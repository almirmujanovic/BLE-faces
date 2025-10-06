import asyncio
import struct
from bleak import BleakClient, BleakScanner
from PIL import Image
import numpy as np

# UUIDs based on ESP32 definitions 
# in main/include/ble_image_transfer.h

# BLE_IMG_INFO_UUID128
INFO_UUID = "00006e40-0003-b5a3-f393-e0a9e54dca3e"  
# BLE_IMG_DATA_UUID128
DATA_UUID = "00006e40-0004-b5a3-f393-e0a9e54dca3e"  

class Reassembler:
    def __init__(self):
        self.current_frame = None
        self.data_buffer = bytearray()
        self.expected_len = 0
        
    def handle_info(self, data):
        """Handle image info packet"""
        if len(data) < 22:
            print(f"[WARN] INFO packet too short: {len(data)} bytes, expected 22")
            return
            
        # Unpack ble_img_info_t struct (22 bytes total)
        version, fmt, width, height, x, y, total_len, frame_id, crc32 = struct.unpack('<BBHHHHIII', data[:22])
        
        print(f"[INFO] frame {frame_id} {width}x{height} fmt={fmt} len={total_len} roi=({x},{y})")
        
        self.current_frame = {
            'frame_id': frame_id,
            'width': width,
            'height': height,
            'x': x, 'y': y,
            'format': fmt,
            'total_len': total_len,
            'crc32': crc32
        }
        self.data_buffer = bytearray()
        self.expected_len = total_len
        
    def handle_data(self, data):
        """Handle image data chunk"""
        if not self.current_frame:
            print("[WARN] Got DATA without INFO")
            return
            
        self.data_buffer.extend(data)
        progress = len(self.data_buffer) / self.expected_len * 100
        print(f"[DATA] {len(self.data_buffer)}/{self.expected_len} bytes ({progress:.1f}%)")
        
        # Check if we have all data
        if len(self.data_buffer) >= self.expected_len:
            self.reconstruct_image()
            
    def reconstruct_image(self):
        """Reconstruct RGB565 image using byte-swapped decoding"""
        frame = self.current_frame
        data = self.data_buffer[:self.expected_len]  # Trim any excess
        
        width, height = frame['width'], frame['height']
        expected_bytes = width * height * 2  # RGB565 = 2 bytes per pixel
        
        if len(data) != expected_bytes:
            print(f"[ERROR] Data length mismatch: got {len(data)}, expected {expected_bytes}")
            return
            
        try:
            # Method 3: Byte-swapped RGB565 (primary working method)
            swapped = bytearray()
            for i in range(0, len(data), 2):
                if i + 1 < len(data):
                    swapped.extend([data[i+1], data[i]])
            rgb565 = np.frombuffer(swapped, dtype=np.uint16).reshape(height, width)
            
            # Alternative Method 2: Big-endian RGB565 
            # rgb565 = np.frombuffer(data, dtype='>u2').reshape(height, width)
            # For reliablity, keep using primary method (byte-swapped)

            # Convert RGB565 to RGB888
            r = ((rgb565 & 0xF800) >> 11) << 3
            g = ((rgb565 & 0x07E0) >> 5) << 2
            b = (rgb565 & 0x001F) << 3
            
            # Expand to full 8-bit range
            r = r | (r >> 5)
            g = g | (g >> 6)
            b = b | (b >> 5)
            
            # Stack into RGB image
            rgb = np.stack([r, g, b], axis=2).astype(np.uint8)
            
            # Create PIL image
            img = Image.fromarray(rgb, mode="RGB")
            
            # Save image with clean filename
            filename = f"frame_{frame['frame_id']}_{width}x{height}.png"
            img.save(filename)
            print(f"[SAVED] {filename} (using byte-swapped decode)")
            
        except Exception as e:
            print(f"[ERROR] Failed to reconstruct image: {e}")
            # Debug: Save raw data for analysis
            with open(f"debug_frame_{frame['frame_id']}.raw", 'wb') as f:
                f.write(data)
            print(f"[DEBUG] Saved raw data for analysis")
        finally:
            # Reset for next frame
            self.current_frame = None
            self.data_buffer = bytearray()
            self.expected_len = 0

async def info_handler(sender, data):
    """Handle INFO characteristic notifications"""
    reassembler.handle_info(data)

async def data_handler(sender, data):
    """Handle DATA characteristic notifications"""
    reassembler.handle_data(data)

# Global reassembler instance
reassembler = Reassembler()

async def run(addr=None, name_hint="ESP-Face-Detector"):
    """Main client function"""
    
    if not addr:
        print("🔍 Scanning for ESP32...")
        devices = await BleakScanner.discover(timeout=5.0)
        for d in devices:
            if name_hint in (d.name or ""):
                addr = d.address
                print(f"📡 Found {d.name} at {addr}")
                break
        else:
            print(f"No device found with name containing '{name_hint}'")
            return
    
    print(f"Connecting to {addr}...")
    
    async with BleakClient(addr) as client:
        if not client.is_connected:
            print("Failed to connect")
            return
            
        print(f"Connected to {addr}")
        
        # Subscribe to notifications
        await client.start_notify(INFO_UUID, info_handler)
        print("Subscribed to INFO")
        
        await client.start_notify(DATA_UUID, data_handler)
        print("Subscribed to DATA")
        
        print("Using byte-swapped RGB565 decoding")
        print("\nReady! Point a face at the ESP32 camera...")
        print("Press Ctrl+C to quit.")
        
        # Keep alive and handle notifications
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nDisconnecting...")

if __name__ == "__main__":
    # Optional: specify exact MAC address if you know it
    asyncio.run(run("74:4D:BD:89:2C:92"))
    # Or scan for device:
    # asyncio.run(run())