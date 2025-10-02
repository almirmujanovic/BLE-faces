import asyncio, struct, argparse, time
from bleak import BleakClient, BleakScanner
import numpy as np
from PIL import Image

# CORRECT UUIDs based on your ESP32 definitions
INFO_UUID = "00006e40-0003-b5a3-f393-e0a9e54dca3e"  # BLE_IMG_INFO_UUID128
DATA_UUID = "00006e40-0004-b5a3-f393-e0a9e54dca3e"  # BLE_IMG_DATA_UUID128

class Reassembler:
    def __init__(self):
        self.expect = 0
        self.buf = bytearray()
        self.meta = None

    def on_info(self, sender, data: bytes):
        if len(data) != 22:  # Correct size is 22 bytes
            print(f"[ERROR] INFO packet wrong size: {len(data)} (expected 22)")
            return
        
        # Parse all 22 bytes: BBHHHHIII = 22 bytes total
        self.meta = struct.unpack("<BBHHHHIII", data)
        _, fmt, w, h, x, y, total_len, frame_id, crc32 = self.meta
        print(f"[INFO] frame {frame_id} {w}x{h} fmt={fmt} len={total_len} roi=({x},{y})")
        self.expect = total_len
        self.buf = bytearray()

    def on_data(self, sender, data: bytes):
        if self.expect == 0:
            print("[WARN] Got DATA without INFO")
            return
        self.buf += data
        print(f"[DATA] {len(self.buf)}/{self.expect} bytes")
        if len(self.buf) >= self.expect:
            payload = bytes(self.buf[:self.expect])
            self.expect = 0
            self.render(payload)

    def render(self, payload: bytes):
        _, fmt, w, h, x, y, total_len, frame_id, crc32 = self.meta
        if fmt == 0: # RGB565
            if len(payload) != w * h * 2:
                print(f"[ERROR] Size mismatch: got {len(payload)}, expected {w*h*2}")
                return
            arr = np.frombuffer(payload, dtype='<u2').reshape((h, w))  # Little-endian
            # split 5:6:5 and scale to 8-bit
            r = ((arr >> 11) & 0x1F) << 3  # 5->8 bits
            g = ((arr >> 5)  & 0x3F) << 2  # 6->8 bits  
            b = (arr         & 0x1F) << 3  # 5->8 bits
            rgb = np.dstack((r, g, b)).astype(np.uint8)
            img = Image.fromarray(rgb, mode="RGB")
            fn = f"frame_{frame_id}_{w}x{h}.png"
            img.save(fn)
            print(f"[SAVED] {fn}")
        elif fmt == 1: # JPEG
            fn = f"frame_{frame_id}.jpg"
            with open(fn, "wb") as f:
                f.write(payload)
            print(f"[SAVED] {fn}")
        else:
            print(f"[WARN] Unknown format {fmt}")

async def run(addr=None, name_hint="ESP-Face-Detector"):
    if not addr:
        print("Scanning for device...")
        try:
            devs = await BleakScanner.discover(timeout=10.0)
            print(f"Found {len(devs)} devices:")
            for d in devs:
                name = d.name or "Unknown"
                print(f"  {name} - {d.address}")
                if name_hint and name_hint.lower() in name.lower():
                    addr = d.address
                    print(f"  ↳ Selected: {name}")
            if not addr:
                print("No matching device found")
                return
        except Exception as e:
            print(f"Scan failed: {e}")
            return

    print(f"\nConnecting to {addr}...")
    r = Reassembler()
    
    try:
        async with BleakClient(addr, timeout=20.0) as cli:
            print(f"✅ Connected to {addr}")
            
            # DEBUG: List all available services and characteristics
            print("\n📋 Available services:")
            info_found = data_found = False
            
            for service in cli.services:
                print(f"  Service: {service.uuid}")
                for char in service.characteristics:
                    uuid_str = str(char.uuid).lower()
                    props = ", ".join(char.properties)
                    print(f"    {uuid_str} ({props})")
                    
                    # Check if our UUIDs are found
                    if uuid_str == INFO_UUID.lower():
                        info_found = True
                        print(f"      ↳ Found INFO characteristic!")
                    if uuid_str == DATA_UUID.lower():
                        data_found = True
                        print(f"      ↳ Found DATA characteristic!")
            
            if not info_found:
                print(f"\n❌ INFO UUID not found: {INFO_UUID}")
                print("Looking for notify-capable characteristics:")
                for service in cli.services:
                    for char in service.characteristics:
                        if "notify" in char.properties:
                            print(f"  {char.uuid} (can notify)")
                return
                
            if not data_found:
                print(f"\n❌ DATA UUID not found: {DATA_UUID}")
                return
            
            print(f"\n✅ Both characteristics found!")
            
            # Subscribe to notifications
            await cli.start_notify(INFO_UUID, r.on_info)
            print("✅ Subscribed to INFO")
            
            await cli.start_notify(DATA_UUID, r.on_data)
            print("✅ Subscribed to DATA")
            
            print("\n🎯 Ready! Point a face at the ESP32 camera...")
            print("Press Ctrl+C to quit.\n")
            
            try:
                while True:
                    await asyncio.sleep(1.0)
            except KeyboardInterrupt:
                print("\n👋 Disconnecting...")
                
    except Exception as e:
        print(f"❌ Connection failed: {e}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", help="BLE MAC/UUID of ESP")
    ap.add_argument("--name", default="ESP-Face-Detector")
    args = ap.parse_args()
    
    try:
        asyncio.run(run(args.addr, args.name))
    except KeyboardInterrupt:
        print("👋 Goodbye!")