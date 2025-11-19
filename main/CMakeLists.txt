idf_component_register(
    SRCS
        "app_main.c"
        "camera_config.c"
        "face_detection.cpp"
        "ble_server.c"
        "ble_image_transfer.c"
        "gesture.c"
        "ble_hid.c"
        "ble_gatts_utils.c"
        "gesture_ei.c"
        "ei_infer.cpp"
    INCLUDE_DIRS
        "." "include"
    REQUIRES
        bt
        nvs_flash
        esp_timer
        esp32-camera
        human_face_detect
        esp-dl
        ei
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Suppress warnings from Edge Impulse SDK headers (they contain template code compiled here)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-parameter
    -Wno-unused-but-set-parameter
    -Wno-missing-field-initializers
    -Wno-maybe-uninitialized
    -Wno-type-limits
    -Wno-deprecated-declarations
)