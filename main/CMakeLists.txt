idf_component_register(
    SRCS
        "app_main.c"
        "camera_config.c"
        "face_detection.cpp"
        "ble_server.c"
    INCLUDE_DIRS
        "." "include"
    REQUIRES
        bt
        nvs_flash
        esp_timer
        esp32-camera
        human_face_detect
        esp-dl
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)