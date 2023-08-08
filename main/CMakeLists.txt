set(srcs "main.c"
        "key.c" "my_common.c" "mpu6050.c" "ws2812_encoder.c" "ws2812.c" "quaternion_filter.c"
        "font8.c" "font12.c" "font16.c" "font_hzk16.c"
        "wifi/wifi_ap.c" "wifi/my_wifi_ota.c" "wifi/my_mount.c" "wifi/my_http_server.c" "wifi/my_http_file_server.c")

idf_component_register(SRCS ${srcs}
        EMBED_FILES "HZK16.bin"
        EMBED_FILES "static/favicon.ico" "static/ota.html"  "static/upload_script.html"
        INCLUDE_DIRS ".")
