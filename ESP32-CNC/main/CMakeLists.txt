idf_component_register(
    SRCS
        "main.c"
        "wifi_manager.c"
        "http_manager.c"
        "gcode_parser.c"
        "uart_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_event
        nvs_flash
        esp_netif
        esp_http_server
        spiffs
        driver
        freertos
        
)