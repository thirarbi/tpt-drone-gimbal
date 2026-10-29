idf_component_register(
    SRCS "dxl_packet.c" "xl320.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_timer
)