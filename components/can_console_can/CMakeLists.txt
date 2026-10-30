idf_component_register(
    SRCS "src/can_console_can.c" "src/can_console_can_tx_sync.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_twai esp_timer can_console_bsp
)
