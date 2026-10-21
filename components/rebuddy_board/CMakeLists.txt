idf_component_register(
    SRCS "src/rb_board.c" "src/rb_board_input.c" "src/rb_memory.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_spi esp_lcd
        esp_lcd_touch_gt911 esp_lvgl_port lvgl esp_psram esp_timer)
