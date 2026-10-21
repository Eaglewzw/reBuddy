idf_build_get_property(rb_build_dir BUILD_DIR)
idf_component_register(
    KCONFIG_PROJBUILD "${rb_build_dir}/rebuddy.Kconfig.projbuild"
    SRCS "main.c" "pet_ui.c" "character_asset.c"
        "${REBUDDY_CONSOLE_DIR}/main/platform/console_board.c"
        "${REBUDDY_CONSOLE_DIR}/main/platform/console_memory.c"
        "${REBUDDY_CONSOLE_DIR}/main/input/console_input.c"
        "${REBUDDY_CONSOLE_DIR}/main/communication/console_can.c"
    INCLUDE_DIRS "." "${REBUDDY_CONSOLE_DIR}/main"
    REQUIRES rebuddy_core ui_console_display ui_console_bsp ui_console_can
        ui_console_iicbus ui_console_ioexpansion lvgl esp_psram esp_timer nvs_flash)
