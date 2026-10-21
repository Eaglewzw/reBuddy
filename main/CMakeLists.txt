idf_component_register(
    SRCS "main.c" "pet_ui.c" "character_asset.c"
    INCLUDE_DIRS "."
    REQUIRES rebuddy_core rebuddy_board lvgl esp_timer nvs_flash)
