idf_component_register(
    SRCS
        "app/main.c"
        "app/app_arm.c" "app/app_dance.c" "app/app_screen.c" "app/app_diagnostics.c"
        "ui/pet_ui.c" "ui/choreo_ui.c"
        "storage/choreo_store.c"
        "animation/pet_assets.c"
        "generated/pet_animation_table.c" "generated/room_font.c"
    INCLUDE_DIRS "ui" "storage" "animation" "generated"
    EMBED_FILES "../assets/animation/frames.bin" "../assets/animation/room.bin"
    REQUIRES rebuddy_core rebuddy_board rebuddy_arm lvgl esp_timer esp_hw_support spi_flash esp_partition nvs_flash)
