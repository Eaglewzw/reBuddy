idf_component_register(
    SRCS "choreo_ui.c" "choreo_store.c" "main.c" "pet_ui.c" "pet_assets.c" "pet_animation_table.c" "room_font.c"
    INCLUDE_DIRS "."
    EMBED_FILES "../assets/animation/frames.bin" "../assets/animation/room.bin"
    REQUIRES rebuddy_core rebuddy_board rebuddy_arm lvgl esp_timer spi_flash esp_partition nvs_flash)
