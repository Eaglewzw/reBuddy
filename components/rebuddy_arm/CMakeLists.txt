idf_component_register(
    SRCS "src/rb_arm.c" "src/rb_daily_control.c" "src/rb_arm_daily.c" "src/rb_daily_screen.c"
    INCLUDE_DIRS "include"
    REQUIRES can_console_arm_motion can_console_motor can_console_can can_console_bsp rebuddy_core
    PRIV_REQUIRES rebuddy_board esp_timer)
