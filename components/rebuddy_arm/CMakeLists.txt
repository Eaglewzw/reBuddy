idf_component_register(
    SRCS "src/rb_arm.c"
    INCLUDE_DIRS "include"
    REQUIRES can_console_arm_motion can_console_motor can_console_can can_console_bsp
    PRIV_REQUIRES rebuddy_board esp_timer)
