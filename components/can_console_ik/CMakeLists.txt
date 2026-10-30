idf_component_register(
    SRCS "src/can_console_ik.c" "src/can_console_pose_math.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer log
)
