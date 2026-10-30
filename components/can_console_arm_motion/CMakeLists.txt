idf_component_register(
    SRCS "src/can_console_arm_motion.c" "src/arm_trajectory.c"
         "src/arm_gravity.c" "src/arm_gravity_model.c" "src/arm_teach.c"
         "src/arm_joint_path.c"
         "src/arm_stop.c"
         "src/arm_teach_file.c"
         "src/arm_playback.c"
         "src/arm_gripper.c"
    INCLUDE_DIRS "include"
    REQUIRES can_console_ik can_console_motor freertos esp_timer log
    PRIV_REQUIRES heap
)
