idf_component_register(
    SRCS
        "config/app_config.c"
        "core/motor_manager.c"
        "core/control_loop.c"
        "core/safety_manager.c"
        "protocol/host_protocol.c"
        "service/can_console_motor_client.c"
        "service/can_console_motor_runtime.c"
        "service/frame_router.c"
        "service/motor_service.c"
        "service/motor_session.c"
        "transport/can_manager.c"
        "vendors/motor_vendor.c"
        "vendors/damiao/damiao_protocol.c"
        "vendors/damiao/damiao_vendor.c"
        "vendors/robstride/robstride_protocol.c"
        "vendors/robstride/robstride_vendor.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES can_console_can driver esp_driver_twai esp_timer freertos esp_rom
)
