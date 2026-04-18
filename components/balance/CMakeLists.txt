idf_component_register(
    SRCS "wheel_control.c" "balance_task.c"
    INCLUDE_DIRS "include"
    REQUIRES board freertos imu motor uart
)
