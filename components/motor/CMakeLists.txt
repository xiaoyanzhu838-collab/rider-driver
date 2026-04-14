idf_component_register(
    SRCS "dxl_proto.c" "scs_proto.c" "motor_control.c" "motor_test.c" "motor_tuner.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_common
)
