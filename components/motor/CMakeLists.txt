idf_component_register(
    SRCS "scs_proto.c" "motor_control.c" "motor_test.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_common
)
