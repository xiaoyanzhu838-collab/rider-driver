idf_component_register(
    SRCS "ble_control.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash motor balance
)
