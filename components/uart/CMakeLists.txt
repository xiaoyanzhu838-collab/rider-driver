idf_component_register(
    SRCS "uart_protocol.c" "uart_bus.c" "uart_app.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_driver_gpio freertos imu rgb board motor
)
