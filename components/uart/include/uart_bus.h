#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART_BUS_ID_CM4 = 0,   // RX=GPIO4  TX=GPIO5
    UART_BUS_ID_EXT = 1,   // RX=GPIO13 TX=GPIO14
    UART_BUS_ID_MAX,
} uart_bus_id_t;

typedef struct {
    int baud_rate;
} uart_bus_config_t;

typedef void (*uart_bus_rx_cb_t)(uart_bus_id_t id, const uint8_t *data, size_t len, void *user_ctx);

esp_err_t uart_bus_init(uart_bus_id_t id, const uart_bus_config_t *cfg);

esp_err_t uart_bus_start(uart_bus_id_t id);

esp_err_t uart_bus_register_rx_cb(uart_bus_id_t id, uart_bus_rx_cb_t cb, void *user_ctx);

int uart_bus_write(uart_bus_id_t id, const void *data, size_t len);

#ifdef __cplusplus
}
#endif
