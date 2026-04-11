#include "uart_bus.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"

#define UART_BUS_RX_BUF_SIZE (1024)
#define UART_BUS_EVT_QUEUE_LEN (20)

static const char *TAG = "uart_bus";

typedef struct {
    bool inited;
    bool started;

    uart_port_t port;
    gpio_num_t tx;
    gpio_num_t rx;

    int baud_rate;

    uart_bus_rx_cb_t rx_cb;
    void *rx_cb_ctx;

    QueueHandle_t evt_queue;
    TaskHandle_t rx_task;
} uart_bus_ctx_t;

static uart_bus_ctx_t s_bus[UART_BUS_ID_MAX] = {
    [UART_BUS_ID_CM4] = {
        .inited = false,
        .started = false,
        .port = UART_NUM_1,
        .tx = GPIO_NUM_5,
        .rx = GPIO_NUM_4,
        .baud_rate = 115200,
        .rx_cb = NULL,
        .rx_cb_ctx = NULL,
        .evt_queue = NULL,
        .rx_task = NULL,
    },
    [UART_BUS_ID_EXT] = {
        .inited = false,
        .started = false,
        .port = UART_NUM_2,
        .tx = GPIO_NUM_14,
        .rx = GPIO_NUM_13,
        .baud_rate = 115200,
        .rx_cb = NULL,
        .rx_cb_ctx = NULL,
        .evt_queue = NULL,
        .rx_task = NULL,
    },
};

static void uart_bus_rx_task(void *arg)
{
    uart_bus_id_t id = (uart_bus_id_t)(uintptr_t)arg;
    uart_bus_ctx_t *b = &s_bus[id];

    uint8_t *buf = (uint8_t *)malloc(256);
    if (!buf) {
        ESP_LOGE(TAG, "malloc rx buf failed (id=%d)", id);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "rx task running (id=%d)", id);

    uart_event_t evt;
    while (1) {
        if (xQueueReceive(b->evt_queue, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (evt.type == UART_DATA && evt.size > 0) {
            // 有数据可读（包括 FIFO 超时触发的 UART_DATA 事件）
            size_t remaining = evt.size;
            while (remaining > 0) {
                size_t to_read = (remaining > 256) ? 256 : remaining;
                int rx_len = uart_read_bytes(b->port, buf, to_read, pdMS_TO_TICKS(10));
                if (rx_len > 0 && b->rx_cb) {
                    b->rx_cb(id, buf, (size_t)rx_len, b->rx_cb_ctx);
                }
                if (rx_len <= 0) break;
                remaining -= (size_t)rx_len;
            }
        }
    }
}

esp_err_t uart_bus_init(uart_bus_id_t id, const uart_bus_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(id >= 0 && id < UART_BUS_ID_MAX, ESP_ERR_INVALID_ARG, TAG, "invalid id");

    uart_bus_ctx_t *b = &s_bus[id];
    if (b->inited) {
        return ESP_OK;
    }

    if (cfg && cfg->baud_rate > 0) {
        b->baud_rate = cfg->baud_rate;
    }

    uart_config_t uart_config = {
        .baud_rate = b->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(b->port, UART_BUS_RX_BUF_SIZE, 0, UART_BUS_EVT_QUEUE_LEN, &b->evt_queue, 0), TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(b->port, &uart_config), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(b->port, b->tx, b->rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart_set_pin failed");

    b->inited = true;
    return ESP_OK;
}

esp_err_t uart_bus_register_rx_cb(uart_bus_id_t id, uart_bus_rx_cb_t cb, void *user_ctx)
{
    ESP_RETURN_ON_FALSE(id >= 0 && id < UART_BUS_ID_MAX, ESP_ERR_INVALID_ARG, TAG, "invalid id");

    s_bus[id].rx_cb = cb;
    s_bus[id].rx_cb_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t uart_bus_start(uart_bus_id_t id)
{
    ESP_RETURN_ON_FALSE(id >= 0 && id < UART_BUS_ID_MAX, ESP_ERR_INVALID_ARG, TAG, "invalid id");

    uart_bus_ctx_t *b = &s_bus[id];
    ESP_RETURN_ON_FALSE(b->inited, ESP_ERR_INVALID_STATE, TAG, "bus not inited");

    if (b->started) {
        return ESP_OK;
    }

    const char *task_name = (id == UART_BUS_ID_CM4) ? "uart_cm4_rx" : "uart_ext_rx";
    BaseType_t ok = xTaskCreate(uart_bus_rx_task, task_name, 4096, (void *)(uintptr_t)id, tskIDLE_PRIORITY + 4, &b->rx_task);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "create rx task failed");

    b->started = true;
    return ESP_OK;
}

int uart_bus_write(uart_bus_id_t id, const void *data, size_t len)
{
    if (id < 0 || id >= UART_BUS_ID_MAX || !data || len == 0) {
        return -1;
    }

    uart_bus_ctx_t *b = &s_bus[id];
    if (!b->inited) {
        return -1;
    }

    return uart_write_bytes(b->port, data, len);
}

