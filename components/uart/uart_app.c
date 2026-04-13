#include "uart_app.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uart_bus.h"
#include "uart_protocol.h"

static const char *TAG = "uart_app";

// 每条总线各自独立的帧解析器
static proto_parser_t s_parser_cm4;
static proto_parser_t s_parser_ext;

static void on_uart_rx(uart_bus_id_t id, const uint8_t *data, size_t len, void *user_ctx)
{
    (void)user_ctx;

    // -------- 原始字节抓包（调试用，确认后删除）--------
    {
        const size_t dump_max = (len < 32) ? len : 32;  // 最多打印 32 字节
        char hex[32 * 3 + 1];
        for (size_t i = 0; i < dump_max; i++) {
            sprintf(hex + i * 3, "%02X ", data[i]);
        }
        hex[dump_max * 3] = '\0';
        ESP_LOGW(TAG, "[bus%d] RAW %u bytes: %s%s",
                 (int)id, (unsigned)len, hex, (len > dump_max) ? "..." : "");
    }
    // ---------------------------------------------------

    // 根据来源总线选择对应的解析器（解析器内带 bus_id，回包时同口回）
    if (id == UART_BUS_ID_CM4) {
        proto_parser_feed(&s_parser_cm4, data, len);
    } else if (id == UART_BUS_ID_EXT) {
        proto_parser_feed(&s_parser_ext, data, len);
    }
}

void uart_app_task(void *arg)
{
    (void)arg;

    proto_parser_init(&s_parser_cm4, (int)UART_BUS_ID_CM4);
    proto_parser_init(&s_parser_ext, (int)UART_BUS_ID_EXT);

    uart_bus_config_t cfg = {
        .baud_rate = 115200,
    };

    // 双口同时监听：哪个口收到请求就从哪个口回
    ESP_ERROR_CHECK(uart_bus_init(UART_BUS_ID_CM4, &cfg));
    ESP_ERROR_CHECK(uart_bus_register_rx_cb(UART_BUS_ID_CM4, on_uart_rx, NULL));
    ESP_ERROR_CHECK(uart_bus_start(UART_BUS_ID_CM4));

    // EXT 总线(UART2) 已交由 motor 组件 (dxl_proto) 管理，不在此处初始化

    ESP_LOGI(TAG, "CM4 bus listening: UART1 TX=GPIO5 RX=GPIO4");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

