#include "fan_app.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "fan.h"

static const char *TAG = "fan_app";

void fan_app_task(void *arg)
{
    (void)arg;

    // fan 模块入口：初始化 PWM 风扇
    ESP_ERROR_CHECK(fan_init());

    ESP_LOGI(TAG, "running");

    // 常驻任务：后续可在这里接入队列/事件，动态调速
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

