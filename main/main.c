#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "battery.h"
#include "button_app.h"
#include "fan_app.h"
#include "imu_app.h"
#include "rgb_app.h"
#include "uart_app.h"

void app_main(void)
{
    // 电池 ADC 初始化（非任务，只需上电初始化一次）
    ESP_ERROR_CHECK(battery_init());

    // main 只负责启动各业务模块的任务，避免堆叠初始化/业务逻辑
    // 各模块内部自行完成初始化与业务循环
    xTaskCreate(uart_app_task, "uart_app", 4096, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(rgb_app_task, "rgb_app", 3072, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(fan_app_task, "fan_app", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(button_app_task, "button_app", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(imu_app_task, "imu_app", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
}
