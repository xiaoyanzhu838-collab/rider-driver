#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "battery.h"
#include "motor_test.h"

void app_main(void)
{
    // 电池 ADC 初始化（非任务，只需上电初始化一次）
    ESP_ERROR_CHECK(battery_init());

    // 本轮实验固件只拉起电机诊断任务，避免其他模块占用日志与外设。
    xTaskCreate(motor_test_task, "motor_test", 8192, NULL, tskIDLE_PRIORITY + 4, NULL);
}
