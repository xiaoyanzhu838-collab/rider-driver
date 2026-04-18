#include <assert.h>

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "balance_task.h"
#include "battery.h"
#include "ble_control.h"
#include "board_config.h"
#include "imu_app.h"
#include "uart_app.h"

static void start_task(TaskFunction_t entry,
                       const char *name,
                       uint32_t stack_size,
                       UBaseType_t priority)
{
    BaseType_t ok = xTaskCreate(entry, name, stack_size, NULL, priority, NULL);
    assert(ok == pdPASS);
}

void app_main(void)
{
#if BOARD_IMU_DIAGNOSTIC_MODE
    start_task(imu_app_task, "imu_app", 4096, 6);
#else
    ESP_ERROR_CHECK(battery_init());
    ESP_ERROR_CHECK(ble_control_init());

    start_task(imu_app_task, "imu_app", 4096, 6);
    start_task(uart_app_task, "uart_app", 4096, 5);
    start_task(balance_task, "balance", 4096, 7);
#endif
}
