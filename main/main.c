#include "esp_check.h"

#include "battery.h"
#include "ble_control.h"

void app_main(void)
{
    ESP_ERROR_CHECK(battery_init());
    ESP_ERROR_CHECK(ble_control_init());
}
