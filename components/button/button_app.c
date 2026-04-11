#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"
#include "button.h"
#include "rgb_control.h"
#include "rgb_service.h"

static const char *TAG = "button_app";

static bool s_rgb_enabled;

static void on_button(bool pressed, void *user_ctx)
{
    (void)user_ctx;

    // EN 引脚：低电平有效。
    // 你的需求：默认关闭RGB；按一下切换一次开/关（toggle）
    // pressed==true 表示“按下事件”（有效边沿），只在此时切换状态
    if (pressed) {
        s_rgb_enabled = !s_rgb_enabled;
        ESP_LOGI(TAG, "RGB toggle -> %s", s_rgb_enabled ? "ON" : "OFF");
        (void)rgb_control_set_enabled(s_rgb_enabled);
    }
}

void button_app_task(void *arg)
{
    (void)arg;

    // 上电默认关闭RGB
    s_rgb_enabled = false;
    ESP_ERROR_CHECK(rgb_control_set_enabled(false));

    ESP_ERROR_CHECK(button_init(BOARD_BTN_EN_GPIO, true));
    ESP_ERROR_CHECK(button_register_callback(on_button, NULL));

    ESP_LOGI(TAG, "running");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
