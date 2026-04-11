#include "rgb_app.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"
#include "rgb_control.h"
#include "rgb_effects.h"
#include "rgb_service.h"
#include "ws2812.h"

static const char *TAG = "rgb_app";

static void apply_default_effects(void)
{
    // 默认效果参数（亮度/速度/颜色等）
    // 注意：是否真正点亮由 rgb_control_set_enabled(true/false) 控制
    rgb_effect_config_t cfg = {
        .effect = RGB_EFFECT_AURORA,
        .brightness = BOARD_RGB_DEFAULT_BRIGHTNESS,
        .speed_ms = BOARD_RGB_DEFAULT_SPEED_MS,
        .r = BOARD_RGB_DEFAULT_R,
        .g = BOARD_RGB_DEFAULT_G,
        .b = BOARD_RGB_DEFAULT_B,
        .param = 0,
    };
    ESP_ERROR_CHECK(rgb_service_set_effect(&cfg));
}

void rgb_app_task(void *arg)
{
    (void)arg;

    // 这里是 RGB 模块的“总入口任务”：
    // 1) 从 board_config.h 读取硬件参数（WS2812 的 GPIO、灯珠数量等）
    // 2) 初始化底层 ws2812(RMT)
    // 3) 启动上层动画/效果任务（rgb_service）
    ws2812_config_t cfg = {
        .gpio_num = BOARD_WS2812_GPIO,
        .led_count = BOARD_WS2812_LED_COUNT,
        .enable_level = 0,
    };

    ESP_ERROR_CHECK(ws2812_init(&cfg));
    ESP_ERROR_CHECK(rgb_service_start());

    // 设置默认效果参数（亮度/速度/颜色等）
    apply_default_effects();

    ESP_LOGI(TAG, "running");

    // 上电默认关闭：先灭灯
    ESP_ERROR_CHECK(rgb_control_set_enabled(false));

    // 演示轮播：只有在按键开启后才会运行
    const rgb_effect_t effects[] = {
        RGB_EFFECT_AURORA,
        RGB_EFFECT_RAINBOW,
        RGB_EFFECT_CHASE,
        RGB_EFFECT_BREATH,
        RGB_EFFECT_TEMPERATURE,
    };

    int idx = 0;
    while (1) {
        if (rgb_control_get_enabled()) {
            rgb_effect_t mode = effects[idx];
            ESP_LOGI(TAG, "switch effect -> %d", mode);
            ESP_ERROR_CHECK(rgb_service_set_mode(mode));

            idx = (idx + 1) % (sizeof(effects) / sizeof(effects[0]));
            vTaskDelay(pdMS_TO_TICKS(5000));
        } else {
            // 关闭状态下不切换效果，保持灭灯
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
