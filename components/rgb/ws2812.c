#include "ws2812.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "led_strip.h"

static const char *TAG = "ws2812";

static led_strip_handle_t s_strip;
static int s_led_count;

esp_err_t ws2812_init(const ws2812_config_t *cfg)
{
    // 初始化 WS2812 驱动（使用 led_strip 组件 + RMT 外设输出时序）
    // 这里只做“底层硬件初始化”，不包含任何动画/业务逻辑。
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");
    ESP_RETURN_ON_FALSE(cfg->led_count > 0, ESP_ERR_INVALID_ARG, TAG, "invalid led_count");

    // 避免重复初始化：如果已经创建过 strip 句柄，直接返回
    if (s_strip != NULL) {
        return ESP_OK;
    }

    // WS2812 常见为 GRB 排列
    led_strip_config_t strip_config = {
        .strip_gpio_num = cfg->gpio_num,
        .max_leds = cfg->led_count,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = 0,
    };

    // RMT 分辨率：10MHz（每 tick 0.1us），足够生成 WS2812 的时序
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags.with_dma = 0,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    ESP_RETURN_ON_ERROR(err, TAG, "create led_strip failed");

    s_led_count = cfg->led_count;

    // 上电先清灯，避免残影
    return ws2812_clear();
}

esp_err_t ws2812_set_pixel(int index, uint8_t r, uint8_t g, uint8_t b)
{
    ESP_RETURN_ON_FALSE(s_strip != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(index >= 0 && index < s_led_count, ESP_ERR_INVALID_ARG, TAG, "index out of range");

    return led_strip_set_pixel(s_strip, index, r, g, b);
}

esp_err_t ws2812_clear(void)
{
    ESP_RETURN_ON_FALSE(s_strip != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    esp_err_t err = led_strip_clear(s_strip);
    ESP_RETURN_ON_ERROR(err, TAG, "clear failed");
    return led_strip_refresh(s_strip);
}

esp_err_t ws2812_refresh(void)
{
    ESP_RETURN_ON_FALSE(s_strip != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    return led_strip_refresh(s_strip);
}

void ws2812_hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (!r || !g || !b) return;

    if (s == 0) {
        *r = v;
        *g = v;
        *b = v;
        return;
    }

    uint8_t region = h / 43;
    uint8_t remainder = (h - (region * 43)) * 6;

    uint8_t p = (uint8_t)((v * (255 - s)) >> 8);
    uint8_t q = (uint8_t)((v * (255 - ((s * remainder) >> 8))) >> 8);
    uint8_t t = (uint8_t)((v * (255 - ((s * (255 - remainder)) >> 8))) >> 8);

    switch (region) {
        case 0:
            *r = v; *g = t; *b = p;
            break;
        case 1:
            *r = q; *g = v; *b = p;
            break;
        case 2:
            *r = p; *g = v; *b = t;
            break;
        case 3:
            *r = p; *g = q; *b = v;
            break;
        case 4:
            *r = t; *g = p; *b = v;
            break;
        default:
            *r = v; *g = p; *b = q;
            break;
    }
}
