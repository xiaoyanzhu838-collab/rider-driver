#include "rgb_service.h"

#include <stdatomic.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"
#include "rgb_effects_internal.h"
#include "ws2812.h"

static const char *TAG = "rgb_service";

static TaskHandle_t s_task;
static atomic_bool s_running;
static portMUX_TYPE s_cfg_mux = portMUX_INITIALIZER_UNLOCKED;

static rgb_effect_config_t s_cfg = {
    .effect = RGB_EFFECT_RAINBOW,
    .brightness = BOARD_RGB_DEFAULT_BRIGHTNESS,
    .speed_ms = BOARD_RGB_DEFAULT_SPEED_MS,
    .r = BOARD_RGB_DEFAULT_R,
    .g = BOARD_RGB_DEFAULT_G,
    .b = BOARD_RGB_DEFAULT_B,
};

static inline uint8_t tri_u8(uint16_t x)
{
    x %= 512;
    return (x < 256) ? (uint8_t)x : (uint8_t)(511 - x);
}

static void render_background(int leds, uint8_t brightness)
{
    // 暗底色：深蓝偏青，避免“全灭”的突兀
    uint8_t r = 0, g = 6, b = 16;
    rgb_apply_brightness(brightness, &r, &g, &b);
    rgb_gamma_correct(&r, &g, &b);

    for (int i = 0; i < leds; i++) {
        (void)ws2812_set_pixel(i, r, g, b);
    }
}

static void render_off(int leds)
{
    // 强制全灭：保留该模式，但业务上尽量别用（会很突兀）
    for (int i = 0; i < leds; i++) {
        (void)ws2812_set_pixel(i, 0, 0, 0);
    }
}

static void render_standby(int leds, const rgb_effect_config_t *cfg, uint16_t step)
{
    // 柔和待机：极低亮度的慢速流动，不会出现任何一颗“灭掉”的突兀感
    // 用 aurora 的简化版：低亮度 + 慢速
    uint8_t base_h1 = (uint8_t)(step);
    uint8_t base_h2 = (uint8_t)(step * 2);

    uint8_t standby_br = (cfg->brightness > 12) ? 12 : cfg->brightness; // 待机亮度上限

    for (int i = 0; i < leds; i++) {
        uint8_t r1, g1, b1;
        uint8_t r2, g2, b2;

        ws2812_hsv_to_rgb((uint8_t)(base_h1 + i * 40), 220, 255, &r1, &g1, &b1);
        ws2812_hsv_to_rgb((uint8_t)(base_h2 + i * 70), 255, 255, &r2, &g2, &b2);

        uint16_t r = (uint16_t)r1 + (uint16_t)r2 / 3;
        uint16_t g = (uint16_t)g1 + (uint16_t)g2 / 3;
        uint16_t b = (uint16_t)b1 + (uint16_t)b2 / 3;
        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;

        uint8_t rr = (uint8_t)r;
        uint8_t gg = (uint8_t)g;
        uint8_t bb = (uint8_t)b;

        // 每颗灯轻微不同相位亮度变化（仍然保证不会到 0）
        uint8_t wave = tri_u8((uint16_t)(step * 6 + i * 140));
        uint8_t br = (uint8_t)((((uint16_t)standby_br) * (80 + wave)) / 335); // 约 0.24~0.95 倍

        rgb_apply_brightness(br, &rr, &gg, &bb);
        rgb_gamma_correct(&rr, &gg, &bb);
        (void)ws2812_set_pixel(i, rr, gg, bb);
    }
}

static void render_solid(int leds, const rgb_effect_config_t *cfg)
{
    uint8_t r = cfg->r, g = cfg->g, b = cfg->b;
    rgb_apply_brightness(cfg->brightness, &r, &g, &b);
    rgb_gamma_correct(&r, &g, &b);

    for (int i = 0; i < leds; i++) {
        (void)ws2812_set_pixel(i, r, g, b);
    }
}

static void render_breath(int leds, const rgb_effect_config_t *cfg, uint16_t phase)
{
    // 呼吸也不允许到 0，最低亮度给一个小底
    uint8_t r = cfg->r, g = cfg->g, b = cfg->b;
    uint8_t k = tri_u8(phase);
    uint8_t min_br = 6;
    uint8_t br = (uint8_t)(min_br + ((((uint16_t)cfg->brightness) * k) / 255));

    rgb_apply_brightness(br, &r, &g, &b);
    rgb_gamma_correct(&r, &g, &b);

    for (int i = 0; i < leds; i++) {
        (void)ws2812_set_pixel(i, r, g, b);
    }
}

static void render_rainbow(int leds, const rgb_effect_config_t *cfg, uint8_t base_hue)
{
    for (int i = 0; i < leds; i++) {
        uint8_t r, g, b;
        ws2812_hsv_to_rgb(base_hue + (i * (uint8_t)(256 / (leds > 0 ? leds : 1))), 255, 255, &r, &g, &b);
        rgb_apply_brightness(cfg->brightness, &r, &g, &b);
        rgb_gamma_correct(&r, &g, &b);
        (void)ws2812_set_pixel(i, r, g, b);
    }
}

static void render_chase(int leds, const rgb_effect_config_t *cfg, uint16_t step)
{
    // 跑马灯也不清空：先铺一层暗底色，再叠加高亮头/尾迹
    render_background(leds, 8);

    int head = (leds > 0) ? (step % leds) : 0;
    uint8_t r = cfg->r, g = cfg->g, b = cfg->b;
    rgb_apply_brightness(cfg->brightness, &r, &g, &b);
    rgb_gamma_correct(&r, &g, &b);

    (void)ws2812_set_pixel(head, r, g, b);

    // 多级尾迹（不会出现灭灯）
    if (leds >= 2) {
        uint8_t tr = (uint8_t)(r / 3), tg = (uint8_t)(g / 3), tb = (uint8_t)(b / 3);
        rgb_gamma_correct(&tr, &tg, &tb);
        (void)ws2812_set_pixel((head - 1 + leds) % leds, tr, tg, tb);
    }
    if (leds >= 3) {
        uint8_t tr = (uint8_t)(r / 8), tg = (uint8_t)(g / 8), tb = (uint8_t)(b / 8);
        rgb_gamma_correct(&tr, &tg, &tb);
        (void)ws2812_set_pixel((head - 2 + leds) % leds, tr, tg, tb);
    }
}

static void render_blink(int leds, const rgb_effect_config_t *cfg, uint16_t step)
{
    // 闪烁：熄灭时也保留暗底色
    bool on = ((step / 8) % 2) == 0;
    if (!on) {
        render_background(leds, 6);
        return;
    }

    render_solid(leds, cfg);
}

static void render_temperature(int leds, const rgb_effect_config_t *cfg, uint16_t step)
{
    // 在 2700K~6500K 之间来回摆动
    uint16_t k = (uint16_t)(2700 + ((uint32_t)tri_u8(step * 2) * (6500 - 2700)) / 255);

    uint8_t r, g, b;
    rgb_color_temperature(k, &r, &g, &b);
    rgb_apply_brightness(cfg->brightness, &r, &g, &b);
    rgb_gamma_correct(&r, &g, &b);

    for (int i = 0; i < leds; i++) {
        (void)ws2812_set_pixel(i, r, g, b);
    }
}

static void render_aurora(int leds, const rgb_effect_config_t *cfg, uint16_t step)
{
    uint8_t base_h1 = (uint8_t)(step * 2);
    uint8_t base_h2 = (uint8_t)(step * 5);

    for (int i = 0; i < leds; i++) {
        uint8_t r1, g1, b1;
        uint8_t r2, g2, b2;

        ws2812_hsv_to_rgb((uint8_t)(base_h1 + i * 40), 200, 255, &r1, &g1, &b1);
        ws2812_hsv_to_rgb((uint8_t)(base_h2 + i * 70), 255, 255, &r2, &g2, &b2);

        // 亮度波：不让任何一颗掉到 0
        uint8_t wave = tri_u8((uint16_t)(step * 12 + i * 140));
        uint8_t br = (uint8_t)(8 + ((((uint16_t)cfg->brightness) * wave) / 255));

        uint16_t r = (uint16_t)r1 + (uint16_t)r2 / 2;
        uint16_t g = (uint16_t)g1 + (uint16_t)g2 / 2;
        uint16_t b = (uint16_t)b1 + (uint16_t)b2 / 2;

        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;

        uint8_t rr = (uint8_t)r;
        uint8_t gg = (uint8_t)g;
        uint8_t bb = (uint8_t)b;

        rgb_apply_brightness(br, &rr, &gg, &bb);

        uint8_t w = (uint8_t)(tri_u8((uint16_t)(step * 8 + i * 90)) / 10);
        rr = (uint8_t)((rr + w) > 255 ? 255 : (rr + w));
        gg = (uint8_t)((gg + w) > 255 ? 255 : (gg + w));
        bb = (uint8_t)((bb + w) > 255 ? 255 : (bb + w));

        rgb_gamma_correct(&rr, &gg, &bb);
        (void)ws2812_set_pixel(i, rr, gg, bb);
    }
}

static void rgb_task(void *arg)
{
    (void)arg;

    uint16_t step = 0;
    uint8_t hue = 0;

    while (atomic_load(&s_running)) {
        rgb_effect_config_t cfg;
        portENTER_CRITICAL(&s_cfg_mux);
        cfg = s_cfg;
        portEXIT_CRITICAL(&s_cfg_mux);

        int leds = BOARD_WS2812_LED_COUNT;
        uint16_t delay_ms = (cfg.speed_ms == 0) ? 1 : cfg.speed_ms;

        switch (cfg.effect) {
            case RGB_EFFECT_OFF:
                render_off(leds);
                break;
            case RGB_EFFECT_STANDBY:
                render_standby(leds, &cfg, step);
                break;
            case RGB_EFFECT_SOLID:
                render_solid(leds, &cfg);
                break;
            case RGB_EFFECT_BREATH:
                render_breath(leds, &cfg, step * 8);
                break;
            case RGB_EFFECT_RAINBOW:
                render_rainbow(leds, &cfg, hue);
                hue++;
                break;
            case RGB_EFFECT_CHASE:
                render_chase(leds, &cfg, step);
                break;
            case RGB_EFFECT_BLINK:
                render_blink(leds, &cfg, step);
                break;
            case RGB_EFFECT_TEMPERATURE:
                render_temperature(leds, &cfg, step);
                break;
            case RGB_EFFECT_AURORA:
                render_aurora(leds, &cfg, step);
                break;
            default:
                render_standby(leds, &cfg, step);
                break;
        }

        (void)ws2812_refresh();

        step++;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    (void)ws2812_clear();

    TaskHandle_t self = s_task;
    s_task = NULL;
    vTaskDelete(self);
}

esp_err_t rgb_service_start(void)
{
    if (s_task != NULL) {
        return ESP_OK;
    }

    atomic_store(&s_running, true);

    BaseType_t ok = xTaskCreate(rgb_task, "rgb", 3072, NULL, tskIDLE_PRIORITY + 1, &s_task);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create task failed");

    ESP_LOGI(TAG, "started");
    return ESP_OK;
}

esp_err_t rgb_service_stop(void)
{
    if (s_task == NULL) {
        return ESP_OK;
    }

    atomic_store(&s_running, false);
    ESP_LOGI(TAG, "stopping");
    return ESP_OK;
}

esp_err_t rgb_service_set_effect(const rgb_effect_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");

    portENTER_CRITICAL(&s_cfg_mux);
    s_cfg = *cfg;
    portEXIT_CRITICAL(&s_cfg_mux);

    return ESP_OK;
}

esp_err_t rgb_service_set_mode(rgb_effect_t effect)
{
    portENTER_CRITICAL(&s_cfg_mux);
    s_cfg.effect = effect;
    portEXIT_CRITICAL(&s_cfg_mux);

    return ESP_OK;
}
