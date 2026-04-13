#include "led.h"

#include "esp_check.h"
#include "esp_log.h"

#include "driver/ledc.h"

#include "board_config.h"

static const char *TAG = "led";

// 使用 LEDC 定时器 1（定时器 0 已被 fan 占用）
#define LED_LEDC_TIMER          LEDC_TIMER_1
#define LED_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LED_LEDC_DUTY_RES       LEDC_TIMER_10_BIT
#define LED_LEDC_CH_RED         LEDC_CHANNEL_1
#define LED_LEDC_CH_BLUE        LEDC_CHANNEL_2

static bool s_inited;

// 低电平点亮：duty_percent=100 时，PWM 输出应反相（output_invert=1）
// 反相后：duty=max -> GPIO 低 -> LED 最亮
//         duty=0   -> GPIO 高 -> LED 熄灭
static esp_err_t led_apply_duty(ledc_channel_t ch, int duty_percent)
{
    if (duty_percent < 0) duty_percent = 0;
    if (duty_percent > 100) duty_percent = 100;

    uint32_t max_duty = (1U << LED_LEDC_DUTY_RES) - 1;
    uint32_t duty = (max_duty * (uint32_t)duty_percent) / 100;

    ESP_RETURN_ON_ERROR(ledc_set_duty(LED_LEDC_MODE, ch, duty), TAG, "ledc_set_duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LED_LEDC_MODE, ch), TAG, "ledc_update_duty failed");

    return ESP_OK;
}

esp_err_t led_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    // 1) 配置 LEDC 定时器
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LED_LEDC_MODE,
        .duty_resolution = LED_LEDC_DUTY_RES,
        .timer_num = LED_LEDC_TIMER,
        .freq_hz = BOARD_LED_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "ledc_timer_config failed");

    // 2) 配置红色 LED 通道（output_invert=1，低电平点亮）
    ledc_channel_config_t red_cfg = {
        .gpio_num = BOARD_LED_RED_GPIO,
        .speed_mode = LED_LEDC_MODE,
        .channel = LED_LEDC_CH_RED,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LED_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 1,   // 低电平点亮，反相输出
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&red_cfg), TAG, "ledc_channel_config red failed");

    // 3) 配置蓝色 LED 通道
    ledc_channel_config_t blue_cfg = {
        .gpio_num = BOARD_LED_BLUE_GPIO,
        .speed_mode = LED_LEDC_MODE,
        .channel = LED_LEDC_CH_BLUE,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LED_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 1,   // 低电平点亮，反相输出
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&blue_cfg), TAG, "ledc_channel_config blue failed");

    s_inited = true;

    ESP_LOGI(TAG, "inited red=GPIO%d blue=GPIO%d freq=%dHz",
             BOARD_LED_RED_GPIO, BOARD_LED_BLUE_GPIO, BOARD_LED_PWM_FREQ_HZ);

    return ESP_OK;
}

// ========== 红色 LED ==========

esp_err_t led_red_on(void)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "led not initialized");
    return led_apply_duty(LED_LEDC_CH_RED, 100);
}

esp_err_t led_red_off(void)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "led not initialized");
    return led_apply_duty(LED_LEDC_CH_RED, 0);
}

esp_err_t led_red_set_duty(int duty_percent)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "led not initialized");
    return led_apply_duty(LED_LEDC_CH_RED, duty_percent);
}

// ========== 蓝色 LED ==========

esp_err_t led_blue_on(void)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "led not initialized");
    return led_apply_duty(LED_LEDC_CH_BLUE, 100);
}

esp_err_t led_blue_off(void)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "led not initialized");
    return led_apply_duty(LED_LEDC_CH_BLUE, 0);
}

esp_err_t led_blue_set_duty(int duty_percent)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "led not initialized");
    return led_apply_duty(LED_LEDC_CH_BLUE, duty_percent);
}
