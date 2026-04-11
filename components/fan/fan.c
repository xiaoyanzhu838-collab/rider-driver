#include "fan.h"

#include "esp_check.h"
#include "esp_log.h"

#include "driver/ledc.h"

#include "board_config.h"

static const char *TAG = "fan";

// 使用 LEDC 生成 PWM：
// - 低速风扇/无刷风扇常用 25kHz 控制口
// - 这里用 10-bit 分辨率，占空比 0~1023
#define FAN_LEDC_TIMER          LEDC_TIMER_0
#define FAN_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define FAN_LEDC_CHANNEL        LEDC_CHANNEL_0
#define FAN_LEDC_DUTY_RES       LEDC_TIMER_10_BIT

static bool s_inited;

static esp_err_t fan_apply_duty_percent(int duty_percent)
{
    if (duty_percent < 0) duty_percent = 0;
    if (duty_percent > 100) duty_percent = 100;

    uint32_t max_duty = (1U << FAN_LEDC_DUTY_RES) - 1;
    uint32_t duty = (max_duty * (uint32_t)duty_percent) / 100;

    ESP_RETURN_ON_ERROR(ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty), TAG, "ledc_set_duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL), TAG, "ledc_update_duty failed");

    return ESP_OK;
}

esp_err_t fan_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    // 1) 配置 LEDC 定时器：决定 PWM 频率和分辨率
    ledc_timer_config_t timer_cfg = {
        .speed_mode = FAN_LEDC_MODE,
        .duty_resolution = FAN_LEDC_DUTY_RES,
        .timer_num = FAN_LEDC_TIMER,
        .freq_hz = BOARD_FAN_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "ledc_timer_config failed");

    // 2) 配置 LEDC 通道：把 PWM 输出映射到指定 GPIO
    ledc_channel_config_t ch_cfg = {
        .gpio_num = BOARD_FAN_PWM_GPIO,
        .speed_mode = FAN_LEDC_MODE,
        .channel = FAN_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = FAN_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "ledc_channel_config failed");

    s_inited = true;

    // 3) 上电默认占空比（60%）
    ESP_RETURN_ON_ERROR(fan_apply_duty_percent(BOARD_FAN_DEFAULT_DUTY), TAG, "apply default duty failed");

    ESP_LOGI(TAG, "inited gpio=%d freq=%dHz duty=%d%%",
             BOARD_FAN_PWM_GPIO, BOARD_FAN_PWM_FREQ_HZ, BOARD_FAN_DEFAULT_DUTY);

    return ESP_OK;
}

esp_err_t fan_set_duty_percent(int duty_percent)
{
    ESP_RETURN_ON_FALSE(s_inited, ESP_ERR_INVALID_STATE, TAG, "fan not initialized");
    return fan_apply_duty_percent(duty_percent);
}

