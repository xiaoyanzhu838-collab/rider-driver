#include "battery.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "board_config.h"

static const char *TAG = "battery";

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_cali_handle;
static bool s_inited;

// IO33 = ADC1_CH5
#define BAT_ADC_UNIT     ADC_UNIT_1
#define BAT_ADC_CHANNEL  ADC_CHANNEL_5
#define BAT_ADC_ATTEN    ADC_ATTEN_DB_12

esp_err_t battery_init(void)
{
    if (s_inited) return ESP_OK;

    // 初始化 ADC oneshot
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BAT_ADC_UNIT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle), TAG, "adc unit init failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg), TAG, "adc chan config failed");

    // 初始化校准（线性拟合方案，ESP32 支持）
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = BAT_ADC_UNIT,
        .atten = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration not available, using raw values");
        s_cali_handle = NULL;
    }
#else
    s_cali_handle = NULL;
    ESP_LOGW(TAG, "ADC calibration scheme not supported");
#endif

    s_inited = true;
    ESP_LOGI(TAG, "inited, gpio=%d, divider_ratio=%.1f", BOARD_BAT_ADC_GPIO, BOARD_BAT_DIVIDER_RATIO);
    return ESP_OK;
}

float battery_read_voltage(void)
{
    if (!s_inited) return 0.0f;

    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &raw) != ESP_OK) {
        return 0.0f;
    }

    float voltage_mv;
    if (s_cali_handle) {
        int mv = 0;
        adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
        voltage_mv = (float)mv;
    } else {
        // 无校准时粗略估算: 12-bit, 12dB衰减 ≈ 0~3.3V
        voltage_mv = (float)raw / 4095.0f * 3300.0f;
    }

    // 乘以分压比还原电池电压
    float bat_v = (voltage_mv / 1000.0f) * BOARD_BAT_DIVIDER_RATIO;
    return bat_v;
}

uint8_t battery_read_percent(void)
{
    float v = battery_read_voltage();

    if (v <= BOARD_BAT_VOLTAGE_MIN) return 0;
    if (v >= BOARD_BAT_VOLTAGE_MAX) return 100;

    float pct = (v - BOARD_BAT_VOLTAGE_MIN) / (BOARD_BAT_VOLTAGE_MAX - BOARD_BAT_VOLTAGE_MIN) * 100.0f;
    return (uint8_t)pct;
}
