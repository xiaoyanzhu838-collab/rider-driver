#include "imu_app.h"

#include <math.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"
#include "icm42670.h"
#include "imu.h"
#include "ahrs.h"

// static const char *TAG = "imu_app";
static const char *TAG = "imu_app";

#define IMU_GYRO_CALIBRATION_SAMPLES 200

typedef struct {
    float gx_bias;
    float gy_bias;
    float gz_bias;
} imu_gyro_bias_t;

static imu_gyro_bias_t calibrate_gyro_bias(icm42670_handle_t imu)
{
    imu_gyro_bias_t bias = {0};
    int valid_samples = 0;

    for (int i = 0; i < IMU_GYRO_CALIBRATION_SAMPLES; i++) {
        icm42670_value_t gyro = {0};
        if (icm42670_get_gyro_value(imu, &gyro) == ESP_OK) {
            bias.gx_bias += gyro.x;
            bias.gy_bias += gyro.y;
            bias.gz_bias += gyro.z;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(BOARD_IMU_SAMPLE_PERIOD_MS));
    }

    if (valid_samples > 0) {
        bias.gx_bias /= (float)valid_samples;
        bias.gy_bias /= (float)valid_samples;
        bias.gz_bias /= (float)valid_samples;
    }

    ESP_LOGI(TAG,
             "gyro bias calibrated: samples=%d gx=%.3f gy=%.3f gz=%.3f dps",
             valid_samples,
             (double)bias.gx_bias,
             (double)bias.gy_bias,
             (double)bias.gz_bias);
    return bias;
}

void imu_app_task(void *arg)
{
    (void)arg;

    ESP_ERROR_CHECK(imu_init());
    icm42670_handle_t imu = imu_get_handle();
    imu_gyro_bias_t gyro_bias = {0};

    // 初始化 AHRS 滤波器，采样频率 = 1000 / BOARD_IMU_SAMPLE_PERIOD_MS
    ahrs_init(1000.0f / (float)BOARD_IMU_SAMPLE_PERIOD_MS);
    gyro_bias = calibrate_gyro_bias(imu);

    while (1) {
        imu_sample_t s = {0};
        s.ts_us = esp_timer_get_time();

        icm42670_value_t acc = {0};
        icm42670_value_t gyro = {0};
        float temp_c = 0;

        esp_err_t err = icm42670_get_acce_value(imu, &acc);
        if (err == ESP_OK) err = icm42670_get_gyro_value(imu, &gyro);
        if (err == ESP_OK) err = icm42670_get_temp_value(imu, &temp_c);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2C read failed: %s, skip cycle", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(BOARD_IMU_SAMPLE_PERIOD_MS));
            continue;
        }

        s.ax = acc.x; s.ay = acc.y; s.az = acc.z;
        s.gx = gyro.x - gyro_bias.gx_bias;
        s.gy = gyro.y - gyro_bias.gy_bias;
        s.gz = gyro.z - gyro_bias.gz_bias;
        s.temp_c = temp_c;

        imu_set_latest(&s);

        // 喂入 AHRS 滤波器
        ahrs_update(s.ax, s.ay, s.az, s.gx, s.gy, s.gz);

        vTaskDelay(pdMS_TO_TICKS(BOARD_IMU_SAMPLE_PERIOD_MS));
    }
}
