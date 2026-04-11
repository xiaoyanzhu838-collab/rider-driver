#include "imu_app.h"

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

void imu_app_task(void *arg)
{
    (void)arg;

    ESP_ERROR_CHECK(imu_init());
    icm42670_handle_t imu = imu_get_handle();

    // 初始化 AHRS 滤波器，采样频率 = 1000 / BOARD_IMU_SAMPLE_PERIOD_MS
    ahrs_init(1000.0f / (float)BOARD_IMU_SAMPLE_PERIOD_MS);

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
        s.gx = gyro.x; s.gy = gyro.y; s.gz = gyro.z;
        s.temp_c = temp_c;

        imu_set_latest(&s);

        // 喂入 AHRS 滤波器
        ahrs_update(s.ax, s.ay, s.az, s.gx, s.gy, s.gz);

        vTaskDelay(pdMS_TO_TICKS(BOARD_IMU_SAMPLE_PERIOD_MS));
    }
}
