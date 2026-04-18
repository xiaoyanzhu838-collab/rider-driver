#include "imu_app.h"

#include <math.h>
#include <stdbool.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ahrs.h"
#include "board_config.h"
#include "icm42670.h"
#include "imu.h"

static const char *TAG = "imu_app";

#define IMU_GYRO_CALIBRATION_SAMPLES 200

typedef struct {
    float gx_bias;
    float gy_bias;
    float gz_bias;
} imu_gyro_bias_t;

#if BOARD_IMU_DIAGNOSTIC_MODE
typedef struct {
    const char *name;
    const char *instruction;
} imu_diag_phase_t;
#endif

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

    ahrs_init(1000.0f / (float)BOARD_IMU_SAMPLE_PERIOD_MS);
    gyro_bias = calibrate_gyro_bias(imu);

#if BOARD_IMU_DIAGNOSTIC_MODE
    static const imu_diag_phase_t diag_phases[] = {
        {.name = "upright", .instruction = "保持扶正"},
        {.name = "forward", .instruction = "向前倾"},
        {.name = "backward", .instruction = "向后倾"},
    };
    const int diag_phase_count = (int)(sizeof(diag_phases) / sizeof(diag_phases[0]));
    int diag_phase_index = 0;
    bool diag_capture_active = false;
    bool diag_complete = false;
    int64_t diag_state_start_us = 0;
    int64_t next_diag_log_us = 0;

    ESP_LOGI(TAG,
             "IMU 诊断模式：扶正 3 秒，前倾 3 秒，后倾 3 秒，每段准备 %d 毫秒",
             BOARD_IMU_DIAGNOSTIC_PREP_MS);
#endif

    while (1) {
        imu_sample_t s = {0};
        s.ts_us = esp_timer_get_time();

        icm42670_value_t acc = {0};
        icm42670_value_t gyro = {0};
        float temp_c = 0.0f;

        esp_err_t err = icm42670_get_acce_value(imu, &acc);
        if (err == ESP_OK) {
            err = icm42670_get_gyro_value(imu, &gyro);
        }
        if (err == ESP_OK) {
            err = icm42670_get_temp_value(imu, &temp_c);
        }

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2C read failed: %s, skip cycle", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(BOARD_IMU_SAMPLE_PERIOD_MS));
            continue;
        }

        s.ax = acc.x;
        s.ay = acc.y;
        s.az = acc.z;
        s.gx = gyro.x - gyro_bias.gx_bias;
        s.gy = gyro.y - gyro_bias.gy_bias;
        s.gz = gyro.z - gyro_bias.gz_bias;
        s.temp_c = temp_c;

        imu_set_latest(&s);
        ahrs_update(s.ax, s.ay, s.az, s.gx, s.gy, s.gz);

#if BOARD_IMU_DIAGNOSTIC_MODE
        if (diag_state_start_us == 0) {
            diag_state_start_us = s.ts_us;
            ESP_LOGI(TAG, "诊断准备完成");
            ESP_LOGI(TAG,
                     "准备阶段 %d/%d：%s，%d 毫秒后开始",
                     diag_phase_index + 1,
                     diag_phase_count,
                     diag_phases[diag_phase_index].instruction,
                     BOARD_IMU_DIAGNOSTIC_PREP_MS);
        }

        if (!diag_complete) {
            const int64_t elapsed_ms = (s.ts_us - diag_state_start_us) / 1000;

            if (!diag_capture_active) {
                if (elapsed_ms >= BOARD_IMU_DIAGNOSTIC_PREP_MS) {
                    diag_capture_active = true;
                    diag_state_start_us = s.ts_us;
                    next_diag_log_us = s.ts_us;
                    ESP_LOGI(TAG,
                             "采集阶段 %d/%d 开始：%s，持续 %d 毫秒",
                             diag_phase_index + 1,
                             diag_phase_count,
                             diag_phases[diag_phase_index].instruction,
                             BOARD_IMU_DIAGNOSTIC_STAGE_MS);
                }
            } else {
                if (s.ts_us >= next_diag_log_us) {
                    ahrs_euler_t euler = {0};
                    const int64_t phase_ms = (s.ts_us - diag_state_start_us) / 1000;
                    const float acc_norm = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
                    const float gyro_norm = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);

                    ahrs_get_euler(&euler);
                    ESP_LOGI(TAG,
                             "phase=%s t=%lldms ax=%.4f ay=%.4f az=%.4f gx=%.3f gy=%.3f gz=%.3f |a|=%.4f |g|=%.3f roll=%.2f pitch=%.2f yaw=%.2f temp=%.2f",
                             diag_phases[diag_phase_index].name,
                             (long long)phase_ms,
                             (double)s.ax,
                             (double)s.ay,
                             (double)s.az,
                             (double)s.gx,
                             (double)s.gy,
                             (double)s.gz,
                             (double)acc_norm,
                             (double)gyro_norm,
                             (double)euler.roll,
                             (double)euler.pitch,
                             (double)euler.yaw,
                             (double)s.temp_c);
                    next_diag_log_us += (int64_t)BOARD_IMU_DIAGNOSTIC_LOG_INTERVAL_MS * 1000;
                }

                if (elapsed_ms >= BOARD_IMU_DIAGNOSTIC_STAGE_MS) {
                    ESP_LOGI(TAG,
                             "采集阶段 %d/%d 完成：%s",
                             diag_phase_index + 1,
                             diag_phase_count,
                             diag_phases[diag_phase_index].name);
                    diag_phase_index++;
                    diag_capture_active = false;
                    diag_state_start_us = s.ts_us;

                    if (diag_phase_index >= diag_phase_count) {
                        diag_complete = true;
                        ESP_LOGI(TAG, "诊断采集完成，日志输出已停止");
                    } else {
                        ESP_LOGI(TAG,
                                 "准备阶段 %d/%d：%s，%d 毫秒后开始",
                                 diag_phase_index + 1,
                                 diag_phase_count,
                                 diag_phases[diag_phase_index].instruction,
                                 BOARD_IMU_DIAGNOSTIC_PREP_MS);
                    }
                }
            }
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(BOARD_IMU_SAMPLE_PERIOD_MS));
    }
}
