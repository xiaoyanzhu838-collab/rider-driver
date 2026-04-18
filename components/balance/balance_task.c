#include "balance_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ahrs.h"
#include "board_config.h"
#include "imu.h"
#include "uart_protocol.h"
#include "wheel_control.h"

static const char *TAG = "balance";

static int clamp_range(int value, int limit)
{
    if (value < -limit) {
        return -limit;
    }
    if (value > limit) {
        return limit;
    }
    return value;
}

static int map_centered_u8_to_speed(uint8_t raw, int max_abs)
{
    int centered = (int)raw - BOARD_CONTROL_INPUT_CENTER;
    if (centered > -BOARD_CONTROL_INPUT_DEADBAND &&
        centered < BOARD_CONTROL_INPUT_DEADBAND) {
        return 0;
    }

    return (centered * max_abs) / 127;
}

static int map_height_u8_to_percent(uint8_t raw)
{
    int centered = (int)raw - BOARD_CONTROL_INPUT_CENTER;
    int target = BOARD_BODY_HEIGHT_DEFAULT_PERCENT +
                 (centered * BOARD_BODY_HEIGHT_RANGE_PERCENT) / 127;

    if (target < 0) {
        return 0;
    }
    if (target > 100) {
        return 100;
    }
    return target;
}

static int clamp_percent(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 100) {
        return 100;
    }
    return value;
}

static int clamp_step_towards(int current, int target, int max_step)
{
    if (target > current + max_step) {
        return current + max_step;
    }
    if (target < current - max_step) {
        return current - max_step;
    }
    return target;
}

static float vector_norm3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

void balance_task(void *arg)
{
    (void)arg;

    const TickType_t loop_delay = pdMS_TO_TICKS(BOARD_BALANCE_LOOP_PERIOD_MS);
    const float loop_hz = 1000.0f / (float)BOARD_BALANCE_LOOP_PERIOD_MS;
    bool was_enabled = false;
    bool in_tilt_guard = false;
    bool body_pose_initialized = false;
    int last_base_height_percent = BOARD_BODY_HEIGHT_DEFAULT_PERCENT;
    int last_left_height_percent = BOARD_BODY_HEIGHT_DEFAULT_PERCENT;
    int last_right_height_percent = BOARD_BODY_HEIGHT_DEFAULT_PERCENT;
    TickType_t last_body_level_tick = 0;
    TickType_t last_log_tick = 0;
    TickType_t last_idle_log_tick = 0;
    float pitch_zero_deg = 0.0f;
    float prev_pitch_error_deg = 0.0f;
    float pitch_rate_deg_s = 0.0f;

    if (wheel_control_init() != ESP_OK) {
        ESP_LOGE(TAG, "wheel control init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "started: loop=%dms waiting for UART imu_balance=1, input_center=%d deadband=%d body_level=%dms",
             BOARD_BALANCE_LOOP_PERIOD_MS,
             BOARD_CONTROL_INPUT_CENTER,
             BOARD_CONTROL_INPUT_DEADBAND,
             BOARD_BODY_LEVEL_LOOP_PERIOD_MS);

    while (1) {
        proto_write_state_t proto = {0};
        imu_sample_t sample = {0};
        ahrs_euler_t euler = {0};
        bool balance_enabled = false;
        int target_base_height_percent = 0;
        TickType_t now = xTaskGetTickCount();

        proto_get_write_state(&proto);
        balance_enabled = proto.imu_balance != 0;
        target_base_height_percent = map_height_u8_to_percent(proto.translation_z);

        if (!imu_get_latest(&sample)) {
            if (balance_enabled && (now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                ESP_LOGW(TAG, "balance requested but IMU sample not ready");
                last_log_tick = now;
            }
            vTaskDelay(loop_delay);
            continue;
        }

        ahrs_get_euler(&euler);

        if (!body_pose_initialized ||
            (now - last_body_level_tick) >= pdMS_TO_TICKS(BOARD_BODY_LEVEL_LOOP_PERIOD_MS)) {
            float roll_error_deg = euler.roll - BOARD_BODY_LEVEL_ROLL_TARGET_DEG;
            int trim_percent = 0;
            int desired_left_percent = target_base_height_percent;
            int desired_right_percent = target_base_height_percent;

            if (fabsf(roll_error_deg) >= BOARD_BODY_LEVEL_ROLL_DEADBAND_DEG) {
                trim_percent = (int)lroundf(BOARD_BODY_LEVEL_ROLL_SIGN *
                                            BOARD_BODY_LEVEL_KP *
                                            roll_error_deg);
                trim_percent = clamp_range(trim_percent, BOARD_BODY_LEVEL_MAX_TRIM_PERCENT);
            }

            desired_left_percent = clamp_percent(target_base_height_percent + trim_percent);
            desired_right_percent = clamp_percent(target_base_height_percent - trim_percent);

            if (body_pose_initialized) {
                desired_left_percent = clamp_step_towards(last_left_height_percent,
                                                          desired_left_percent,
                                                          BOARD_BODY_LEVEL_MAX_STEP_PERCENT);
                desired_right_percent = clamp_step_towards(last_right_height_percent,
                                                           desired_right_percent,
                                                           BOARD_BODY_LEVEL_MAX_STEP_PERCENT);
            }

            if (!body_pose_initialized ||
                desired_left_percent != last_left_height_percent ||
                desired_right_percent != last_right_height_percent ||
                target_base_height_percent != last_base_height_percent) {
                esp_err_t body_err = wheel_control_set_body_pose_percent(desired_left_percent,
                                                                         desired_right_percent);
                if (body_err == ESP_OK) {
                    if (!body_pose_initialized || (now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                        ESP_LOGI(TAG,
                                 "body level: roll=%.2f base=%d trim=%d left=%d right=%d",
                                 (double)roll_error_deg,
                                 target_base_height_percent,
                                 trim_percent,
                                 desired_left_percent,
                                 desired_right_percent);
                        last_log_tick = now;
                    }
                    body_pose_initialized = true;
                    last_base_height_percent = target_base_height_percent;
                    last_left_height_percent = desired_left_percent;
                    last_right_height_percent = desired_right_percent;
                    last_body_level_tick = now;
                } else if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "body level failed: %s", esp_err_to_name(body_err));
                    last_log_tick = now;
                }
            } else {
                last_body_level_tick = now;
            }
        }

        if (!balance_enabled) {
            if (was_enabled) {
                (void)wheel_control_stop();
                ESP_LOGI(TAG, "balance disabled, wheels stopped");
            }

            if ((now - last_idle_log_tick) >= pdMS_TO_TICKS(2000)) {
                const float acc_norm = vector_norm3(sample.ax, sample.ay, sample.az);
                const float gyro_norm = vector_norm3(sample.gx, sample.gy, sample.gz);
                const bool roughly_still =
                    fabsf(acc_norm - 1.0f) < 0.15f && gyro_norm < 12.0f;
                ESP_LOGI(TAG,
                         "idle: imu_balance=0 roll=%.2f pitch=%.2f acc=%.3fg gyro=%.2fdps still=%d vx_raw=%u vyaw_raw=%u z_raw=%u z_percent=%d body_l=%d body_r=%d",
                         (double)euler.roll,
                         (double)euler.pitch,
                         (double)acc_norm,
                         (double)gyro_norm,
                         roughly_still ? 1 : 0,
                         proto.vx,
                         proto.vyaw,
                         proto.translation_z,
                         target_base_height_percent,
                         last_left_height_percent,
                         last_right_height_percent);
                last_idle_log_tick = now;
            }
            was_enabled = false;
            in_tilt_guard = false;
            prev_pitch_error_deg = 0.0f;
            pitch_rate_deg_s = 0.0f;
            vTaskDelay(loop_delay);
            continue;
        }

        if (!was_enabled) {
            pitch_zero_deg = euler.pitch;
            prev_pitch_error_deg = 0.0f;
            pitch_rate_deg_s = 0.0f;
            in_tilt_guard = false;
            ESP_LOGI(TAG, "balance enabled, pitch zero=%.2f deg", (double)pitch_zero_deg);
        }
        was_enabled = true;

        {
            const float pitch_error_deg = euler.pitch - pitch_zero_deg;
            const float raw_pitch_rate_deg_s = (pitch_error_deg - prev_pitch_error_deg) * loop_hz;
            const int feedforward_speed = map_centered_u8_to_speed(proto.vx, BOARD_BALANCE_VX_MAX_SPEED);
            const int yaw_speed = map_centered_u8_to_speed(proto.vyaw, BOARD_BALANCE_VYAW_MAX_SPEED);
            float control_f = 0.0f;
            int forward_speed = 0;
            esp_err_t err = ESP_OK;

            prev_pitch_error_deg = pitch_error_deg;
            pitch_rate_deg_s = pitch_rate_deg_s * 0.65f + raw_pitch_rate_deg_s * 0.35f;

            if (fabsf(pitch_error_deg) > BOARD_BALANCE_TILT_CUTOFF_DEG) {
                err = wheel_control_stop();
                if (!in_tilt_guard) {
                    ESP_LOGW(TAG, "tilt guard active, pitch=%.2f deg", (double)pitch_error_deg);
                }
                in_tilt_guard = true;
                if (err != ESP_OK && (now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "wheel stop failed: %s", esp_err_to_name(err));
                    last_log_tick = now;
                }
                vTaskDelay(loop_delay);
                continue;
            }

            in_tilt_guard = false;

            control_f = BOARD_BALANCE_OUTPUT_SIGN *
                        (BOARD_BALANCE_KP * pitch_error_deg +
                         BOARD_BALANCE_KD * pitch_rate_deg_s);
            forward_speed = clamp_range((int)lroundf(control_f) + feedforward_speed,
                                        BOARD_WHEEL_SPEED_RECOMMENDED);
            err = wheel_control_set_twist(forward_speed, yaw_speed);
            if (err != ESP_OK) {
                if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "wheel control failed: %s", esp_err_to_name(err));
                    last_log_tick = now;
                }
                vTaskDelay(loop_delay);
                continue;
            }

            if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                ESP_LOGI(TAG,
                         "pitch=%.2f zero=%.2f rate=%.2f vx=%d yaw=%d out=%d",
                         (double)pitch_error_deg,
                         (double)pitch_zero_deg,
                         (double)pitch_rate_deg_s,
                         feedforward_speed,
                         yaw_speed,
                         forward_speed);
                last_log_tick = now;
            }
        }

        vTaskDelay(loop_delay);
    }
}
