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
#include "motor_control.h"
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

static float vector_norm3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static int apply_min_effective_speed(int value, int min_effective)
{
    if (value == 0 || min_effective <= 0) {
        return value;
    }

    if (value > 0 && value < min_effective) {
        return min_effective;
    }
    if (value < 0 && value > -min_effective) {
        return -min_effective;
    }
    return value;
}

static int decode_signed_u16(uint16_t value)
{
    return (int16_t)value;
}

void balance_task(void *arg)
{
    (void)arg;

    const TickType_t loop_delay = pdMS_TO_TICKS(BOARD_BALANCE_LOOP_PERIOD_MS);
    bool was_enabled = false;
    bool in_tilt_guard = false;
    bool body_pose_initialized = false;
    TickType_t last_log_tick = 0;
    TickType_t last_idle_log_tick = 0;
    float pitch_rate_deg_s = 0.0f;

    if (wheel_control_init() != ESP_OK) {
        ESP_LOGE(TAG, "wheel control init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "started: loop=%dms force_enable=%d fixed_height=%d%% pitch_target=%.2f input_center=%d deadband=%d",
             BOARD_BALANCE_LOOP_PERIOD_MS,
             BOARD_BALANCE_FORCE_ENABLE,
             BOARD_BODY_HEIGHT_DEFAULT_PERCENT,
             (double)BOARD_BALANCE_PITCH_TARGET_DEG,
             BOARD_CONTROL_INPUT_CENTER,
             BOARD_CONTROL_INPUT_DEADBAND);

    while (1) {
        proto_write_state_t proto = {0};
        imu_sample_t sample = {0};
        ahrs_euler_t euler = {0};
        bool balance_enabled = false;
        TickType_t now = xTaskGetTickCount();

        proto_get_write_state(&proto);
        balance_enabled = BOARD_BALANCE_FORCE_ENABLE || (proto.imu_balance != 0);

        if (!imu_get_latest(&sample)) {
            if (balance_enabled && (now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                ESP_LOGW(TAG, "balance requested but IMU sample not ready");
                last_log_tick = now;
            }
            vTaskDelay(loop_delay);
            continue;
        }

        ahrs_get_euler(&euler);

        if (!body_pose_initialized) {
            esp_err_t body_err = wheel_control_set_height_percent(BOARD_BODY_HEIGHT_DEFAULT_PERCENT);
            if (body_err == ESP_OK) {
                body_pose_initialized = true;
                ESP_LOGI(TAG, "body pose locked at %d%% for balance bring-up",
                         BOARD_BODY_HEIGHT_DEFAULT_PERCENT);
            } else if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                ESP_LOGW(TAG, "set fixed body pose failed: %s", esp_err_to_name(body_err));
                last_log_tick = now;
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
                         "idle: imu_balance=0 roll=%.2f pitch=%.2f acc=%.3fg gyro=%.2fdps still=%d vx_raw=%u vyaw_raw=%u z_raw=%u fixed_height=%d%%",
                         (double)euler.roll,
                         (double)euler.pitch,
                         (double)acc_norm,
                         (double)gyro_norm,
                         roughly_still ? 1 : 0,
                         proto.vx,
                         proto.vyaw,
                         proto.translation_z,
                         BOARD_BODY_HEIGHT_DEFAULT_PERCENT);
                last_idle_log_tick = now;
            }
            was_enabled = false;
            in_tilt_guard = false;
            pitch_rate_deg_s = 0.0f;
            vTaskDelay(loop_delay);
            continue;
        }

        if (!was_enabled) {
            pitch_rate_deg_s = 0.0f;
            in_tilt_guard = false;
            ESP_LOGI(TAG,
                     "balance enabled, pitch=%.2f deg target=%.2f deg",
                     (double)euler.pitch,
                     (double)BOARD_BALANCE_PITCH_TARGET_DEG);
        }
        was_enabled = true;

        {
            const float pitch_error_deg = euler.pitch - BOARD_BALANCE_PITCH_TARGET_DEG;
            const float raw_pitch_rate_deg_s = sample.gy * BOARD_BALANCE_PITCH_RATE_SIGN;
            const int feedforward_speed = map_centered_u8_to_speed(proto.vx, BOARD_BALANCE_VX_MAX_SPEED);
            const int yaw_speed = map_centered_u8_to_speed(proto.vyaw, BOARD_BALANCE_VYAW_MAX_SPEED);
            float control_f = 0.0f;
            int raw_forward_speed = 0;
            int forward_speed = 0;
            esp_err_t err = ESP_OK;

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

            if (fabsf(pitch_error_deg) < BOARD_BALANCE_PITCH_DEADBAND_DEG &&
                feedforward_speed == 0) {
                control_f = 0.0f;
            } else {
                control_f = BOARD_BALANCE_OUTPUT_SIGN *
                            (BOARD_BALANCE_KP * pitch_error_deg +
                             BOARD_BALANCE_KD * pitch_rate_deg_s);
            }

            raw_forward_speed = clamp_range((int)lroundf(control_f) + feedforward_speed,
                                            BOARD_WHEEL_SPEED_RECOMMENDED);
            forward_speed = apply_min_effective_speed(raw_forward_speed,
                                                      BOARD_WHEEL_MIN_EFFECTIVE_SPEED);
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
                motor_feedback_t fb_left = {0};
                motor_feedback_t fb_right = {0};
                bool wheel_fb_ok =
                    motor_read_feedback(BOARD_WHEEL_ID_LEFT, &fb_left) == ESP_OK &&
                    motor_read_feedback(BOARD_WHEEL_ID_RIGHT, &fb_right) == ESP_OK;

                ESP_LOGI(TAG,
                         "pitch=%.2f target=%.2f gyro_y=%.2f rate=%.2f vx=%d yaw=%d raw=%d out=%d fb_ok=%d si11=%d si21=%d pos11=%u pos21=%u",
                         (double)pitch_error_deg,
                         (double)BOARD_BALANCE_PITCH_TARGET_DEG,
                         (double)sample.gy,
                         (double)pitch_rate_deg_s,
                         feedforward_speed,
                         yaw_speed,
                         raw_forward_speed,
                         forward_speed,
                         wheel_fb_ok ? 1 : 0,
                         wheel_fb_ok ? decode_signed_u16(fb_left.speed) : 0,
                         wheel_fb_ok ? decode_signed_u16(fb_right.speed) : 0,
                         wheel_fb_ok ? fb_left.position : 0,
                         wheel_fb_ok ? fb_right.position : 0);
                last_log_tick = now;
            }
        }

        vTaskDelay(loop_delay);
    }
}
