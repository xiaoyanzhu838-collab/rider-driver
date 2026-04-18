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

static float clampf_range(float value, float limit)
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

static int apply_breakaway_speed(int value,
                                 int min_effective,
                                 int breakaway_speed,
                                 float pitch_error_deg,
                                 float wheel_speed_feedback)
{
    int floor_speed = min_effective;

    if (fabsf(pitch_error_deg) >= BOARD_BALANCE_STICTION_PITCH_DEG &&
        fabsf(wheel_speed_feedback) < BOARD_BALANCE_STICTION_SPEED_THRESHOLD &&
        breakaway_speed > floor_speed) {
        floor_speed = breakaway_speed;
    }

    return apply_min_effective_speed(value, floor_speed);
}

static int decode_signed_u16(uint16_t value)
{
    return (int16_t)value;
}

static int decode_wheel_logical_speed(uint16_t raw_speed, int board_sign)
{
    return decode_signed_u16(raw_speed) * board_sign;
}

static bool read_chassis_wheel_speeds(int *left_speed_out,
                                      int *right_speed_out,
                                      int *avg_speed_out)
{
    motor_feedback_t fb_left = {0};
    motor_feedback_t fb_right = {0};
    if (motor_read_feedback(BOARD_WHEEL_ID_LEFT, &fb_left) != ESP_OK ||
        motor_read_feedback(BOARD_WHEEL_ID_RIGHT, &fb_right) != ESP_OK) {
        return false;
    }

    int left_speed = decode_wheel_logical_speed(fb_left.speed, BOARD_WHEEL_LEFT_SIGN);
    int right_speed = decode_wheel_logical_speed(fb_right.speed, BOARD_WHEEL_RIGHT_SIGN);
    int avg_speed = (left_speed + right_speed) / 2;

    if (left_speed_out) {
        *left_speed_out = left_speed;
    }
    if (right_speed_out) {
        *right_speed_out = right_speed;
    }
    if (avg_speed_out) {
        *avg_speed_out = avg_speed;
    }
    return true;
}

static void log_wheel_probe_state(const char *tag,
                                  float pitch_deg,
                                  float gyro_y_deg_s,
                                  int command_speed)
{
    motor_feedback_t fb_left = {0};
    motor_feedback_t fb_right = {0};
    bool wheel_fb_ok =
        motor_read_feedback(BOARD_WHEEL_ID_LEFT, &fb_left) == ESP_OK &&
        motor_read_feedback(BOARD_WHEEL_ID_RIGHT, &fb_right) == ESP_OK;

    ESP_LOGI(TAG,
             "%s: pitch=%.2f gyro_y=%.2f cmd=%d fb_ok=%d si11=%d si21=%d pos11=%u pos21=%u",
             tag,
             (double)pitch_deg,
             (double)gyro_y_deg_s,
             command_speed,
             wheel_fb_ok ? 1 : 0,
             wheel_fb_ok ? decode_signed_u16(fb_left.speed) : 0,
             wheel_fb_ok ? decode_signed_u16(fb_right.speed) : 0,
             wheel_fb_ok ? fb_left.position : 0,
             wheel_fb_ok ? fb_right.position : 0);
}

void balance_task(void *arg)
{
    (void)arg;

    const TickType_t loop_delay = pdMS_TO_TICKS(BOARD_BALANCE_LOOP_PERIOD_MS);
    bool was_enabled = false;
    bool in_tilt_guard = false;
    bool body_pose_initialized = false;
    bool balance_armed = false;
    bool wheel_probe_started = false;
    bool wheel_probe_finished = false;
    TickType_t last_log_tick = 0;
    TickType_t last_idle_log_tick = 0;
    TickType_t last_body_pose_retry_tick = 0;
    TickType_t arm_candidate_tick = 0;
    TickType_t wheel_probe_start_tick = 0;
    float pitch_rate_deg_s = 0.0f;
    float wheel_speed_feedback = 0.0f;
    TickType_t last_wheel_feedback_tick = 0;
    bool wheel_feedback_valid = false;

    if (wheel_control_init() != ESP_OK) {
        ESP_LOGE(TAG, "wheel control init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "started: loop=%dms force_enable=%d fixed_height=%d%% pitch_target=%.2f input_center=%d deadband=%d probe=%d",
             BOARD_BALANCE_LOOP_PERIOD_MS,
             BOARD_BALANCE_FORCE_ENABLE,
             BOARD_BODY_HEIGHT_DEFAULT_PERCENT,
             (double)BOARD_BALANCE_PITCH_TARGET_DEG,
             BOARD_CONTROL_INPUT_CENTER,
             BOARD_CONTROL_INPUT_DEADBAND,
             BOARD_WHEEL_PROBE_ENABLE);

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
            if (last_body_pose_retry_tick == 0 ||
                (now - last_body_pose_retry_tick) >= pdMS_TO_TICKS(250)) {
                esp_err_t body_err = wheel_control_set_height_percent(BOARD_BODY_HEIGHT_DEFAULT_PERCENT);
                last_body_pose_retry_tick = now;
                if (body_err == ESP_OK) {
                    body_pose_initialized = true;
                    ESP_LOGI(TAG, "body pose locked at %d%% for balance bring-up",
                             BOARD_BODY_HEIGHT_DEFAULT_PERCENT);
                } else if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "set fixed body pose failed: %s", esp_err_to_name(body_err));
                    last_log_tick = now;
                }
            }

            vTaskDelay(loop_delay);
            continue;
        }

        if (BOARD_WHEEL_PROBE_ENABLE) {
            if (!body_pose_initialized) {
                vTaskDelay(loop_delay);
                continue;
            }

            if (!wheel_probe_started) {
                wheel_probe_start_tick = now;
                wheel_probe_started = true;
                ESP_LOGI(TAG,
                         "wheel probe armed: hold body upright, start in %d ms, run %d ms at speed=%d",
                         BOARD_WHEEL_PROBE_START_DELAY_MS,
                         BOARD_WHEEL_PROBE_RUN_MS,
                         BOARD_WHEEL_PROBE_SPEED);
            }

            if (!wheel_probe_finished) {
                TickType_t elapsed = now - wheel_probe_start_tick;
                TickType_t start_delay = pdMS_TO_TICKS(BOARD_WHEEL_PROBE_START_DELAY_MS);
                TickType_t run_ticks = pdMS_TO_TICKS(BOARD_WHEEL_PROBE_RUN_MS);

                if (elapsed < start_delay) {
                    (void)wheel_control_stop();
                    if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                        log_wheel_probe_state("probe_wait", euler.pitch, sample.gy, 0);
                        last_log_tick = now;
                    }
                    vTaskDelay(loop_delay);
                    continue;
                }

                if (elapsed < (start_delay + run_ticks)) {
                    esp_err_t err = wheel_control_set_twist(BOARD_WHEEL_PROBE_SPEED, 0);
                    if (err != ESP_OK) {
                        if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                            ESP_LOGW(TAG, "wheel probe drive failed: %s", esp_err_to_name(err));
                            last_log_tick = now;
                        }
                        vTaskDelay(loop_delay);
                        continue;
                    }

                    if ((now - last_log_tick) >= pdMS_TO_TICKS(250)) {
                        log_wheel_probe_state("probe_run", euler.pitch, sample.gy,
                                             BOARD_WHEEL_PROBE_SPEED);
                        last_log_tick = now;
                    }
                    vTaskDelay(loop_delay);
                    continue;
                }

                wheel_probe_finished = true;
                (void)wheel_control_stop();
                ESP_LOGI(TAG, "wheel probe complete, wheels stopped");
                log_wheel_probe_state("probe_done", euler.pitch, sample.gy, 0);
            }

            vTaskDelay(loop_delay);
            continue;
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
            balance_armed = false;
            pitch_rate_deg_s = 0.0f;
            wheel_speed_feedback = 0.0f;
            wheel_feedback_valid = false;
            arm_candidate_tick = 0;
            vTaskDelay(loop_delay);
            continue;
        }

        if (!was_enabled) {
            balance_armed = false;
            pitch_rate_deg_s = 0.0f;
            wheel_speed_feedback = 0.0f;
            wheel_feedback_valid = false;
            last_wheel_feedback_tick = 0;
            in_tilt_guard = false;
            arm_candidate_tick = 0;
            ESP_LOGI(TAG,
                     "balance requested, waiting for upright arm window: pitch=%.2f deg target=%.2f deg",
                     (double)euler.pitch,
                     (double)BOARD_BALANCE_PITCH_TARGET_DEG);
        }
        was_enabled = true;

        if (!balance_armed) {
            const float arm_pitch_error_deg = euler.pitch - BOARD_BALANCE_PITCH_TARGET_DEG;
            const float arm_pitch_rate_deg_s = sample.gy * BOARD_BALANCE_PITCH_RATE_SIGN;
            const bool arm_pitch_ok = fabsf(arm_pitch_error_deg) <= BOARD_BALANCE_ARM_PITCH_DEG;
            const bool arm_rate_ok = fabsf(arm_pitch_rate_deg_s) <= BOARD_BALANCE_ARM_GYRO_DPS;

            (void)wheel_control_stop();

            if (arm_pitch_ok && arm_rate_ok) {
                if (arm_candidate_tick == 0) {
                    arm_candidate_tick = now;
                }
                if ((now - arm_candidate_tick) >= pdMS_TO_TICKS(BOARD_BALANCE_ARM_HOLD_MS)) {
                    balance_armed = true;
                    pitch_rate_deg_s = 0.0f;
                    wheel_speed_feedback = 0.0f;
                    wheel_feedback_valid = false;
                    last_wheel_feedback_tick = 0;
                    ESP_LOGI(TAG,
                             "balance armed: pitch=%.2f deg gyro_y=%.2f dps",
                             (double)euler.pitch,
                             (double)sample.gy);
                }
            } else {
                arm_candidate_tick = 0;
                if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                    ESP_LOGI(TAG,
                             "arm_wait: pitch=%.2f gyro_y=%.2f need |pitch|<=%.1f |gyro|<=%.1f",
                             (double)euler.pitch,
                             (double)sample.gy,
                             (double)BOARD_BALANCE_ARM_PITCH_DEG,
                             (double)BOARD_BALANCE_ARM_GYRO_DPS);
                    last_log_tick = now;
                }
            }

            vTaskDelay(loop_delay);
            continue;
        }

        {
            const float pitch_error_deg = euler.pitch - BOARD_BALANCE_PITCH_TARGET_DEG;
            const float raw_pitch_rate_deg_s = sample.gy * BOARD_BALANCE_PITCH_RATE_SIGN;
            const int feedforward_speed = map_centered_u8_to_speed(proto.vx, BOARD_BALANCE_VX_MAX_SPEED);
            const int yaw_speed = map_centered_u8_to_speed(proto.vyaw, BOARD_BALANCE_VYAW_MAX_SPEED);
            float control_f = 0.0f;
            float p_term = 0.0f;
            float d_term = 0.0f;
            float w_term = 0.0f;
            int raw_forward_speed = 0;
            int forward_speed = 0;
            esp_err_t err = ESP_OK;
            int left_wheel_speed = 0;
            int right_wheel_speed = 0;
            int avg_wheel_speed = 0;

            // Use near-real-time gyro damping here; a heavily lagged D term was
            // pushing through the upright crossing and amplifying overshoot.
            pitch_rate_deg_s = raw_pitch_rate_deg_s;

            if (last_wheel_feedback_tick == 0 ||
                (now - last_wheel_feedback_tick) >= pdMS_TO_TICKS(BOARD_BALANCE_WHEEL_FEEDBACK_INTERVAL_MS)) {
                if (read_chassis_wheel_speeds(&left_wheel_speed, &right_wheel_speed, &avg_wheel_speed)) {
                    wheel_speed_feedback = wheel_feedback_valid
                        ? (wheel_speed_feedback * 0.7f + (float)avg_wheel_speed * 0.3f)
                        : (float)avg_wheel_speed;
                    wheel_feedback_valid = true;
                }
                last_wheel_feedback_tick = now;
            }

            if (fabsf(pitch_error_deg) > BOARD_BALANCE_TILT_CUTOFF_DEG) {
                err = wheel_control_stop();
                if (!in_tilt_guard) {
                    ESP_LOGW(TAG, "tilt guard active, pitch=%.2f deg", (double)pitch_error_deg);
                }
                in_tilt_guard = true;
                balance_armed = false;
                arm_candidate_tick = 0;
                pitch_rate_deg_s = 0.0f;
                wheel_speed_feedback = 0.0f;
                wheel_feedback_valid = false;
                last_wheel_feedback_tick = 0;
                if (err != ESP_OK && (now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "wheel stop failed: %s", esp_err_to_name(err));
                    last_log_tick = now;
                }
                vTaskDelay(loop_delay);
                continue;
            }

            in_tilt_guard = false;

            if (fabsf(pitch_error_deg) < BOARD_BALANCE_PITCH_DEADBAND_DEG &&
                fabsf(pitch_rate_deg_s) < BOARD_BALANCE_RATE_DEADBAND_DPS &&
                feedforward_speed == 0) {
                control_f = 0.0f;
            } else {
                p_term = BOARD_BALANCE_KP * pitch_error_deg;
                d_term = clampf_range(BOARD_BALANCE_KD * pitch_rate_deg_s,
                                      BOARD_BALANCE_D_TERM_LIMIT);
                w_term = wheel_feedback_valid ? (BOARD_BALANCE_KW * wheel_speed_feedback) : 0.0f;
                control_f = BOARD_BALANCE_OUTPUT_SIGN *
                            (p_term + d_term + w_term);
            }

            raw_forward_speed = clamp_range((int)lroundf(control_f) + feedforward_speed,
                                            BOARD_WHEEL_SPEED_LIMIT);
            forward_speed = apply_breakaway_speed(raw_forward_speed,
                                                  BOARD_WHEEL_MIN_EFFECTIVE_SPEED,
                                                  BOARD_WHEEL_BREAKAWAY_SPEED,
                                                  pitch_error_deg,
                                                  wheel_speed_feedback);
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
                         "pitch=%.2f target=%.2f gyro_y=%.2f rate=%.2f p=%.2f d=%.2f w=%.2f ws=%.1f vx=%d yaw=%d raw=%d out=%d fb_ok=%d si11=%d si21=%d pos11=%u pos21=%u",
                         (double)pitch_error_deg,
                         (double)BOARD_BALANCE_PITCH_TARGET_DEG,
                         (double)sample.gy,
                         (double)pitch_rate_deg_s,
                         (double)p_term,
                         (double)d_term,
                         (double)w_term,
                         (double)wheel_speed_feedback,
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
