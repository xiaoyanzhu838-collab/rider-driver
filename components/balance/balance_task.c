#include "balance_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "ahrs.h"
#include "board_config.h"
#include "imu.h"
#include "motor_control.h"
#include "uart_protocol.h"
#include "wheel_control.h"

static const char *TAG = "balance";
static portMUX_TYPE s_runtime_mux = portMUX_INITIALIZER_UNLOCKED;
static balance_pid_config_t s_pid_cfg = {
    .kp = BOARD_BALANCE_KP,
    .ki = BOARD_BALANCE_KI,
    .kd = BOARD_BALANCE_KD,
    .kw = BOARD_BALANCE_KW,
    .target_deg = BOARD_BALANCE_PITCH_TARGET_DEG,
    .i_term_limit = BOARD_BALANCE_I_TERM_LIMIT,
    .d_term_limit = BOARD_BALANCE_D_TERM_LIMIT,
};
static balance_runtime_config_t s_runtime_cfg = {
    .speed_limit = BOARD_WHEEL_SPEED_LIMIT,
    .tilt_cutoff_deg = BOARD_BALANCE_TILT_CUTOFF_DEG,
};
static balance_telemetry_t s_telemetry = {0};
static bool s_runtime_log_enabled = false;

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

esp_err_t balance_get_pid_config(balance_pid_config_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_runtime_mux);
    *out = s_pid_cfg;
    portEXIT_CRITICAL(&s_runtime_mux);
    return ESP_OK;
}

esp_err_t balance_set_pid_config(const balance_pid_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_runtime_mux);
    s_pid_cfg.kp = clampf_range(cfg->kp, 200.0f);
    if (s_pid_cfg.kp < 0.0f) {
        s_pid_cfg.kp = 0.0f;
    }
    s_pid_cfg.ki = clampf_range(cfg->ki, 200.0f);
    if (s_pid_cfg.ki < 0.0f) {
        s_pid_cfg.ki = 0.0f;
    }
    s_pid_cfg.kd = clampf_range(cfg->kd, 100.0f);
    if (s_pid_cfg.kd < 0.0f) {
        s_pid_cfg.kd = 0.0f;
    }
    s_pid_cfg.kw = clampf_range(cfg->kw, 20.0f);
    s_pid_cfg.target_deg = clampf_range(cfg->target_deg, 15.0f);
    s_pid_cfg.i_term_limit = clampf_range(cfg->i_term_limit, 200.0f);
    if (s_pid_cfg.i_term_limit < 0.0f) {
        s_pid_cfg.i_term_limit = 0.0f;
    }
    s_pid_cfg.d_term_limit = clampf_range(cfg->d_term_limit, 200.0f);
    if (s_pid_cfg.d_term_limit < 0.0f) {
        s_pid_cfg.d_term_limit = 0.0f;
    }
    portEXIT_CRITICAL(&s_runtime_mux);
    return ESP_OK;
}

esp_err_t balance_get_runtime_config(balance_runtime_config_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_runtime_mux);
    *out = s_runtime_cfg;
    portEXIT_CRITICAL(&s_runtime_mux);
    return ESP_OK;
}

esp_err_t balance_set_runtime_config(const balance_runtime_config_t *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_runtime_mux);
    s_runtime_cfg.speed_limit = clamp_range(cfg->speed_limit, BOARD_WHEEL_SPEED_LIMIT);
    if (s_runtime_cfg.speed_limit < 1) {
        s_runtime_cfg.speed_limit = 1;
    }
    s_runtime_cfg.tilt_cutoff_deg = clampf_range(cfg->tilt_cutoff_deg, 89.0f);
    if (s_runtime_cfg.tilt_cutoff_deg < 20.0f) {
        s_runtime_cfg.tilt_cutoff_deg = 20.0f;
    }
    portEXIT_CRITICAL(&s_runtime_mux);
    return ESP_OK;
}

bool balance_get_telemetry(balance_telemetry_t *out)
{
    if (!out) {
        return false;
    }

    portENTER_CRITICAL(&s_runtime_mux);
    *out = s_telemetry;
    portEXIT_CRITICAL(&s_runtime_mux);
    return true;
}

void balance_set_runtime_log_enabled(bool enabled)
{
    portENTER_CRITICAL(&s_runtime_mux);
    s_runtime_log_enabled = enabled;
    portEXIT_CRITICAL(&s_runtime_mux);
}

bool balance_get_runtime_log_enabled(void)
{
    bool enabled = false;

    portENTER_CRITICAL(&s_runtime_mux);
    enabled = s_runtime_log_enabled;
    portEXIT_CRITICAL(&s_runtime_mux);
    return enabled;
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

static float blendf(float current, float target, float alpha)
{
    return current + alpha * (target - current);
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
    if (motor_read_feedback_fast(BOARD_WHEEL_ID_LEFT, &fb_left) != ESP_OK ||
        motor_read_feedback_fast(BOARD_WHEEL_ID_RIGHT, &fb_right) != ESP_OK) {
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
        motor_read_feedback_fast(BOARD_WHEEL_ID_LEFT, &fb_left) == ESP_OK &&
        motor_read_feedback_fast(BOARD_WHEEL_ID_RIGHT, &fb_right) == ESP_OK;

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
    bool balance_zero_captured = false;
    bool wheel_probe_started = false;
    bool wheel_probe_finished = false;
    TickType_t last_log_tick = 0;
    TickType_t last_idle_log_tick = 0;
    TickType_t last_body_pose_retry_tick = 0;
    TickType_t arm_candidate_tick = 0;
    TickType_t wheel_probe_start_tick = 0;
    float arm_theta_sum_deg = 0.0f;
    int arm_theta_samples = 0;
    float pitch_rate_deg_s = 0.0f;
    float error_integral = 0.0f;
    float wheel_speed_feedback = 0.0f;
    float balance_zero_deg = 0.0f;
    float theta_est_deg = 0.0f;
    float rate_est_deg_s = 0.0f;
    bool rate_filter_initialized = false;
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
        ahrs_state_t ahrs_state = {0};
        bool balance_enabled = false;
        TickType_t now = xTaskGetTickCount();
        bool runtime_logs_enabled = balance_get_runtime_log_enabled();

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
        ahrs_get_state(&ahrs_state);
        {
            const float raw_rate_deg_s = -(sample.gx - ahrs_state.bias_dps[0]);

            theta_est_deg = ahrs_state.theta_fb_deg;
            if (!rate_filter_initialized) {
                rate_est_deg_s = raw_rate_deg_s;
                rate_filter_initialized = true;
            } else {
                rate_est_deg_s =
                    blendf(rate_est_deg_s, raw_rate_deg_s, BOARD_BALANCE_RATE_LPF_ALPHA);
            }
        }
        {
            portENTER_CRITICAL(&s_runtime_mux);
            s_telemetry.enabled = balance_enabled;
            s_telemetry.armed = balance_armed;
            s_telemetry.in_tilt_guard = in_tilt_guard;
            s_telemetry.zero_captured = balance_zero_captured;
            s_telemetry.zero_deg = balance_zero_deg;
            s_telemetry.theta_fb_deg = theta_est_deg;
            s_telemetry.rate_deg_s = rate_est_deg_s;
            s_telemetry.imu = sample;
            s_telemetry.euler = euler;
            portEXIT_CRITICAL(&s_runtime_mux);
        }

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

            if (runtime_logs_enabled &&
                (now - last_idle_log_tick) >= pdMS_TO_TICKS(2000)) {
                const float acc_norm = vector_norm3(sample.ax, sample.ay, sample.az);
                const float gyro_norm = vector_norm3(sample.gx, sample.gy, sample.gz);
                const bool roughly_still =
                    fabsf(acc_norm - 1.0f) < 0.15f && gyro_norm < 12.0f;
                ESP_LOGI(TAG,
                         "idle: imu_balance=0 theta_fb=%.2f roll=%.2f pitch=%.2f acc=%.3fg gyro=%.2fdps still=%d vx_raw=%u vyaw_raw=%u z_raw=%u fixed_height=%d%%",
                         (double)theta_est_deg,
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
            balance_zero_captured = false;
            arm_theta_sum_deg = 0.0f;
            arm_theta_samples = 0;
            pitch_rate_deg_s = 0.0f;
            error_integral = 0.0f;
            wheel_speed_feedback = 0.0f;
            wheel_feedback_valid = false;
            arm_candidate_tick = 0;
            theta_est_deg = ahrs_state.theta_fb_deg;
            rate_est_deg_s = -(sample.gx - ahrs_state.bias_dps[0]);
            rate_filter_initialized = true;
            vTaskDelay(loop_delay);
            continue;
        }

        if (!was_enabled) {
            balance_armed = false;
            balance_zero_captured = false;
            arm_theta_sum_deg = 0.0f;
            arm_theta_samples = 0;
            pitch_rate_deg_s = 0.0f;
            error_integral = 0.0f;
            wheel_speed_feedback = 0.0f;
            wheel_feedback_valid = false;
            last_wheel_feedback_tick = 0;
            in_tilt_guard = false;
            arm_candidate_tick = 0;
            ESP_LOGI(TAG,
                     "balance requested, waiting for upright arm window: theta_fb=%.2f deg target=%.2f deg",
                     (double)theta_est_deg,
                     (double)BOARD_BALANCE_PITCH_TARGET_DEG);
        }
        was_enabled = true;

        if (!balance_armed) {
            const float arm_tilt_error_deg =
                theta_est_deg - BOARD_BALANCE_PITCH_TARGET_DEG;
            const float arm_tilt_rate_deg_s = rate_est_deg_s;
            const bool arm_pitch_ok = fabsf(arm_tilt_error_deg) <= BOARD_BALANCE_ARM_PITCH_DEG;
            const bool arm_rate_ok = fabsf(arm_tilt_rate_deg_s) <= BOARD_BALANCE_ARM_GYRO_DPS;

            (void)wheel_control_stop();

            if (arm_pitch_ok && arm_rate_ok) {
                if (arm_candidate_tick == 0) {
                    arm_candidate_tick = now;
                    arm_theta_sum_deg = 0.0f;
                    arm_theta_samples = 0;
                }
                arm_theta_sum_deg += theta_est_deg;
                arm_theta_samples += 1;
                if ((now - arm_candidate_tick) >= pdMS_TO_TICKS(BOARD_BALANCE_ARM_HOLD_MS)) {
                    const float avg_theta_deg =
                        (arm_theta_samples > 0)
                            ? (arm_theta_sum_deg / (float)arm_theta_samples)
                            : theta_est_deg;
                    balance_armed = true;
                    balance_zero_deg = clampf_range(avg_theta_deg,
                                                    BOARD_BALANCE_ZERO_CAPTURE_LIMIT_DEG);
                    balance_zero_captured = true;
                    pitch_rate_deg_s = 0.0f;
                    error_integral = 0.0f;
                    wheel_speed_feedback = 0.0f;
                    wheel_feedback_valid = false;
                    last_wheel_feedback_tick = 0;
                    ESP_LOGI(TAG,
                             "balance armed: theta_fb=%.2f deg zero=%.2f deg avg=%.2f deg gx=%.2f dps",
                             (double)theta_est_deg,
                             (double)balance_zero_deg,
                             (double)avg_theta_deg,
                             (double)rate_est_deg_s);
                }
            } else {
                arm_candidate_tick = 0;
                arm_theta_sum_deg = 0.0f;
                arm_theta_samples = 0;
                if (runtime_logs_enabled &&
                    (now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                    ESP_LOGI(TAG,
                             "arm_wait: theta_fb=%.2f gx=%.2f need |theta|<=%.1f |rate|<=%.1f",
                             (double)theta_est_deg,
                             (double)rate_est_deg_s,
                             (double)BOARD_BALANCE_ARM_PITCH_DEG,
                             (double)BOARD_BALANCE_ARM_GYRO_DPS);
                    last_log_tick = now;
                }
            }

            vTaskDelay(loop_delay);
            continue;
        }

        {
            balance_pid_config_t pid_cfg = {0};
            balance_runtime_config_t runtime_cfg = {0};
            const float theta_fb_deg = theta_est_deg;
            float balance_target_deg = 0.0f;
            float pitch_error_deg = 0.0f;
            const int feedforward_speed = map_centered_u8_to_speed(proto.vx, BOARD_BALANCE_VX_MAX_SPEED);
            const int yaw_speed = map_centered_u8_to_speed(proto.vyaw, BOARD_BALANCE_VYAW_MAX_SPEED);
            float control_f = 0.0f;
            float p_term = 0.0f;
            float i_term = 0.0f;
            float d_term = 0.0f;
            float w_term = 0.0f;
            int raw_forward_speed = 0;
            int forward_speed = 0;
            esp_err_t err = ESP_OK;
            int left_wheel_speed = 0;
            int right_wheel_speed = 0;
            int avg_wheel_speed = 0;

            (void)balance_get_pid_config(&pid_cfg);
            (void)balance_get_runtime_config(&runtime_cfg);
            balance_target_deg =
                (balance_zero_captured ? balance_zero_deg : 0.0f) + pid_cfg.target_deg;
            pitch_error_deg = theta_fb_deg - balance_target_deg;

            pitch_rate_deg_s = rate_est_deg_s;

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

            if (fabsf(pitch_error_deg) > runtime_cfg.tilt_cutoff_deg) {
                err = wheel_control_stop();
                if (!in_tilt_guard) {
                    ESP_LOGW(TAG, "tilt guard active, theta_fb=%.2f deg cutoff=%.2f deg",
                             (double)pitch_error_deg, (double)runtime_cfg.tilt_cutoff_deg);
                }
                in_tilt_guard = true;
                balance_armed = false;
                arm_candidate_tick = 0;
                arm_theta_sum_deg = 0.0f;
                arm_theta_samples = 0;
                pitch_rate_deg_s = 0.0f;
                error_integral = 0.0f;
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
                error_integral *= 0.9f;
            } else {
                const float dt_s = (float)BOARD_BALANCE_LOOP_PERIOD_MS / 1000.0f;
                error_integral += pitch_error_deg * dt_s;
                error_integral = clampf_range(error_integral, pid_cfg.i_term_limit);
                p_term = pid_cfg.kp * pitch_error_deg;
                i_term = pid_cfg.ki * error_integral;
                d_term = clampf_range(pid_cfg.kd * pitch_rate_deg_s,
                                      pid_cfg.d_term_limit);
                w_term = 0.0f;
                control_f = BOARD_BALANCE_OUTPUT_SIGN *
                            (p_term + i_term + d_term);
            }

            raw_forward_speed = clamp_range((int)lroundf(control_f) + feedforward_speed,
                                            runtime_cfg.speed_limit);
            forward_speed = raw_forward_speed;
            {
                wheel_control_state_t wheel_state = {0};
                (void)wheel_control_get_state(&wheel_state);
                portENTER_CRITICAL(&s_runtime_mux);
                s_telemetry.seq += 1;
                s_telemetry.enabled = balance_enabled;
                s_telemetry.armed = balance_armed;
                s_telemetry.in_tilt_guard = in_tilt_guard;
                s_telemetry.zero_captured = balance_zero_captured;
                s_telemetry.wheel_feedback_valid = wheel_feedback_valid;
                s_telemetry.theta_fb_deg = theta_fb_deg;
                s_telemetry.target_deg = balance_target_deg;
                s_telemetry.error_deg = pitch_error_deg;
                s_telemetry.zero_deg = balance_zero_deg;
                s_telemetry.gx_deg_s = sample.gx - ahrs_state.bias_dps[0];
                s_telemetry.rate_deg_s = pitch_rate_deg_s;
                s_telemetry.p_term = p_term;
                s_telemetry.i_term = i_term;
                s_telemetry.d_term = d_term;
                s_telemetry.w_term = w_term;
                s_telemetry.wheel_speed_feedback = wheel_speed_feedback;
                s_telemetry.feedforward_speed = feedforward_speed;
                s_telemetry.yaw_speed = yaw_speed;
                s_telemetry.raw_forward_speed = raw_forward_speed;
                s_telemetry.forward_speed = forward_speed;
                s_telemetry.imu = sample;
                s_telemetry.euler = euler;
                s_telemetry.wheel = wheel_state;
                portEXIT_CRITICAL(&s_runtime_mux);
            }
            err = wheel_control_set_twist(forward_speed, yaw_speed);
            if (err != ESP_OK) {
                if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "wheel control failed: %s", esp_err_to_name(err));
                    last_log_tick = now;
                }
                vTaskDelay(loop_delay);
                continue;
            }

            if (runtime_logs_enabled &&
                (now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                motor_feedback_t fb_left = {0};
                motor_feedback_t fb_right = {0};
                bool wheel_fb_ok =
                    motor_read_feedback_fast(BOARD_WHEEL_ID_LEFT, &fb_left) == ESP_OK &&
                    motor_read_feedback_fast(BOARD_WHEEL_ID_RIGHT, &fb_right) == ESP_OK;

                ESP_LOGI(TAG,
                         "theta_fb=%.2f target=%.2f err=%.2f gx=%.2f rate=%.2f p=%.2f i=%.2f d=%.2f w=%.2f ws=%.1f vx=%d yaw=%d raw=%d out=%d fb_ok=%d si11=%d si21=%d pos11=%u pos21=%u",
                         (double)theta_fb_deg,
                         (double)balance_target_deg,
                         (double)pitch_error_deg,
                         (double)(sample.gx - ahrs_state.bias_dps[0]),
                         (double)pitch_rate_deg_s,
                         (double)p_term,
                         (double)i_term,
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
