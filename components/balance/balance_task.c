#include "balance_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "ahrs.h"
#include "board_config.h"
#include "imu.h"
#include "led.h"
#include "motor_control.h"
#include "rgb_control.h"
#include "uart_protocol.h"
#include "wheel_control.h"

static const char *TAG = "balance";
#define BAL_LOGI(...) do { if (BOARD_BALANCE_SERIAL_LOG_ENABLE) { ESP_LOGI(TAG, __VA_ARGS__); } } while (0)
#define BAL_LOGW(...) do { if (BOARD_BALANCE_SERIAL_LOG_ENABLE) { ESP_LOGW(TAG, __VA_ARGS__); } } while (0)
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

static void update_zero_calibration_led(bool leds_ready, bool zero_captured)
{
    if (!leds_ready) {
        return;
    }

    if (zero_captured) {
        (void)led_red_off();
        (void)led_blue_on();
    } else {
        (void)led_blue_off();
        (void)led_red_on();
    }
}

static void update_zero_range_rgb(bool theta_in_zero_window, bool zero_captured)
{
    // 进入零点姿态范围时点亮绿色提示；一旦零点已锁定，立即熄灭。
    (void)rgb_control_set_zero_indicator(theta_in_zero_window && !zero_captured);
}

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

static int sign_int(int value)
{
    if (value > 0) {
        return 1;
    }
    if (value < 0) {
        return -1;
    }
    return 0;
}

static int apply_stiction_compensation(int raw_speed,
                                       float pitch_error_deg,
                                       float pitch_rate_deg_s,
                                       int feedforward_speed,
                                       int prev_output_speed,
                                       TickType_t now,
                                       TickType_t *boost_start_tick)
{
    const int sign = sign_int(raw_speed);
    const int prev_sign = sign_int(prev_output_speed);
    const int abs_raw = abs(raw_speed);
    const float abs_error = fabsf(pitch_error_deg);
    const float abs_rate = fabsf(pitch_rate_deg_s);
    const float comp_start_error = BOARD_BALANCE_STICTION_PITCH_DEG * 0.5f;
    const float comp_full_error = BOARD_BALANCE_STICTION_HIGH_PITCH_DEG;
    const float comp_start_rate = BOARD_BALANCE_STICTION_SPEED_THRESHOLD;
    const float comp_full_rate = BOARD_BALANCE_DIVERGE_RATE_DPS;
    const TickType_t boost_window = pdMS_TO_TICKS(BOARD_BALANCE_STICTION_BOOST_MS);
    int compensated_abs = abs_raw;
    int boost_speed = BOARD_WHEEL_BREAKAWAY_SPEED;
    float error_severity = 0.0f;
    float rate_severity = 0.0f;
    float severity = 0.0f;
    float boost_severity = 0.0f;

    if (boost_start_tick == NULL) {
        return raw_speed;
    }

    if (feedforward_speed != 0) {
        *boost_start_tick = 0;
        return raw_speed;
    }

    if (sign == 0) {
        *boost_start_tick = 0;
        return 0;
    }

    // Crossing the balance point should clear the previous boost state, but
    // the new direction still needs breakaway compensation immediately.
    if (prev_sign != 0 && sign != prev_sign) {
        *boost_start_tick = 0;
    }

    if (abs_error < BOARD_BALANCE_PITCH_DEADBAND_DEG &&
        abs_rate < BOARD_BALANCE_RATE_DEADBAND_DPS) {
        *boost_start_tick = 0;
        return 0;
    }

    if (comp_full_error > comp_start_error && abs_error > comp_start_error) {
        error_severity = (abs_error - comp_start_error) /
                         (comp_full_error - comp_start_error);
    }
    if (comp_full_rate > comp_start_rate && abs_rate > comp_start_rate) {
        rate_severity = (abs_rate - comp_start_rate) /
                        (comp_full_rate - comp_start_rate);
    }
    error_severity = clampf_range(error_severity, 1.0f);
    rate_severity = clampf_range(rate_severity, 1.0f);
    severity = fmaxf(error_severity, rate_severity);

    // Near the balance point keep the output mostly linear so the chassis
    // doesn't "kick" itself into oscillation.
    if (severity <= 0.0f) {
        *boost_start_tick = 0;
        return raw_speed;
    }

    if (abs_raw < BOARD_WHEEL_MIN_EFFECTIVE_SPEED) {
        const float lifted =
            (float)abs_raw +
            severity * (float)(BOARD_WHEEL_MIN_EFFECTIVE_SPEED - abs_raw);
        compensated_abs = (int)lroundf(lifted);
    }

    boost_severity = clampf_range((severity - 0.7f) / 0.3f, 1.0f);
    if (boost_severity > 0.0f) {
        const float lifted_boost =
            (float)BOARD_WHEEL_BREAKAWAY_SPEED +
            boost_severity *
                (float)(BOARD_WHEEL_BREAKAWAY_HIGH_SPEED -
                        BOARD_WHEEL_BREAKAWAY_SPEED);
        boost_speed = (int)lroundf(lifted_boost);
    }

    if ((prev_sign == 0 || sign != prev_sign) && boost_severity > 0.0f &&
        abs_raw >= (BOARD_WHEEL_MIN_EFFECTIVE_SPEED / 2)) {
        *boost_start_tick = now;
    }

    if (*boost_start_tick != 0 &&
        (now - *boost_start_tick) <= boost_window &&
        compensated_abs < boost_speed) {
        return sign * boost_speed;
    }

    if (*boost_start_tick != 0 &&
        (now - *boost_start_tick) > boost_window) {
        *boost_start_tick = 0;
    }

    return sign * compensated_abs;
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

    BAL_LOGI(
             "pid updated: kp=%.3f ki=%.3f kd=%.3f kw=%.3f target=%.3f",
             (double)s_pid_cfg.kp,
             (double)s_pid_cfg.ki,
             (double)s_pid_cfg.kd,
             (double)s_pid_cfg.kw,
             (double)s_pid_cfg.target_deg);
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

static float blendf(float current, float target, float alpha)
{
    return current + alpha * (target - current);
}

static void log_wheel_probe_state(const char *tag,
                                  float theta_deg,
                                  float rate_deg_s,
                                  float delta_theta_deg,
                                  int command_speed)
{
    motor_feedback_t fb_left = {0};
    motor_feedback_t fb_right = {0};
    const bool wheel_fb_ok =
        motor_read_feedback_fast(BOARD_WHEEL_ID_LEFT, &fb_left) == ESP_OK &&
        motor_read_feedback_fast(BOARD_WHEEL_ID_RIGHT, &fb_right) == ESP_OK;

    BAL_LOGI(
             "%s: theta=%.2f rate=%.2f dtheta=%.2f cmd=%d fb=%d pos11=%u sp11=%d pos21=%u sp21=%d",
             tag,
             (double)theta_deg,
             (double)rate_deg_s,
             (double)delta_theta_deg,
             command_speed,
             wheel_fb_ok ? 1 : 0,
             wheel_fb_ok ? fb_left.position : 0,
             wheel_fb_ok ? (int16_t)fb_left.speed : 0,
             wheel_fb_ok ? fb_right.position : 0,
             wheel_fb_ok ? (int16_t)fb_right.speed : 0);
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
    bool leds_ready = false;
    bool wheel_probe_started = false;
    bool wheel_probe_finished = false;
    int wheel_probe_phase = 0;
    TickType_t last_log_tick = 0;
    TickType_t last_focus_log_tick = 0;
    TickType_t last_body_pose_retry_tick = 0;
    TickType_t arm_candidate_tick = 0;
    TickType_t wheel_probe_start_tick = 0;
    TickType_t wheel_probe_phase_tick = 0;
    TickType_t wheel_probe_ready_tick = 0;
    float arm_theta_sum_deg = 0.0f;
    float arm_theta_min_deg = 0.0f;
    float arm_theta_max_deg = 0.0f;
    int arm_theta_samples = 0;
    float pitch_rate_deg_s = 0.0f;
    float error_integral = 0.0f;
    float balance_zero_deg = 0.0f;
    float theta_est_deg = 0.0f;
    float rate_est_deg_s = 0.0f;
    float wheel_speed_feedback = 0.0f;
    bool wheel_feedback_valid = false;
    TickType_t last_wheel_feedback_tick = 0;
    float wheel_probe_theta_ref_deg = 0.0f;
    bool rate_filter_initialized = false;
    int last_forward_output = 0;
    TickType_t stiction_boost_start_tick = 0;

    if (wheel_control_init() != ESP_OK) {
        ESP_LOGE(TAG, "wheel control init failed");
        vTaskDelete(NULL);
        return;
    }

    if (led_init() == ESP_OK) {
        leds_ready = true;
        update_zero_calibration_led(leds_ready, false);
    } else {
        BAL_LOGW("board led init failed");
    }

    BAL_LOGI(
             "started: loop=%dms force_enable=%d fixed_height=%d%% pitch_target=%.2f input_center=%d deadband=%d probe=%d",
             BOARD_BALANCE_LOOP_PERIOD_MS,
             BOARD_BALANCE_FORCE_ENABLE,
             BOARD_BODY_HEIGHT_DEFAULT_PERCENT,
             (double)BOARD_BALANCE_PITCH_TARGET_DEG,
             BOARD_CONTROL_INPUT_CENTER,
             BOARD_CONTROL_INPUT_DEADBAND,
             BOARD_WHEEL_PROBE_ENABLE);
    BAL_LOGI(
             "pid defaults: kp=%.3f ki=%.3f kd=%.3f sign=%.1f deadband=%.2fdeg %.2fdps",
             (double)BOARD_BALANCE_KP,
             (double)BOARD_BALANCE_KI,
             (double)BOARD_BALANCE_KD,
             (double)BOARD_BALANCE_OUTPUT_SIGN,
             (double)BOARD_BALANCE_PITCH_DEADBAND_DEG,
             (double)BOARD_BALANCE_RATE_DEADBAND_DPS);

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
                BAL_LOGW("balance requested but IMU sample not ready");
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
        if ((now - last_wheel_feedback_tick) >=
            pdMS_TO_TICKS(BOARD_BALANCE_WHEEL_FEEDBACK_INTERVAL_MS)) {
            motor_feedback_t fb_left = {0};
            motor_feedback_t fb_right = {0};
            if (motor_read_feedback_fast(BOARD_WHEEL_ID_LEFT, &fb_left) == ESP_OK &&
                motor_read_feedback_fast(BOARD_WHEEL_ID_RIGHT, &fb_right) == ESP_OK) {
                const float left_logical =
                    (float)((int16_t)fb_left.speed) * (float)BOARD_WHEEL_LEFT_SIGN;
                const float right_logical =
                    (float)((int16_t)fb_right.speed) * (float)BOARD_WHEEL_RIGHT_SIGN;
                wheel_speed_feedback = 0.5f * (left_logical + right_logical);
                wheel_feedback_valid = true;
            } else {
                wheel_feedback_valid = false;
            }
            last_wheel_feedback_tick = now;
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
                    BAL_LOGI("body pose locked at %d%% for balance bring-up",
                             BOARD_BODY_HEIGHT_DEFAULT_PERCENT);
                } else if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    BAL_LOGW("set fixed body pose failed: %s", esp_err_to_name(body_err));
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
                wheel_probe_phase_tick = 0;
                wheel_probe_ready_tick = 0;
                wheel_probe_theta_ref_deg = theta_est_deg;
                wheel_probe_phase = 0;
                wheel_probe_started = true;
                BAL_LOGI(
                         "定向探针已启动：先扶正等待 %d ms，然后正转 %d 持续 %d ms；之后再次扶正稳定，再反转 %d 持续 %d ms",
                         BOARD_WHEEL_PROBE_START_DELAY_MS,
                         BOARD_WHEEL_PROBE_SPEED_POS,
                         BOARD_WHEEL_PROBE_RUN_MS,
                         BOARD_WHEEL_PROBE_SPEED_NEG,
                         BOARD_WHEEL_PROBE_RUN_MS);
            }

            if (!wheel_probe_finished) {
                TickType_t elapsed = now - wheel_probe_start_tick;
                TickType_t start_delay = pdMS_TO_TICKS(BOARD_WHEEL_PROBE_START_DELAY_MS);
                TickType_t run_ticks = pdMS_TO_TICKS(BOARD_WHEEL_PROBE_RUN_MS);
                TickType_t brake_ticks = pdMS_TO_TICKS(BOARD_WHEEL_PROBE_BRAKE_MS);
                const bool probe_upright_ok =
                    fabsf(theta_est_deg) <= BOARD_BALANCE_ARM_PITCH_DEG &&
                    fabsf(rate_est_deg_s) <= BOARD_BALANCE_ARM_GYRO_DPS;
                const TickType_t probe_hold_ticks = pdMS_TO_TICKS(BOARD_BALANCE_ARM_HOLD_MS);

                if (elapsed < start_delay) {
                    (void)wheel_control_stop();
                    if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                        log_wheel_probe_state("probe_wait",
                                             theta_est_deg,
                                             rate_est_deg_s,
                                             theta_est_deg - wheel_probe_theta_ref_deg,
                                             0);
                        last_log_tick = now;
                    }
                    vTaskDelay(loop_delay);
                    continue;
                }

                if (wheel_probe_phase == 0) {
                    (void)wheel_control_stop();
                    if (!probe_upright_ok) {
                        wheel_probe_ready_tick = 0;
                        if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                            BAL_LOGI("探针等待扶正：theta=%.2f rate=%.2f",
                                     (double)theta_est_deg,
                                     (double)rate_est_deg_s);
                            last_log_tick = now;
                        }
                        vTaskDelay(loop_delay);
                        continue;
                    }

                    if (wheel_probe_ready_tick == 0) {
                        wheel_probe_ready_tick = now;
                    }
                    if ((now - wheel_probe_ready_tick) < probe_hold_ticks) {
                        if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                            BAL_LOGI("探针扶正稳定中：theta=%.2f rate=%.2f",
                                     (double)theta_est_deg,
                                     (double)rate_est_deg_s);
                            last_log_tick = now;
                        }
                        vTaskDelay(loop_delay);
                        continue;
                    }

                    wheel_probe_phase = 1;
                    wheel_probe_phase_tick = now;
                    wheel_probe_theta_ref_deg = theta_est_deg;
                    wheel_probe_ready_tick = 0;
                    BAL_LOGI("探针正转开始：轮子应开始转动 cmd=%d theta_ref=%.2f",
                             BOARD_WHEEL_PROBE_SPEED_POS,
                             (double)wheel_probe_theta_ref_deg);
                }

                if (wheel_probe_phase == 1 &&
                    (now - wheel_probe_phase_tick) < run_ticks) {
                    esp_err_t err = wheel_control_set_twist(BOARD_WHEEL_PROBE_SPEED_POS, 0);
                    if (err != ESP_OK) {
                        if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                            BAL_LOGW("wheel probe drive failed: %s", esp_err_to_name(err));
                            last_log_tick = now;
                        }
                        vTaskDelay(loop_delay);
                        continue;
                    }

                    if ((now - last_log_tick) >= pdMS_TO_TICKS(250)) {
                        log_wheel_probe_state("probe_pos",
                                             theta_est_deg,
                                             rate_est_deg_s,
                                             theta_est_deg - wheel_probe_theta_ref_deg,
                                             BOARD_WHEEL_PROBE_SPEED_POS);
                        last_log_tick = now;
                    }
                    vTaskDelay(loop_delay);
                    continue;
                }

                if (wheel_probe_phase == 1) {
                    wheel_probe_phase = 2;
                    wheel_probe_phase_tick = now;
                    wheel_probe_theta_ref_deg = theta_est_deg;
                    wheel_probe_ready_tick = 0;
                    (void)wheel_control_stop();
                    BAL_LOGI("探针第一段结束，请扶正机身：theta_ref=%.2f",
                             (double)wheel_probe_theta_ref_deg);
                }

                if (wheel_probe_phase == 2 &&
                    (now - wheel_probe_phase_tick) < brake_ticks) {
                    (void)wheel_control_stop();
                    if ((now - last_log_tick) >= pdMS_TO_TICKS(250)) {
                        log_wheel_probe_state("probe_stop",
                                             theta_est_deg,
                                             rate_est_deg_s,
                                             theta_est_deg - wheel_probe_theta_ref_deg,
                                             0);
                        last_log_tick = now;
                    }
                    vTaskDelay(loop_delay);
                    continue;
                }

                if (wheel_probe_phase == 2) {
                    (void)wheel_control_stop();
                    if (!probe_upright_ok) {
                        wheel_probe_ready_tick = 0;
                        if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                            BAL_LOGI("探针等待再次扶正：theta=%.2f rate=%.2f",
                                     (double)theta_est_deg,
                                     (double)rate_est_deg_s);
                            last_log_tick = now;
                        }
                        vTaskDelay(loop_delay);
                        continue;
                    }

                    if (wheel_probe_ready_tick == 0) {
                        wheel_probe_ready_tick = now;
                    }
                    if ((now - wheel_probe_ready_tick) < probe_hold_ticks) {
                        if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                            BAL_LOGI("探针第二段稳定中：theta=%.2f rate=%.2f",
                                     (double)theta_est_deg,
                                     (double)rate_est_deg_s);
                            last_log_tick = now;
                        }
                        vTaskDelay(loop_delay);
                        continue;
                    }

                    wheel_probe_phase = 3;
                    wheel_probe_phase_tick = now;
                    wheel_probe_theta_ref_deg = theta_est_deg;
                    wheel_probe_ready_tick = 0;
                    BAL_LOGI("探针反转开始：轮子应开始转动 cmd=%d theta_ref=%.2f",
                             BOARD_WHEEL_PROBE_SPEED_NEG,
                             (double)wheel_probe_theta_ref_deg);
                }

                if (wheel_probe_phase == 3 &&
                    (now - wheel_probe_phase_tick) < run_ticks) {
                    esp_err_t err = wheel_control_set_twist(BOARD_WHEEL_PROBE_SPEED_NEG, 0);
                    if (err != ESP_OK) {
                        if ((now - last_log_tick) >= pdMS_TO_TICKS(500)) {
                            BAL_LOGW("wheel probe drive failed: %s", esp_err_to_name(err));
                            last_log_tick = now;
                        }
                        vTaskDelay(loop_delay);
                        continue;
                    }

                    if ((now - last_log_tick) >= pdMS_TO_TICKS(250)) {
                        log_wheel_probe_state("probe_neg",
                                             theta_est_deg,
                                             rate_est_deg_s,
                                             theta_est_deg - wheel_probe_theta_ref_deg,
                                             BOARD_WHEEL_PROBE_SPEED_NEG);
                        last_log_tick = now;
                    }
                    vTaskDelay(loop_delay);
                    continue;
                }

                wheel_probe_finished = true;
                (void)wheel_control_stop();
                BAL_LOGI("定向探针完成，轮子已停止");
                log_wheel_probe_state("probe_done",
                                     theta_est_deg,
                                     rate_est_deg_s,
                                     theta_est_deg - wheel_probe_theta_ref_deg,
                                     0);
            }

            vTaskDelay(loop_delay);
            continue;
        }

        if (!balance_enabled) {
            if (was_enabled) {
                (void)wheel_control_stop();
                BAL_LOGI("balance disabled, wheels stopped");
            }

            was_enabled = false;
            in_tilt_guard = false;
            balance_armed = false;
            balance_zero_captured = false;
            update_zero_calibration_led(leds_ready, false);
            update_zero_range_rgb(false, false);
            arm_theta_sum_deg = 0.0f;
            arm_theta_min_deg = 0.0f;
            arm_theta_max_deg = 0.0f;
            arm_theta_samples = 0;
            pitch_rate_deg_s = 0.0f;
            error_integral = 0.0f;
            arm_candidate_tick = 0;
            theta_est_deg = ahrs_state.theta_fb_deg;
            rate_est_deg_s = -(sample.gx - ahrs_state.bias_dps[0]);
            rate_filter_initialized = true;
            wheel_speed_feedback = 0.0f;
            wheel_feedback_valid = false;
            last_wheel_feedback_tick = 0;
            last_forward_output = 0;
            stiction_boost_start_tick = 0;
            vTaskDelay(loop_delay);
            continue;
        }

        if (!was_enabled) {
            balance_armed = false;
            balance_zero_captured = false;
            update_zero_calibration_led(leds_ready, false);
            update_zero_range_rgb(false, false);
            arm_theta_sum_deg = 0.0f;
            arm_theta_min_deg = 0.0f;
            arm_theta_max_deg = 0.0f;
            arm_theta_samples = 0;
            pitch_rate_deg_s = 0.0f;
            error_integral = 0.0f;
            in_tilt_guard = false;
            arm_candidate_tick = 0;
            wheel_speed_feedback = 0.0f;
            wheel_feedback_valid = false;
            last_wheel_feedback_tick = 0;
            BAL_LOGI(
                     "balance requested, waiting for zero capture window: theta_fb=%.2f deg target=%.2f deg",
                     (double)theta_est_deg,
                     (double)BOARD_BALANCE_PITCH_TARGET_DEG);
        }
        was_enabled = true;

        if (!balance_armed) {
            const float arm_tilt_error_deg =
                theta_est_deg - BOARD_BALANCE_PITCH_TARGET_DEG;
            const float arm_tilt_rate_deg_s = rate_est_deg_s;
            const bool zero_capture_pitch_ok =
                fabsf(arm_tilt_error_deg) <= BOARD_BALANCE_ZERO_CAPTURE_PITCH_DEG;
            const bool zero_capture_rate_ok =
                fabsf(arm_tilt_rate_deg_s) <= BOARD_BALANCE_ZERO_CAPTURE_RATE_DPS;

            (void)wheel_control_stop();
            update_zero_calibration_led(leds_ready, false);
            update_zero_range_rgb(zero_capture_pitch_ok, false);

            if (zero_capture_pitch_ok && zero_capture_rate_ok) {
                if (arm_candidate_tick == 0) {
                    arm_candidate_tick = now;
                    arm_theta_sum_deg = 0.0f;
                    arm_theta_min_deg = theta_est_deg;
                    arm_theta_max_deg = theta_est_deg;
                    arm_theta_samples = 0;
                }
                arm_theta_sum_deg += theta_est_deg;
                if (theta_est_deg < arm_theta_min_deg) {
                    arm_theta_min_deg = theta_est_deg;
                }
                if (theta_est_deg > arm_theta_max_deg) {
                    arm_theta_max_deg = theta_est_deg;
                }
                arm_theta_samples += 1;
                {
                    const float avg_theta_deg =
                        (arm_theta_samples > 0)
                            ? (arm_theta_sum_deg / (float)arm_theta_samples)
                            : theta_est_deg;
                    const float theta_span_deg = arm_theta_max_deg - arm_theta_min_deg;

                    if (theta_span_deg <= BOARD_BALANCE_ZERO_CAPTURE_SPAN_DEG &&
                        (now - arm_candidate_tick) >= pdMS_TO_TICKS(BOARD_BALANCE_ZERO_CAPTURE_HOLD_MS)) {
                        balance_armed = true;
                        // Use the current stabilized pose as the saved zero reference.
                        balance_zero_deg = clampf_range(theta_est_deg,
                                                        BOARD_BALANCE_ZERO_CAPTURE_LIMIT_DEG);
                        balance_zero_captured = true;
                        update_zero_calibration_led(leds_ready, true);
                        update_zero_range_rgb(false, true);
                        pitch_rate_deg_s = 0.0f;
                        error_integral = 0.0f;
                        BAL_LOGI(
                                 "balance armed: theta_fb=%.2f deg zero=%.2f deg avg=%.2f deg span=%.2f deg gx=%.2f dps",
                                 (double)theta_est_deg,
                                 (double)balance_zero_deg,
                                 (double)avg_theta_deg,
                                 (double)theta_span_deg,
                                 (double)rate_est_deg_s);
                    } else if ((now - last_log_tick) >= pdMS_TO_TICKS(300)) {
                        const int hold_ms =
                            (int)((now - arm_candidate_tick) * portTICK_PERIOD_MS);
                        BAL_LOGI(
                                 "零点标定中：theta=%.2f rate=%.2f hold=%dms span=%.2f，需要 |theta|<=%.1f |rate|<=%.1f span<=%.1f 且保持 %dms",
                                 (double)theta_est_deg,
                                 (double)rate_est_deg_s,
                                 hold_ms,
                                 (double)theta_span_deg,
                                 (double)BOARD_BALANCE_ZERO_CAPTURE_PITCH_DEG,
                                 (double)BOARD_BALANCE_ZERO_CAPTURE_RATE_DPS,
                                 (double)BOARD_BALANCE_ZERO_CAPTURE_SPAN_DEG,
                                 BOARD_BALANCE_ZERO_CAPTURE_HOLD_MS);
                        last_log_tick = now;
                    }
                }
            } else {
                if ((now - last_log_tick) >= pdMS_TO_TICKS(300)) {
                    if (!zero_capture_pitch_ok) {
                        BAL_LOGI(
                                 "等待零点放置：theta=%.2f，需要 |theta|<=%.1f",
                                 (double)theta_est_deg,
                                 (double)BOARD_BALANCE_ZERO_CAPTURE_PITCH_DEG);
                    } else {
                        BAL_LOGI(
                                 "等待零点静止：theta=%.2f rate=%.2f，需要 |rate|<=%.1f",
                                 (double)theta_est_deg,
                                 (double)rate_est_deg_s,
                                 (double)BOARD_BALANCE_ZERO_CAPTURE_RATE_DPS);
                    }
                    last_log_tick = now;
                }
                arm_candidate_tick = 0;
                arm_theta_sum_deg = 0.0f;
                arm_theta_min_deg = 0.0f;
                arm_theta_max_deg = 0.0f;
                arm_theta_samples = 0;
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

            (void)balance_get_pid_config(&pid_cfg);
            (void)balance_get_runtime_config(&runtime_cfg);
            balance_target_deg =
                (balance_zero_captured ? balance_zero_deg : 0.0f) + pid_cfg.target_deg;
            pitch_error_deg = theta_fb_deg - balance_target_deg;

            pitch_rate_deg_s = rate_est_deg_s;

            if (fabsf(pitch_error_deg) > runtime_cfg.tilt_cutoff_deg) {
                err = wheel_control_stop();
                if (!in_tilt_guard) {
                    BAL_LOGW("tilt guard active, theta_fb=%.2f deg cutoff=%.2f deg",
                             (double)pitch_error_deg, (double)runtime_cfg.tilt_cutoff_deg);
                }
                in_tilt_guard = true;
                balance_armed = false;
                balance_zero_captured = false;
                update_zero_calibration_led(leds_ready, false);
                update_zero_range_rgb(false, false);
                arm_candidate_tick = 0;
                arm_theta_sum_deg = 0.0f;
                arm_theta_min_deg = 0.0f;
                arm_theta_max_deg = 0.0f;
                arm_theta_samples = 0;
                pitch_rate_deg_s = 0.0f;
                error_integral = 0.0f;
                wheel_speed_feedback = 0.0f;
                wheel_feedback_valid = false;
                last_wheel_feedback_tick = 0;
                last_forward_output = 0;
                stiction_boost_start_tick = 0;
                if (err != ESP_OK && (now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    BAL_LOGW("wheel stop failed: %s", esp_err_to_name(err));
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
                if (wheel_feedback_valid) {
                    // Wheel-speed damping: when the chassis is already rolling in one
                    // direction, subtract part of that measured motion so the robot
                    // stops "chasing" the center of mass indefinitely.
                    w_term = -pid_cfg.kw * wheel_speed_feedback;
                } else {
                    w_term = 0.0f;
                }
                control_f = BOARD_BALANCE_OUTPUT_SIGN *
                            (p_term + i_term + d_term) + w_term;
            }

            raw_forward_speed = clamp_range((int)lroundf(control_f) + feedforward_speed,
                                            runtime_cfg.speed_limit);
            forward_speed = apply_stiction_compensation(raw_forward_speed,
                                                        pitch_error_deg,
                                                        pitch_rate_deg_s,
                                                        feedforward_speed,
                                                        last_forward_output,
                                                        now,
                                                        &stiction_boost_start_tick);
            forward_speed = clamp_range(forward_speed, runtime_cfg.speed_limit);
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
                last_forward_output = 0;
                stiction_boost_start_tick = 0;
                if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                    BAL_LOGW("wheel control failed: %s", esp_err_to_name(err));
                    last_log_tick = now;
                }
                vTaskDelay(loop_delay);
                continue;
            }

            last_forward_output = forward_speed;

            if (runtime_logs_enabled &&
                (now - last_focus_log_tick) >= pdMS_TO_TICKS(100)) {
                wheel_control_state_t wheel_state = {0};
                (void)wheel_control_get_state(&wheel_state);

                BAL_LOGI(
                         "bal theta=%.2f err=%.2f gx=%.2f rate=%.2f p=%.2f d=%.2f raw=%d out=%d l=%d r=%d",
                         (double)theta_fb_deg,
                         (double)pitch_error_deg,
                         (double)(sample.gx - ahrs_state.bias_dps[0]),
                         (double)pitch_rate_deg_s,
                         (double)p_term,
                         (double)d_term,
                         raw_forward_speed,
                         forward_speed,
                         wheel_state.left_speed,
                         wheel_state.right_speed);
                last_focus_log_tick = now;
            }
        }

        vTaskDelay(loop_delay);
    }
}
