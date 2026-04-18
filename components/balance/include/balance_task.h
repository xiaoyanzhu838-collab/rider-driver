#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ahrs.h"
#include "esp_err.h"
#include "imu.h"
#include "wheel_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float kw;
    float target_deg;
    float i_term_limit;
    float d_term_limit;
} balance_pid_config_t;

typedef struct {
    int speed_limit;
    float tilt_cutoff_deg;
} balance_runtime_config_t;

typedef struct {
    uint32_t seq;
    bool enabled;
    bool armed;
    bool in_tilt_guard;
    bool zero_captured;
    bool wheel_feedback_valid;
    float theta_fb_deg;
    float target_deg;
    float error_deg;
    float zero_deg;
    float gx_deg_s;
    float rate_deg_s;
    float p_term;
    float i_term;
    float d_term;
    float w_term;
    float wheel_speed_feedback;
    int feedforward_speed;
    int yaw_speed;
    int raw_forward_speed;
    int forward_speed;
    imu_sample_t imu;
    ahrs_euler_t euler;
    wheel_control_state_t wheel;
} balance_telemetry_t;

void balance_task(void *arg);
esp_err_t balance_get_pid_config(balance_pid_config_t *out);
esp_err_t balance_set_pid_config(const balance_pid_config_t *cfg);
esp_err_t balance_get_runtime_config(balance_runtime_config_t *out);
esp_err_t balance_set_runtime_config(const balance_runtime_config_t *cfg);
bool balance_get_telemetry(balance_telemetry_t *out);
void balance_set_runtime_log_enabled(bool enabled);
bool balance_get_runtime_log_enabled(void);

#ifdef __cplusplus
}
#endif
