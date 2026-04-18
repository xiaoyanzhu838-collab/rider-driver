#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t left_speed;
    int16_t right_speed;
    int16_t left_raw_speed;
    int16_t right_raw_speed;
    int16_t height_percent;
    int16_t left_height_percent;
    int16_t right_height_percent;
    bool wheel_torque_enabled;
    bool body_pose_applied;
} wheel_control_state_t;

esp_err_t wheel_control_init(void);
int wheel_control_clamp_speed(int speed);
esp_err_t wheel_control_set_body_pose_percent(int left_percent, int right_percent);
esp_err_t wheel_control_set_height_percent(int percent);
esp_err_t wheel_control_set_chassis_speeds(int left_speed, int right_speed);
esp_err_t wheel_control_set_twist(int forward_speed, int yaw_speed);
esp_err_t wheel_control_stop(void);
bool wheel_control_get_state(wheel_control_state_t *out);

#ifdef __cplusplus
}
#endif
