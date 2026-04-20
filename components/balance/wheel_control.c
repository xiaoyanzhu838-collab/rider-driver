#include "wheel_control.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "board_config.h"
#include "motor_control.h"
#include "scs_proto.h"

static const char *TAG = "wheel_control";

static atomic_bool s_initialized;
static atomic_bool s_body_home_ready;
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;
static int s_body_home12 = BOARD_BODY_HEIGHT_HOME_12;
static int s_body_home22 = BOARD_BODY_HEIGHT_HOME_22;
static wheel_control_state_t s_state = {
    .height_percent = BOARD_BODY_HEIGHT_DEFAULT_PERCENT,
    .left_height_percent = BOARD_BODY_HEIGHT_DEFAULT_PERCENT,
    .right_height_percent = BOARD_BODY_HEIGHT_DEFAULT_PERCENT,
    .wheel_torque_enabled = false,
    .body_pose_applied = false,
};

static int clamp_percent(int percent)
{
    if (percent < 0) {
        return 0;
    }
    if (percent > 100) {
        return 100;
    }
    return percent;
}

int wheel_control_clamp_speed(int speed)
{
    if (speed < -BOARD_WHEEL_SPEED_LIMIT) {
        return -BOARD_WHEEL_SPEED_LIMIT;
    }
    if (speed > BOARD_WHEEL_SPEED_LIMIT) {
        return BOARD_WHEEL_SPEED_LIMIT;
    }
    return speed;
}

static uint16_t encode_signed_i16(int speed)
{
    return (uint16_t)(int16_t)wheel_control_clamp_speed(speed);
}

static esp_err_t capture_body_home_if_needed(void)
{
    if (atomic_load(&s_body_home_ready)) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "using fixed body home: home12=%d home22=%d",
             BOARD_BODY_HEIGHT_HOME_12, BOARD_BODY_HEIGHT_HOME_22);
    atomic_store(&s_body_home_ready, true);
    return ESP_OK;
}

static int body_goal_for_servo(uint8_t id, int percent)
{
    int home12 = 0;
    int home22 = 0;

    percent = clamp_percent(percent);
    portENTER_CRITICAL(&s_state_mux);
    home12 = s_body_home12;
    home22 = s_body_home22;
    portEXIT_CRITICAL(&s_state_mux);

    if (id == 12) {
        int delta = (BOARD_BODY_HEIGHT_DELTA_12 * percent + 50) / 100;
        return home12 - delta;
    }

    int delta = (BOARD_BODY_HEIGHT_DELTA_22 * percent + 50) / 100;
    return home22 + delta;
}

static void state_update_body_pose(int base_percent, int left_percent, int right_percent, bool applied)
{
    portENTER_CRITICAL(&s_state_mux);
    s_state.height_percent = (int16_t)base_percent;
    s_state.left_height_percent = (int16_t)left_percent;
    s_state.right_height_percent = (int16_t)right_percent;
    s_state.body_pose_applied = applied;
    portEXIT_CRITICAL(&s_state_mux);
}

static void state_update_speeds(int left, int right, int left_raw, int right_raw, bool torque_enabled)
{
    portENTER_CRITICAL(&s_state_mux);
    s_state.left_speed = (int16_t)left;
    s_state.right_speed = (int16_t)right;
    s_state.left_raw_speed = (int16_t)left_raw;
    s_state.right_raw_speed = (int16_t)right_raw;
    s_state.wheel_torque_enabled = torque_enabled;
    portEXIT_CRITICAL(&s_state_mux);
}

static esp_err_t write_wheel_speed_with_retry(uint8_t id, uint16_t raw, const char *label)
{
    static const int kMaxAttempts = 3;

    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        esp_err_t err = scs_write_word(id, SCS_ADDR_GOAL_SPEED_L, raw);
        if (err == ESP_OK) {
            if (attempt > 1) {
                ESP_LOGW(TAG, "%s recovered on retry %d", label, attempt);
            }
            return ESP_OK;
        }

        if (attempt < kMaxAttempts) {
            ESP_LOGW(TAG, "%s attempt %d failed: %s", label, attempt, esp_err_to_name(err));
            (void)motor_set_torque(id, true);
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        return err;
    }

    return ESP_FAIL;
}

static esp_err_t set_wheel_torque_enabled(bool enabled)
{
    ESP_RETURN_ON_ERROR(motor_set_torque(BOARD_WHEEL_ID_LEFT, enabled),
                        TAG, "set left wheel torque failed");
    ESP_RETURN_ON_ERROR(motor_set_torque(BOARD_WHEEL_ID_RIGHT, enabled),
                        TAG, "set right wheel torque failed");
    return ESP_OK;
}

static esp_err_t ensure_wheel_torque_enabled(void)
{
    bool enabled = false;

    portENTER_CRITICAL(&s_state_mux);
    enabled = s_state.wheel_torque_enabled;
    portEXIT_CRITICAL(&s_state_mux);

    if (enabled) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(set_wheel_torque_enabled(true),
                        TAG, "enable wheel torque failed");

    state_update_speeds(0, 0, 0, 0, true);
    return ESP_OK;
}

esp_err_t wheel_control_init(void)
{
    if (atomic_load(&s_initialized)) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor init failed");
    ESP_LOGI(TAG,
             "init ok: left=%u(sign=%d) right=%u(sign=%d) height_default=%d%% speed_limit=%d",
             BOARD_WHEEL_ID_LEFT, BOARD_WHEEL_LEFT_SIGN,
             BOARD_WHEEL_ID_RIGHT, BOARD_WHEEL_RIGHT_SIGN,
             BOARD_BODY_HEIGHT_DEFAULT_PERCENT, BOARD_WHEEL_SPEED_LIMIT);
    atomic_store(&s_initialized, true);
    return ESP_OK;
}

esp_err_t wheel_control_set_body_pose_percent(int left_percent, int right_percent)
{
    const int clamped_left = clamp_percent(left_percent);
    const int clamped_right = clamp_percent(right_percent);
    const int base_percent = (clamped_left + clamped_right + 1) / 2;
    uint16_t goal12 = 0;
    uint16_t goal22 = 0;
    const uint16_t speed = BOARD_BODY_HEIGHT_MOVE_SPEED;
    uint8_t payload[10] = {0};

    ESP_RETURN_ON_ERROR(wheel_control_init(), TAG, "wheel control init failed");
    ESP_RETURN_ON_ERROR(capture_body_home_if_needed(), TAG, "capture body home failed");

    goal12 = (uint16_t)body_goal_for_servo(12, clamped_left);
    goal22 = (uint16_t)body_goal_for_servo(22, clamped_right);
    payload[0] = 12;
    payload[1] = (uint8_t)(goal12 & 0xFF);
    payload[2] = (uint8_t)(goal12 >> 8);
    payload[3] = (uint8_t)(speed & 0xFF);
    payload[4] = (uint8_t)(speed >> 8);
    payload[5] = 22;
    payload[6] = (uint8_t)(goal22 & 0xFF);
    payload[7] = (uint8_t)(goal22 >> 8);
    payload[8] = (uint8_t)(speed & 0xFF);
    payload[9] = (uint8_t)(speed >> 8);

    ESP_RETURN_ON_ERROR(motor_set_torque(12, true), TAG, "enable body servo 12 failed");
    ESP_RETURN_ON_ERROR(motor_set_torque(22, true), TAG, "enable body servo 22 failed");
    ESP_RETURN_ON_ERROR(scs_sync_write(SCS_ADDR_GOAL_POSITION_L, 4, payload, 2),
                        TAG, "set body height failed");

    ESP_LOGI(TAG,
             "body pose -> base=%d left=%d right=%d goal12=%u goal22=%u speed=%u",
             base_percent, clamped_left, clamped_right, goal12, goal22, speed);
    state_update_body_pose(base_percent, clamped_left, clamped_right, true);
    return ESP_OK;
}

esp_err_t wheel_control_set_height_percent(int percent)
{
    percent = clamp_percent(percent);
    return wheel_control_set_body_pose_percent(percent, percent);
}

esp_err_t wheel_control_set_chassis_speeds(int left_speed, int right_speed)
{
    int left_logical = wheel_control_clamp_speed(left_speed);
    int right_logical = wheel_control_clamp_speed(right_speed);
    int left_raw_signed = left_logical * BOARD_WHEEL_LEFT_SIGN;
    int right_raw_signed = right_logical * BOARD_WHEEL_RIGHT_SIGN;
    uint16_t left_raw = encode_signed_i16(left_raw_signed);
    uint16_t right_raw = encode_signed_i16(right_raw_signed);
    int desired_height = BOARD_BODY_HEIGHT_DEFAULT_PERCENT;
    bool body_pose_applied = false;
    bool wheel_torque_enabled = false;
    int current_left_speed = 0;
    int current_right_speed = 0;

    ESP_RETURN_ON_ERROR(wheel_control_init(), TAG, "wheel control init failed");

    portENTER_CRITICAL(&s_state_mux);
    desired_height = s_state.height_percent;
    body_pose_applied = s_state.body_pose_applied;
    wheel_torque_enabled = s_state.wheel_torque_enabled;
    current_left_speed = s_state.left_speed;
    current_right_speed = s_state.right_speed;
    portEXIT_CRITICAL(&s_state_mux);

    if (!body_pose_applied) {
        ESP_RETURN_ON_ERROR(wheel_control_set_height_percent(desired_height),
                            TAG, "apply safe body pose failed");
    }

    if (wheel_torque_enabled &&
        current_left_speed == left_logical &&
        current_right_speed == right_logical) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ensure_wheel_torque_enabled(), TAG, "wheel torque failed");
    ESP_RETURN_ON_ERROR(write_wheel_speed_with_retry(BOARD_WHEEL_ID_LEFT, left_raw, "left wheel speed write"),
                        TAG, "left wheel speed write failed");
    ESP_RETURN_ON_ERROR(write_wheel_speed_with_retry(BOARD_WHEEL_ID_RIGHT, right_raw, "right wheel speed write"),
                        TAG, "right wheel speed write failed");

    state_update_speeds(left_logical, right_logical, left_raw_signed, right_raw_signed, true);
    return ESP_OK;
}

esp_err_t wheel_control_set_twist(int forward_speed, int yaw_speed)
{
    int forward = wheel_control_clamp_speed(forward_speed);
    int yaw = wheel_control_clamp_speed(yaw_speed);
    int left = wheel_control_clamp_speed(forward + yaw);
    int right = wheel_control_clamp_speed(forward - yaw);

    return wheel_control_set_chassis_speeds(left, right);
}

esp_err_t wheel_control_stop(void)
{
    return wheel_control_set_chassis_speeds(0, 0);
}

bool wheel_control_get_state(wheel_control_state_t *out)
{
    if (!out) {
        return false;
    }

    portENTER_CRITICAL(&s_state_mux);
    *out = s_state;
    portEXIT_CRITICAL(&s_state_mux);
    return true;
}
