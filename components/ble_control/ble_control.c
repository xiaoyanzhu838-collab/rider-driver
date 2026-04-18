#include "ble_control.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "balance_task.h"
#include "board_config.h"
#include "host/util/util.h"
#include "motor_control.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "scs_proto.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_control";

#define BLE_DEVICE_NAME "RiderDriver"
#define BLE_MAX_TEXT_LEN 512

#define MOTOR_ID_12 12
#define MOTOR_ID_22 22
#define MOTOR_ID_11 11
#define MOTOR_ID_21 21
#define MOTOR_12_SAFE_DELTA 104
#define MOTOR_22_SAFE_DELTA 112
#define DEFAULT_MOVE_SPEED 200
#define WHEEL_SAFE_OPEN_PERCENT 20
#define WHEEL_HOLD_SPEED 10
#define WHEEL_MAX_GOAL_SPEED 1064
#define WHEEL_SIGNED_SPEED_LIMIT BOARD_WHEEL_SPEED_LIMIT
#define WHEEL_SIGNED_SPEED_RECOMMENDED BOARD_WHEEL_SPEED_RECOMMENDED

static uint8_t s_own_addr_type;
static bool s_ble_started = false;
static uint16_t s_status_handle;
static char s_status_text[BLE_MAX_TEXT_LEN] = "ready";
static char s_last_command[BLE_MAX_TEXT_LEN] = "";
static int s_home12 = 599;
static int s_home22 = 439;
static QueueHandle_t s_command_queue;
static portMUX_TYPE s_stream_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_balance_stream_enabled = false;
static uint32_t s_balance_stream_interval_ms = 100;

static const ble_uuid128_t s_service_uuid =
    BLE_UUID128_INIT(0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe,
                     0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe);
static const ble_uuid128_t s_command_uuid =
    BLE_UUID128_INIT(0x11, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe,
                     0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe);
static const ble_uuid128_t s_status_uuid =
    BLE_UUID128_INIT(0x12, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe,
                     0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe);

static bool ble_control_advertise(void);
static void status_scan_ids(int start_id, int end_id);

typedef enum {
    WHEEL_MODE_ABS = 0,
    WHEEL_MODE_RAW = 1,
    WHEEL_MODE_I16 = 2,
    WHEEL_MODE_DXL = 3,
} wheel_write_mode_t;

typedef struct {
    uint16_t model;
    uint8_t firmware;
    uint8_t id;
    uint8_t baud;
    uint8_t return_delay;
    uint16_t min_angle;
    uint16_t max_angle;
    uint8_t temp_limit;
    uint8_t min_voltage_raw;
    uint8_t max_voltage_raw;
    uint16_t max_torque;
    uint8_t status_return_level;
    uint8_t alarm_led;
    uint8_t shutdown;
    uint8_t led;
    uint8_t cw_margin;
    uint8_t ccw_margin;
    uint8_t cw_slope;
    uint8_t ccw_slope;
    uint16_t torque_limit;
    uint8_t lock;
} servo_config_t;

typedef struct {
    char text[BLE_MAX_TEXT_LEN];
} ble_command_item_t;

static void status_set_v(bool log_to_serial, const char *fmt, va_list args)
{
    vsnprintf(s_status_text, sizeof(s_status_text), fmt, args);

    if (log_to_serial) {
        ESP_LOGI(TAG, "%s", s_status_text);
    }
    if (s_status_handle != 0) {
        ble_gatts_chr_updated(s_status_handle);
    }
}

static void status_set(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    status_set_v(true, fmt, args);
    va_end(args);
}

static void status_set_quiet(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    status_set_v(false, fmt, args);
    va_end(args);
}

static int gatt_write_text(struct os_mbuf *om, char *dst, uint16_t max_len)
{
    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(om, dst, max_len - 1, &out_len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    dst[out_len] = '\0';
    return 0;
}

static uint32_t clamp_stream_interval_ms(uint32_t interval_ms)
{
    if (interval_ms < 50) {
        return 50;
    }
    if (interval_ms > 1000) {
        return 1000;
    }
    return interval_ms;
}

static void balance_stream_set(bool enabled, uint32_t interval_ms)
{
    portENTER_CRITICAL(&s_stream_mux);
    s_balance_stream_enabled = enabled;
    s_balance_stream_interval_ms = clamp_stream_interval_ms(interval_ms);
    portEXIT_CRITICAL(&s_stream_mux);
}

static void balance_stream_get(bool *enabled_out, uint32_t *interval_ms_out)
{
    bool enabled = false;
    uint32_t interval_ms = 100;

    portENTER_CRITICAL(&s_stream_mux);
    enabled = s_balance_stream_enabled;
    interval_ms = s_balance_stream_interval_ms;
    portEXIT_CRITICAL(&s_stream_mux);

    if (enabled_out) {
        *enabled_out = enabled;
    }
    if (interval_ms_out) {
        *interval_ms_out = interval_ms;
    }
}

static void status_balance_pid(void)
{
    balance_pid_config_t cfg = {0};
    if (balance_get_pid_config(&cfg) != ESP_OK) {
        status_set("ERR BGETPID read_failed");
        return;
    }

    status_set(
        "{\"kind\":\"pid\",\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"kw\":%.4f,"
        "\"target\":%.3f,\"ilim\":%.3f,\"dlim\":%.3f,\"log\":%d}",
        (double)cfg.kp,
        (double)cfg.ki,
        (double)cfg.kd,
        (double)cfg.kw,
        (double)cfg.target_deg,
        (double)cfg.i_term_limit,
        (double)cfg.d_term_limit,
        balance_get_runtime_log_enabled() ? 1 : 0);
}

static void status_balance_runtime_cfg(void)
{
    balance_runtime_config_t cfg = {0};
    if (balance_get_runtime_config(&cfg) != ESP_OK) {
        status_set("ERR BGETCFG read_failed");
        return;
    }

    status_set(
        "{\"kind\":\"cfg_balance\",\"speed_limit\":%d,\"tilt_cutoff\":%.2f}",
        cfg.speed_limit,
        (double)cfg.tilt_cutoff_deg);
}

static void status_balance_telemetry(bool quiet)
{
    balance_telemetry_t tel = {0};
    if (!balance_get_telemetry(&tel)) {
        if (quiet) {
            status_set_quiet("{\"kind\":\"telemetry\",\"ok\":0}");
        } else {
            status_set("{\"kind\":\"telemetry\",\"ok\":0}");
        }
        return;
    }

    if (quiet) {
        status_set_quiet(
            "{\"kind\":\"telemetry\",\"ok\":1,\"seq\":%lu,\"en\":%d,\"arm\":%d,\"guard\":%d,"
            "\"theta\":%.3f,\"target\":%.3f,\"err\":%.3f,\"zero\":%.3f,"
            "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
            "\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,"
            "\"rate\":%.3f,\"p\":%.3f,\"i\":%.3f,\"d\":%.3f,\"w\":%.3f,"
            "\"ws\":%.3f,\"raw\":%d,\"out\":%d,\"ff\":%d,\"yaw_out\":%d,"
            "\"ls\":%d,\"rs\":%d}",
            (unsigned long)tel.seq,
            tel.enabled ? 1 : 0,
            tel.armed ? 1 : 0,
            tel.in_tilt_guard ? 1 : 0,
            (double)tel.theta_fb_deg,
            (double)tel.target_deg,
            (double)tel.error_deg,
            (double)tel.zero_deg,
            (double)tel.imu.ax,
            (double)tel.imu.ay,
            (double)tel.imu.az,
            (double)tel.imu.gx,
            (double)tel.imu.gy,
            (double)tel.imu.gz,
            (double)tel.euler.roll,
            (double)tel.euler.pitch,
            (double)tel.euler.yaw,
            (double)tel.rate_deg_s,
            (double)tel.p_term,
            (double)tel.i_term,
            (double)tel.d_term,
            (double)tel.w_term,
            (double)tel.wheel_speed_feedback,
            tel.raw_forward_speed,
            tel.forward_speed,
            tel.feedforward_speed,
            tel.yaw_speed,
            tel.wheel.left_speed,
            tel.wheel.right_speed);
        return;
    }

    status_set(
        "{\"kind\":\"telemetry\",\"ok\":1,\"seq\":%lu,\"en\":%d,\"arm\":%d,\"guard\":%d,"
        "\"theta\":%.3f,\"target\":%.3f,\"err\":%.3f,\"zero\":%.3f,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
        "\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,"
        "\"rate\":%.3f,\"p\":%.3f,\"i\":%.3f,\"d\":%.3f,\"w\":%.3f,"
        "\"ws\":%.3f,\"raw\":%d,\"out\":%d,\"ff\":%d,\"yaw_out\":%d,"
        "\"ls\":%d,\"rs\":%d}",
        (unsigned long)tel.seq,
        tel.enabled ? 1 : 0,
        tel.armed ? 1 : 0,
        tel.in_tilt_guard ? 1 : 0,
        (double)tel.theta_fb_deg,
        (double)tel.target_deg,
        (double)tel.error_deg,
        (double)tel.zero_deg,
        (double)tel.imu.ax,
        (double)tel.imu.ay,
        (double)tel.imu.az,
        (double)tel.imu.gx,
        (double)tel.imu.gy,
        (double)tel.imu.gz,
        (double)tel.euler.roll,
        (double)tel.euler.pitch,
        (double)tel.euler.yaw,
        (double)tel.rate_deg_s,
        (double)tel.p_term,
        (double)tel.i_term,
        (double)tel.d_term,
        (double)tel.w_term,
        (double)tel.wheel_speed_feedback,
        tel.raw_forward_speed,
        tel.forward_speed,
        tel.feedforward_speed,
        tel.yaw_speed,
        tel.wheel.left_speed,
        tel.wheel.right_speed);
}

static bool is_supported_servo(uint8_t id)
{
    return id == MOTOR_ID_12 || id == MOTOR_ID_22;
}

static bool is_supported_wheel(uint8_t id)
{
    return id == MOTOR_ID_11 || id == MOTOR_ID_21;
}

static bool is_supported_node(uint8_t id)
{
    return is_supported_servo(id) || is_supported_wheel(id);
}

static int servo_home(uint8_t id)
{
    return id == MOTOR_ID_12 ? s_home12 : s_home22;
}

static int servo_edge(uint8_t id)
{
    return id == MOTOR_ID_12
        ? (s_home12 - MOTOR_12_SAFE_DELTA)
        : (s_home22 + MOTOR_22_SAFE_DELTA);
}

static int servo_min_goal(uint8_t id)
{
    return id == MOTOR_ID_12 ? servo_edge(id) : servo_home(id);
}

static int servo_max_goal(uint8_t id)
{
    return id == MOTOR_ID_12 ? servo_home(id) : servo_edge(id);
}

static int clamp_speed(int speed)
{
    if (speed <= 0) {
        return DEFAULT_MOVE_SPEED;
    }
    if (speed > 400) {
        return 400;
    }
    return speed;
}

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

static int clamp_goal_for_servo(uint8_t id, int goal)
{
    int min_goal = servo_min_goal(id);
    int max_goal = servo_max_goal(id);
    if (goal < min_goal) {
        return min_goal;
    }
    if (goal > max_goal) {
        return max_goal;
    }
    return goal;
}

static int clamp_u8_range(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int clamp_torque_limit(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 1023) {
        return 1023;
    }
    return value;
}

static int clamp_wheel_goal_speed(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > WHEEL_MAX_GOAL_SPEED) {
        return WHEEL_MAX_GOAL_SPEED;
    }
    return value;
}

static int clamp_wheel_signed_speed(int value)
{
    if (value < -WHEEL_SIGNED_SPEED_LIMIT) {
        return -WHEEL_SIGNED_SPEED_LIMIT;
    }
    if (value > WHEEL_SIGNED_SPEED_LIMIT) {
        return WHEEL_SIGNED_SPEED_LIMIT;
    }
    return value;
}

static int clamp_i16_range(int value)
{
    if (value < -32768) {
        return -32768;
    }
    if (value > 32767) {
        return 32767;
    }
    return value;
}

static int clamp_u16_range(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 65535) {
        return 65535;
    }
    return value;
}

static int clamp_probe_samples(int value)
{
    if (value < 2) {
        return 2;
    }
    if (value > 40) {
        return 40;
    }
    return value;
}

static int clamp_probe_interval_ms(int value)
{
    if (value < 20) {
        return 20;
    }
    if (value > 500) {
        return 500;
    }
    return value;
}

static int clamp_probe_settle_ms(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 1000) {
        return 1000;
    }
    return value;
}

static const char *wheel_mode_name(wheel_write_mode_t mode)
{
    switch (mode) {
    case WHEEL_MODE_ABS:
        return "ABS";
    case WHEEL_MODE_RAW:
        return "RAW";
    case WHEEL_MODE_I16:
        return "I16";
    case WHEEL_MODE_DXL:
        return "DXL";
    default:
        return "UNK";
    }
}

static bool parse_wheel_mode(const char *text, wheel_write_mode_t *mode_out)
{
    if (!text || !mode_out) {
        return false;
    }

    if (strcmp(text, "ABS") == 0) {
        *mode_out = WHEEL_MODE_ABS;
        return true;
    }
    if (strcmp(text, "RAW") == 0) {
        *mode_out = WHEEL_MODE_RAW;
        return true;
    }
    if (strcmp(text, "I16") == 0) {
        *mode_out = WHEEL_MODE_I16;
        return true;
    }
    if (strcmp(text, "DXL") == 0) {
        *mode_out = WHEEL_MODE_DXL;
        return true;
    }
    return false;
}

static uint16_t encode_wheel_signed_i16(int value)
{
    return (uint16_t)(int16_t)clamp_i16_range(value);
}

static int decode_wheel_signed_i16(uint16_t value)
{
    return (int16_t)value;
}

static uint16_t encode_wheel_dxl_signed(int value)
{
    int magnitude = value < 0 ? -value : value;
    if (magnitude > 1023) {
        magnitude = 1023;
    }
    if (value < 0) {
        return (uint16_t)(0x0400 | magnitude);
    }
    return (uint16_t)magnitude;
}

static uint16_t encode_wheel_goal_value(wheel_write_mode_t mode, int value)
{
    switch (mode) {
    case WHEEL_MODE_ABS:
        return (uint16_t)clamp_wheel_goal_speed(value);
    case WHEEL_MODE_RAW:
        return (uint16_t)clamp_u16_range(value);
    case WHEEL_MODE_I16:
        return encode_wheel_signed_i16(value);
    case WHEEL_MODE_DXL:
        return encode_wheel_dxl_signed(value);
    default:
        return 0;
    }
}

static int wheel_step_delta(uint16_t prev, uint16_t curr)
{
    int delta = (int)curr - (int)prev;
    if (delta > 512) {
        delta -= 1024;
    } else if (delta < -512) {
        delta += 1024;
    }
    return delta;
}

static int pair_goal_for_percent(uint8_t id, int percent)
{
    percent = clamp_percent(percent);
    if (id == MOTOR_ID_12) {
        int delta = (MOTOR_12_SAFE_DELTA * percent + 50) / 100;
        return s_home12 - delta;
    }

    int delta = (MOTOR_22_SAFE_DELTA * percent + 50) / 100;
    return s_home22 + delta;
}

static bool read_goal_and_torque(uint8_t id, uint16_t *goal_out, uint8_t *torque_out)
{
    if (goal_out && scs_read_word(id, SCS_ADDR_GOAL_POSITION_L, goal_out) != ESP_OK) {
        return false;
    }
    if (torque_out && scs_read_byte(id, SCS_ADDR_TORQUE_ENABLE, torque_out) != ESP_OK) {
        return false;
    }
    return true;
}

static bool read_servo_config(uint8_t id, servo_config_t *out)
{
    memset(out, 0, sizeof(*out));
    return scs_read_word(id, SCS_ADDR_MODEL_L, &out->model) == ESP_OK &&
           scs_read_byte(id, SCS_ADDR_VERSION, &out->firmware) == ESP_OK &&
           scs_read_byte(id, SCS_ADDR_ID, &out->id) == ESP_OK &&
           scs_read_byte(id, SCS_ADDR_BAUD_RATE, &out->baud) == ESP_OK &&
           scs_read_byte(id, SCS_ADDR_RETURN_DELAY, &out->return_delay) == ESP_OK &&
           scs_read_word(id, SCS_ADDR_MIN_ANGLE_L, &out->min_angle) == ESP_OK &&
           scs_read_word(id, SCS_ADDR_MAX_ANGLE_L, &out->max_angle) == ESP_OK &&
           scs_read_byte(id, 0x0B, &out->temp_limit) == ESP_OK &&
           scs_read_byte(id, 0x0C, &out->min_voltage_raw) == ESP_OK &&
           scs_read_byte(id, 0x0D, &out->max_voltage_raw) == ESP_OK &&
           scs_read_word(id, 0x0E, &out->max_torque) == ESP_OK &&
           scs_read_byte(id, 0x10, &out->status_return_level) == ESP_OK &&
           scs_read_byte(id, 0x11, &out->alarm_led) == ESP_OK &&
           scs_read_byte(id, 0x12, &out->shutdown) == ESP_OK &&
           scs_read_byte(id, 0x19, &out->led) == ESP_OK &&
           scs_read_byte(id, 0x1A, &out->cw_margin) == ESP_OK &&
           scs_read_byte(id, 0x1B, &out->ccw_margin) == ESP_OK &&
           scs_read_byte(id, 0x1C, &out->cw_slope) == ESP_OK &&
           scs_read_byte(id, 0x1D, &out->ccw_slope) == ESP_OK &&
           scs_read_word(id, 0x22, &out->torque_limit) == ESP_OK &&
           scs_read_byte(id, 0x2F, &out->lock) == ESP_OK;
}

static void status_scan_ids(int start_id, int end_id)
{
    char ids_text[256] = {0};
    size_t used = 0;
    int count = 0;

    if (start_id < 1) {
        start_id = 1;
    }
    if (end_id > 253) {
        end_id = 253;
    }
    if (start_id > end_id) {
        int tmp = start_id;
        start_id = end_id;
        end_id = tmp;
    }

    if (motor_init() != ESP_OK) {
        status_set("ERR SCAN motor_init_failed");
        return;
    }

    ESP_LOGI(TAG, "starting UART2 bus scan: ids %d..%d", start_id, end_id);

    for (int id = start_id; id <= end_id; id++) {
        scs_status_t st = {0};
        if (scs_ping((uint8_t)id, &st) == ESP_OK) {
            int written = snprintf(ids_text + used, sizeof(ids_text) - used,
                                   "%s%d", count == 0 ? "" : ",", id);
            if (written > 0 && (size_t)written < (sizeof(ids_text) - used)) {
                used += (size_t)written;
            }
            count++;
            ESP_LOGI(TAG, "SCAN hit: id=%d error=0x%02X params=%u",
                     id, st.error, st.param_count);
        }
        vTaskDelay(pdMS_TO_TICKS(8));
    }

    status_set(
        "{\"kind\":\"scan\",\"start\":%d,\"end\":%d,\"count\":%d,\"ids\":[%s]}",
        start_id, end_id, count, ids_text);
}

static void status_config(uint8_t id)
{
    servo_config_t cfg = {0};
    if (!is_supported_node(id)) {
        status_set("ERR CFG id=%u unsupported", id);
        return;
    }
    if (!read_servo_config(id, &cfg)) {
        status_set("ERR CFG id=%u read_failed", id);
        return;
    }

    status_set(
        "{\"kind\":\"cfg\",\"id\":%u,\"model\":%u,\"fw\":%u,\"baud\":%u,"
        "\"delay\":%u,\"amin\":%u,\"amax\":%u,\"tmpl\":%u,\"vmin\":%u,"
        "\"vmax\":%u,\"tmax\":%u,\"sret\":%u,\"alarm\":%u,\"shutdown\":%u,"
        "\"led\":%u,\"cwm\":%u,\"ccwm\":%u,\"cws\":%u,\"ccws\":%u,"
        "\"tlim\":%u,\"lock\":%u}",
        cfg.id, cfg.model, cfg.firmware, cfg.baud, cfg.return_delay,
        cfg.min_angle, cfg.max_angle, cfg.temp_limit, cfg.min_voltage_raw,
        cfg.max_voltage_raw, cfg.max_torque, cfg.status_return_level,
        cfg.alarm_led, cfg.shutdown, cfg.led, cfg.cw_margin, cfg.ccw_margin,
        cfg.cw_slope, cfg.ccw_slope, cfg.torque_limit, cfg.lock);
}

static void status_wheel_snapshot(void)
{
    motor_feedback_t fb11 = {0};
    motor_feedback_t fb21 = {0};
    uint16_t goal11 = 0;
    uint16_t goal21 = 0;
    uint16_t goal_speed11 = 0;
    uint16_t goal_speed21 = 0;
    uint8_t torque11 = 0;
    uint8_t torque21 = 0;

    bool ok11 = motor_read_feedback(MOTOR_ID_11, &fb11) == ESP_OK &&
                read_goal_and_torque(MOTOR_ID_11, &goal11, &torque11) &&
                scs_read_word(MOTOR_ID_11, SCS_ADDR_GOAL_SPEED_L, &goal_speed11) == ESP_OK;
    bool ok21 = motor_read_feedback(MOTOR_ID_21, &fb21) == ESP_OK &&
                read_goal_and_torque(MOTOR_ID_21, &goal21, &torque21) &&
                scs_read_word(MOTOR_ID_21, SCS_ADDR_GOAL_SPEED_L, &goal_speed21) == ESP_OK;

    if (!ok11 || !ok21) {
        status_set("ERR WSNAP ok11=%d ok21=%d", ok11 ? 1 : 0, ok21 ? 1 : 0);
        return;
    }

    status_set(
        "{\"kind\":\"wheel\",\"safe\":%d,\"limit\":%d,\"recommend\":%d,"
        "\"p11\":%u,\"g11\":%u,\"t11\":%u,\"gs11\":%u,\"gsi11\":%d,\"s11\":%u,\"si11\":%d,\"l11\":%u,\"v11\":%u,\"tmp11\":%u,\"m11\":%u,"
        "\"p21\":%u,\"g21\":%u,\"t21\":%u,\"gs21\":%u,\"gsi21\":%d,\"s21\":%u,\"si21\":%d,\"l21\":%u,\"v21\":%u,\"tmp21\":%u,\"m21\":%u}",
        WHEEL_SAFE_OPEN_PERCENT, WHEEL_SIGNED_SPEED_LIMIT, WHEEL_SIGNED_SPEED_RECOMMENDED,
        fb11.position, goal11, torque11, goal_speed11, decode_wheel_signed_i16(goal_speed11),
        fb11.speed, decode_wheel_signed_i16(fb11.speed), fb11.load, fb11.voltage, fb11.temperature, fb11.moving,
        fb21.position, goal21, torque21, goal_speed21, decode_wheel_signed_i16(goal_speed21),
        fb21.speed, decode_wheel_signed_i16(fb21.speed), fb21.load, fb21.voltage, fb21.temperature, fb21.moving);
}

static void status_snapshot(void)
{
    motor_feedback_t fb12 = {0};
    motor_feedback_t fb22 = {0};
    uint16_t goal12 = 0;
    uint16_t goal22 = 0;
    uint8_t torque12 = 0;
    uint8_t torque22 = 0;

    bool ok12 = motor_read_feedback(MOTOR_ID_12, &fb12) == ESP_OK &&
                read_goal_and_torque(MOTOR_ID_12, &goal12, &torque12);
    bool ok22 = motor_read_feedback(MOTOR_ID_22, &fb22) == ESP_OK &&
                read_goal_and_torque(MOTOR_ID_22, &goal22, &torque22);

    if (!ok12 || !ok22) {
        status_set("ERR SNAP ok12=%d ok22=%d", ok12 ? 1 : 0, ok22 ? 1 : 0);
        return;
    }

    status_set(
        "{\"kind\":\"snap\",\"h12\":%d,\"e12\":%d,\"p12\":%u,\"g12\":%u,"
        "\"t12\":%u,\"s12\":%u,\"l12\":%u,\"v12\":%u,\"tmp12\":%u,\"m12\":%u,"
        "\"h22\":%d,\"e22\":%d,\"p22\":%u,\"g22\":%u,\"t22\":%u,\"s22\":%u,"
        "\"l22\":%u,\"v22\":%u,\"tmp22\":%u,\"m22\":%u}",
        s_home12, servo_edge(MOTOR_ID_12), fb12.position, goal12, torque12,
        fb12.speed, fb12.load, fb12.voltage, fb12.temperature, fb12.moving,
        s_home22, servo_edge(MOTOR_ID_22), fb22.position, goal22, torque22,
        fb22.speed, fb22.load, fb22.voltage, fb22.temperature, fb22.moving);
}

static esp_err_t set_single_safe_goal(uint8_t id, int goal, int speed)
{
    if (!is_supported_servo(id)) {
        return ESP_ERR_INVALID_ARG;
    }

    goal = clamp_goal_for_servo(id, goal);
    speed = clamp_speed(speed);

    ESP_RETURN_ON_ERROR(motor_set_torque(id, true), TAG, "torque on failed");
    ESP_RETURN_ON_ERROR(scs_write_word(id, SCS_ADDR_GOAL_POSITION_L, (uint16_t)goal),
                        TAG, "goal write failed");
    return scs_write_word(id, SCS_ADDR_GOAL_SPEED_L, (uint16_t)speed);
}

static esp_err_t set_pair_percent(int percent, int speed)
{
    const uint16_t goal12 = (uint16_t)pair_goal_for_percent(MOTOR_ID_12, percent);
    const uint16_t goal22 = (uint16_t)pair_goal_for_percent(MOTOR_ID_22, percent);
    const uint16_t speed_u16 = (uint16_t)clamp_speed(speed);
    uint8_t pair_data[10] = {
        MOTOR_ID_12,
        (uint8_t)(goal12 & 0xFF),
        (uint8_t)(goal12 >> 8),
        (uint8_t)(speed_u16 & 0xFF),
        (uint8_t)(speed_u16 >> 8),
        MOTOR_ID_22,
        (uint8_t)(goal22 & 0xFF),
        (uint8_t)(goal22 >> 8),
        (uint8_t)(speed_u16 & 0xFF),
        (uint8_t)(speed_u16 >> 8),
    };

    ESP_RETURN_ON_ERROR(motor_set_torque(MOTOR_ID_12, true), TAG, "torque on 12 failed");
    ESP_RETURN_ON_ERROR(motor_set_torque(MOTOR_ID_22, true), TAG, "torque on 22 failed");
    return scs_sync_write(SCS_ADDR_GOAL_POSITION_L, 4, pair_data, 2);
}

static esp_err_t relax_all(void)
{
    ESP_RETURN_ON_ERROR(motor_set_torque(MOTOR_ID_12, false), TAG, "relax 12 failed");
    return motor_set_torque(MOTOR_ID_22, false);
}

static esp_err_t capture_home_positions(void)
{
    motor_feedback_t fb12 = {0};
    motor_feedback_t fb22 = {0};
    ESP_RETURN_ON_ERROR(motor_read_feedback(MOTOR_ID_12, &fb12), TAG, "read 12 failed");
    ESP_RETURN_ON_ERROR(motor_read_feedback(MOTOR_ID_22, &fb22), TAG, "read 22 failed");

    s_home12 = fb12.position;
    s_home22 = fb22.position;
    return ESP_OK;
}

static esp_err_t set_led(uint8_t id, int value)
{
    if (!is_supported_servo(id)) {
        return ESP_ERR_INVALID_ARG;
    }
    return scs_write_byte(id, 0x19, value != 0 ? 1 : 0);
}

static esp_err_t set_torque_limit_value(uint8_t id, int value)
{
    if (!is_supported_servo(id)) {
        return ESP_ERR_INVALID_ARG;
    }
    return scs_write_word(id, 0x22, (uint16_t)clamp_torque_limit(value));
}

static esp_err_t set_compliance(uint8_t id, int cw_margin, int ccw_margin, int cw_slope, int ccw_slope)
{
    if (!is_supported_servo(id)) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(scs_write_byte(id, 0x1A, (uint8_t)clamp_u8_range(cw_margin, 0, 16)),
                        TAG, "cw margin write failed");
    ESP_RETURN_ON_ERROR(scs_write_byte(id, 0x1B, (uint8_t)clamp_u8_range(ccw_margin, 0, 16)),
                        TAG, "ccw margin write failed");
    ESP_RETURN_ON_ERROR(scs_write_byte(id, 0x1C, (uint8_t)clamp_u8_range(cw_slope, 1, 64)),
                        TAG, "cw slope write failed");
    return scs_write_byte(id, 0x1D, (uint8_t)clamp_u8_range(ccw_slope, 1, 64));
}

static esp_err_t restore_known_runtime_defaults(uint8_t id)
{
    ESP_RETURN_ON_ERROR(set_led(id, 0), TAG, "restore led failed");
    ESP_RETURN_ON_ERROR(set_torque_limit_value(id, 1023), TAG, "restore torque limit failed");
    return set_compliance(id, 1, 1, 32, 32);
}

static esp_err_t ensure_wheel_safe_open(void)
{
    return set_pair_percent(WHEEL_SAFE_OPEN_PERCENT, DEFAULT_MOVE_SPEED);
}

static esp_err_t wheel_set_speed(uint8_t id, int speed)
{
    uint16_t raw = encode_wheel_signed_i16(clamp_wheel_signed_speed(speed));

    if (!is_supported_wheel(id)) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_wheel_safe_open(), TAG, "safe open failed");
    ESP_RETURN_ON_ERROR(motor_set_torque(id, true), TAG, "wheel torque on failed");
    return scs_write_word(id, SCS_ADDR_GOAL_SPEED_L, raw);
}

static esp_err_t wheel_set_encoded(uint8_t id, wheel_write_mode_t mode, int value, uint16_t *raw_out)
{
    uint16_t raw = encode_wheel_goal_value(mode, value);

    if (!is_supported_wheel(id)) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_wheel_safe_open(), TAG, "safe open failed");
    ESP_RETURN_ON_ERROR(motor_set_torque(id, true), TAG, "wheel torque on failed");
    ESP_RETURN_ON_ERROR(scs_write_word(id, SCS_ADDR_GOAL_SPEED_L, raw), TAG, "wheel raw speed write failed");
    if (raw_out) {
        *raw_out = raw;
    }
    return ESP_OK;
}

static esp_err_t wheel_hold_current(uint8_t id)
{
    motor_feedback_t fb = {0};

    if (!is_supported_wheel(id)) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_wheel_safe_open(), TAG, "safe open failed");
    ESP_RETURN_ON_ERROR(motor_read_feedback(id, &fb), TAG, "wheel read failed");
    ESP_RETURN_ON_ERROR(motor_set_torque(id, true), TAG, "wheel torque on failed");
    ESP_RETURN_ON_ERROR(scs_write_word(id, SCS_ADDR_GOAL_POSITION_L, fb.position), TAG, "wheel goal write failed");
    return scs_write_word(id, SCS_ADDR_GOAL_SPEED_L, WHEEL_HOLD_SPEED);
}

static esp_err_t wheel_stop(uint8_t id)
{
    if (!is_supported_wheel(id)) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ensure_wheel_safe_open(), TAG, "safe open failed");
    ESP_RETURN_ON_ERROR(motor_set_torque(id, true), TAG, "wheel torque on failed");
    return scs_write_word(id, SCS_ADDR_GOAL_SPEED_L, 0);
}

static esp_err_t wheel_relax(uint8_t id)
{
    if (!is_supported_wheel(id)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(ensure_wheel_safe_open(), TAG, "safe open failed");
    return motor_set_torque(id, false);
}

static void status_wheel_probe(wheel_write_mode_t mode, uint8_t id, int value,
                               int settle_ms, int samples, int interval_ms)
{
    motor_feedback_t start = {0};
    motor_feedback_t fb = {0};
    motor_feedback_t stop_fb = {0};
    uint16_t raw = 0;
    int sum_delta = 0;
    int sum_abs_delta = 0;
    int pos_steps = 0;
    int neg_steps = 0;
    int max_speed = 0;
    int last_delta = 0;
    bool stop_ok = false;

    settle_ms = clamp_probe_settle_ms(settle_ms);
    samples = clamp_probe_samples(samples);
    interval_ms = clamp_probe_interval_ms(interval_ms);

    if (!is_supported_wheel(id)) {
        status_set("ERR WPROBE id=%u unsupported", id);
        return;
    }

    if (motor_read_feedback(id, &start) != ESP_OK) {
        status_set("ERR WPROBE id=%u start_read_failed", id);
        return;
    }

    esp_err_t err = wheel_set_encoded(id, mode, value, &raw);
    if (err != ESP_OK) {
        status_set("ERR WPROBE id=%u %s", id, esp_err_to_name(err));
        return;
    }

    if (settle_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(settle_ms));
    }

    uint16_t prev_pos = start.position;
    for (int i = 0; i < samples; i++) {
        if (motor_read_feedback(id, &fb) != ESP_OK) {
            status_set("ERR WPROBE id=%u sample_failed idx=%d", id, i);
            (void)wheel_stop(id);
            return;
        }

        last_delta = wheel_step_delta(prev_pos, fb.position);
        sum_delta += last_delta;
        sum_abs_delta += last_delta < 0 ? -last_delta : last_delta;
        if (last_delta > 0) {
            pos_steps++;
        } else if (last_delta < 0) {
            neg_steps++;
        }
        if ((int)fb.speed > max_speed) {
            max_speed = fb.speed;
        }

        prev_pos = fb.position;
        if (i + 1 < samples) {
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }
    }

    stop_ok = wheel_stop(id) == ESP_OK;
    if (stop_ok) {
        vTaskDelay(pdMS_TO_TICKS(80));
        (void)motor_read_feedback(id, &stop_fb);
    }

    status_set(
        "{\"kind\":\"wprobe\",\"mode\":\"%s\",\"id\":%u,\"input\":%d,\"raw\":%u,"
        "\"settle\":%d,\"samples\":%d,\"dt\":%d,"
        "\"start_p\":%u,\"end_p\":%u,\"sum\":%d,\"sum_abs\":%d,"
        "\"pos\":%d,\"neg\":%d,\"last\":%d,\"max_s\":%d,"
        "\"last_s\":%u,\"last_l\":%u,\"last_m\":%u,"
        "\"stop_ok\":%d,\"stop_p\":%u,\"stop_s\":%u}",
        wheel_mode_name(mode), id, value, raw,
        settle_ms, samples, interval_ms,
        start.position, fb.position, sum_delta, sum_abs_delta,
        pos_steps, neg_steps, last_delta, max_speed,
        fb.speed, fb.load, fb.moving,
        stop_ok ? 1 : 0, stop_fb.position, stop_fb.speed);
}

static void execute_command(const char *cmd)
{
    char op[24] = {0};
    char mode_text[24] = {0};
    int id = 0;
    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;

    strncpy(s_last_command, cmd, sizeof(s_last_command) - 1);
    s_last_command[sizeof(s_last_command) - 1] = '\0';

    if (sscanf(cmd, "%23s", op) != 1) {
        status_set("ERR empty command");
        return;
    }

    if (strcmp(op, "PING") == 0) {
        status_set("OK PONG");
        return;
    }

    if (strcmp(op, "HELP") == 0) {
        status_set("OK cmds: GET WGET BGETPID BSETPID BGETCFG BSETCFG BGETTEL BTELE BLOG SCAN CFG LED TL COMP RESTORE_RUNTIME PAIR HOME EDGE SET TORQUE RELAX CAPTURE_HOME WSET(signed) WRAW WI16 WDXL WPROBE WSTOP WHOLD WRELAX PING");
        return;
    }

    if (strcmp(op, "BGETPID") == 0) {
        status_balance_pid();
        return;
    }

    if (strcmp(op, "BGETTEL") == 0) {
        status_balance_telemetry(false);
        return;
    }

    if (strcmp(op, "BGETCFG") == 0) {
        status_balance_runtime_cfg();
        return;
    }

    if (strcmp(op, "BLOG") == 0 && sscanf(cmd, "%23s %d", op, &a) == 2) {
        balance_set_runtime_log_enabled(a != 0);
        status_set("{\"kind\":\"blog\",\"enabled\":%d}", a != 0 ? 1 : 0);
        return;
    }

    if (strcmp(op, "BTELE") == 0) {
        unsigned long interval_ms = 100;
        int enabled = 0;
        int matched = sscanf(cmd, "%23s %d %lu", op, &enabled, &interval_ms);
        if (matched >= 2) {
            balance_stream_set(enabled != 0, (uint32_t)interval_ms);
            status_set("{\"kind\":\"btele\",\"enabled\":%d,\"interval_ms\":%lu}",
                       enabled != 0 ? 1 : 0,
                       (unsigned long)clamp_stream_interval_ms((uint32_t)interval_ms));
            return;
        }
    }

    if (strcmp(op, "BSETPID") == 0) {
        balance_pid_config_t cfg = {0};
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float kw = 0.0f;
        float target = 0.0f;
        int matched = 0;

        if (balance_get_pid_config(&cfg) != ESP_OK) {
            status_set("ERR BSETPID read_failed");
            return;
        }

        matched = sscanf(cmd, "%23s %f %f %f %f %f", op, &kp, &ki, &kd, &kw, &target);
        if (matched < 4) {
            status_set("ERR BSETPID need kp ki kd [kw] [target]");
            return;
        }

        cfg.kp = kp;
        cfg.ki = ki;
        cfg.kd = kd;
        if (matched >= 5) {
            cfg.kw = kw;
        }
        if (matched >= 6) {
            cfg.target_deg = target;
        }

        if (balance_set_pid_config(&cfg) != ESP_OK) {
            status_set("ERR BSETPID write_failed");
            return;
        }
        status_balance_pid();
        return;
    }

    if (strcmp(op, "BSETCFG") == 0) {
        balance_runtime_config_t cfg = {0};
        int matched = 0;

        if (balance_get_runtime_config(&cfg) != ESP_OK) {
            status_set("ERR BSETCFG read_failed");
            return;
        }

        matched = sscanf(cmd, "%23s %d %f",
                         op,
                         &cfg.speed_limit,
                         &cfg.tilt_cutoff_deg);
        if (matched < 3) {
            status_set("ERR BSETCFG need speed_limit tilt_cutoff");
            return;
        }

        if (balance_set_runtime_config(&cfg) != ESP_OK) {
            status_set("ERR BSETCFG write_failed");
            return;
        }
        status_balance_runtime_cfg();
        return;
    }

    if (strcmp(op, "GET") == 0 || strcmp(op, "READ") == 0) {
        status_snapshot();
        return;
    }

    if (strcmp(op, "WGET") == 0) {
        status_wheel_snapshot();
        return;
    }

    if (strcmp(op, "SCAN") == 0) {
        int start_id = 1;
        int end_id = 253;
        (void)sscanf(cmd, "%23s %d %d", op, &start_id, &end_id);
        status_scan_ids(start_id, end_id);
        return;
    }

    if (strcmp(op, "CFG") == 0 && sscanf(cmd, "%23s %d", op, &id) == 2) {
        status_config((uint8_t)id);
        return;
    }

    if (strcmp(op, "CAPTURE_HOME") == 0) {
        esp_err_t err = capture_home_positions();
        if (err == ESP_OK) {
            status_snapshot();
        } else {
            status_set("ERR CAPTURE_HOME %s", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "RELAX") == 0) {
        esp_err_t err = relax_all();
        if (err == ESP_OK) {
            status_snapshot();
        } else {
            status_set("ERR RELAX %s", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "HOME") == 0) {
        int speed = DEFAULT_MOVE_SPEED;
        (void)sscanf(cmd, "%23s %d", op, &speed);
        esp_err_t err = set_pair_percent(0, speed);
        if (err == ESP_OK) {
            status_snapshot();
        } else {
            status_set("ERR HOME %s", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "EDGE") == 0) {
        int speed = DEFAULT_MOVE_SPEED;
        (void)sscanf(cmd, "%23s %d", op, &speed);
        esp_err_t err = set_pair_percent(100, speed);
        if (err == ESP_OK) {
            status_snapshot();
        } else {
            status_set("ERR EDGE %s", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "PAIR") == 0 && sscanf(cmd, "%23s %d %d", op, &a, &b) >= 2) {
        int percent = clamp_percent(a);
        int speed = clamp_speed(b);
        esp_err_t err = set_pair_percent(percent, speed);
        if (err == ESP_OK) {
            status_snapshot();
        } else {
            status_set("ERR PAIR %s", esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "SET") == 0 && sscanf(cmd, "%23s %d %d %d", op, &id, &a, &b) >= 3) {
        int speed = clamp_speed(b);
        esp_err_t err = set_single_safe_goal((uint8_t)id, a, speed);
        if (err == ESP_OK) {
            status_snapshot();
        } else {
            status_set("ERR SET id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "TORQUE") == 0 && sscanf(cmd, "%23s %d %d", op, &id, &a) == 3) {
        if (!is_supported_servo((uint8_t)id)) {
            status_set("ERR TORQUE id=%d unsupported", id);
            return;
        }
        esp_err_t err = motor_set_torque((uint8_t)id, a != 0);
        if (err == ESP_OK) {
            status_snapshot();
        } else {
            status_set("ERR TORQUE id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "LED") == 0 && sscanf(cmd, "%23s %d %d", op, &id, &a) == 3) {
        esp_err_t err = set_led((uint8_t)id, a);
        if (err == ESP_OK) {
            status_config((uint8_t)id);
        } else {
            status_set("ERR LED id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "TL") == 0 && sscanf(cmd, "%23s %d %d", op, &id, &a) == 3) {
        esp_err_t err = set_torque_limit_value((uint8_t)id, a);
        if (err == ESP_OK) {
            status_config((uint8_t)id);
        } else {
            status_set("ERR TL id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "COMP") == 0 && sscanf(cmd, "%23s %d %d %d %d %d", op, &id, &a, &b, &c, &d) == 6) {
        esp_err_t err = set_compliance((uint8_t)id, a, b, c, d);
        if (err == ESP_OK) {
            status_config((uint8_t)id);
        } else {
            status_set("ERR COMP id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "RESTORE_RUNTIME") == 0 && sscanf(cmd, "%23s %d", op, &id) == 2) {
        esp_err_t err = restore_known_runtime_defaults((uint8_t)id);
        if (err == ESP_OK) {
            status_config((uint8_t)id);
        } else {
            status_set("ERR RESTORE_RUNTIME id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "WSET") == 0 && sscanf(cmd, "%23s %d %d", op, &id, &a) == 3) {
        esp_err_t err = wheel_set_speed((uint8_t)id, a);
        if (err == ESP_OK) {
            status_wheel_snapshot();
        } else {
            status_set("ERR WSET id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "WRAW") == 0 && sscanf(cmd, "%23s %d %d", op, &id, &a) == 3) {
        uint16_t raw = 0;
        esp_err_t err = wheel_set_encoded((uint8_t)id, WHEEL_MODE_RAW, a, &raw);
        if (err == ESP_OK) {
            status_set("{\"kind\":\"wheel_write\",\"mode\":\"RAW\",\"id\":%d,\"input\":%d,\"raw\":%u}", id, a, raw);
        } else {
            status_set("ERR WRAW id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "WI16") == 0 && sscanf(cmd, "%23s %d %d", op, &id, &a) == 3) {
        uint16_t raw = 0;
        esp_err_t err = wheel_set_encoded((uint8_t)id, WHEEL_MODE_I16, a, &raw);
        if (err == ESP_OK) {
            status_set("{\"kind\":\"wheel_write\",\"mode\":\"I16\",\"id\":%d,\"input\":%d,\"raw\":%u}", id, a, raw);
        } else {
            status_set("ERR WI16 id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "WDXL") == 0 && sscanf(cmd, "%23s %d %d", op, &id, &a) == 3) {
        uint16_t raw = 0;
        esp_err_t err = wheel_set_encoded((uint8_t)id, WHEEL_MODE_DXL, a, &raw);
        if (err == ESP_OK) {
            status_set("{\"kind\":\"wheel_write\",\"mode\":\"DXL\",\"id\":%d,\"input\":%d,\"raw\":%u}", id, a, raw);
        } else {
            status_set("ERR WDXL id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "WPROBE") == 0 &&
        sscanf(cmd, "%23s %23s %d %d %d %d %d", op, mode_text, &id, &a, &b, &c, &d) >= 4) {
        wheel_write_mode_t mode = WHEEL_MODE_ABS;
        if (!parse_wheel_mode(mode_text, &mode)) {
            status_set("ERR WPROBE mode=%s unsupported", mode_text);
            return;
        }
        if (!is_supported_wheel((uint8_t)id)) {
            status_set("ERR WPROBE id=%d unsupported", id);
            return;
        }
        if (b == 0) {
            b = 120;
        }
        if (c == 0) {
            c = 10;
        }
        if (d == 0) {
            d = 80;
        }
        status_wheel_probe(mode, (uint8_t)id, a, b, c, d);
        return;
    }

    if (strcmp(op, "WSTOP") == 0 && sscanf(cmd, "%23s %d", op, &id) == 2) {
        esp_err_t err = wheel_stop((uint8_t)id);
        if (err == ESP_OK) {
            status_wheel_snapshot();
        } else {
            status_set("ERR WSTOP id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "WHOLD") == 0 && sscanf(cmd, "%23s %d", op, &id) == 2) {
        esp_err_t err = wheel_hold_current((uint8_t)id);
        if (err == ESP_OK) {
            status_wheel_snapshot();
        } else {
            status_set("ERR WHOLD id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    if (strcmp(op, "WRELAX") == 0 && sscanf(cmd, "%23s %d", op, &id) == 2) {
        esp_err_t err = wheel_relax((uint8_t)id);
        if (err == ESP_OK) {
            status_wheel_snapshot();
        } else {
            status_set("ERR WRELAX id=%d %s", id, esp_err_to_name(err));
        }
        return;
    }

    status_set("ERR unknown command: %s", cmd);
}

static void ble_command_task(void *arg)
{
    (void)arg;

    ble_command_item_t item;
    while (1) {
        if (xQueueReceive(s_command_queue, &item, portMAX_DELAY) == pdTRUE) {
            execute_command(item.text);
        }
    }
}

static void ble_balance_stream_task(void *arg)
{
    (void)arg;

    while (1) {
        bool enabled = false;
        uint32_t interval_ms = 100;

        balance_stream_get(&enabled, &interval_ms);
        if (enabled) {
            status_balance_telemetry(true);
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static int ble_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)arg;
    int rc;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (attr_handle == s_status_handle) {
            return os_mbuf_append(ctxt->om, s_status_text, strlen(s_status_text)) == 0
                ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        return os_mbuf_append(ctxt->om, s_last_command, strlen(s_last_command)) == 0
            ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        ble_command_item_t item = {0};
        rc = gatt_write_text(ctxt->om, item.text, sizeof(item.text));
        if (rc != 0) {
            return rc;
        }
        if (s_command_queue == NULL) {
            status_set("ERR command queue unavailable");
            return BLE_ATT_ERR_UNLIKELY;
        }
        if (xQueueSend(s_command_queue, &item, 0) != pdTRUE) {
            status_set("ERR command queue full");
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_command_uuid.u,
                .access_cb = ble_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &s_status_uuid.u,
                .access_cb = ble_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_status_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        status_set(event->connect.status == 0 ? "OK connected" : "ERR connect failed");
        if (event->connect.status != 0) {
            ble_control_advertise();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        status_set("OK disconnected");
        ble_control_advertise();
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_control_advertise();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe attr=%u notify=%d",
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        return 0;
    default:
        return 0;
    }
}

static bool ble_control_advertise(void)
{
    struct ble_hs_adv_fields adv_fields;
    struct ble_hs_adv_fields rsp_fields;
    struct ble_gap_adv_params adv_params;
    int rc;

    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.name = (uint8_t *)BLE_DEVICE_NAME;
    adv_fields.name_len = strlen(BLE_DEVICE_NAME);
    adv_fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv set fields failed: %d", rc);
        return false;
    }

    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.uuids128 = (ble_uuid128_t[]) { s_service_uuid };
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv rsp set fields failed: %d", rc);
        return false;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv start failed: %d", rc);
        return false;
    }

    ESP_LOGI(TAG, "advertising started: name=%s", BLE_DEVICE_NAME);
    return true;
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "nimble reset: %d", reason);
}

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure addr failed: %d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer addr failed: %d", rc);
        return;
    }

    if (ble_control_advertise()) {
        status_set("OK advertising");
    } else {
        status_set("ERR advertising");
    }
}

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_control_init(void)
{
    if (s_ble_started) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs init failed");
    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor init failed");
    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "nimble init failed");

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }
    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0) {
        return ESP_FAIL;
    }

    s_command_queue = xQueueCreate(8, sizeof(ble_command_item_t));
    if (s_command_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(ble_command_task, "ble_cmd", 6144, NULL, 5, NULL) != pdPASS) {
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(ble_balance_stream_task, "ble_btel", 4096, NULL, 4, NULL) != pdPASS) {
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    nimble_port_freertos_init(ble_host_task);
    s_ble_started = true;
    status_set("OK ble init");
    return ESP_OK;
}
