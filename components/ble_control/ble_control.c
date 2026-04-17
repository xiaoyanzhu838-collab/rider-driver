#include "ble_control.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
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
#define BLE_MAX_TEXT_LEN 192

#define MOTOR_ID_12 12
#define MOTOR_ID_22 22
#define MOTOR_12_SAFE_DELTA 104
#define MOTOR_22_SAFE_DELTA 112
#define DEFAULT_MOVE_SPEED 200

static uint8_t s_own_addr_type;
static bool s_ble_started = false;
static uint16_t s_status_handle;
static char s_status_text[BLE_MAX_TEXT_LEN] = "ready";
static char s_last_command[BLE_MAX_TEXT_LEN] = "";
static int s_home12 = 599;
static int s_home22 = 439;

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

static void status_set(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(s_status_text, sizeof(s_status_text), fmt, args);
    va_end(args);

    ESP_LOGI(TAG, "%s", s_status_text);
    if (s_status_handle != 0) {
        ble_gatts_chr_updated(s_status_handle);
    }
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

static bool is_supported_servo(uint8_t id)
{
    return id == MOTOR_ID_12 || id == MOTOR_ID_22;
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
        "{\"h12\":%d,\"e12\":%d,\"p12\":%u,\"g12\":%u,\"t12\":%u,"
        "\"h22\":%d,\"e22\":%d,\"p22\":%u,\"g22\":%u,\"t22\":%u}",
        s_home12, servo_edge(MOTOR_ID_12), fb12.position, goal12, torque12,
        s_home22, servo_edge(MOTOR_ID_22), fb22.position, goal22, torque22);
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

static void execute_command(const char *cmd)
{
    char op[24] = {0};
    int id = 0;
    int a = 0;
    int b = 0;

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
        status_set("OK cmds: GET PAIR HOME EDGE SET TORQUE RELAX CAPTURE_HOME PING");
        return;
    }

    if (strcmp(op, "GET") == 0 || strcmp(op, "READ") == 0) {
        status_snapshot();
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

    status_set("ERR unknown command: %s", cmd);
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
        char cmd[BLE_MAX_TEXT_LEN];
        rc = gatt_write_text(ctxt->om, cmd, sizeof(cmd));
        if (rc != 0) {
            return rc;
        }
        execute_command(cmd);
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

    nimble_port_freertos_init(ble_host_task);
    s_ble_started = true;
    status_set("OK ble init");
    return ESP_OK;
}
