#include "ble_control.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "motor_control.h"
#include "scs_proto.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_control";

#define BLE_DEVICE_NAME "RiderDriver"
#define BLE_MAX_TEXT_LEN 192

static uint8_t s_own_addr_type;
static bool s_ble_started = false;
static uint16_t s_status_handle;
static char s_status_text[BLE_MAX_TEXT_LEN] = "ready";
static char s_last_command[BLE_MAX_TEXT_LEN] = "";

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

static bool clamp_goal(int *goal)
{
    bool changed = false;
    if (*goal < 80) {
        *goal = 80;
        changed = true;
    }
    if (*goal > 940) {
        *goal = 940;
        changed = true;
    }
    return changed;
}

static bool read_motor_snapshot(uint8_t id, motor_feedback_t *fb)
{
    return motor_read_feedback(id, fb) == ESP_OK;
}

static void fill_one_motor_snapshot(uint8_t id, char *out, size_t out_len)
{
    motor_feedback_t fb = {0};
    if (!read_motor_snapshot(id, &fb)) {
        snprintf(out, out_len, "{\"id\":%u,\"ok\":0}", id);
        return;
    }

    uint8_t torque = 0xFF;
    uint8_t mode = 0xFF;
    uint16_t goal = 0xFFFF;
    uint16_t goal_speed = 0xFFFF;
    (void)scs_read_byte(id, SCS_ADDR_TORQUE_ENABLE, &torque);
    (void)scs_read_byte(id, SCS_ADDR_MODE, &mode);
    (void)scs_read_word(id, SCS_ADDR_GOAL_POSITION_L, &goal);
    (void)scs_read_word(id, SCS_ADDR_GOAL_SPEED_L, &goal_speed);

    snprintf(out, out_len,
             "{\"id\":%u,\"ok\":1,\"pos\":%u,\"spd\":%u,\"load\":%u,\"v\":%u,\"t\":%u,\"moving\":%u,"
             "\"torque\":%u,\"mode\":%u,\"goal\":%u,\"goal_speed\":%u}",
             id, fb.position, fb.speed, fb.load, fb.voltage, fb.temperature, fb.moving,
             torque, mode, goal, goal_speed);
}

static void fill_status_snapshot(char *out, size_t out_len)
{
    motor_feedback_t fb1 = {0};
    motor_feedback_t fb2 = {0};
    esp_err_t err1 = motor_read_feedback(11, &fb1);
    esp_err_t err2 = motor_read_feedback(12, &fb2);

    snprintf(out, out_len,
             "{\"m1\":{\"pos\":%u,\"spd\":%u,\"load\":%u,\"v\":%u,\"t\":%u},"
             "\"m2\":{\"pos\":%u,\"spd\":%u,\"load\":%u,\"v\":%u,\"t\":%u},"
             "\"ok1\":%d,\"ok2\":%d}",
             fb1.position, fb1.speed, fb1.load, fb1.voltage, fb1.temperature,
             fb2.position, fb2.speed, fb2.load, fb2.voltage, fb2.temperature,
             err1 == ESP_OK ? 1 : 0, err2 == ESP_OK ? 1 : 0);
}

static void execute_command(const char *cmd)
{
    char op[16] = {0};
    int id = 0;
    int a = 0;
    int b = 0;
    int c = 0;

    strncpy(s_last_command, cmd, sizeof(s_last_command) - 1);
    s_last_command[sizeof(s_last_command) - 1] = '\0';

    if (sscanf(cmd, "%15s", op) != 1) {
        status_set("ERR empty command");
        return;
    }

    if (strcmp(op, "PING") == 0) {
        status_set("OK PONG");
        return;
    }

    if (strcmp(op, "HELP") == 0) {
        status_set("OK commands: HELP PING GET READ TORQUE WHEEL WHEELSTOP STOP");
        return;
    }

    if (strcmp(op, "GET") == 0) {
        if (sscanf(cmd, "%15s %d", op, &id) == 2) {
            fill_one_motor_snapshot((uint8_t)id, s_status_text, sizeof(s_status_text));
            ble_gatts_chr_updated(s_status_handle);
            ESP_LOGI(TAG, "%s", s_status_text);
            return;
        }

        fill_status_snapshot(s_status_text, sizeof(s_status_text));
        ble_gatts_chr_updated(s_status_handle);
        ESP_LOGI(TAG, "%s", s_status_text);
        return;
    }

    if (strcmp(op, "TORQUE") == 0 && sscanf(cmd, "%15s %d %d", op, &id, &a) == 3) {
        esp_err_t err = motor_set_torque((uint8_t)id, a != 0);
        status_set("%s TORQUE id=%d enable=%d", err == ESP_OK ? "OK" : "ERR", id, a);
        return;
    }

    if (strcmp(op, "WHEEL") == 0 && sscanf(cmd, "%15s %d %d", op, &id, &a) == 3) {
        int16_t wheel_speed = (int16_t)a;
        esp_err_t err = motor_set_torque((uint8_t)id, true);
        if (err == ESP_OK) {
            err = scs_write_word((uint8_t)id, SCS_ADDR_GOAL_SPEED_L, (uint16_t)wheel_speed);
        }
        status_set("%s WHEEL id=%d speed=%d", err == ESP_OK ? "OK" : "ERR", id, a);
        return;
    }

    if ((strcmp(op, "WHEELSTOP") == 0 && sscanf(cmd, "%15s %d", op, &id) == 2) ||
        (strcmp(op, "STOP") == 0 && sscanf(cmd, "%15s %d", op, &id) == 2)) {
        esp_err_t err = scs_write_word((uint8_t)id, SCS_ADDR_GOAL_SPEED_L, 0);
        if (err == ESP_OK) {
            err = motor_set_torque((uint8_t)id, false);
        }
        status_set("%s WHEELSTOP id=%d", err == ESP_OK ? "OK" : "ERR", id);
        return;
    }

    if (strcmp(op, "READ") == 0 && sscanf(cmd, "%15s %d", op, &id) == 2) {
        fill_one_motor_snapshot((uint8_t)id, s_status_text, sizeof(s_status_text));
        ble_gatts_chr_updated(s_status_handle);
        ESP_LOGI(TAG, "%s", s_status_text);
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
        ESP_LOGI(TAG, "subscribe attr=%u notify=%d", event->subscribe.attr_handle, event->subscribe.cur_notify);
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
