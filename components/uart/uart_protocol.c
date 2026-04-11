#include "uart_protocol.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "ahrs.h"
#include "battery.h"
#include "imu.h"
#include "rgb_control.h"
#include "rgb_effects.h"
#include "rgb_service.h"
#include "uart_bus.h"
#include "ws2812.h"

static const char *TAG = "proto";

// ============================================================
// 写命令状态（线程安全读写）
// ============================================================
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;
static proto_write_state_t s_state;

void proto_get_write_state(proto_write_state_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_state_mux);
    *out = s_state;
    portEXIT_CRITICAL(&s_state_mux);
}

static void state_set_u8(uint8_t *field, uint8_t val)
{
    portENTER_CRITICAL(&s_state_mux);
    *field = val;
    portEXIT_CRITICAL(&s_state_mux);
}

static void state_set_led(int idx, const uint8_t *rgb)
{
    if (idx < 0 || idx >= 4) return;
    portENTER_CRITICAL(&s_state_mux);
    s_state.led_rgb[idx][0] = rgb[0];
    s_state.led_rgb[idx][1] = rgb[1];
    s_state.led_rgb[idx][2] = rgb[2];
    portEXIT_CRITICAL(&s_state_mux);
}

// ============================================================
// 校验和
// ============================================================
static uint8_t calc_checksum(const uint8_t *frame, size_t frame_len)
{
    // CHECK = 255 - ((LEN + TYPE + ADDR + sum(DATA)) % 256)
    uint16_t sum = 0;
    // LEN at [2], TYPE at [3], ADDR at [4], DATA from [5] to [frame_len-3]
    for (size_t i = PROTO_OFF_LEN; i < frame_len - 3; i++) {
        sum += frame[i];
    }
    return (uint8_t)(255 - (sum % 256));
}

static bool verify_checksum(const uint8_t *frame, size_t frame_len)
{
    uint8_t expected = calc_checksum(frame, frame_len);
    uint8_t got = frame[frame_len - 3];
    return expected == got;
}

// ============================================================
// 构建并发送响应帧
// ============================================================
static void send_read_response(int bus_id, uint8_t addr, const uint8_t *data, size_t data_len)
{
    // 响应帧: 55 00 LEN 02 ADDR DATA... CHECK 00 AA
    size_t frame_len = 8 + data_len;  // header(2) + LEN(1) + TYPE(1) + ADDR(1) + DATA(n) + CHECK(1) + footer(2)
    if (frame_len > PROTO_MAX_FRAME_LEN) {
        ESP_LOGW(TAG, "response too long: %u", (unsigned)frame_len);
        return;
    }

    uint8_t frame[PROTO_MAX_FRAME_LEN];
    frame[0] = PROTO_HEADER_0;
    frame[1] = PROTO_HEADER_1;
    frame[2] = (uint8_t)(2 + data_len);  // LEN = TYPE(1) + ADDR(1) + DATA(n)
    frame[3] = PROTO_TYPE_READ;
    frame[4] = addr;
    if (data_len > 0 && data) {
        memcpy(&frame[5], data, data_len);
    }
    frame[frame_len - 3] = calc_checksum(frame, frame_len);
    frame[frame_len - 2] = PROTO_FOOTER_0;
    frame[frame_len - 1] = PROTO_FOOTER_1;

    uart_bus_write((uart_bus_id_t)bus_id, frame, frame_len);
}

// ============================================================
// 读命令处理
// ============================================================
static void handle_read(int bus_id, uint8_t addr, uint8_t read_len)
{
    uint8_t resp[32];
    size_t resp_len = 0;

    switch (addr) {
    case ADDR_FIRMWARE_VERSION: {
        // 10字节 ASCII，首字符 'R' 表示 RIDER
        const char *fw = "RIDER-ESP\0";
        resp_len = 10;
        memcpy(resp, fw, resp_len);
        break;
    }

    case ADDR_BATTERY: {
        // 真实 ADC 电池电压采样
        resp[0] = battery_read_percent();
        resp_len = 1;
        break;
    }

    case ADDR_YAW: {
        // 4字节 little-endian float
        ahrs_euler_t euler;
        ahrs_get_euler(&euler);
        float yaw = euler.yaw;
        memcpy(resp, &yaw, sizeof(float));
        resp_len = 4;
        break;
    }

    case ADDR_ROLL: {
        ahrs_euler_t euler;
        ahrs_get_euler(&euler);
        float roll = euler.roll;
        memcpy(resp, &roll, sizeof(float));
        resp_len = 4;
        break;
    }

    case ADDR_PITCH: {
        ahrs_euler_t euler;
        ahrs_get_euler(&euler);
        float pitch = euler.pitch;
        memcpy(resp, &pitch, sizeof(float));
        resp_len = 4;
        break;
    }

    case ADDR_IMU_ROLL_INT16:
    case ADDR_IMU_PITCH_INT16:
    case ADDR_IMU_YAW_INT16: {
        // 2字节 big-endian int16
        ahrs_euler_t euler;
        ahrs_get_euler(&euler);
        int16_t val = 0;
        if (addr == ADDR_IMU_ROLL_INT16) {
            val = (int16_t)euler.roll;
        } else if (addr == ADDR_IMU_PITCH_INT16) {
            val = (int16_t)euler.pitch;
        } else {
            val = (int16_t)euler.yaw;
        }
        // big-endian
        resp[0] = (uint8_t)((val >> 8) & 0xFF);
        resp[1] = (uint8_t)(val & 0xFF);
        resp_len = 2;
        break;
    }

    case ADDR_READ_MOTOR_ANGLE: {
        // 15字节电机角度（暂时返回全0）
        memset(resp, 0, 15);
        resp_len = 15;
        break;
    }

    case ADDR_IMU_BALANCE: {
        portENTER_CRITICAL(&s_state_mux);
        resp[0] = s_state.imu_balance;
        portEXIT_CRITICAL(&s_state_mux);
        resp_len = 1;
        break;
    }

    case ADDR_PERFORM: {
        portENTER_CRITICAL(&s_state_mux);
        resp[0] = s_state.perform;
        portEXIT_CRITICAL(&s_state_mux);
        resp_len = 1;
        break;
    }

    default:
        ESP_LOGW(TAG, "unknown read addr=0x%02X len=%d", addr, read_len);
        return;
    }

    send_read_response(bus_id, addr, resp, resp_len);
}

// ============================================================
// 写命令处理
// ============================================================
static void apply_led_color(int led_idx, const uint8_t *rgb)
{
    state_set_led(led_idx, rgb);

    // CM4 通过协议控制单颗LED时，切到直接硬件控制模式
    // 先确保 RGB 效果引擎不会覆盖我们的设置
    rgb_control_set_enabled(false);

    // 直接设置对应灯珠颜色
    ws2812_set_pixel(led_idx, rgb[0], rgb[1], rgb[2]);
    ws2812_refresh();

    ESP_LOGI(TAG, "LED[%d] R=%u G=%u B=%u", led_idx, rgb[0], rgb[1], rgb[2]);
}

static void handle_write(int bus_id, uint8_t addr, const uint8_t *data, size_t data_len)
{
    switch (addr) {
    case ADDR_VX:
        if (data_len >= 1) {
            state_set_u8(&s_state.vx, data[0]);
            ESP_LOGI(TAG, "VX=%u", data[0]);
        }
        break;

    case ADDR_VYAW:
        if (data_len >= 1) {
            state_set_u8(&s_state.vyaw, data[0]);
            ESP_LOGI(TAG, "VYAW=%u", data[0]);
        }
        break;

    case ADDR_TRANSLATION_Z:
        if (data_len >= 1) {
            state_set_u8(&s_state.translation_z, data[0]);
            ESP_LOGI(TAG, "TRANS_Z=%u", data[0]);
        }
        break;

    case ADDR_ATTITUDE_R:
        if (data_len >= 1) {
            state_set_u8(&s_state.attitude_r, data[0]);
            ESP_LOGI(TAG, "ATT_R=%u", data[0]);
        }
        break;

    case ADDR_PERIODIC_ROT_R:
        if (data_len >= 1) {
            state_set_u8(&s_state.periodic_rot_r, data[0]);
            ESP_LOGI(TAG, "PROT_R=%u", data[0]);
        }
        break;

    case ADDR_PERIODIC_TRAN_Z:
        if (data_len >= 1) {
            state_set_u8(&s_state.periodic_tran_z, data[0]);
            ESP_LOGI(TAG, "PTRAN_Z=%u", data[0]);
        }
        break;

    case ADDR_ACTION:
        if (data_len >= 1) {
            state_set_u8(&s_state.action, data[0]);
            if (data[0] == 255) {
                ESP_LOGI(TAG, "ACTION=RESET");
                // TODO: 执行机器人复位
            } else {
                ESP_LOGI(TAG, "ACTION=%u", data[0]);
                // TODO: 执行指定动作
            }
        }
        break;

    case ADDR_IMU_BALANCE:
        if (data_len >= 1) {
            state_set_u8(&s_state.imu_balance, data[0]);
            ESP_LOGI(TAG, "IMU_BALANCE=%u", data[0]);
        }
        break;

    case ADDR_PERFORM:
        if (data_len >= 1) {
            state_set_u8(&s_state.perform, data[0]);
            ESP_LOGI(TAG, "PERFORM=%u", data[0]);
        }
        break;

    case ADDR_LED_COLOR_1:
        if (data_len >= 3) apply_led_color(0, data);
        break;
    case ADDR_LED_COLOR_2:
        if (data_len >= 3) apply_led_color(1, data);
        break;
    case ADDR_LED_COLOR_3:
        if (data_len >= 3) apply_led_color(2, data);
        break;
    case ADDR_LED_COLOR_4:
        if (data_len >= 3) apply_led_color(3, data);
        break;

    case ADDR_SET_ORIGIN:
        // rider_reset_odom(): 重置里程/原点
        if (data_len >= 1 && data[0] == 0x01) {
            ahrs_reset();
            ESP_LOGI(TAG, "SET_ORIGIN: reset AHRS + odom");
        }
        break;

    case ADDR_UPGRADE:
        // rider_upgrade(): 升级握手入口
        // CM4 发送 0x05 data=0x01，期望回包首字节 0x55
        // 然后 CM4 切换波特率到 350000 发 bin 流
        if (data_len >= 1 && data[0] == 0x01) {
            ESP_LOGI(TAG, "UPGRADE: handshake requested");
            // 回应握手 ACK: 首字节 0x55
            uint8_t ack = 0x55;
            send_read_response(bus_id, ADDR_UPGRADE, &ack, 1);
            // TODO: 进入升级模式，切换波特率到 350000，接收 bin 流
        }
        break;

    default:
        ESP_LOGW(TAG, "unknown write addr=0x%02X len=%u", addr, (unsigned)data_len);
        break;
    }
}

// ============================================================
// 帧处理分发
// ============================================================
static void process_frame(int bus_id, const uint8_t *frame, size_t frame_len)
{
    if (!verify_checksum(frame, frame_len)) {
        ESP_LOGW(TAG, "[bus%d] checksum mismatch", bus_id);
        return;
    }

    uint8_t type = frame[PROTO_OFF_TYPE];
    uint8_t addr = frame[PROTO_OFF_ADDR];

    if (type == PROTO_TYPE_READ) {
        uint8_t payload_len = frame[PROTO_OFF_LEN];
        uint8_t read_len = 0;

        // 兼容两种读请求：
        // 1) 最小读请求: 55 00 02 02 ADDR CHECK 00 AA
        // 2) 显式长度:   55 00 03 02 ADDR READ_LEN CHECK 00 AA
        if (payload_len >= 3 && frame_len >= 9) {
            read_len = frame[PROTO_OFF_DATA];
        }

        ESP_LOGI(TAG, "[bus%d] READ addr=0x%02X len=%u", bus_id, addr, read_len);
        handle_read(bus_id, addr, read_len);
    } else if (type == PROTO_TYPE_WRITE) {
        // 写命令帧: 55 00 LEN 01 ADDR DATA... CHECK 00 AA
        size_t data_len = frame_len - 8;  // 减去 header(2)+LEN(1)+TYPE(1)+ADDR(1)+CHECK(1)+footer(2)
        const uint8_t *data = &frame[PROTO_OFF_DATA];
        ESP_LOGI(TAG, "[bus%d] WRITE addr=0x%02X data_len=%u", bus_id, addr, (unsigned)data_len);
        handle_write(bus_id, addr, data, data_len);
    } else {
        ESP_LOGW(TAG, "[bus%d] unknown type=0x%02X", bus_id, type);
    }
}

// ============================================================
// 状态机帧解析器
// ============================================================
void proto_parser_init(proto_parser_t *p, int bus_id)
{
    memset(p, 0, sizeof(*p));
    p->bus_id = bus_id;
}

void proto_parser_feed(proto_parser_t *p, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];

        if (!p->synced) {
            // 寻找帧头 55 00
            if (p->pos == 0 && b == PROTO_HEADER_0) {
                p->buf[0] = b;
                p->pos = 1;
            } else if (p->pos == 1 && b == PROTO_HEADER_1) {
                p->buf[1] = b;
                p->pos = 2;
                p->synced = true;
            } else {
                p->pos = 0;
            }
            continue;
        }

        // 已同步，继续收字节
        if (p->pos < PROTO_MAX_FRAME_LEN) {
            p->buf[p->pos++] = b;
        } else {
            // 溢出，丢弃并重新同步
            ESP_LOGW(TAG, "frame overflow, resync");
            p->pos = 0;
            p->synced = false;
            continue;
        }

        // 收到 LEN 字段后，检测是否已收齐整帧
        if (p->pos >= 3) {
            // LEN 是 payload 长度 (TYPE+ADDR+DATA)，总帧长 = header(2)+LEN(1)+payload(LEN)+CHECK(1)+footer(2) = LEN+6
            size_t payload_len = p->buf[PROTO_OFF_LEN];
            size_t expected_len = payload_len + 6;
            if (expected_len < PROTO_MIN_FRAME_LEN || expected_len > PROTO_MAX_FRAME_LEN) {
                // 不合法的长度，丢弃重新同步
                ESP_LOGW(TAG, "invalid frame len=%u, resync", (unsigned)expected_len);
                p->pos = 0;
                p->synced = false;
                continue;
            }

            if (p->pos >= expected_len) {
                // 校验帧尾
                if (p->buf[expected_len - 2] == PROTO_FOOTER_0 &&
                    p->buf[expected_len - 1] == PROTO_FOOTER_1) {
                    process_frame(p->bus_id, p->buf, expected_len);
                } else {
                    ESP_LOGW(TAG, "[bus%d] bad footer, discard", p->bus_id);
                }
                p->pos = 0;
                p->synced = false;
            }
        }
    }
}
