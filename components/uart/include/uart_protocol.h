#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// CM4 ↔ ESP32 串口协议
// 帧格式: 55 00 LEN TYPE ADDR [DATA...] CHECK 00 AA
//   LEN   = 整帧长度（含帧头/帧尾）
//   TYPE  = 0x01(写) / 0x02(读请求及读响应)
//   ADDR  = 命令地址
//   CHECK = 255 - ((LEN + TYPE + ADDR + sum(DATA)) % 256)
// ============================================================

#define PROTO_HEADER_0      0x55
#define PROTO_HEADER_1      0x00
#define PROTO_FOOTER_0      0x00
#define PROTO_FOOTER_1      0xAA

#define PROTO_TYPE_WRITE    0x01
#define PROTO_TYPE_READ     0x02

// 帧各字段偏移
#define PROTO_OFF_H0        0
#define PROTO_OFF_H1        1
#define PROTO_OFF_LEN       2
#define PROTO_OFF_TYPE      3
#define PROTO_OFF_ADDR      4
#define PROTO_OFF_DATA      5

// 最小帧长度：8字节
// 例：55 00 08 01 ADDR CHECK 00 AA
#define PROTO_MIN_FRAME_LEN 8
// 最大帧长度（合理上限）
#define PROTO_MAX_FRAME_LEN 64

// ---------- 读命令地址 ----------
#define ADDR_BATTERY            0x01
#define ADDR_PERFORM            0x03
#define ADDR_UPGRADE            0x05
#define ADDR_SET_ORIGIN         0x06
#define ADDR_FIRMWARE_VERSION   0x07
#define ADDR_READ_MOTOR_ANGLE   0x50
#define ADDR_IMU_BALANCE        0x61
#define ADDR_ROLL               0x62
#define ADDR_PITCH              0x63
#define ADDR_YAW                0x64
#define ADDR_IMU_ROLL_INT16     0x66
#define ADDR_IMU_PITCH_INT16    0x67
#define ADDR_IMU_YAW_INT16     0x68

// ---------- 写命令地址 ----------
#define ADDR_VX                 0x30
#define ADDR_VYAW               0x32
#define ADDR_TRANSLATION_Z      0x35
#define ADDR_ATTITUDE_R         0x36
#define ADDR_PERIODIC_ROT_R     0x39
#define ADDR_ACTION             0x3E
#define ADDR_PERIODIC_TRAN_Z    0x82

// ---------- LED 写命令地址 (3 bytes RGB) ----------
#define ADDR_LED_COLOR_1        0x69
#define ADDR_LED_COLOR_2        0x6A
#define ADDR_LED_COLOR_3        0x6B
#define ADDR_LED_COLOR_4        0x6C

// ---------- 写命令状态（供外部模块读取） ----------
typedef struct {
    uint8_t vx;                 // 0x30: 前后速度
    uint8_t vyaw;               // 0x32: 转向速度
    uint8_t translation_z;      // 0x35: 高度
    uint8_t attitude_r;         // 0x36: 横滚
    uint8_t periodic_rot_r;     // 0x39: 周期横滚
    uint8_t periodic_tran_z;    // 0x82: 周期升降
    uint8_t action;             // 0x3E: 当前动作号
    uint8_t imu_balance;        // 0x61: 自平衡开关
    uint8_t perform;            // 0x03: 表演模式开关
    uint8_t led_rgb[4][3];      // 0x69~0x6C: 4组LED颜色
} proto_write_state_t;

// 获取写状态快照（线程安全）
void proto_get_write_state(proto_write_state_t *out);

// ---------- 帧解析器 ----------

// 状态机式帧解析器上下文
typedef struct {
    uint8_t buf[PROTO_MAX_FRAME_LEN];
    size_t  pos;
    bool    synced;  // 已匹配到 55 00
    int     bus_id;  // 绑定的 UART 总线ID，哪个口收的就从哪个口回
} proto_parser_t;

// 初始化解析器，bus_id 对应 uart_bus_id_t
void proto_parser_init(proto_parser_t *p, int bus_id);

// 逐字节喂入数据；每次解析到完整帧时通过内部回调处理
void proto_parser_feed(proto_parser_t *p, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
