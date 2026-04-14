#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// ============================================================
// Dynamixel Protocol 1.0 底层驱动（备选协议层）
//
// 本文件实现了 Dynamixel 协议 1.0 的通信驱动，用于兼容
// Dynamixel MX / AX 系列舵机。
//
// 【与 scs_proto 的关系】
// scs_proto 和 dxl_proto 都实现了相同的 Protocol 1.0 帧格式，
// 只是针对不同品牌的舵机做了各自的适配：
//   - scs_proto: 针对 Feetech SCS/STS/SMS 系列
//   - dxl_proto: 针对 Dynamixel MX/AX 系列
// 两者共用相同的 UART 硬件（UART2, GPIO14/13），不能同时使用。
//
// 当前项目主要使用 scs_proto，dxl_proto 作为备选/参考保留。
//
// 【半双工通信说明】
// 硬件使用 BC807（PNP 三极管）+ SN74（缓冲器）实现自动方向切换：
//   - 发送时：TX 信号驱动总线，SN74 缓冲器放大信号
//   - 接收时：总线信号通过 SN74 进入 RX 引脚
//   - 方向切换由硬件自动完成，软件无需额外控制
// ============================================================

// 帧头标识：每帧都以两个 0xFF 开头
#define DXL_HEADER          0xFF
// 广播地址：0xFE，向总线上所有设备发送，不等待回复
#define DXL_BROADCAST_ID    0xFE

// ============================================================
// Dynamixel 指令码
// ============================================================
#define DXL_INST_PING       0x01    // PING：探测设备在线状态
#define DXL_INST_READ       0x02    // READ：读取控制表寄存器
#define DXL_INST_WRITE      0x03    // WRITE：写入控制表寄存器
#define DXL_INST_REG_WRITE  0x04    // REG_WRITE：预写入（等待 ACTION 触发）
#define DXL_INST_ACTION     0x05    // ACTION：触发 REG_WRITE 的数据执行
#define DXL_INST_SYNC_WRITE 0x83    // SYNC_WRITE：广播同步写入

// ============================================================
// Dynamixel 控制表常用地址（MX / AX 系列共通）
// ============================================================
#define DXL_ADDR_TORQUE_ENABLE  0x18   // 扭矩使能 (1 byte): 0=失能, 1=使能
#define DXL_ADDR_GOAL_POSITION  0x1E   // 目标位置 (2 bytes, Little-Endian)
#define DXL_ADDR_MOVING_SPEED   0x20   // 目标速度 (2 bytes, Little-Endian)
#define DXL_ADDR_PRESENT_POS    0x24   // 当前位置 (2 bytes, 只读)
#define DXL_ADDR_PRESENT_SPEED  0x26   // 当前速度 (2 bytes, 只读)
#define DXL_ADDR_PRESENT_LOAD   0x28   // 当前负载 (2 bytes, 只读)

// ============================================================
// 状态包错误码位掩码
// 当舵机检测到异常时，在状态包的 ERR 字段中置位相应的位
// ============================================================
#define DXL_ERR_VOLTAGE     (1 << 0)   // bit 0: 输入电压超出工作范围
#define DXL_ERR_ANGLE       (1 << 1)   // bit 1: 目标位置超出角度限制
#define DXL_ERR_OVERHEAT    (1 << 2)   // bit 2: 内部温度过高
#define DXL_ERR_RANGE       (1 << 3)   // bit 3: 指令参数超出范围
#define DXL_ERR_CHECKSUM    (1 << 4)   // bit 4: 校验和错误
#define DXL_ERR_OVERLOAD    (1 << 5)   // bit 5: 负载过大（扭矩超过限制）
#define DXL_ERR_INSTRUCTION (1 << 6)   // bit 6: 收到未定义的指令

// 最大收发缓冲大小
#define DXL_MAX_PKT_LEN     64

// ============================================================
// 状态包解析结果
// ============================================================
typedef struct {
    uint8_t  id;                            // 回复的设备 ID
    uint8_t  error;                         // 错误码（参考上面的 DXL_ERR_xxx 位掩码）
    uint8_t  params[DXL_MAX_PKT_LEN];       // 参数数据
    uint8_t  param_count;                   // 有效参数字节数
} dxl_status_t;

/**
 * @brief 初始化 Dynamixel 总线
 * 配置 UART2 (GPIO14 TX, GPIO13 RX)
 * @param baud_rate  波特率（典型 1000000）
 */
esp_err_t dxl_init(int baud_rate);

/**
 * @brief 发送指令包并等待状态包
 * @param id          目标设备 ID (0xFE=广播时不等回复)
 * @param instruction 指令码
 * @param params      参数数组
 * @param param_len   参数长度
 * @param status      输出：状态包解析结果（广播时可传 NULL）
 * @param timeout_ms  等待回复超时 (ms)
 */
esp_err_t dxl_txrx(uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t param_len,
                    dxl_status_t *status, uint32_t timeout_ms);

// ============ 便捷 API ============

/** PING：探测指定 ID 的舵机是否在线 */
esp_err_t dxl_ping(uint8_t id, dxl_status_t *status);

/** READ：读取舵机控制表 */
esp_err_t dxl_read(uint8_t id, uint8_t addr, uint8_t len, dxl_status_t *status);

/** WRITE：向舵机控制表写入数据 */
esp_err_t dxl_write(uint8_t id, uint8_t addr, const uint8_t *data, size_t data_len);

/** SYNC_WRITE：广播同步写（同时向多个舵机写入数据，不等回复） */
esp_err_t dxl_sync_write(uint8_t addr, uint8_t data_len,
                          const uint8_t *id_data_pairs, size_t pair_count);
