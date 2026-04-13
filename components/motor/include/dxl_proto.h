#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// ============================================================
// Dynamixel Protocol 1.0 低层驱动
// 半双工单线总线，硬件自动切换方向 (BC807 + SN74 缓冲)
// ============================================================

#define DXL_HEADER          0xFF
#define DXL_BROADCAST_ID    0xFE

// 指令码
#define DXL_INST_PING       0x01
#define DXL_INST_READ       0x02
#define DXL_INST_WRITE      0x03
#define DXL_INST_REG_WRITE  0x04
#define DXL_INST_ACTION     0x05
#define DXL_INST_SYNC_WRITE 0x83

// 常用控制表地址 (MX / AX 系列共通部分)
#define DXL_ADDR_TORQUE_ENABLE  0x18   // 1 byte
#define DXL_ADDR_GOAL_POSITION  0x1E   // 2 bytes
#define DXL_ADDR_MOVING_SPEED   0x20   // 2 bytes
#define DXL_ADDR_PRESENT_POS    0x24   // 2 bytes
#define DXL_ADDR_PRESENT_SPEED  0x26   // 2 bytes
#define DXL_ADDR_PRESENT_LOAD   0x28   // 2 bytes

// 状态包错误位
#define DXL_ERR_VOLTAGE     (1 << 0)
#define DXL_ERR_ANGLE       (1 << 1)
#define DXL_ERR_OVERHEAT    (1 << 2)
#define DXL_ERR_RANGE       (1 << 3)
#define DXL_ERR_CHECKSUM    (1 << 4)
#define DXL_ERR_OVERLOAD    (1 << 5)
#define DXL_ERR_INSTRUCTION (1 << 6)

// 最大收发缓冲
#define DXL_MAX_PKT_LEN     64

// 状态包解析结果
typedef struct {
    uint8_t  id;
    uint8_t  error;
    uint8_t  params[DXL_MAX_PKT_LEN];
    uint8_t  param_count;
} dxl_status_t;

/**
 * @brief 初始化 Dynamixel 总线 (UART2, GPIO14 TX, GPIO13 RX)
 * @param baud_rate  波特率 (典型 1000000)
 */
esp_err_t dxl_init(int baud_rate);

/**
 * @brief 发送指令包并等待状态包
 * @param id          目标设备 ID (0xFE=广播，广播不等回包)
 * @param instruction 指令码
 * @param params      参数数组
 * @param param_len   参数长度
 * @param status      输出：状态包 (广播时可传 NULL)
 * @param timeout_ms  等待回包超时 (ms)
 * @return ESP_OK 成功收到状态包 / ESP_ERR_TIMEOUT 超时
 */
esp_err_t dxl_txrx(uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t param_len,
                    dxl_status_t *status, uint32_t timeout_ms);

// ============ 便捷 API ============

/** PING 某个 ID，返回是否在线 */
esp_err_t dxl_ping(uint8_t id, dxl_status_t *status);

/** 读取控制表 */
esp_err_t dxl_read(uint8_t id, uint8_t addr, uint8_t len, dxl_status_t *status);

/** 写入控制表 (任意长度) */
esp_err_t dxl_write(uint8_t id, uint8_t addr, const uint8_t *data, size_t data_len);

/** 同步写 (广播，不等回包) */
esp_err_t dxl_sync_write(uint8_t addr, uint8_t data_len,
                          const uint8_t *id_data_pairs, size_t pair_count);
