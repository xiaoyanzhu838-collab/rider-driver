#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// ============================================================
// Feetech SCS / STS / SMS 系列串行舵机低层驱动
// 协议帧与 Dynamixel Protocol 1.0 相同:
//   FF FF ID LEN INST PARAM... CHECKSUM
// 校验: ~(ID + LEN + INST + PARAMS...)  & 0xFF
// 半双工单线总线，硬件自动切换方向
// ============================================================

#define SCS_HEADER          0xFF
#define SCS_BROADCAST_ID    0xFE

// 指令码 (与 Dynamixel Protocol 1.0 / Feetech INST.h 一致)
#define SCS_INST_PING       0x01
#define SCS_INST_READ       0x02
#define SCS_INST_WRITE      0x03
#define SCS_INST_REG_WRITE  0x04
#define SCS_INST_ACTION     0x05
#define SCS_INST_SYNC_READ  0x82
#define SCS_INST_SYNC_WRITE 0x83
#define SCS_INST_RECOVERY   0x06
#define SCS_INST_RESET      0x0A

// ============================================================
// 当前实机确认的控制表地址
// 这批舵机的帧格式兼容 Feetech / Dynamixel Protocol 1.0，
// 但寄存器布局更接近 legacy Dynamixel AX 风格。
// ============================================================

// --- EPROM (只读/读写) ---
#define SCS_ADDR_MODEL_L            0
#define SCS_ADDR_MODEL_H            1
#define SCS_ADDR_VERSION            2
#define SCS_ADDR_ID                 3
#define SCS_ADDR_BAUD_RATE          4
#define SCS_ADDR_RETURN_DELAY       5
#define SCS_ADDR_MIN_ANGLE_L        6
#define SCS_ADDR_MIN_ANGLE_H        7
#define SCS_ADDR_MAX_ANGLE_L        8
#define SCS_ADDR_MAX_ANGLE_H        9

// --- 兼容保留：本批舵机未使用 STS 扩展区 ---
#define SCS_ADDR_CW_DEAD            26
#define SCS_ADDR_CCW_DEAD           27
#define SCS_ADDR_OFS_L              31
#define SCS_ADDR_OFS_H              32
#define SCS_ADDR_MODE               33

// --- SRAM (读写) ---
#define SCS_ADDR_TORQUE_ENABLE      24   // 0x18
#define SCS_ADDR_ACC                29   // legacy profile 不使用，保留兼容
#define SCS_ADDR_GOAL_POSITION_L    30   // 0x1E
#define SCS_ADDR_GOAL_POSITION_H    31   // 0x1F
#define SCS_ADDR_GOAL_SPEED_L       32   // 0x20
#define SCS_ADDR_GOAL_SPEED_H       33   // 0x21
#define SCS_ADDR_GOAL_TIME_L        34
#define SCS_ADDR_GOAL_TIME_H        35
#define SCS_ADDR_TORQUE_LIMIT_L     34
#define SCS_ADDR_TORQUE_LIMIT_H     35
#define SCS_ADDR_LOCK               47
#define SCS_ADDR_LOCK_STS           55

// --- SRAM (只读) ---
#define SCS_ADDR_PRESENT_POSITION_L 36   // 0x24
#define SCS_ADDR_PRESENT_POSITION_H 37   // 0x25
#define SCS_ADDR_PRESENT_SPEED_L    38   // 0x26
#define SCS_ADDR_PRESENT_SPEED_H    39   // 0x27
#define SCS_ADDR_PRESENT_LOAD_L     40   // 0x28
#define SCS_ADDR_PRESENT_LOAD_H     41   // 0x29
#define SCS_ADDR_PRESENT_VOLTAGE    42   // 0x2A
#define SCS_ADDR_PRESENT_TEMPERATURE 43  // 0x2B
#define SCS_ADDR_MOVING             46   // 0x2E
#define SCS_ADDR_PRESENT_CURRENT_L  44
#define SCS_ADDR_PRESENT_CURRENT_H  45

#define SCS_DEFAULT_BAUD_RATE       1000000

// 最大收发缓冲
#define SCS_MAX_PKT_LEN     128

// ============================================================
// 状态包解析结果
// ============================================================
typedef struct {
    uint8_t  id;
    uint8_t  error;
    uint8_t  params[SCS_MAX_PKT_LEN];
    uint8_t  param_count;
} scs_status_t;

// ============================================================
// API
// ============================================================

/**
 * @brief 初始化 SCS/STS 总线 (UART2, GPIO14 TX, GPIO13 RX)
 * @param baud_rate  波特率 (典型 1000000)
 */
esp_err_t scs_init(int baud_rate);

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
esp_err_t scs_txrx(uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t param_len,
                    scs_status_t *status, uint32_t timeout_ms);

// ============ 便捷 API ============

/** PING 某个 ID，返回是否在线 */
esp_err_t scs_ping(uint8_t id, scs_status_t *status);

/** 读取控制表 */
esp_err_t scs_read(uint8_t id, uint8_t addr, uint8_t len, scs_status_t *status);

/** 写入控制表 (任意长度) */
esp_err_t scs_write(uint8_t id, uint8_t addr, const uint8_t *data, size_t data_len);

/** 写 1 字节 */
esp_err_t scs_write_byte(uint8_t id, uint8_t addr, uint8_t val);

/** 写 2 字节 (little-endian on wire: LO, HI) */
esp_err_t scs_write_word(uint8_t id, uint8_t addr, uint16_t val);

/** 读 1 字节, 结果写入 *out */
esp_err_t scs_read_byte(uint8_t id, uint8_t addr, uint8_t *out);

/** 读 2 字节 (LE), 结果写入 *out */
esp_err_t scs_read_word(uint8_t id, uint8_t addr, uint16_t *out);

/** 同步写 (广播，不等回包) */
esp_err_t scs_sync_write(uint8_t addr, uint8_t data_len,
                          const uint8_t *id_data_pairs, size_t pair_count);

// ============ 高级 API (STS/SMS WritePos 风格) ============

/**
 * @brief 写位置+速度+加速度 (STS/SMS WritePosEx 风格)
 *   写入 addr=0x29 开始连续 7 字节:
 *     ACC(1) + GoalPos_L(1) + GoalPos_H(1) + GoalTime_L(1) + GoalTime_H(1)
 *           + GoalSpeed_L(1) + GoalSpeed_H(1)
 */
esp_err_t scs_write_pos(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
