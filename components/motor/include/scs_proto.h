#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// ============================================================
// Feetech SCS / STS / SMS 系列串行总线舵机 —— 底层通信驱动
// ============================================================
//
// 【整体架构说明】
//
// 本项目的电机驱动分为三层：
//   1. scs_proto (本文件) — 串行总线协议底层驱动
//      负责通过 UART 与舵机进行字节级通信，包括：
//      - 组装/解析协议帧（帧头、校验和）
//      - 管理 UART 半双工收发（TX 回环处理）
//      - 提供便捷的读写 API
//
//   2. motor_control — 电机控制中间层
//      使用 scs_proto 提供的 API，实现面向业务的控制逻辑：
//      - 初始化电机总线
//      - 开关扭矩（使能/失能电机）
//      - 设置目标位置和速度
//      - 读取反馈（位置、速度、负载、电压、温度等）
//
//   3. motor_test / motor_tuner — 应用层
//      调用 motor_control API，实现具体的应用场景：
//      - 上电自检和寄存器扫描诊断
//      - 简单的运动演示
//      - 参数调试工具
//
// 【通信原理 —— 半双工单线总线】
//
// 串行总线舵机（如 Dynamixel、Feetech SCS 系列）使用半双工单线通信：
//   - 只有一根数据线（DATA），TX 和 RX 共享同一根信号线
//   - 主控发送指令时，数据从 TX 引脚发出，经过方向控制电路到达总线
//   - 舵机回复时，主控切换为接收模式，从 RX 引脚读取数据
//   - 发送的字节会"回环"到 RX 端（即主控会收到自己发出的数据），
//     因此需要在接收舵机回复之前，先丢弃这些回环字节
//
// 硬件上使用 BC807（PNP 三极管）+ SN74（缓冲器）实现自动方向切换，
// 软件无需手动控制方向引脚。
//
// 【协议帧格式 —— 与 Dynamixel Protocol 1.0 兼容】
//
//   指令包（主控 → 舵机）:
//     [0xFF] [0xFF] [ID] [LEN] [INST] [PARAM0...] [CHECKSUM]
//      帧头1   帧头2  目标ID 包长度  指令码   参数      校验和
//
//   状态包（舵机 → 主控）:
//     [0xFF] [0xFF] [ID] [LEN] [ERR] [PARAM0...] [CHECKSUM]
//      帧头1   帧头2  源ID  包长度  错误码   参数      校验和
//
//   其中:
//     LEN    = 后续字节数 = 指令码(1) + 参数(N) + 校验和(1)
//     CHECKSUM = ~(ID + LEN + INST/ERR + PARAM0 + ... + PARAMn) & 0xFF
//              即：从 ID 到最后一个参数按字节求和，取反后保留低 8 位
//
// 【舵机控制表（寄存器映射）】
//
//   每个舵机内部有一组寄存器（控制表），通过 READ/WRITE 指令访问：
//     - EPROM 区（掉电保持）：型号、ID、波特率、角度限制等
//     - SRAM 区（掉电丢失）：扭矩使能、目标位置、目标速度、实时反馈等
//
//   控制流程：
//     1. 上电 → 初始化 UART 总线
//     2. PING 探测 → 确认舵机在线
//     3. 写 TORQUE_ENABLE = 1 → 使能电机（扭矩上锁）
//     4. 写 GOAL_POSITION + GOAL_SPEED → 设置目标位置和运动速度
//     5. 读 PRESENT_POSITION 等反馈寄存器 → 获取当前状态
//     6. 写 TORQUE_ENABLE = 0 → 失能电机（扭矩释放，可手动转动）
// ============================================================

// 协议常量
#define SCS_HEADER          0xFF    // 帧头字节（两个连续的 0xFF 标识一帧的开始）
#define SCS_BROADCAST_ID    0xFE    // 广播地址（254），向总线上所有舵机发送指令，不等待回复

// ============================================================
// 指令码（Instruction）
// 与 Dynamixel Protocol 1.0 以及 Feetech INST.h 定义一致
// 主控通过这些指令码告诉舵机要执行什么操作
// ============================================================
#define SCS_INST_PING       0x01    // PING：探测舵机是否在线，舵机会回复状态包
#define SCS_INST_READ       0x02    // READ：读取舵机控制表中指定地址的寄存器值
#define SCS_INST_WRITE      0x03    // WRITE：向舵机控制表中指定地址写入数据
#define SCS_INST_REG_WRITE  0x04    // REG_WRITE：预先写入数据但不立即执行，等待 ACTION 指令触发
#define SCS_INST_ACTION     0x05    // ACTION：触发之前 REG_WRITE 写入的数据执行（用于多舵机同步）
#define SCS_INST_SYNC_READ  0x82    // SYNC_READ：同时读取多个舵机的寄存器（广播发送，逐个回复）
#define SCS_INST_SYNC_WRITE 0x83    // SYNC_WRITE：同时向多个舵机写入数据（广播发送，不等回复）
#define SCS_INST_RECOVERY   0x06    // RECOVERY：恢复舵机出厂设置
#define SCS_INST_RESET      0x0A    // RESET：重置舵机参数

// ============================================================
// 舵机控制表地址映射（寄存器地址）
//
// 本批舵机的帧格式兼容 Feetech / Dynamixel Protocol 1.0，
// 但寄存器布局更接近传统 Dynamixel AX 系列风格。
//
// 控制表分为两个区域：
//   - EPROM 区：掉电保持，存储型号、ID、波特率等出厂/配置参数
//   - SRAM 区：掉电丢失，存储运行时控制命令和实时反馈数据
// ============================================================

// --- EPROM 区（只读或掉电保持的配置参数）---
// 这些地址存储舵机的固有属性和出厂配置
#define SCS_ADDR_MODEL_L            0       // 型号低字节（只读）
#define SCS_ADDR_MODEL_H            1       // 型号高字节（只读）
#define SCS_ADDR_VERSION            2       // 固件版本号（只读）
#define SCS_ADDR_ID                 3       // 舵机 ID 地址（可写，掉电保持，默认出厂设定）
#define SCS_ADDR_BAUD_RATE          4       // 波特率配置（可写，掉电保持，0=9600, 1=19200, ...）
#define SCS_ADDR_RETURN_DELAY       5       // 回复延时（单位 2μs，舵机收到指令后延迟多久才回复）
#define SCS_ADDR_MIN_ANGLE_L        6       // 最小角度限制低字节
#define SCS_ADDR_MIN_ANGLE_H        7       // 最小角度限制高字节
#define SCS_ADDR_MAX_ANGLE_L        8       // 最大角度限制低字节
#define SCS_ADDR_MAX_ANGLE_H        9       // 最大角度限制高字节

// --- 兼容保留地址：本批舵机未使用 STS 扩展区 ---
#define SCS_ADDR_CW_DEAD            26      // 顺时针方向死区宽度
#define SCS_ADDR_CCW_DEAD           27      // 逆时针方向死区宽度
#define SCS_ADDR_OFS_L              31      // 位置偏移低字节（零点校准用）
#define SCS_ADDR_OFS_H              32      // 位置偏移高字节
#define SCS_ADDR_MODE               33      // 工作模式：0=位置伺服模式，1=连续旋转（电机）模式

// --- SRAM 区 - 可写控制寄存器（运行时命令）---
// 写入这些寄存器可以立即控制舵机行为
#define SCS_ADDR_TORQUE_ENABLE      24      // 扭矩使能：0=失能（可自由转动），1=使能（锁定位置）
#define SCS_ADDR_ACC                29      // 加速度配置（legacy profile 不使用，保留兼容）
#define SCS_ADDR_GOAL_POSITION_L    30      // 目标位置低字节（0x1E）
#define SCS_ADDR_GOAL_POSITION_H    31      // 目标位置高字节（0x1F）
#define SCS_ADDR_GOAL_SPEED_L       32      // 目标速度低字节（0x20）
#define SCS_ADDR_GOAL_SPEED_H       33      // 目标速度高字节（0x21）
#define SCS_ADDR_GOAL_TIME_L        34      // 运动时间低字节（部分型号用于指定到达目标的毫秒数）
#define SCS_ADDR_GOAL_TIME_H        35      // 运动时间高字节
#define SCS_ADDR_TORQUE_LIMIT_L     34      // 扭矩限制低字节（与 GOAL_TIME 地址重叠，不同型号不同含义）
#define SCS_ADDR_TORQUE_LIMIT_H     35      // 扭矩限制高字节
#define SCS_ADDR_LOCK               47      // 寄存器锁定：设为 1 后禁止写入 EPROM，防止误修改
#define SCS_ADDR_LOCK_STS           55      // 状态锁定

// --- SRAM 区 - 只读反馈寄存器（实时状态）---
// 读取这些寄存器可以获取舵机的当前状态
#define SCS_ADDR_PRESENT_POSITION_L 36      // 当前位置低字节（0x24）
#define SCS_ADDR_PRESENT_POSITION_H 37      // 当前位置高字节（0x25）
#define SCS_ADDR_PRESENT_SPEED_L    38      // 当前速度低字节（0x26）
#define SCS_ADDR_PRESENT_SPEED_H    39      // 当前速度高字节（0x27）
#define SCS_ADDR_PRESENT_LOAD_L     40      // 当前负载低字节（0x28）
#define SCS_ADDR_PRESENT_LOAD_H     41      // 当前负载高字节（0x29）
#define SCS_ADDR_PRESENT_VOLTAGE    42      // 当前供电电压（0x2A），单位约 0.1V
#define SCS_ADDR_PRESENT_TEMPERATURE 43     // 当前温度（0x2B），单位约 1°C
#define SCS_ADDR_MOVING             46      // 运动标志（0x2E）：0=静止，1=正在运动
#define SCS_ADDR_PRESENT_CURRENT_L  44      // 当前电流低字节
#define SCS_ADDR_PRESENT_CURRENT_H  45      // 当前电流高字节

// 默认通信波特率（1Mbps，本项目中舵机出厂配置的波特率）
#define SCS_DEFAULT_BAUD_RATE       1000000

// 最大收发缓冲大小（单帧最大字节数，用于定义缓冲区）
#define SCS_MAX_PKT_LEN     128

// ============================================================
// 状态包解析结果结构体
// 当舵机回复状态包时，解析后的数据存入此结构体
// ============================================================
typedef struct {
    uint8_t  id;                                    // 回复的舵机 ID
    uint8_t  error;                                 // 错误码（0=无错误，各位含义见下方说明）
    uint8_t  params[SCS_MAX_PKT_LEN];               // 参数数据（实际寄存器值）
    uint8_t  param_count;                           // 参数有效字节数
} scs_status_t;

// 错误码各位含义（error 字段的位掩码）:
//   bit 0 (0x01) = 输入电压异常
//   bit 1 (0x02) = 角度超出限制
//   bit 2 (0x04) = 过热
//   bit 3 (0x08) = 指令范围错误
//   bit 4 (0x10) = 校验和错误
//   bit 5 (0x20) = 过载（负载过大）
//   bit 6 (0x40) = 非法指令
//   bit 7        = 保留

// ============================================================
// API 函数声明
// ============================================================

/**
 * @brief 初始化 SCS/STS 串行总线
 *
 * 配置 UART2（TX=GPIO14, RX=GPIO13）为半双工通信模式。
 * 首次调用会安装 UART 驱动并配置引脚，后续调用直接返回成功。
 *
 * @param baud_rate  波特率（典型值 1000000，需与舵机出厂配置一致）
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t scs_init(int baud_rate);

/**
 * @brief 串行总线收发核心函数 —— 发送指令包并等待状态包回复
 *
 * 这是所有通信的基础函数，工作流程：
 *   1. 组装协议帧：FF FF ID LEN INST [PARAMS...] CHECKSUM
 *   2. 清空 UART 接收缓冲区
 *   3. 发送指令帧
 *   4. 等待发送完成后，延时并丢弃 TX 回环字节
 *   5. 接收舵机回复的状态包（如果不是广播地址）
 *   6. 验证校验和，解析状态包内容
 *
 * @param id          目标舵机 ID（0xFE=广播，广播时不等待回复）
 * @param instruction 指令码（PING/READ/WRITE 等）
 * @param params      参数数组（如 READ 需要传 [地址, 长度]）
 * @param param_len   参数数组长度
 * @param status      输出：解析后的状态包（广播时可传 NULL）
 * @param timeout_ms  等待回复超时时间（毫秒）
 * @return ESP_OK 成功收到并校验通过的状态包
 *         ESP_ERR_TIMEOUT 接收超时
 *         ESP_ERR_INVALID_CRC 校验和错误
 */
esp_err_t scs_txrx(uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t param_len,
                    scs_status_t *status, uint32_t timeout_ms);

// ============ 便捷 API（基于 scs_txrx 封装）============

/** PING：探测指定 ID 的舵机是否在线，在线则返回状态包 */
esp_err_t scs_ping(uint8_t id, scs_status_t *status);

/** READ：从舵机控制表的指定地址读取指定长度的数据 */
esp_err_t scs_read(uint8_t id, uint8_t addr, uint8_t len, scs_status_t *status);

/** WRITE：向舵机控制表的指定地址写入任意长度的数据 */
esp_err_t scs_write(uint8_t id, uint8_t addr, const uint8_t *data, size_t data_len);

/** WRITE_BYTE：向舵机控制表写入 1 个字节（常用于写扭矩使能等单字节寄存器） */
esp_err_t scs_write_byte(uint8_t id, uint8_t addr, uint8_t val);

/** WRITE_WORD：向舵机控制表写入 2 个字节（低字节在前，Little-Endian 格式）
 *  常用于写目标位置（16 位）、目标速度（16 位）等双字节寄存器 */
esp_err_t scs_write_word(uint8_t id, uint8_t addr, uint16_t val);

/** READ_BYTE：从舵机控制表读取 1 个字节 */
esp_err_t scs_read_byte(uint8_t id, uint8_t addr, uint8_t *out);

/** READ_WORD：从舵机控制表读取 2 个字节（Little-Endian），组合为 uint16_t */
esp_err_t scs_read_word(uint8_t id, uint8_t addr, uint16_t *out);

/** SYNC_WRITE：广播同步写
 *  在一帧内向多个舵机写入相同地址的寄存器数据
 *  参数 id_data_pairs 格式: [ID1, D1, D2, ..., ID2, D1, D2, ...] */
esp_err_t scs_sync_write(uint8_t addr, uint8_t data_len,
                          const uint8_t *id_data_pairs, size_t pair_count);

// ============ 高级 API（STS/SMS WritePos 风格）============

/**
 * @brief 设置舵机目标位置和速度（WritePos 风格）
 *
 * 依次写入两个寄存器：
 *   1. GOAL_POSITION（地址 0x1E，2 字节）—— 目标位置
 *   2. GOAL_SPEED（地址 0x20，2 字节）—— 运动速度
 *
 * @param id        舵机 ID
 * @param position  目标位置（0~1023 对应 0°~300°）
 * @param speed     运动速度（0=最大速度，单位约 0.111rpm）
 * @param acc       加速度（本实现中未使用，保留参数）
 */
esp_err_t scs_write_pos(uint8_t id, int16_t position, uint16_t speed, uint8_t acc);
