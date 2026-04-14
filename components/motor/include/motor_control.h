#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// 电机控制中间层（Motor Control Layer）
//
// 本文件是电机驱动三层架构的中间层，职责：
//   - 对上（motor_test / motor_tuner）：提供简洁的业务 API
//   - 对下（scs_proto）：调用底层协议驱动完成实际通信
//
// 使用者只需关注"使能/设置位置/读取反馈"等高层语义，
// 无需了解协议帧格式、寄存器地址等底层细节。
// ============================================================

// 电机数量（本项目中使用两个舵机）
#define MOTOR_COUNT 2

/**
 * @brief 电机反馈数据结构体
 *
 * 包含从舵机读取的所有实时状态信息，用于监控电机运行状态。
 * 各字段的含义和单位取决于舵机型号：
 *   - position: 当前位置（通常 0~1023 对应 0°~300°）
 *   - speed: 当前速度（单位约 0.111rpm）
 *   - load: 当前负载（0~2047，bit10=方向，bit0-9=大小）
 *   - voltage: 当前电压（单位约 0.1V，如 120 表示 12.0V）
 *   - temperature: 当前温度（单位约 1°C）
 *   - moving: 运动标志（0=静止，1=正在运动到目标位置）
 */
typedef struct {
    uint8_t id;             // 舵机 ID
    uint16_t position;      // 当前位置
    uint16_t speed;         // 当前速度
    uint16_t load;          // 当前负载
    uint8_t voltage;        // 当前电压
    uint8_t temperature;    // 当前温度
    uint8_t moving;         // 运动标志（0=静止，1=运动中）
} motor_feedback_t;

/**
 * @brief 初始化电机总线
 * 内部调用 scs_init() 初始化 UART 串行总线
 * @return ESP_OK 成功
 */
esp_err_t motor_init(void);

/**
 * @brief 获取所有电机的 ID 列表
 * @param ids_out  输出：指向内部 ID 数组的指针
 * @return 电机数量
 */
size_t motor_get_ids(const uint8_t **ids_out);

/**
 * @brief 开关电机扭矩
 *
 * 扭矩使能（enable=true）时，电机锁定当前位置，可以接受位置指令运动；
 * 扭矩失能（enable=false）时，电机释放扭矩，可以用手自由转动输出轴。
 *
 * @param id      舵机 ID
 * @param enable  true=使能扭矩（锁定），false=失能扭矩（释放）
 */
esp_err_t motor_set_torque(uint8_t id, bool enable);

/**
 * @brief 设置电机目标位置和运动速度
 *
 * 电机将以此速度从当前位置运动到目标位置。
 * 位置值范围取决于舵机型号（通常 0~1023 对应 0°~300°）。
 *
 * @param id        舵机 ID
 * @param position  目标位置值
 * @param speed     运动速度（0=最大速度）
 */
esp_err_t motor_set_position(uint8_t id, uint16_t position, uint16_t speed);

/**
 * @brief 读取电机完整反馈数据
 *
 * 从舵机读取 6 字节连续数据（位置+速度+负载），
 * 再分别读取电压、温度、运动标志，全部存入 motor_feedback_t。
 *
 * @param id   舵机 ID
 * @param out  输出：反馈数据结构体
 */
esp_err_t motor_read_feedback(uint8_t id, motor_feedback_t *out);

/**
 * @brief 批量读取所有电机的当前位置
 *
 * @param positions  输出数组（容量需 >= MOTOR_COUNT）
 * @param capacity   数组容量
 * @param written    输出：实际写入的数量
 */
esp_err_t motor_read_positions(uint16_t *positions, size_t capacity, size_t *written);

#ifdef __cplusplus
}
#endif
