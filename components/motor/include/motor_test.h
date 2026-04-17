#pragma once

/**
 * @brief 电机总线自检与演示任务
 *
 * 本任务在系统上电后自动运行，用于验证电机系统工作正常。
 *
 * 【运行模式】
 * 通过编译宏控制两种模式：
 *
 * 1. 简单演示模式（默认 MOTOR_RUN_SIMPLE_DEMO=0，手动打开）：
 *    - 快速使能所有电机
 *    - 每个电机做一次小幅度往复运动（正偏移 60 → 回到起点）
 *    - 最后失能所有电机
 *    - 适合日常上电快速验证
 *
 * 2. 完整诊断模式（需手动设置 MOTOR_RUN_FULL_DIAGNOSTIC=1）：
 *    - Step 1: PING 探测所有舵机是否在线
 *    - Step 2: 逐字节扫描寄存器 0~70，生成完整寄存器映射表
 *    - Step 3: 多字节 READ 测试（与单字节扫描结果对比验证）
 *    - Step 4: 扭矩测试 + 小步位置测试（写入后读回验证）
 *    - 最后进入持续监控循环，周期性读取位置
 *    - 适合首次调试或排查通信问题
 *
 * @param arg  FreeRTOS 任务参数（未使用）
 */
void motor_test_task(void *arg);

/**
 * @brief 低风险寄存器/协议探索任务
 *
 * 上电后只执行一次，优先验证：
 * - 已识别但未系统整理的只读寄存器
 * - LED (0x19) 的直接读写
 * - REG_WRITE + ACTION 的延迟执行语义
 *
 * 任务完成后自动退出，不进入持续循环。
 *
 * @param arg FreeRTOS 任务参数（未使用）
 */
void motor_explore_task(void *arg);

/**
 * @brief 四节点只读识别任务
 *
 * 本任务用于对 UART2 上当前在线的 `11 / 12 / 21 / 22` 四个节点做一轮
 * 只读识别：
 * - 先将 `12 / 22` 拉到安全的 20% 展开位，避免刮机身
 * - 再对四个节点执行 PING / 逐字节寄存器扫描 / 多字节读取
 * - 不对 `11 / 21` 下发运动命令
 *
 * @param arg FreeRTOS 任务参数（未使用）
 */
void motor_identify_four_nodes_task(void *arg);

/**
 * @brief 轮毂节点主动探测任务
 *
 * 用于在保证 `12 / 22` 已安全展开的前提下，
 * 对 `11 / 21` 做最小风险的主动探测：
 * - 重新确认协议 profile
 * - 验证 torque on/off
 * - 做极小位置步进与恢复
 *
 * @param arg FreeRTOS 任务参数（未使用）
 */
void motor_probe_wheels_task(void *arg);

/**
 * @brief 轮毂节点驱动语义探测任务
 *
 * 在保证 `12 / 22` 已安全展开的前提下，对 `11 / 21` 继续做低风险探测：
 * - 当前位置保持写入是否真的能“保持”
 * - 单独改 `goal_speed` 是否会改变轮子行为
 * - 相同小步目标在不同 speed 下的反馈差异
 *
 * 用于判断 `11 / 21` 更接近哪种驱动语义。
 *
 * @param arg FreeRTOS 任务参数（未使用）
 */
void motor_probe_wheel_drive_task(void *arg);

/**
 * @brief 轮毂节点方向语义探测任务
 *
 * 在 `12 / 22` 已安全展开后，聚焦探测 `11 / 21` 的方向控制：
 * - `goal_position` 的正/负大偏移是否会改变转向
 * - 经典 DXL 风格的 speed 方向位是否有效
 *
 * @param arg FreeRTOS 任务参数（未使用）
 */
void motor_probe_wheel_direction_task(void *arg);

/**
 * @brief 轮毂节点控制接口探测任务
 *
 * 在 `12 / 22` 已安全展开后，继续验证 `11 / 21` 的最小可用控制接口：
 * - `goal_speed` 的低速死区与稳定起转范围
 * - `speed = 0` 是否可作为主动停转命令
 * - `torque off` 与 `speed = 0` 的停转差异
 *
 * @param arg FreeRTOS 任务参数（未使用）
 */
void motor_probe_wheel_control_task(void *arg);

/**
 * @brief 轮毂节点中间速度档位补测任务
 *
 * 只对 `21` 做较窄范围的速度补测，用于补齐 `20 / 25 / 30 / 40`
 * 这几个关键档位的起转阈值结论。
 *
 * @param arg FreeRTOS 任务参数（未使用）
 */
void motor_probe_wheel_midrange_task(void *arg);
