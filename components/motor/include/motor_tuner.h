#pragma once

/**
 * @brief 独立电机调参任务
 *
 * 提供一个独立运行的电机参数调试工具。
 * 通过修改文件顶部的常量来定义调参动作：
 *   - TUNER_SPEED: 运动速度
 *   - TUNER_HOLD_MS: 到达目标位置后的保持时间（毫秒）
 *   - TUNER_PAUSE_MS: 两个电机动作之间的间隔（毫秒）
 *   - TUNER_LOOPS: 重复执行次数
 *   - TUNER_TARGETS[]: 目标电机列表及其位置偏移量
 *
 * 典型用途：调整电机运动参数、验证位置范围、测试响应速度等。
 *
 * @param arg  FreeRTOS 任务参数（未使用）
 */
void motor_tuner_task(void *arg);
