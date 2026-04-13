#pragma once

/**
 * @brief 电机总线自检任务
 *
 * 默认仅完成总线初始化并保持空闲。
 * 如需再次启用完整诊断，可在 motor_test.c 中将
 * `MOTOR_RUN_FULL_DIAGNOSTIC` 设为 1。
 */
void motor_test_task(void *arg);
