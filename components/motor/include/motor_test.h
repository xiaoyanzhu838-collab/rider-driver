#pragma once

/**
 * @brief 电机总线自检任务
 *
 * 默认会上电执行一次低幅度双电机演示，然后保持空闲。
 * 如需再次启用完整诊断，可在 motor_test.c 中将
 * `MOTOR_RUN_FULL_DIAGNOSTIC` 设为 1；
 * 如需关闭默认演示，可将 `MOTOR_RUN_SIMPLE_DEMO` 设为 0。
 */
void motor_test_task(void *arg);
