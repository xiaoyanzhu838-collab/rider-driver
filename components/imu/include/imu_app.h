#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// IMU 任务入口：周期读取 ICM42670 并打印/输出
void imu_app_task(void *arg);

#ifdef __cplusplus
}
#endif

