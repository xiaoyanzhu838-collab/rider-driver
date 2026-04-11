#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float roll;     // degrees
    float pitch;    // degrees
    float yaw;      // degrees
} ahrs_euler_t;

// 初始化 AHRS（Madgwick 滤波器）
void ahrs_init(float sample_freq_hz);

// 喂入 IMU 数据并更新姿态
// accel: g, gyro: dps（内部会转 rad/s）
void ahrs_update(float ax, float ay, float az, float gx, float gy, float gz);

// 获取当前欧拉角（degrees）
void ahrs_get_euler(ahrs_euler_t *out);

// 重置姿态到零点
void ahrs_reset(void);

#ifdef __cplusplus
}
#endif
