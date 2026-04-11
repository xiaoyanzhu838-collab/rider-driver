#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "icm42670.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t ts_us;        // esp_timer_get_time()
    float ax, ay, az;     // g
    float gx, gy, gz;     // dps
    float temp_c;         // Celsius
} imu_sample_t;

// 初始化I2C与IMU（ICM42670）
esp_err_t imu_init(void);

// 获取 ICM42670 句柄（imu_init 后有效）
icm42670_handle_t imu_get_handle(void);

// 写入/读取最近一次采样
void imu_set_latest(const imu_sample_t *s);

bool imu_get_latest(imu_sample_t *out);

#ifdef __cplusplus
}
#endif
