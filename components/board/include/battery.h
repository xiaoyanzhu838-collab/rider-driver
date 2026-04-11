#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化电池ADC（使用 board_config.h 中的 GPIO）
esp_err_t battery_init(void);

// 读取电池电压（V）
float battery_read_voltage(void);

// 读取电池百分比（0~100）
uint8_t battery_read_percent(void);

#ifdef __cplusplus
}
#endif
