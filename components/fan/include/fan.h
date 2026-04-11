#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化风扇 PWM（使用 board_config.h 中的 GPIO/频率/默认占空比）
esp_err_t fan_init(void);

// 设置占空比（0~100）
esp_err_t fan_set_duty_percent(int duty_percent);

#ifdef __cplusplus
}
#endif

