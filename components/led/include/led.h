#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化板载 LED（红色 + 蓝色），使用 LEDC PWM
 * 初始化后默认熄灭
 */
esp_err_t led_init(void);

// ========== 红色 LED (IO22, 低电平点亮) ==========

/** 点亮红色 LED（100% 亮度） */
esp_err_t led_red_on(void);

/** 熄灭红色 LED */
esp_err_t led_red_off(void);

/**
 * 设置红色 LED 亮度
 * @param duty_percent 0~100，0=熄灭，100=最亮
 */
esp_err_t led_red_set_duty(int duty_percent);

// ========== 蓝色 LED (IO23, 低电平点亮) ==========

/** 点亮蓝色 LED（100% 亮度） */
esp_err_t led_blue_on(void);

/** 熄灭蓝色 LED */
esp_err_t led_blue_off(void);

/**
 * 设置蓝色 LED 亮度
 * @param duty_percent 0~100，0=熄灭，100=最亮
 */
esp_err_t led_blue_set_duty(int duty_percent);

#ifdef __cplusplus
}
#endif
