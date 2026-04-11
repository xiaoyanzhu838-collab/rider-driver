#pragma once

// 统一管理全项目的硬件引脚与关键参数（建议：只放“硬件事实”，不要放业务逻辑）
// 后续新增模块（motor/fan/sensor/uart/...）都从这里取引脚，避免散落在各组件里。

// ========== RGB / WS2812 ==========
#define BOARD_WS2812_GPIO          (27)
#define BOARD_WS2812_LED_COUNT     (4)

// RGB 默认效果参数（业务默认值放这里，方便统一调参）
#define BOARD_RGB_DEFAULT_BRIGHTNESS   (128)   // 0..255
#define BOARD_RGB_DEFAULT_SPEED_MS     (50)
#define BOARD_RGB_DEFAULT_R            (255)
#define BOARD_RGB_DEFAULT_G            (80)
#define BOARD_RGB_DEFAULT_B            (16)

// 预留：如果以后加电平反相、供电使能等，可以继续在这里追加
// #define BOARD_RGB_PWR_EN_GPIO    (...)

// ========== BUTTON / EN ==========
// EN 引脚：低电平有效（按下/拉低=有效）
#define BOARD_BTN_EN_GPIO          (0)

// ========== FAN / PWM ==========
#define BOARD_FAN_PWM_GPIO         (2)
#define BOARD_FAN_PWM_FREQ_HZ      (25000)  // 25kHz, a common frequency for PC fans
#define BOARD_FAN_DEFAULT_DUTY     (80)     // 60%

// ========== I2C / IMU ==========
#define BOARD_I2C_SCL_GPIO         (19)
#define BOARD_I2C_SDA_GPIO         (18)
#define BOARD_ICM42670_ADDR        (0x69)   // AD0 is pulled high
#define BOARD_IMU_SAMPLE_PERIOD_MS (5)      // 5ms -> 200Hz

// ========== BATTERY ADC ==========
// 分压电路: GND --[10K]-- IO33 --[20K]-- BAT
// V_ADC = V_BAT * 10K / (10K + 20K) = V_BAT * 1/3
// 所以 V_BAT = V_ADC * 3
#define BOARD_BAT_ADC_GPIO         (33)
#define BOARD_BAT_DIVIDER_RATIO    (3.0f)   // (10K + 20K) / 10K
// 电池电压范围（用于百分比映射）
#define BOARD_BAT_VOLTAGE_MIN      (6.4f)   // 2S锂电 空电 ~3.2V*2
#define BOARD_BAT_VOLTAGE_MAX      (8.4f)   // 2S锂电 满电 ~4.2V*2

