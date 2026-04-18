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
#define BOARD_FAN_DEFAULT_DUTY     (60)     // 60%

// ========== I2C / IMU ==========
#define BOARD_I2C_SCL_GPIO         (19)
#define BOARD_I2C_SDA_GPIO         (18)
#define BOARD_ICM42670_ADDR        (0x69)   // AD0 is pulled high
#define BOARD_IMU_SAMPLE_PERIOD_MS (5)      // 5ms -> 200Hz
#define BOARD_IMU_DIAGNOSTIC_MODE  (0)
#define BOARD_IMU_DIAGNOSTIC_LOG_INTERVAL_MS (50)
#define BOARD_IMU_DIAGNOSTIC_PREP_MS (2000)
#define BOARD_IMU_DIAGNOSTIC_STAGE_MS (3000)
#define BOARD_VQF_TAU_ACC          (3.0f)

// ========== LED (板载指示灯，低电平点亮) ==========
#define BOARD_LED_RED_GPIO         (22)
#define BOARD_LED_BLUE_GPIO        (23)
#define BOARD_LED_PWM_FREQ_HZ      (5000)   // 5kHz，适合 LED 调光

// ========== BATTERY ADC ==========
// 分压电路: GND --[10K]-- IO33 --[20K]-- BAT
// V_ADC = V_BAT * 10K / (10K + 20K) = V_BAT * 1/3
// 所以 V_BAT = V_ADC * 3
#define BOARD_BAT_ADC_GPIO         (33)
#define BOARD_BAT_DIVIDER_RATIO    (3.0f)   // (10K + 20K) / 10K
// 电池电压范围（用于百分比映射）
#define BOARD_BAT_VOLTAGE_MIN      (6.4f)   // 2S锂电 空电 ~3.2V*2
#define BOARD_BAT_VOLTAGE_MAX      (8.4f)   // 2S锂电 满电 ~4.2V*2

// ========== CHASSIS / BALANCE ==========
// 11 / 21 are used as the two wheel nodes on UART2.
#define BOARD_WHEEL_ID_LEFT        (11)
#define BOARD_WHEEL_ID_RIGHT       (21)

// Safe body-open posture derived from the 12 / 22 servo boundary exploration.
#define BOARD_BODY_HEIGHT_HOME_12  (599)
#define BOARD_BODY_HEIGHT_HOME_22  (439)
#define BOARD_BODY_HEIGHT_DELTA_12 (104)
#define BOARD_BODY_HEIGHT_DELTA_22 (112)
#define BOARD_BODY_HEIGHT_DEFAULT_PERCENT (25)
#define BOARD_BODY_HEIGHT_MOVE_SPEED      (200)

// Signed speed envelope confirmed on the wheel nodes.
#define BOARD_WHEEL_SPEED_LIMIT    (1000)
#define BOARD_WHEEL_MIN_EFFECTIVE_SPEED (30)
#define BOARD_WHEEL_BREAKAWAY_SPEED (60)
#define BOARD_WHEEL_BREAKAWAY_HIGH_SPEED (80)
#define BOARD_WHEEL_DIVERGE_BOOST_SPEED (80)
#define BOARD_WHEEL_ARM_ASSIST_SPEED (50)
#define BOARD_WHEEL_SPEED_RECOMMENDED (1000)

// Logical chassis wheel direction to raw wheel signed-speed mapping.
// Keep these as board-level constants so field tuning only needs one edit.
#define BOARD_WHEEL_LEFT_SIGN      (1)
#define BOARD_WHEEL_RIGHT_SIGN     (-1)

// Balance control loop defaults.
#define BOARD_CONTROL_INPUT_CENTER (128)
#define BOARD_CONTROL_INPUT_DEADBAND (4)
#define BOARD_BALANCE_LOOP_PERIOD_MS (10)
#define BOARD_BALANCE_TILT_CUTOFF_DEG (75.0f)
#define BOARD_BALANCE_FORCE_ENABLE (1)
#define BOARD_BALANCE_PITCH_TARGET_DEG (0.0f)
#define BOARD_BALANCE_ARM_PITCH_DEG (6.0f)
#define BOARD_BALANCE_ARM_GYRO_DPS (10.0f)
#define BOARD_BALANCE_ARM_HOLD_MS (350)
#define BOARD_BALANCE_ZERO_CAPTURE_LIMIT_DEG (2.5f)
#define BOARD_BALANCE_PITCH_DEADBAND_DEG (0.4f)
#define BOARD_BALANCE_RATE_DEADBAND_DPS (2.0f)
#define BOARD_BALANCE_WHEEL_FEEDBACK_INTERVAL_MS (20)
#define BOARD_BALANCE_THETA_COMPLEMENTARY_ALPHA (0.96f)
#define BOARD_BALANCE_RATE_LPF_ALPHA (0.35f)
#define BOARD_BALANCE_STICTION_PITCH_DEG (2.0f)
#define BOARD_BALANCE_STICTION_HIGH_PITCH_DEG (8.0f)
#define BOARD_BALANCE_STICTION_SPEED_THRESHOLD (5.0f)
#define BOARD_BALANCE_DIVERGE_PITCH_DEG (6.0f)
#define BOARD_BALANCE_DIVERGE_RATE_DPS (20.0f)
#define BOARD_BALANCE_ARM_ASSIST_MS (300)
#define BOARD_BALANCE_ARM_ASSIST_PITCH_DEG (5.0f)
#define BOARD_BALANCE_ARM_ASSIST_RATE_DPS (18.0f)
#define BOARD_BALANCE_SOFTSTART_MS (500)
#define BOARD_BALANCE_SOFTSTART_PITCH_DEG (6.0f)
#define BOARD_BALANCE_SOFTSTART_SPEED (45)
#define BOARD_BALANCE_OUTPUT_SIGN  (1.0f)
#define BOARD_BALANCE_PITCH_RATE_SIGN (1.0f)
#define BOARD_BALANCE_KP           (4.4f)
#define BOARD_BALANCE_KI           (0.0f)
#define BOARD_BALANCE_KD           (0.35f)
#define BOARD_BALANCE_I_TERM_LIMIT (30.0f)
#define BOARD_BALANCE_D_TERM_LIMIT (45.0f)
#define BOARD_BALANCE_KW           (0.0f)
#define BOARD_BALANCE_VX_MAX_SPEED (45)
#define BOARD_BALANCE_VYAW_MAX_SPEED (35)

// Temporary bring-up probe: keep body height fixed, skip PD balance,
// then drive the chassis in one direction so we can observe pitch response.
#define BOARD_WHEEL_PROBE_ENABLE       (0)
#define BOARD_WHEEL_PROBE_START_DELAY_MS (3000)
#define BOARD_WHEEL_PROBE_RUN_MS       (5000)
#define BOARD_WHEEL_PROBE_SPEED        (-45)

