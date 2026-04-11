#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RGB_EFFECT_OFF = 0,         // 强制全灭（保留，但业务上少用）
    RGB_EFFECT_STANDBY,         // 柔和待机（替代OFF，避免突兀）
    RGB_EFFECT_SOLID,
    RGB_EFFECT_BREATH,
    RGB_EFFECT_RAINBOW,
    RGB_EFFECT_CHASE,
    RGB_EFFECT_BLINK,
    RGB_EFFECT_TEMPERATURE,
    RGB_EFFECT_AURORA,          // 极光流动
} rgb_effect_t;

typedef struct {
    rgb_effect_t effect;
    uint8_t brightness;    // 0..255
    uint16_t speed_ms;     // 动画帧间隔
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t param;        // 通用参数：例如色温(K)、模式参数等
} rgb_effect_config_t;

#ifdef __cplusplus
}
#endif
