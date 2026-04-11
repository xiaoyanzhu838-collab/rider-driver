#include "rgb_effects.h"

#include <stdint.h>

#include "ws2812.h"

static inline uint8_t scale_u8(uint8_t v, uint8_t scale)
{
    return (uint8_t)((((uint16_t)v) * scale) / 255);
}

void rgb_apply_brightness(uint8_t brightness, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = scale_u8(*r, brightness);
    *g = scale_u8(*g, brightness);
    *b = scale_u8(*b, brightness);
}

void rgb_color_temperature(uint16_t kelvin, uint8_t *r, uint8_t *g, uint8_t *b)
{
    // 简化版色温到RGB：2700K(暖) -> 6500K(冷)
    if (kelvin < 2700) kelvin = 2700;
    if (kelvin > 6500) kelvin = 6500;
    uint16_t t = (uint16_t)(kelvin - 2700);
    uint16_t range = (uint16_t)(6500 - 2700);

    // 暖光：偏红黄；冷光：偏蓝
    uint8_t warm_r = 255, warm_g = 160, warm_b = 48;
    uint8_t cool_r = 200, cool_g = 220, cool_b = 255;

    *r = (uint8_t)(warm_r + ((int)(cool_r - warm_r) * t) / range);
    *g = (uint8_t)(warm_g + ((int)(cool_g - warm_g) * t) / range);
    *b = (uint8_t)(warm_b + ((int)(cool_b - warm_b) * t) / range);
}

// 8-bit gamma 校正（γ≈2.2），让低亮度变化更细腻，“高级感”更强
static const uint8_t s_gamma8[256] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,
    1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,
    2,3,3,3,3,3,3,3,4,4,4,4,4,5,5,5,
    5,6,6,6,6,7,7,7,8,8,8,9,9,9,10,10,
    10,11,11,12,12,13,13,13,14,14,15,15,16,16,17,17,
    18,18,19,19,20,20,21,22,22,23,23,24,25,25,26,27,
    27,28,29,29,30,31,32,32,33,34,35,35,36,37,38,39,
    39,40,41,42,43,44,45,46,47,48,49,50,50,51,52,54,
    55,56,57,58,59,60,61,62,63,64,66,67,68,69,70,72,
    73,74,75,77,78,79,81,82,83,85,86,87,89,90,92,93,
    95,96,98,99,101,102,104,105,107,109,110,112,114,115,117,119,
    120,122,124,126,127,129,131,133,135,137,138,140,142,144,146,148,
    150,152,154,156,158,160,162,164,167,169,171,173,175,177,180,182,
    184,186,189,191,193,196,198,200,203,205,208,210,213,215,218,220,
    223,225,228,231,233,236,239,241,244,247,249,252,255,255,255,255
};

void rgb_gamma_correct(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (!r || !g || !b) return;
    *r = s_gamma8[*r];
    *g = s_gamma8[*g];
    *b = s_gamma8[*b];
}
