#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void rgb_apply_brightness(uint8_t brightness, uint8_t *r, uint8_t *g, uint8_t *b);

void rgb_color_temperature(uint16_t kelvin, uint8_t *r, uint8_t *g, uint8_t *b);

// gamma 校正：改善低亮度细节，让渐变更顺滑
void rgb_gamma_correct(uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif
