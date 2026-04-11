#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 这里不使用 gpio_num_t，避免头文件强依赖 driver/gpio.h
// gpio_num_t 本质是 int 的枚举类型，对外用 int 更利于组件解耦/复用
typedef struct {
    int gpio_num;
    int led_count;
    int enable_level;
} ws2812_config_t;

esp_err_t ws2812_init(const ws2812_config_t *cfg);

esp_err_t ws2812_set_pixel(int index, uint8_t r, uint8_t g, uint8_t b);

esp_err_t ws2812_clear(void);

esp_err_t ws2812_refresh(void);

void ws2812_hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif
