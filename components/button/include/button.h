#pragma once

#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*button_callback_t)(bool pressed, void *user_ctx);

// 初始化按键GPIO与中断（低电平有效/高电平有效由参数决定）
esp_err_t button_init(gpio_num_t gpio_num, bool active_low);

// 注册回调（在ISR里通过任务通知转发，在任务上下文回调）
esp_err_t button_register_callback(button_callback_t cb, void *user_ctx);

#ifdef __cplusplus
}
#endif

