#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// RGB 开关控制：
// - enable=true  : 开启RGB（恢复默认效果/开始轮播）
// - enable=false : 进入柔和待机（不全灭，避免突兀）
esp_err_t rgb_control_set_enabled(bool enable);

bool rgb_control_get_enabled(void);

// 零点范围提示：
// - active=true  : RGB 变为绿色常亮，用于提示当前姿态已进入零点范围
// - active=false : 关闭该提示（熄灭）
esp_err_t rgb_control_set_zero_indicator(bool active);

#ifdef __cplusplus
}
#endif
