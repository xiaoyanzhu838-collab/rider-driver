#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "rgb_effects.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rgb_service_start(void);

esp_err_t rgb_service_stop(void);

// 设置当前效果（线程安全：内部会拷贝一份配置）
esp_err_t rgb_service_set_effect(const rgb_effect_config_t *cfg);

// 便捷接口：仅切换模式，其它参数保持不变
esp_err_t rgb_service_set_mode(rgb_effect_t effect);

#ifdef __cplusplus
}
#endif
