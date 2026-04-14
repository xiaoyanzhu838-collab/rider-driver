#pragma once

#include <stddef.h>
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

// 直接设置每颗灯珠颜色，实际刷新仍由 rgb_service 任务完成
esp_err_t rgb_service_set_pixels(const uint8_t (*rgb)[3], size_t count);

esp_err_t rgb_service_disable_output(void);

#ifdef __cplusplus
}
#endif
