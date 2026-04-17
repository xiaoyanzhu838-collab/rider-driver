#include "rgb_control.h"

#include <stdatomic.h>

#include "esp_err.h"
#include "rgb_effects.h"
#include "rgb_service.h"

static atomic_bool s_enabled;

esp_err_t rgb_control_set_enabled(bool enable)
{
    atomic_store(&s_enabled, enable);

    // 关闭时保持全灭，避免后台持续刷新带来的额外干扰
    if (!enable) {
        return rgb_service_set_mode(RGB_EFFECT_OFF);
    }

    // 开启时恢复默认效果
    return rgb_service_set_mode(RGB_EFFECT_AURORA);
}

bool rgb_control_get_enabled(void)
{
    return atomic_load(&s_enabled);
}
