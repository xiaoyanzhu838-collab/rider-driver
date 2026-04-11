#include "rgb_control.h"

#include <stdatomic.h>

#include "esp_err.h"
#include "rgb_effects.h"
#include "rgb_service.h"

static atomic_bool s_enabled;

esp_err_t rgb_control_set_enabled(bool enable)
{
    atomic_store(&s_enabled, enable);

    // 关闭时进入柔和待机：不全灭，避免突兀
    if (!enable) {
        return rgb_service_set_mode(RGB_EFFECT_STANDBY);
    }

    // 开启时恢复到更高级的默认效果
    return rgb_service_set_mode(RGB_EFFECT_AURORA);
}

bool rgb_control_get_enabled(void)
{
    return atomic_load(&s_enabled);
}
