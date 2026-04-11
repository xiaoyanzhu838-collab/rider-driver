#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// fan 模块入口任务：负责初始化 PWM 风扇，并长期驻留（后续可扩展控制逻辑）
void fan_app_task(void *arg);

#ifdef __cplusplus
}
#endif

