#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// button 模块入口任务：负责读取 EN 按键并控制 RGB 开关
void button_app_task(void *arg);

#ifdef __cplusplus
}
#endif

