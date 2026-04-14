#include "motor_tuner.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"

static const char *TAG = "motor_tuner";

// ============================================================
// 独立电机调参程序
//
// 【使用方法】
// 修改下面的常量来定义调试动作：
//   - 速度、保持时间、重复次数等
//   - 目标电机的 ID 和位置偏移量
// 然后编译烧录，观察串口日志中的运动反馈数据。
//
// 【运动流程】
// 对每个目标电机：
//   1. 读取当前位置（作为起始位置 start）
//   2. 计算目标位置 goal = start + delta（并限制在安全范围内）
//   3. 使能扭矩 → 运动到 goal → 保持 TUNER_HOLD_MS → 读反馈
//   4. 运动回 start → 保持 TUNER_HOLD_MS → 读反馈
//   5. 失能扭矩
// ============================================================

// --- 调参参数（根据需要修改）---
#define TUNER_SPEED         120     // 运动速度（0~1023，越大越快）
#define TUNER_HOLD_MS       1200    // 到达目标后的保持等待时间（毫秒）
#define TUNER_PAUSE_MS      250     // 两个电机之间的间隔时间（毫秒）
#define TUNER_LOOPS         1       // 重复执行次数（用于稳定性测试）
#define TUNER_TORQUE_OFF_AT_END 1   // 运动结束后是否关闭扭矩（1=关闭，0=保持使能）

/**
 * @brief 单个调参目标的配置
 * @param id      舵机 ID
 * @param delta   位置偏移量（正值=正方向，负值=反方向）
 * @param enabled 是否启用此目标的调参动作
 */
typedef struct {
    uint8_t id;
    int delta;
    bool enabled;
} tuner_target_t;

// 调参目标列表：ID=11 正偏移 20，ID=12 未启用
static const tuner_target_t TUNER_TARGETS[] = {
    { .id = 11, .delta = 20,  .enabled = true  },
    { .id = 12, .delta = 0, .enabled = false  },
};

/**
 * @brief 将位置值限制在安全范围 [80, 940] 内
 *
 * 舵机的位置范围通常是 0~1023，但为避免到达物理极限导致堵转，
 * 将运动范围限制在 [80, 940] 的安全区间内。
 */
static uint16_t clamp_position(int pos)
{
    if (pos < 80) {
        pos = 80;
    }
    if (pos > 940) {
        pos = 940;
    }
    return (uint16_t)pos;
}

/**
 * @brief 执行单个电机的调参动作
 *
 * 完整流程：
 *   1. 读取当前实际位置 → 作为起始位置 start
 *   2. 计算目标位置 goal = start + delta（安全限幅）
 *   3. 使能扭矩
 *   4. 运动到 goal，等待 TUNER_HOLD_MS，读取中间状态反馈
 *   5. 运动回 start，等待 TUNER_HOLD_MS，读取最终状态反馈
 *   6. 失能扭矩（如果 TUNER_TORQUE_OFF_AT_END=1）
 *
 * @param target  调参目标配置
 */
static void move_one_motor(const tuner_target_t *target)
{
    // 1. 读取当前实际位置
    motor_feedback_t fb = {0};
    esp_err_t err = motor_read_feedback(target->id, &fb);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ID %u read failed: %s", target->id, esp_err_to_name(err));
        return;
    }

    uint16_t start = fb.position;                                   // 起始位置
    uint16_t goal = clamp_position((int)start + target->delta);     // 目标位置（安全限幅）

    ESP_LOGI(TAG,
             "ID %u start=%u goal=%u delta=%d speed=%u",
             target->id, start, goal, target->delta, TUNER_SPEED);

    // 2. 使能扭矩（锁定电机，准备运动）
    err = motor_set_torque(target->id, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ID %u torque on failed: %s", target->id, esp_err_to_name(err));
        return;
    }

    // 3. 运动到目标位置
    err = motor_set_position(target->id, goal, TUNER_SPEED);
    ESP_LOGI(TAG, "ID %u move out: %s", target->id, esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(TUNER_HOLD_MS));   // 等待运动完成

    // 4. 读取到达目标后的状态（位置、负载、运动标志）
    err = motor_read_feedback(target->id, &fb);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ID %u mid pos=%u load=%u moving=%u",
                 target->id, fb.position, fb.load, fb.moving);
    }

    // 5. 运动回起始位置
    err = motor_set_position(target->id, start, TUNER_SPEED);
    ESP_LOGI(TAG, "ID %u move back: %s", target->id, esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(TUNER_HOLD_MS));   // 等待运动完成

    // 6. 读取最终状态
    err = motor_read_feedback(target->id, &fb);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ID %u final pos=%u load=%u moving=%u",
                 target->id, fb.position, fb.load, fb.moving);
    }

    // 7. 失能扭矩（释放电机，可手动转动）
#if TUNER_TORQUE_OFF_AT_END
    err = motor_set_torque(target->id, false);
    ESP_LOGI(TAG, "ID %u torque off: %s", target->id, esp_err_to_name(err));
#endif
}

/**
 * @brief 电机调参 FreeRTOS 任务入口
 *
 * 初始化电机总线后，按照 TUNER_TARGETS 列表依次执行每个电机的调参动作，
 * 重复 TUNER_LOOPS 次。完成后进入空闲循环，保持任务存活。
 */
void motor_tuner_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Standalone Motor Tuner");
    ESP_LOGI(TAG, "  speed=%u hold_ms=%u loops=%u",
             TUNER_SPEED, TUNER_HOLD_MS, TUNER_LOOPS);
    ESP_LOGI(TAG, "======================================");

    // 初始化电机总线
    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    // 等待舵机上电稳定（舵机需要几百毫秒完成内部初始化）
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 按循环次数重复执行调参动作
    for (int loop = 0; loop < TUNER_LOOPS; loop++) {
        ESP_LOGI(TAG, "---- loop %d/%d ----", loop + 1, TUNER_LOOPS);
        // 遍历所有目标电机
        for (size_t i = 0; i < sizeof(TUNER_TARGETS) / sizeof(TUNER_TARGETS[0]); i++) {
            if (!TUNER_TARGETS[i].enabled) {
                continue;   // 跳过未启用的目标
            }
            move_one_motor(&TUNER_TARGETS[i]);      // 执行调参动作
            vTaskDelay(pdMS_TO_TICKS(TUNER_PAUSE_MS));  // 两个电机之间的间隔
        }
    }

    ESP_LOGI(TAG, "Standalone motor tuner done.");

    // FreeRTOS 任务不能退出，进入空闲循环保持任务存活
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
