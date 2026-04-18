#include "motor_control.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "scs_proto.h"

static const char *TAG = "motor";

// 本项目当前使用的两个舵机 ID
static const uint8_t MOTOR_IDS[MOTOR_COUNT] = { 12, 22 };

/**
 * @brief 初始化电机总线
 *
 * 调用 scs_init() 以默认波特率（1Mbps）初始化 UART 串行总线。
 * 底层会配置 UART2 (TX=GPIO14, RX=GPIO13) 的硬件参数。
 */
esp_err_t motor_init(void)
{
    return scs_init(SCS_DEFAULT_BAUD_RATE);
}

/**
 * @brief 获取所有电机的 ID 列表
 *
 * 返回内部 MOTOR_IDS 数组的指针和元素数量。
 * 上层代码可通过此函数遍历所有电机。
 */
size_t motor_get_ids(const uint8_t **ids_out)
{
    if (ids_out) {
        *ids_out = MOTOR_IDS;
    }
    return MOTOR_COUNT;
}

/**
 * @brief 开关电机扭矩
 *
 * 写入 TORQUE_ENABLE 寄存器（地址 0x18，1 字节）：
 *   - enable=true  → 写入 1，电机锁定，可以接受位置指令
 *   - enable=false → 写入 0，电机释放，可手动转动
 *
 * 【控制流程中的位置】
 *   1. motor_set_torque(id, true)   → 使能电机
 *   2. motor_set_position(...)      → 设置目标位置运动
 *   3. motor_set_torque(id, false)  → 失能电机（释放扭矩）
 */
esp_err_t motor_set_torque(uint8_t id, bool enable)
{
    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor init failed");
    return scs_write_byte(id, SCS_ADDR_TORQUE_ENABLE, enable ? 1 : 0);
}

/**
 * @brief 设置电机目标位置和运动速度
 *
 * 调用 scs_write_pos() 写入：
 *   1. GOAL_POSITION 寄存器（0x1E，2 字节）—— 目标位置
 *   2. GOAL_SPEED 寄存器（0x20，2 字节）—— 运动速度
 *
 * 电机将以指定速度从当前位置平滑运动到目标位置。
 */
esp_err_t motor_set_position(uint8_t id, uint16_t position, uint16_t speed)
{
    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor init failed");
    return scs_write_pos(id, (int16_t)position, speed, 0);
}

/**
 * @brief 读取电机完整反馈数据
 *
 * 分两步读取舵机的实时状态：
 *
 * 第一步 - 连续读取 6 字节（从 PRESENT_POSITION_L 开始）：
 *   params[0:1] → position（当前位置，16 位小端序）
 *   params[2:3] → speed（当前速度，16 位小端序）
 *   params[4:5] → load（当前负载，16 位小端序）
 *
 * 第二步 - 分别读取 3 个单字节寄存器：
 *   PRESENT_VOLTAGE（0x2A）→ 电压
 *   PRESENT_TEMPERATURE（0x2B）→ 温度
 *   MOVING（0x2E）→ 是否正在运动
 */
esp_err_t motor_read_feedback(uint8_t id, motor_feedback_t *out)
{
    esp_err_t err = ESP_OK;

    err = motor_read_feedback_fast(id, out);
    if (err != ESP_OK) {
        return err;
    }

    // 轮电机 11/21 上，附加状态寄存器不是高可靠项。
    // 主控制环只依赖位置/速度/负载，所以这里把扩展字段降级为“尽量读到”，
    // 读不到也不让整次反馈失败。
    (void)scs_read_byte(id, SCS_ADDR_PRESENT_VOLTAGE, &out->voltage);
    (void)scs_read_byte(id, SCS_ADDR_PRESENT_TEMPERATURE, &out->temperature);
    (void)scs_read_byte(id, SCS_ADDR_MOVING, &out->moving);
    return ESP_OK;
}

esp_err_t motor_read_feedback_fast(uint8_t id, motor_feedback_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->id = id;

    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor init failed");

    // 连续读取 6 字节：位置(2) + 速度(2) + 负载(2)
    scs_status_t st = {0};
    ESP_RETURN_ON_ERROR(scs_read(id, SCS_ADDR_PRESENT_POSITION_L, 6, &st),
                        TAG, "read present block failed");
    if (st.param_count < 6) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 解析小端序数据（低字节在前）
    out->position = st.params[0] | ((uint16_t)st.params[1] << 8);
    out->speed = st.params[2] | ((uint16_t)st.params[3] << 8);
    out->load = st.params[4] | ((uint16_t)st.params[5] << 8);

    return ESP_OK;
}

/**
 * @brief 批量读取所有电机的当前位置
 *
 * 遍历 MOTOR_IDS 数组，逐个读取每个电机的当前位置（2 字节）。
 * 适用于需要同时监控多个电机状态的场景。
 */
esp_err_t motor_read_positions(uint16_t *positions, size_t capacity, size_t *written)
{
    if (!positions || capacity < MOTOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        ESP_RETURN_ON_ERROR(scs_read_word(MOTOR_IDS[i], SCS_ADDR_PRESENT_POSITION_L, &positions[i]),
                            TAG, "read position failed");
    }

    if (written) {
        *written = MOTOR_COUNT;
    }
    return ESP_OK;
}
