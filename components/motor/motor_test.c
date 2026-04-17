#include "motor_test.h"
#include "motor_control.h"
#include "scs_proto.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor_test";

// ============================================================
// 编译配置开关
//
// MOTOR_RUN_FULL_DIAGNOSTIC: 设为 1 启用完整诊断模式（见下方说明）
//   0 = 仅运行简单演示（默认）
//   1 = 运行完整的 PING + 寄存器扫描 + 多字节读取 + 运动测试
//
// MOTOR_RUN_SIMPLE_DEMO: 设为 1 在非诊断模式下运行简单演示
//   0 = 不运行任何演示，上电后直接进入空闲
//   1 = 运行简单双电机往复演示（默认）
// ============================================================
#ifndef MOTOR_RUN_FULL_DIAGNOSTIC
#define MOTOR_RUN_FULL_DIAGNOSTIC 1
#endif

#ifndef MOTOR_RUN_SIMPLE_DEMO
#define MOTOR_RUN_SIMPLE_DEMO 0
#endif

#ifndef MOTOR_RUN_FOCUSED_BOUNDARY
#define MOTOR_RUN_FOCUSED_BOUNDARY 1
#endif

// 需要测试的舵机 ID 列表
static const uint8_t TARGET_IDS[] = { 12, 22 };
#define TARGET_COUNT (sizeof(TARGET_IDS) / sizeof(TARGET_IDS[0]))
#define EXPLORATION_SETTLE_MS 120

static const uint8_t IDENTIFY_IDS[] = { 11, 12, 21, 22 };
#define IDENTIFY_COUNT (sizeof(IDENTIFY_IDS) / sizeof(IDENTIFY_IDS[0]))
#define SAFE_OPEN_20_GOAL_12 578
#define SAFE_OPEN_20_GOAL_22 461
static const uint8_t WHEEL_IDS[] = { 11, 21 };
#define WHEEL_COUNT (sizeof(WHEEL_IDS) / sizeof(WHEEL_IDS[0]))

// ============================================================
// 电机型号识别（Profile Detection）
//
// 不同品牌/型号的舵机使用不同的控制表地址映射。
// 本系统需要自动识别舵机型号，才能使用正确的寄存器地址。
//
// 目前支持两种 Profile：
//   - SCSCL/STS: Feetech 新版舵机（控制表起始地址偏大）
//   - Legacy DXL-like: 传统 Dynamixel AX 风格（地址较小）
// ============================================================

typedef enum {
    MOTOR_PROFILE_UNKNOWN = 0,      // 未能识别
    MOTOR_PROFILE_SCSCL,            // Feetech SCSCL/STS 系列
    MOTOR_PROFILE_LEGACY_DXL,       // 传统 Dynamixel AX 风格
} motor_profile_t;

/**
 * @brief 每个 Profile 对应的寄存器地址映射
 *
 * 不同型号舵机的同一功能寄存器在不同地址，需要通过此结构体做适配。
 */
typedef struct {
    uint8_t torque_enable;          // 扭矩使能寄存器地址
    uint8_t goal_position_l;        // 目标位置低字节地址
    uint8_t goal_speed_l;           // 目标速度低字节地址
    uint8_t present_position_l;     // 当前位置低字节地址
    bool    uses_acc;               // 是否使用加速度寄存器
    uint8_t acc_addr;               // 加速度寄存器地址（如果使用）
    const char *name;               // Profile 名称（用于日志输出）
} motor_profile_addrs_t;

static esp_err_t write_goal_for_profile(uint8_t id, motor_profile_t profile,
                                        int16_t goal_pos, uint16_t speed, uint8_t acc);
static int forward_delta_mod1024(uint16_t prev, uint16_t curr);
static int shortest_delta_mod1024(uint16_t from, uint16_t to);

/**
 * @brief 寄存器扫描结果
 * 存储从舵机逐字节读取的完整寄存器映射表
 */
typedef struct {
    bool valid;                     // 扫描结果是否有效
    uint8_t reg_map[72];            // 寄存器值数组（地址 0~71）
    bool reg_ok[72];                // 对应地址是否成功读取
    motor_profile_t profile;        // 识别出的 Profile 类型
} motor_scan_result_t;

// 保存每个舵机的扫描结果
static motor_scan_result_t s_scan_results[TARGET_COUNT];

/**
 * @brief 从寄存器映射表中读取 16 位值（Little-Endian）
 */
static uint16_t read_word_from_map(const uint8_t *reg_map, const bool *reg_ok, uint8_t addr_l)
{
    if (!reg_ok[addr_l] || !reg_ok[addr_l + 1]) {
        return 0;
    }
    return reg_map[addr_l] | ((uint16_t)reg_map[addr_l + 1] << 8);
}

/**
 * @brief 获取指定 Profile 的寄存器地址映射
 *
 * 两种 Profile 的关键寄存器地址对比：
 *   SCSCL/STS:   torque=40, goal_pos=42, goal_speed=46, present_pos=56
 *   Legacy DXL:  torque=24, goal_pos=30, goal_speed=32, present_pos=36
 */
static const motor_profile_addrs_t *get_profile_addrs(motor_profile_t profile)
{
    // Feetech SCSCL/STS 系列的地址映射
    static const motor_profile_addrs_t scscl = {
        .torque_enable = 40,
        .goal_position_l = 42,
        .goal_speed_l = 46,
        .present_position_l = 56,
        .uses_acc = true,
        .acc_addr = 41,
        .name = "SCSCL/STS",
    };
    // 传统 Dynamixel AX 风格的地址映射
    static const motor_profile_addrs_t legacy = {
        .torque_enable = 24,
        .goal_position_l = 30,
        .goal_speed_l = 32,
        .present_position_l = 36,
        .uses_acc = false,
        .acc_addr = 0,
        .name = "Legacy DXL-like",
    };

    switch (profile) {
    case MOTOR_PROFILE_SCSCL:
        return &scscl;
    case MOTOR_PROFILE_LEGACY_DXL:
        return &legacy;
    default:
        return NULL;
    }
}

/**
 * @brief 自动识别舵机的 Profile 类型
 *
 * 通过评分机制判断舵机型号：
 *   对每种 Profile 的关键寄存器进行特征匹配，命中则加分。
 *   最终得分较高的 Profile 被认为是当前舵机的型号。
 *
 * 评分规则：
 *   - ID 寄存器值与预期 ID 匹配 → +4 分（最强信号）
 *   - 扭矩使能寄存器值为 0 或 1 → +2 分
 *   - 电压值在合理范围 → +1~2 分
 *   - 位置值在合理范围 → +1~2 分
 */
static motor_profile_t detect_profile(uint8_t id, const uint8_t *reg_map, const bool *reg_ok)
{
    int scscl_score = 0;
    int legacy_score = 0;

    // --- SCSCL/STS 特征评分 ---
    if (reg_ok[5] && reg_map[5] == id) {            // 地址 5 应为 ID
        scscl_score += 4;
    }
    if (reg_ok[40] && reg_map[40] <= 1) {           // 地址 40 应为扭矩使能 (0或1)
        scscl_score += 2;
    }
    if (reg_ok[62] && reg_map[62] >= 40 && reg_map[62] <= 140) {  // 地址 62 应为电压 (合理范围)
        scscl_score += 2;
    }
    if (reg_ok[56] && reg_ok[57]) {                 // 地址 56-57 应为当前位置
        uint16_t pos = read_word_from_map(reg_map, reg_ok, 56);
        if (pos <= 4095) {                          // SCSCL 位置范围 0~4095
            scscl_score += 1;
        }
    }

    // --- Legacy DXL 特征评分 ---
    if (reg_ok[3] && reg_map[3] == id) {            // 地址 3 应为 ID
        legacy_score += 4;
    }
    if (reg_ok[24] && reg_map[24] <= 1) {           // 地址 24 应为扭矩使能 (0或1)
        legacy_score += 2;
    }
    if (reg_ok[42] && reg_map[42] >= 40 && reg_map[42] <= 140) {  // 地址 42 应为电压
        legacy_score += 1;
    }
    if (reg_ok[30] && reg_ok[31]) {                 // 地址 30-31 应为目标位置
        uint16_t goal = read_word_from_map(reg_map, reg_ok, 30);
        if (goal <= 1023) {                         // Legacy DXL 位置范围 0~1023
            legacy_score += 1;
        }
    }
    if (reg_ok[36] && reg_ok[37]) {                 // 地址 36-37 应为当前位置
        uint16_t pos = read_word_from_map(reg_map, reg_ok, 36);
        if (pos <= 1023) {
            legacy_score += 2;
        }
    }

    ESP_LOGI(TAG, "  profile score: SCSCL=%d legacy=%d", scscl_score, legacy_score);

    // 比较得分，高的胜出
    if (legacy_score > scscl_score) {
        return MOTOR_PROFILE_LEGACY_DXL;
    }
    if (scscl_score > legacy_score) {
        return MOTOR_PROFILE_SCSCL;
    }
    return MOTOR_PROFILE_UNKNOWN;
}

/**
 * @brief 按照 Legacy DXL 风格解读寄存器并输出到日志
 *
 * 将寄存器映射表中的数据按照 Dynamixel AX 的地址含义解读，
 * 输出型号、ID、波特率、角度限制、位置、速度、负载、电压、温度等信息。
 */
static void print_legacy_interpretation(const uint8_t *reg_map, const bool *reg_ok)
{
    ESP_LOGI(TAG, "  --- Legacy DXL-style interpretation ---");

    if (reg_ok[0] || reg_ok[1]) {
        uint16_t model = reg_map[0] | ((uint16_t)reg_map[1] << 8);
        ESP_LOGI(TAG, "  addr 0-1  (model):         %u", model);
    }
    if (reg_ok[2])
        ESP_LOGI(TAG, "  addr 2    (firmware):      %u", reg_map[2]);
    if (reg_ok[3])
        ESP_LOGI(TAG, "  addr 3    (ID):            %u", reg_map[3]);
    if (reg_ok[4])
        ESP_LOGI(TAG, "  addr 4    (baud code):     %u", reg_map[4]);
    if (reg_ok[5])
        ESP_LOGI(TAG, "  addr 5    (return delay):  %u", reg_map[5]);
    if (reg_ok[6]) {
        uint16_t cw = read_word_from_map(reg_map, reg_ok, 6);
        uint16_t ccw = read_word_from_map(reg_map, reg_ok, 8);
        ESP_LOGI(TAG, "  addr 6-9  (angle limits):  cw=%u ccw=%u", cw, ccw);
    }
    if (reg_ok[24])
        ESP_LOGI(TAG, "  addr 24   (torque):        %u", reg_map[24]);
    if (reg_ok[30]) {
        uint16_t goal = read_word_from_map(reg_map, reg_ok, 30);
        ESP_LOGI(TAG, "  addr 30-31(goal pos):      %u", goal);
    }
    if (reg_ok[32]) {
        uint16_t speed = read_word_from_map(reg_map, reg_ok, 32);
        ESP_LOGI(TAG, "  addr 32-33(goal speed):    %u", speed);
    }
    if (reg_ok[36]) {
        uint16_t pos = read_word_from_map(reg_map, reg_ok, 36);
        ESP_LOGI(TAG, "  addr 36-37(present pos):   %u", pos);
    }
    if (reg_ok[38]) {
        uint16_t spd = read_word_from_map(reg_map, reg_ok, 38);
        ESP_LOGI(TAG, "  addr 38-39(present speed): %u", spd);
    }
    if (reg_ok[40]) {
        uint16_t load = read_word_from_map(reg_map, reg_ok, 40);
        ESP_LOGI(TAG, "  addr 40-41(present load):  %u", load);
    }
    if (reg_ok[42])
        ESP_LOGI(TAG, "  addr 42   (voltage raw):   %u", reg_map[42]);
    if (reg_ok[43])
        ESP_LOGI(TAG, "  addr 43   (temperature):   %u", reg_map[43]);
    if (reg_ok[46])
        ESP_LOGI(TAG, "  addr 46   (moving):        %u", reg_map[46]);
}

/** 根据 ID 查找在 TARGET_IDS 中的索引 */
static int target_index_for_id(uint8_t id)
{
    for (int i = 0; i < TARGET_COUNT; i++) {
        if (TARGET_IDS[i] == id) {
            return i;
        }
    }
    return -1;
}

/** 获取指定 ID 舵机的 Profile 类型（优先使用扫描结果） */
static motor_profile_t profile_for_id(uint8_t id)
{
    int idx = target_index_for_id(id);
    if (idx >= 0 && s_scan_results[idx].valid) {
        return s_scan_results[idx].profile;
    }
    return MOTOR_PROFILE_SCSCL;    // 默认使用 SCSCL
}

/**
 * @brief 安全限幅：确保目标位置在合理范围 [80, 940] 内
 * 避免舵机到达物理极限导致堵转过载
 */
static uint16_t clamp_goal(uint16_t base, int delta)
{
    int goal = (int)base + delta;
    if (goal < 80) {
        goal = 80;
    }
    if (goal > 940) {
        goal = 940;
    }
    return (uint16_t)goal;
}

static int16_t speed_raw_to_signed(uint16_t raw)
{
    return (int16_t)raw;
}

static uint16_t load_mag_10bit(uint16_t raw)
{
    return raw & 0x03FF;
}

static uint8_t load_dir_10bit(uint16_t raw)
{
    return (raw & 0x0400) ? 1 : 0;
}

static uint16_t load_mag_11bit(uint16_t raw)
{
    return raw & 0x07FF;
}

static uint8_t load_dir_11bit(uint16_t raw)
{
    return (raw & 0x0800) ? 1 : 0;
}

static int abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (int)(a - b) : (int)(b - a);
}

static bool read_present_position(uint8_t id, motor_profile_t profile, uint16_t *out_pos)
{
    if (!out_pos) {
        return false;
    }

    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint8_t pos_addr = addrs ? addrs->present_position_l : SCS_ADDR_PRESENT_POSITION_L;
    scs_status_t st = {0};
    esp_err_t err = scs_read(id, pos_addr, 2, &st);
    if (err != ESP_OK || st.param_count < 2) {
        ESP_LOGW(TAG, "  ID %u read present pos failed: %s (addr=%u)",
                 id, esp_err_to_name(err), pos_addr);
        return false;
    }

    *out_pos = st.params[0] | ((uint16_t)st.params[1] << 8);
    return true;
}

static bool read_feedback_for_profile(uint8_t id, motor_profile_t profile,
                                      motor_feedback_t *out_fb, uint8_t *out_err)
{
    if (!out_fb) {
        return false;
    }

    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint8_t pos_addr = addrs ? addrs->present_position_l : SCS_ADDR_PRESENT_POSITION_L;
    scs_status_t st = {0};
    if (scs_read(id, pos_addr, 6, &st) != ESP_OK || st.param_count < 6) {
        return false;
    }

    memset(out_fb, 0, sizeof(*out_fb));
    out_fb->id = id;
    out_fb->position = st.params[0] | ((uint16_t)st.params[1] << 8);
    out_fb->speed = st.params[2] | ((uint16_t)st.params[3] << 8);
    out_fb->load = st.params[4] | ((uint16_t)st.params[5] << 8);
    (void)scs_read_byte(id, SCS_ADDR_PRESENT_VOLTAGE, &out_fb->voltage);
    (void)scs_read_byte(id, SCS_ADDR_PRESENT_TEMPERATURE, &out_fb->temperature);
    (void)scs_read_byte(id, SCS_ADDR_MOVING, &out_fb->moving);

    if (out_err) {
        *out_err = st.error;
    }
    return true;
}

static bool read_word_with_status(uint8_t id, uint8_t addr, uint16_t *out_val, uint8_t *out_err)
{
    if (!out_val) {
        return false;
    }

    scs_status_t st = {0};
    if (scs_read(id, addr, 2, &st) != ESP_OK || st.param_count < 2) {
        return false;
    }

    *out_val = st.params[0] | ((uint16_t)st.params[1] << 8);
    if (out_err) {
        *out_err = st.error;
    }
    return true;
}

static bool read_byte_with_status(uint8_t id, uint8_t addr, uint8_t *out_val, uint8_t *out_err)
{
    if (!out_val) {
        return false;
    }

    scs_status_t st = {0};
    if (scs_read(id, addr, 1, &st) != ESP_OK || st.param_count < 1) {
        return false;
    }

    *out_val = st.params[0];
    if (out_err) {
        *out_err = st.error;
    }
    return true;
}

static void log_goal_latch(const char *label, uint8_t id, motor_profile_t profile)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    if (!addrs) {
        ESP_LOGW(TAG, "%s ID=%u skip: unknown profile", label, id);
        return;
    }

    uint16_t goal_pos = 0;
    uint16_t goal_speed = 0;
    uint8_t torque = 0;
    uint8_t err_pos = 0;
    uint8_t err_speed = 0;
    uint8_t err_torque = 0;
    bool ok_pos = read_word_with_status(id, addrs->goal_position_l, &goal_pos, &err_pos);
    bool ok_speed = read_word_with_status(id, addrs->goal_speed_l, &goal_speed, &err_speed);
    bool ok_torque = read_byte_with_status(id, addrs->torque_enable, &torque, &err_torque);

    if (!ok_pos || !ok_speed || !ok_torque) {
        ESP_LOGW(TAG, "%s ID=%u latch read failed", label, id);
        return;
    }

    ESP_LOGI(TAG,
             "%s ID=%u torque=%u goal=%u speed=%u err_pos=0x%02X err_speed=0x%02X err_torque=0x%02X",
             label, id, torque, goal_pos, goal_speed, err_pos, err_speed, err_torque);
}

static void trace_motion(const char *label, uint8_t id, motor_profile_t profile,
                         int samples, int interval_ms)
{
    bool have_prev = false;
    uint16_t prev_pos = 0;
    int total_forward = 0;
    int total_shortest = 0;
    int nonzero_steps = 0;

    for (int i = 0; i < samples; i++) {
        motor_feedback_t fb = {0};
        uint8_t err = 0;
        if (read_feedback_for_profile(id, profile, &fb, &err)) {
            int16_t speed_s16 = speed_raw_to_signed(fb.speed);
            if (have_prev) {
                int step_forward = forward_delta_mod1024(prev_pos, fb.position);
                int step_shortest = shortest_delta_mod1024(prev_pos, fb.position);
                total_forward += step_forward;
                total_shortest += step_shortest;
                if (step_forward != 0) {
                    nonzero_steps++;
                }
            }
            ESP_LOGI(TAG,
                     "TRACE %s ID=%u sample=%02d pos=%u speed_raw=%u speed_s16=%d load_raw=%u load10=%u dir10=%u load11=%u dir11=%u moving=%u volt=%u temp=%u err=0x%02X",
                     label, id, i, fb.position, fb.speed, speed_s16, fb.load,
                     load_mag_10bit(fb.load), load_dir_10bit(fb.load),
                     load_mag_11bit(fb.load), load_dir_11bit(fb.load),
                     fb.moving, fb.voltage, fb.temperature, err);
            prev_pos = fb.position;
            have_prev = true;
        } else {
            ESP_LOGW(TAG, "TRACE %s ID=%u sample=%02d read failed", label, id, i);
        }
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }

    if (have_prev && samples > 1) {
        ESP_LOGI(TAG,
                 "TRACE %s SUMMARY ID=%u samples=%d total_forward=%d total_shortest=%d nonzero_steps=%d",
                 label, id, samples, total_forward, total_shortest, nonzero_steps);
    }
}

// ============================================================
// 简单演示模式（默认上电运行）
//
// 对每个电机执行一次往复运动：
//   使能 → 读当前位置 → 偏移运动 → 读反馈 → 回到起点 → 读反馈 → 失能
// ============================================================
static void run_simple_demo(void)
{
    const uint16_t speed = 120;                   // 运动速度
    const int deltas[MOTOR_COUNT] = { 60, -60 };  // 每个电机的位置偏移（对称运动）

    ESP_LOGI(TAG, "===== SIMPLE MOTOR DEMO =====");

    // 1. 使能所有电机
    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        esp_err_t err = motor_set_torque(id, true);
        ESP_LOGI(TAG, "  ID %u torque on: %s", id, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    vTaskDelay(pdMS_TO_TICKS(300));    // 等待扭矩稳定

    // 2. 逐个执行往复运动
    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        motor_feedback_t fb = {0};
        esp_err_t err = motor_read_feedback(id, &fb);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "  ID %u read before move failed: %s", id, esp_err_to_name(err));
            continue;
        }

        uint16_t start = fb.position;
        uint16_t target = clamp_goal(start, deltas[i]);
        ESP_LOGI(TAG, "  ID %u start=%u target=%u speed=%u", id, start, target, speed);

        // 运动到偏移位置
        err = motor_set_position(id, target, speed);
        ESP_LOGI(TAG, "  ID %u move out: %s", id, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1200));    // 等待运动完成

        // 读取中间状态
        err = motor_read_feedback(id, &fb);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ID %u mid pos=%u load=%u moving=%u", id, fb.position, fb.load, fb.moving);
        }

        // 运动回起始位置
        err = motor_set_position(id, start, speed);
        ESP_LOGI(TAG, "  ID %u move back: %s", id, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1200));    // 等待运动完成

        // 读取最终状态
        err = motor_read_feedback(id, &fb);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ID %u final pos=%u load=%u moving=%u", id, fb.position, fb.load, fb.moving);
        }

        vTaskDelay(pdMS_TO_TICKS(200));     // 两个电机之间的间隔
    }

    // 3. 失能所有电机（释放扭矩）
    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        esp_err_t err = motor_set_torque(id, false);
        ESP_LOGI(TAG, "  ID %u torque off: %s", id, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    ESP_LOGI(TAG, "===== SIMPLE MOTOR DEMO DONE =====");
}

// ============================================================
// 辅助函数：打印状态包内容到日志
// ============================================================
static void print_status(const char *label, uint8_t id, const scs_status_t *st)
{
    char hex[128] = "";
    for (int i = 0; i < st->param_count && i < 32; i++) {
        sprintf(hex + i * 3, "%02X ", st->params[i]);
    }
    ESP_LOGI(TAG, "%s ID=%u err=0x%02X params(%u): %s",
             label, id, st->error, st->param_count, hex);
}

// ============================================================
// Step 1: PING 探测
// 向每个目标 ID 发送 PING 指令，检查舵机是否在线
// ============================================================
static void step_ping_all(void)
{
    ESP_LOGI(TAG, "===== Step 1: PING all IDs =====");
    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        scs_status_t st = {0};
        esp_err_t err = scs_ping(id, &st);
        if (err == ESP_OK) {
            print_status("PING", id, &st);
        } else {
            ESP_LOGW(TAG, "  ID %u: OFFLINE (%s)", id, esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================
// Step 2: 逐字节寄存器扫描（最核心的诊断手段）
//
// 从地址 0 到 70，每次只读 1 字节，逐个读取所有寄存器。
// 生成完整的寄存器映射表，用于：
//   1. 确认舵机的实际控制表布局
//   2. 自动识别舵机型号（Profile Detection）
//   3. 排查通信异常（对比标准寄存器值是否合理）
// ============================================================
static motor_profile_t step_register_scan(uint8_t id)
{
    ESP_LOGI(TAG, "===== REGISTER SCAN ID=%u (addr 0..70, 1 byte each) =====", id);

    uint8_t reg_map[72];    // 存储每个地址读取到的值
    bool    reg_ok[72];     // 标记每个地址是否成功读取
    memset(reg_map, 0x00, sizeof(reg_map));
    memset(reg_ok, 0, sizeof(reg_ok));

    int ok_count = 0;
    int fail_count = 0;

    // 逐字节读取：每次发送 READ(id, addr, 1) 指令
    for (uint8_t addr = 0; addr <= 70; addr++) {
        scs_status_t st = {0};
        esp_err_t err = scs_read(id, addr, 1, &st);
        if (err == ESP_OK && st.param_count >= 1) {
            reg_map[addr] = st.params[0];   // 记录读取到的值
            reg_ok[addr] = true;            // 标记读取成功
            ok_count++;
        } else {
            fail_count++;                   // 读取失败
        }
        vTaskDelay(pdMS_TO_TICKS(20));      // 间隔 20ms，给总线恢复时间
    }

    ESP_LOGI(TAG, "  scan complete: %d OK, %d FAIL", ok_count, fail_count);

    // 以十六进制格式打印完整的寄存器映射表（每行 16 个地址）
    for (int row = 0; row <= 4; row++) {
        int base = row * 16;
        char line[256];
        int pos = 0;
        pos += sprintf(line + pos, "  [0x%02X] ", base);
        for (int col = 0; col < 16 && (base + col) <= 70; col++) {
            int a = base + col;
            if (reg_ok[a]) {
                pos += sprintf(line + pos, "%02X ", reg_map[a]);
            } else {
                pos += sprintf(line + pos, "-- ");   // 读取失败的地址显示为 --
            }
        }
        ESP_LOGI(TAG, "%s", line);
    }

    // --- 解读关键寄存器含义 ---
    ESP_LOGI(TAG, "  --- Key register interpretation ---");

    // EPROM 区：型号、版本、ID、波特率等
    if (reg_ok[3] || reg_ok[4]) {
        uint16_t ver_model = reg_map[3] | ((uint16_t)reg_map[4] << 8);
        ESP_LOGI(TAG, "  addr 3-4 (model/version): 0x%02X 0x%02X = %u",
                 reg_map[3], reg_map[4], ver_model);
    }
    if (reg_ok[5])
        ESP_LOGI(TAG, "  addr  5 (ID):              %u", reg_map[5]);
    if (reg_ok[6])
        ESP_LOGI(TAG, "  addr  6 (baud rate code):  %u", reg_map[6]);
    if (reg_ok[7])
        ESP_LOGI(TAG, "  addr  7 (return delay):    %u", reg_map[7]);

    // 角度限制
    if (reg_ok[9]) {
        uint16_t min_angle = reg_map[9] | ((uint16_t)reg_map[10] << 8);
        uint16_t max_angle = reg_map[11] | ((uint16_t)reg_map[12] << 8);
        ESP_LOGI(TAG, "  addr 9-12 (angle limits): min=%u max=%u", min_angle, max_angle);
    }

    // 工作模式（0=位置伺服，1=连续旋转）
    if (reg_ok[33])
        ESP_LOGI(TAG, "  addr 33 (mode):            %u (0=servo, 1=wheel)", reg_map[33]);

    // SRAM 控制区
    if (reg_ok[40])
        ESP_LOGI(TAG, "  addr 40 (torque enable):   %u", reg_map[40]);
    if (reg_ok[41])
        ESP_LOGI(TAG, "  addr 41 (acceleration):    %u", reg_map[41]);
    if (reg_ok[42]) {
        uint16_t goal = reg_map[42] | ((uint16_t)reg_map[43] << 8);
        ESP_LOGI(TAG, "  addr 42-43 (goal pos):     %u", goal);
    }
    if (reg_ok[44]) {
        uint16_t goal_t = reg_map[44] | ((uint16_t)reg_map[45] << 8);
        ESP_LOGI(TAG, "  addr 44-45 (goal time):    %u", goal_t);
    }
    if (reg_ok[46]) {
        uint16_t goal_s = reg_map[46] | ((uint16_t)reg_map[47] << 8);
        ESP_LOGI(TAG, "  addr 46-47 (goal speed):   %u", goal_s);
    }

    // 反馈区：位置、速度、负载、电压、温度等实时数据
    if (reg_ok[56]) {
        uint16_t pos = reg_map[56] | ((uint16_t)reg_map[57] << 8);
        ESP_LOGI(TAG, "  addr 56-57 (present pos):  %u", pos);
    }
    if (reg_ok[58]) {
        int16_t spd = (int16_t)(reg_map[58] | ((uint16_t)reg_map[59] << 8));
        ESP_LOGI(TAG, "  addr 58-59 (present spd):  %d", spd);
    }
    if (reg_ok[60]) {
        int16_t load = (int16_t)(reg_map[60] | ((uint16_t)reg_map[61] << 8));
        ESP_LOGI(TAG, "  addr 60-61 (present load): %d", load);
    }
    if (reg_ok[62])
        ESP_LOGI(TAG, "  addr 62 (voltage):         %u (raw)", reg_map[62]);
    if (reg_ok[63])
        ESP_LOGI(TAG, "  addr 63 (temperature):     %u", reg_map[63]);
    if (reg_ok[66])
        ESP_LOGI(TAG, "  addr 66 (moving):          %u", reg_map[66]);
    if (reg_ok[69]) {
        uint16_t cur = reg_map[69] | ((uint16_t)reg_map[70] << 8);
        ESP_LOGI(TAG, "  addr 69-70 (current):      %u", cur);
    }

    // 同时检查 Dynamixel 标准地址，用于对比判断
    ESP_LOGI(TAG, "  --- Dynamixel-style addresses for comparison ---");
    if (reg_ok[24])
        ESP_LOGI(TAG, "  addr 24(0x18) (DXL torque): %u", reg_map[24]);
    if (reg_ok[30]) {
        uint16_t dxl_goal = reg_map[30] | ((uint16_t)reg_map[31] << 8);
        ESP_LOGI(TAG, "  addr 30-31(0x1E) (DXL goal pos): %u", dxl_goal);
    }
    if (reg_ok[36]) {
        uint16_t dxl_pos = reg_map[36] | ((uint16_t)reg_map[37] << 8);
        ESP_LOGI(TAG, "  addr 36-37(0x24) (DXL present pos): %u", dxl_pos);
    }

    // 按 Legacy DXL 风格完整解读一次
    print_legacy_interpretation(reg_map, reg_ok);

    // 自动识别 Profile 并保存结果
    motor_profile_t profile = detect_profile(id, reg_map, reg_ok);
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    if (addrs) {
        ESP_LOGI(TAG, "  selected profile: %s", addrs->name);
    } else {
        ESP_LOGW(TAG, "  selected profile: UNKNOWN");
    }

    int idx = target_index_for_id(id);
    if (idx >= 0) {
        memcpy(s_scan_results[idx].reg_map, reg_map, sizeof(reg_map));
        memcpy(s_scan_results[idx].reg_ok, reg_ok, sizeof(reg_ok));
        s_scan_results[idx].profile = profile;
        s_scan_results[idx].valid = true;
    }
    return profile;
}

// ============================================================
// Step 3: 多字节 READ 测试
//
// 与 Step 2 的逐字节扫描结果进行对比验证。
// 使用 READ(id, addr, N) 一次读取 N 个连续字节，
// 验证单字节扫描和多字节读取的结果是否一致。
// ============================================================
static void step_read_multi_with_profile(uint8_t id, motor_profile_t profile)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint8_t pos_addr = addrs ? addrs->present_position_l : SCS_ADDR_PRESENT_POSITION_L;
    uint8_t torque_addr = addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE;

    ESP_LOGI(TAG, "===== Multi-byte READ tests (ID=%u) =====", id);

    // 读 2 字节 from addr 3 (model/version)
    {
        scs_status_t st = {0};
        esp_err_t err = scs_read(id, 3, 2, &st);
        if (err == ESP_OK) {
            print_status("READ(3,2)", id, &st);
        } else {
            ESP_LOGW(TAG, "  READ(3,2) failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    // 按当前 Profile 读取连续 6 字节（位置+速度+负载）
    {
        scs_status_t st = {0};
        esp_err_t err = scs_read(id, pos_addr, 6, &st);
        if (err == ESP_OK) {
            char label[32];
            snprintf(label, sizeof(label), "READ(%u,6)", pos_addr);
            print_status(label, id, &st);
        } else {
            ESP_LOGW(TAG, "  READ(%u,6) failed: %s", pos_addr, esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    // 读 1 字节 from addr 62 (voltage)
    {
        scs_status_t st = {0};
        esp_err_t err = scs_read(id, 62, 1, &st);
        if (err == ESP_OK) {
            print_status("READ(62,1)", id, &st);
        } else {
            ESP_LOGW(TAG, "  READ(62,1) failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    // 按当前 Profile 读取扭矩使能状态
    {
        scs_status_t st = {0};
        esp_err_t err = scs_read(id, torque_addr, 1, &st);
        if (err == ESP_OK) {
            char label[32];
            snprintf(label, sizeof(label), "READ(%u,1)", torque_addr);
            print_status(label, id, &st);
        } else {
            ESP_LOGW(TAG, "  READ(%u,1) failed: %s", torque_addr, esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static void step_read_multi(uint8_t id)
{
    step_read_multi_with_profile(id, profile_for_id(id));
}

static void move_servo_pair_to_safe_open_20(void)
{
    motor_feedback_t fb12 = {0};
    motor_feedback_t fb22 = {0};

    ESP_LOGI(TAG, "===== SAFE OPEN PRESET =====");
    ESP_LOGI(TAG, "  moving servo 12 -> %u, servo 22 -> %u", SAFE_OPEN_20_GOAL_12, SAFE_OPEN_20_GOAL_22);

    ESP_LOGI(TAG, "  torque on 12: %s", esp_err_to_name(motor_set_torque(12, true)));
    vTaskDelay(pdMS_TO_TICKS(60));
    ESP_LOGI(TAG, "  torque on 22: %s", esp_err_to_name(motor_set_torque(22, true)));
    vTaskDelay(pdMS_TO_TICKS(60));

    ESP_LOGI(TAG, "  set pos 12: %s",
             esp_err_to_name(motor_set_position(12, SAFE_OPEN_20_GOAL_12, 200)));
    vTaskDelay(pdMS_TO_TICKS(80));
    ESP_LOGI(TAG, "  set pos 22: %s",
             esp_err_to_name(motor_set_position(22, SAFE_OPEN_20_GOAL_22, 200)));

    vTaskDelay(pdMS_TO_TICKS(1800));

    if (motor_read_feedback(12, &fb12) == ESP_OK) {
        ESP_LOGI(TAG, "  safe open confirm ID12 pos=%u load=%u moving=%u",
                 fb12.position, fb12.load, fb12.moving);
    } else {
        ESP_LOGW(TAG, "  safe open confirm ID12 read failed");
    }

    if (motor_read_feedback(22, &fb22) == ESP_OK) {
        ESP_LOGI(TAG, "  safe open confirm ID22 pos=%u load=%u moving=%u",
                 fb22.position, fb22.load, fb22.moving);
    } else {
        ESP_LOGW(TAG, "  safe open confirm ID22 read failed");
    }
}

static uint16_t clamp_goal_legacy_full(int goal)
{
    if (goal < 0) {
        return 0;
    }
    if (goal > 1023) {
        return 1023;
    }
    return (uint16_t)goal;
}

static uint16_t wrap_goal_legacy_mod1024(uint16_t base, int delta)
{
    int goal = (int)base + delta;
    while (goal < 0) {
        goal += 1024;
    }
    while (goal > 1023) {
        goal -= 1024;
    }
    return (uint16_t)goal;
}

static int forward_delta_mod1024(uint16_t prev, uint16_t curr)
{
    return (curr >= prev) ? (int)(curr - prev) : (int)(curr + 1024 - prev);
}

static int shortest_delta_mod1024(uint16_t from, uint16_t to)
{
    int delta = (int)to - (int)from;
    while (delta > 512) {
        delta -= 1024;
    }
    while (delta < -512) {
        delta += 1024;
    }
    return delta;
}

static void log_feedback_snapshot(const char *label, uint8_t id, motor_profile_t profile)
{
    motor_feedback_t fb = {0};
    uint8_t err_bits = 0;
    if (read_feedback_for_profile(id, profile, &fb, &err_bits)) {
        ESP_LOGI(TAG,
                 "%s ID=%u pos=%u speed_raw=%u speed_s16=%d load_raw=%u moving=%u volt=%u temp=%u err=0x%02X",
                 label, id, fb.position, fb.speed, speed_raw_to_signed(fb.speed),
                 fb.load, fb.moving, fb.voltage, fb.temperature, err_bits);
    } else {
        ESP_LOGW(TAG, "%s ID=%u read feedback failed", label, id);
    }
}

static void wheel_torque_probe(uint8_t id, motor_profile_t profile)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint8_t torque_before = 0;
    uint8_t torque_after_on = 0;
    uint8_t torque_after_off = 0;

    ESP_LOGI(TAG, "===== WHEEL TORQUE PROBE ID=%u =====", id);
    if (!read_byte_with_status(id,
                               addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE,
                               &torque_before,
                               NULL)) {
        ESP_LOGW(TAG, "WHEEL TORQUE ID=%u baseline read failed", id);
        return;
    }
    ESP_LOGI(TAG, "WHEEL TORQUE ID=%u baseline=%u", id, torque_before);
    log_feedback_snapshot("WHEEL_TORQUE_BASE", id, profile);

    ESP_LOGI(TAG, "WHEEL TORQUE ID=%u -> ON (%s)",
             id, esp_err_to_name(scs_write_byte(id,
                                                addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE,
                                                1)));
    vTaskDelay(pdMS_TO_TICKS(250));
    (void)read_byte_with_status(id,
                                addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE,
                                &torque_after_on,
                                NULL);
    log_feedback_snapshot("WHEEL_TORQUE_ON", id, profile);

    ESP_LOGI(TAG, "WHEEL TORQUE ID=%u -> OFF (%s)",
             id, esp_err_to_name(scs_write_byte(id,
                                                addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE,
                                                0)));
    vTaskDelay(pdMS_TO_TICKS(250));
    (void)read_byte_with_status(id,
                                addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE,
                                &torque_after_off,
                                NULL);
    log_feedback_snapshot("WHEEL_TORQUE_OFF", id, profile);

    ESP_LOGI(TAG, "WHEEL TORQUE SUMMARY ID=%u baseline=%u after_on=%u after_off=%u",
             id, torque_before, torque_after_on, torque_after_off);
}

static void wheel_position_probe(uint8_t id, motor_profile_t profile, int delta)
{
    uint16_t base = 0;
    uint16_t goal = 0;
    uint16_t restore = 0;
    const uint16_t speed = 40;
    const uint8_t acc = 10;
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);

    ESP_LOGI(TAG, "===== WHEEL POSITION PROBE ID=%u delta=%+d =====", id, delta);
    if (!read_present_position(id, profile, &base)) {
        ESP_LOGW(TAG, "WHEEL POS ID=%u baseline pos read failed", id);
        return;
    }

    goal = clamp_goal_legacy_full((int)base + delta);
    restore = base;
    ESP_LOGI(TAG, "WHEEL POS ID=%u base=%u goal=%u restore=%u speed=%u",
             id, base, goal, restore, speed);

    ESP_LOGI(TAG, "WHEEL POS ID=%u torque on -> %s",
             id, esp_err_to_name(scs_write_byte(id,
                                                addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE,
                                                1)));
    vTaskDelay(pdMS_TO_TICKS(120));
    log_goal_latch("WHEEL_BEFORE_MOVE", id, profile);
    log_feedback_snapshot("WHEEL_BEFORE_MOVE_FB", id, profile);

    ESP_LOGI(TAG, "WHEEL POS ID=%u write goal=%u -> %s",
             id, goal, esp_err_to_name(write_goal_for_profile(id, profile, (int16_t)goal, speed, acc)));
    log_goal_latch("WHEEL_AFTER_WRITE", id, profile);
    trace_motion("WHEEL_MOVE", id, profile, 12, 120);

    ESP_LOGI(TAG, "WHEEL POS ID=%u restore goal=%u -> %s",
             id, restore, esp_err_to_name(write_goal_for_profile(id, profile, (int16_t)restore, speed, acc)));
    log_goal_latch("WHEEL_AFTER_RESTORE_WRITE", id, profile);
    trace_motion("WHEEL_RESTORE", id, profile, 12, 120);

    ESP_LOGI(TAG, "WHEEL POS ID=%u torque off -> %s",
             id, esp_err_to_name(scs_write_byte(id,
                                                addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE,
                                                0)));
    vTaskDelay(pdMS_TO_TICKS(120));
    log_feedback_snapshot("WHEEL_AFTER_TORQUE_OFF", id, profile);
}

static esp_err_t write_goal_speed_only_for_profile(uint8_t id, motor_profile_t profile, uint16_t speed)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    if (!addrs) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return scs_write_word(id, addrs->goal_speed_l, speed);
}

static void wheel_hold_probe(uint8_t id, motor_profile_t profile, uint16_t speed)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint16_t hold = 0;

    ESP_LOGI(TAG, "===== WHEEL HOLD PROBE ID=%u speed=%u =====", id, speed);
    if (!read_present_position(id, profile, &hold)) {
        ESP_LOGW(TAG, "WHEEL HOLD ID=%u baseline pos read failed", id);
        return;
    }

    ESP_LOGI(TAG, "WHEEL HOLD ID=%u hold=%u", id, hold);
    ESP_LOGI(TAG, "WHEEL HOLD ID=%u torque on -> %s",
             id, esp_err_to_name(scs_write_byte(id,
                                                addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE,
                                                1)));
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_LOGI(TAG, "WHEEL HOLD ID=%u write goal=%u speed=%u -> %s",
             id, hold, speed,
             esp_err_to_name(write_goal_for_profile(id, profile, (int16_t)hold, speed, 10)));
    log_goal_latch("WHEEL_HOLD_LATCH", id, profile);
    trace_motion("WHEEL_HOLD", id, profile, 10, 120);
}

static void wheel_speed_only_probe(uint8_t id, motor_profile_t profile, uint16_t speed)
{
    ESP_LOGI(TAG, "===== WHEEL SPEED-ONLY PROBE ID=%u speed=%u =====", id, speed);
    ESP_LOGI(TAG, "WHEEL SPEED-ONLY ID=%u write speed=%u -> %s",
             id, speed, esp_err_to_name(write_goal_speed_only_for_profile(id, profile, speed)));
    log_goal_latch("WHEEL_SPEED_ONLY_LATCH", id, profile);
    trace_motion("WHEEL_SPEED_ONLY", id, profile, 10, 120);
}

static void wheel_speed_only_probe_tagged(const char *tag, uint8_t id, motor_profile_t profile,
                                          uint16_t speed, int samples, int interval_ms)
{
    ESP_LOGI(TAG, "===== %s ID=%u speed=%u =====", tag, id, speed);
    ESP_LOGI(TAG, "%s ID=%u write speed=%u -> %s",
             tag, id, speed, esp_err_to_name(write_goal_speed_only_for_profile(id, profile, speed)));
    log_goal_latch("WHEEL_SPEED_TAG_LATCH", id, profile);
    trace_motion(tag, id, profile, samples, interval_ms);
}

static void wheel_goal_speed_probe(uint8_t id, motor_profile_t profile, int delta, uint16_t speed)
{
    uint16_t base = 0;
    uint16_t goal = 0;

    ESP_LOGI(TAG, "===== WHEEL GOAL+SPEED PROBE ID=%u delta=%+d speed=%u =====",
             id, delta, speed);
    if (!read_present_position(id, profile, &base)) {
        ESP_LOGW(TAG, "WHEEL GOAL+SPEED ID=%u baseline pos read failed", id);
        return;
    }

    goal = clamp_goal_legacy_full((int)base + delta);
    ESP_LOGI(TAG, "WHEEL GOAL+SPEED ID=%u base=%u goal=%u", id, base, goal);
    ESP_LOGI(TAG, "WHEEL GOAL+SPEED ID=%u write goal=%u speed=%u -> %s",
             id, goal, speed,
             esp_err_to_name(write_goal_for_profile(id, profile, (int16_t)goal, speed, 10)));
    log_goal_latch("WHEEL_GOAL_SPEED_LATCH", id, profile);
    trace_motion("WHEEL_GOAL_SPEED", id, profile, 10, 120);
}

static void wheel_goal_direction_probe(uint8_t id, motor_profile_t profile, int delta, uint16_t speed)
{
    uint16_t base = 0;
    uint16_t goal = 0;
    int shortest = 0;

    ESP_LOGI(TAG, "===== WHEEL GOAL-DIR PROBE ID=%u delta=%+d speed=%u =====",
             id, delta, speed);
    if (!read_present_position(id, profile, &base)) {
        ESP_LOGW(TAG, "WHEEL GOAL-DIR ID=%u baseline pos read failed", id);
        return;
    }

    goal = wrap_goal_legacy_mod1024(base, delta);
    shortest = shortest_delta_mod1024(base, goal);
    ESP_LOGI(TAG, "WHEEL GOAL-DIR ID=%u base=%u goal=%u shortest=%d",
             id, base, goal, shortest);
    ESP_LOGI(TAG, "WHEEL GOAL-DIR ID=%u write goal=%u speed=%u -> %s",
             id, goal, speed,
             esp_err_to_name(write_goal_for_profile(id, profile, (int16_t)goal, speed, 10)));
    log_goal_latch("WHEEL_GOAL_DIR_LATCH", id, profile);
    trace_motion("WHEEL_GOAL_DIR", id, profile, 8, 120);
}

static void wheel_direction_focus_probe(uint8_t id, motor_profile_t profile)
{
    const uint16_t speed_hold = 10;
    const uint16_t speed_mid = 40;
    const uint16_t speed_dirbit = 0x0400 | 40;

    ESP_LOGI(TAG, "===== WHEEL DIRECTION FOCUS ID=%u =====", id);

    wheel_hold_probe(id, profile, speed_hold);
    vTaskDelay(pdMS_TO_TICKS(250));
    wheel_goal_direction_probe(id, profile, +128, speed_mid);
    vTaskDelay(pdMS_TO_TICKS(250));

    wheel_hold_probe(id, profile, speed_hold);
    vTaskDelay(pdMS_TO_TICKS(250));
    wheel_goal_direction_probe(id, profile, -128, speed_mid);
    vTaskDelay(pdMS_TO_TICKS(250));

    wheel_hold_probe(id, profile, speed_hold);
    vTaskDelay(pdMS_TO_TICKS(250));
    wheel_speed_only_probe_tagged("WHEEL_SPEED_DIRBIT_FWD", id, profile, speed_mid, 8, 120);
    vTaskDelay(pdMS_TO_TICKS(250));

    wheel_hold_probe(id, profile, speed_hold);
    vTaskDelay(pdMS_TO_TICKS(250));
    wheel_speed_only_probe_tagged("WHEEL_SPEED_DIRBIT_REVBIT", id, profile, speed_dirbit, 8, 120);
}

static void wheel_speed_deadband_probe(uint8_t id, motor_profile_t profile)
{
    static const uint16_t speeds[] = { 0, 1, 5, 10, 15, 20, 25, 30, 40, 60 };

    ESP_LOGI(TAG, "===== WHEEL SPEED DEADBAND ID=%u =====", id);
    for (size_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++) {
        wheel_hold_probe(id, profile, 10);
        vTaskDelay(pdMS_TO_TICKS(220));
        wheel_speed_only_probe_tagged("WHEEL_SPEED_SWEEP", id, profile, speeds[i], 6, 100);
        vTaskDelay(pdMS_TO_TICKS(260));
    }
}

static void wheel_stop_semantics_probe(uint8_t id, motor_profile_t profile)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    if (!addrs) {
        ESP_LOGW(TAG, "WHEEL STOP ID=%u unknown profile", id);
        return;
    }

    ESP_LOGI(TAG, "===== WHEEL STOP SEMANTICS ID=%u =====", id);

    wheel_hold_probe(id, profile, 10);
    vTaskDelay(pdMS_TO_TICKS(220));
    wheel_speed_only_probe_tagged("WHEEL_STOP_PRE_SPEED0", id, profile, 40, 5, 100);
    vTaskDelay(pdMS_TO_TICKS(180));
    ESP_LOGI(TAG, "WHEEL STOP ID=%u write speed=0 -> %s",
             id, esp_err_to_name(write_goal_speed_only_for_profile(id, profile, 0)));
    log_goal_latch("WHEEL_STOP_SPEED0_LATCH", id, profile);
    trace_motion("WHEEL_STOP_SPEED0", id, profile, 8, 100);

    wheel_hold_probe(id, profile, 10);
    vTaskDelay(pdMS_TO_TICKS(220));
    wheel_speed_only_probe_tagged("WHEEL_STOP_PRE_TORQUE_OFF", id, profile, 40, 5, 100);
    vTaskDelay(pdMS_TO_TICKS(180));
    ESP_LOGI(TAG, "WHEEL STOP ID=%u torque off -> %s",
             id, esp_err_to_name(scs_write_byte(id, addrs->torque_enable, 0)));
    log_goal_latch("WHEEL_STOP_TORQUE_OFF_LATCH", id, profile);
    trace_motion("WHEEL_STOP_TORQUE_OFF", id, profile, 8, 100);

    ESP_LOGI(TAG, "WHEEL STOP ID=%u torque on -> %s",
             id, esp_err_to_name(scs_write_byte(id, addrs->torque_enable, 1)));
    vTaskDelay(pdMS_TO_TICKS(120));
    wheel_hold_probe(id, profile, 10);
}

static void wheel_drive_semantics_probe(uint8_t id, motor_profile_t profile)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    static const uint16_t speeds[] = { 10, 40, 100 };

    ESP_LOGI(TAG, "===== WHEEL DRIVE SEMANTICS ID=%u =====", id);
    if (!addrs) {
        ESP_LOGW(TAG, "WHEEL DRIVE ID=%u unknown profile", id);
        return;
    }

    ESP_LOGI(TAG, "WHEEL DRIVE ID=%u torque on -> %s",
             id, esp_err_to_name(scs_write_byte(id, addrs->torque_enable, 1)));
    vTaskDelay(pdMS_TO_TICKS(150));
    log_feedback_snapshot("WHEEL_DRIVE_BASE", id, profile);

    wheel_hold_probe(id, profile, speeds[0]);
    vTaskDelay(pdMS_TO_TICKS(250));

    for (int i = 0; i < (int)(sizeof(speeds) / sizeof(speeds[0])); i++) {
        wheel_speed_only_probe(id, profile, speeds[i]);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    for (int i = 0; i < (int)(sizeof(speeds) / sizeof(speeds[0])); i++) {
        wheel_goal_speed_probe(id, profile, +8, speeds[i]);
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    wheel_goal_speed_probe(id, profile, -8, speeds[1]);
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "WHEEL DRIVE ID=%u torque off -> %s",
             id, esp_err_to_name(scs_write_byte(id, addrs->torque_enable, 0)));
    vTaskDelay(pdMS_TO_TICKS(120));
    log_feedback_snapshot("WHEEL_DRIVE_AFTER_TORQUE_OFF", id, profile);
}

// ============================================================
// Step 4: WRITE 扭矩测试 + 小步位置测试
// ============================================================

/**
 * @brief 批量设置所有电机的扭矩使能状态
 * 使用扫描结果中确定的正确寄存器地址
 */
static void step_torque_all(uint8_t enable)
{
    ESP_LOGI(TAG, "===== WRITE torque_enable = %u =====", enable);
    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        motor_profile_t profile = s_scan_results[i].valid ? s_scan_results[i].profile : MOTOR_PROFILE_SCSCL;
        const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
        uint8_t torque_addr = addrs ? addrs->torque_enable : SCS_ADDR_TORQUE_ENABLE;
        esp_err_t err = scs_write_byte(id, torque_addr, enable);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ID %u: torque %s OK (addr=%u, profile=%s)",
                     id, enable ? "ON" : "OFF", torque_addr, addrs ? addrs->name : "unknown");
        } else {
            ESP_LOGW(TAG, "  ID %u: torque write failed (%s)", id, esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/**
 * @brief 根据舵机 Profile 写入目标位置和速度
 * 不同 Profile 使用不同的寄存器地址和写入方式
 */
static esp_err_t write_goal_for_profile(uint8_t id, motor_profile_t profile,
                                        int16_t goal_pos, uint16_t speed, uint8_t acc)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    if (!addrs) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (addrs->uses_acc) {
        // SCSCL/STS：使用 scs_write_pos（包含加速度参数）
        return scs_write_pos(id, goal_pos, speed, acc);
    }

    // Legacy DXL：分别写位置和速度
    esp_err_t err = scs_write_word(id, addrs->goal_position_l, (uint16_t)goal_pos);
    if (err != ESP_OK) {
        return err;
    }
    return scs_write_word(id, addrs->goal_speed_l, speed);
}

/**
 * @brief 小步位置测试：写入目标位置后读回验证
 *
 * 1. 读取当前位置 → 作为基准
 * 2. 在基准位置上偏移 +50，写入目标位置
 * 3. 等待 2 秒让电机运动完成
 * 4. 读回位置验证是否到达
 */
static void step_write_and_verify(uint8_t id, int delta, uint16_t speed, uint8_t acc)
{
    motor_profile_t profile = profile_for_id(id);
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint16_t before = 0;
    bool before_valid = false;

    ESP_LOGI(TAG, "===== WritePos ID=%u delta=%d speed=%u acc=%u profile=%s =====",
             id, delta, speed, acc, addrs ? addrs->name : "unknown");

    // 读取运动前的位置
    if (read_present_position(id, profile, &before)) {
        before_valid = true;
        ESP_LOGI(TAG, "  before: pos=%u (0x%04X)", before, before);
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // 计算目标位置：在当前位置基础上按指定偏移做小步运动
    uint16_t base = before_valid ? before : 600;
    int16_t goal_pos = (int16_t)clamp_goal(base, delta);

    ESP_LOGI(TAG, "  target goal=%d", goal_pos);

    // 写入目标位置
    esp_err_t err = write_goal_for_profile(id, profile, goal_pos, speed, acc);
    ESP_LOGI(TAG, "  write_pos: %s", esp_err_to_name(err));

    // 等待运动完成
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 读回运动后的位置，验证是否到达
    {
        uint16_t after = 0;
        if (read_present_position(id, profile, &after)) {
            ESP_LOGI(TAG, "  after:  pos=%u (0x%04X)", after, after);
        }
    }
}

static void step_sync_write_legacy_pair(uint16_t speed)
{
    if (TARGET_COUNT < 2) {
        return;
    }

    const uint8_t id_a = TARGET_IDS[0];
    const uint8_t id_b = TARGET_IDS[1];
    const motor_profile_t profile_a = profile_for_id(id_a);
    const motor_profile_t profile_b = profile_for_id(id_b);

    if (profile_a != MOTOR_PROFILE_LEGACY_DXL || profile_b != MOTOR_PROFILE_LEGACY_DXL) {
        const motor_profile_addrs_t *addrs_a = get_profile_addrs(profile_a);
        const motor_profile_addrs_t *addrs_b = get_profile_addrs(profile_b);
        ESP_LOGW(TAG,
                 "skip sync_write(0x1E,4): need legacy pair, got %s / %s",
                 addrs_a ? addrs_a->name : "unknown",
                 addrs_b ? addrs_b->name : "unknown");
        return;
    }

    uint16_t before_a = 0;
    uint16_t before_b = 0;
    if (!read_present_position(id_a, profile_a, &before_a) ||
        !read_present_position(id_b, profile_b, &before_b)) {
        ESP_LOGW(TAG, "skip sync_write pair: failed to read starting positions");
        return;
    }

    const uint16_t goal_a = clamp_goal(before_a, +40);
    const uint16_t goal_b = clamp_goal(before_b, -40);
    uint8_t pair_data[10] = {
        id_a,
        (uint8_t)(goal_a & 0xFF),
        (uint8_t)(goal_a >> 8),
        (uint8_t)(speed & 0xFF),
        (uint8_t)(speed >> 8),
        id_b,
        (uint8_t)(goal_b & 0xFF),
        (uint8_t)(goal_b >> 8),
        (uint8_t)(speed & 0xFF),
        (uint8_t)(speed >> 8),
    };

    ESP_LOGI(TAG,
             "===== SYNC_WRITE legacy pair addr=0x%02X len=4 =====", SCS_ADDR_GOAL_POSITION_L);
    ESP_LOGI(TAG,
             "  before: ID %u=%u, ID %u=%u", id_a, before_a, id_b, before_b);
    ESP_LOGI(TAG,
             "  goal:   ID %u=%u speed=%u, ID %u=%u speed=%u",
             id_a, goal_a, speed, id_b, goal_b, speed);

    esp_err_t err = scs_sync_write(SCS_ADDR_GOAL_POSITION_L, 4, pair_data, 2);
    ESP_LOGI(TAG, "  sync_write move: %s", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint16_t after_move_a = 0;
    uint16_t after_move_b = 0;
    if (read_present_position(id_a, profile_a, &after_move_a) &&
        read_present_position(id_b, profile_b, &after_move_b)) {
        ESP_LOGI(TAG,
                 "  after move: ID %u=%u, ID %u=%u",
                 id_a, after_move_a, id_b, after_move_b);
    }

    pair_data[1] = (uint8_t)(before_a & 0xFF);
    pair_data[2] = (uint8_t)(before_a >> 8);
    pair_data[6] = (uint8_t)(before_b & 0xFF);
    pair_data[7] = (uint8_t)(before_b >> 8);

    err = scs_sync_write(SCS_ADDR_GOAL_POSITION_L, 4, pair_data, 2);
    ESP_LOGI(TAG, "  sync_write restore: %s", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint16_t after_restore_a = 0;
    uint16_t after_restore_b = 0;
    if (read_present_position(id_a, profile_a, &after_restore_a) &&
        read_present_position(id_b, profile_b, &after_restore_b)) {
        ESP_LOGI(TAG,
                 "  after restore: ID %u=%u, ID %u=%u",
                 id_a, after_restore_a, id_b, after_restore_b);
    }
}

static void run_probe_with_trace_ex(uint8_t id, int delta, uint16_t speed, uint8_t acc,
                                    int samples, int interval_ms)
{
    motor_profile_t profile = profile_for_id(id);
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint16_t base = 0;
    if (!read_present_position(id, profile, &base)) {
        ESP_LOGW(TAG, "probe ID=%u read base failed", id);
        return;
    }

    uint16_t goal = clamp_goal(base, delta);
    ESP_LOGI(TAG,
             "===== PROBE ID=%u delta=%d speed=%u acc=%u base=%u goal=%u samples=%d interval=%d profile=%s =====",
             id, delta, speed, acc, base, goal, samples, interval_ms,
             addrs ? addrs->name : "unknown");

    esp_err_t err = write_goal_for_profile(id, profile, (int16_t)goal, speed, acc);
    ESP_LOGI(TAG, "  probe write: %s", esp_err_to_name(err));
    log_goal_latch("LATCH", id, profile);
    trace_motion("probe", id, profile, samples, interval_ms);

    err = write_goal_for_profile(id, profile, (int16_t)base, speed, acc);
    ESP_LOGI(TAG, "  probe restore: %s", esp_err_to_name(err));
    log_goal_latch("RESTORE_LATCH", id, profile);
    trace_motion("restore", id, profile, samples, interval_ms);
}

static void run_probe_with_trace(uint8_t id, int delta, uint16_t speed, uint8_t acc)
{
    run_probe_with_trace_ex(id, delta, speed, acc, 8, 200);
}

static void step_characterize_motor(uint8_t id, int preferred_sign)
{
    const int delta_cases[] = {
        40 * preferred_sign,
        80 * preferred_sign,
        -40 * preferred_sign,
        -80 * preferred_sign,
    };
    const uint16_t speed_cases[] = { 80, 200, 400 };

    ESP_LOGI(TAG, "===== CHARACTERIZE ID=%u preferred_sign=%+d =====", id, preferred_sign);

    for (int i = 0; i < (int)(sizeof(delta_cases) / sizeof(delta_cases[0])); i++) {
        run_probe_with_trace(id, delta_cases[i], 200, 20);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    for (int i = 0; i < (int)(sizeof(speed_cases) / sizeof(speed_cases[0])); i++) {
        run_probe_with_trace(id, 80 * preferred_sign, speed_cases[i], 20);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static void step_speed_profile_free_direction(uint8_t id, int free_sign)
{
    const uint16_t speed_cases[] = { 80, 200, 400 };

    ESP_LOGI(TAG, "===== FREE-DIR SPEED PROFILE ID=%u free_sign=%+d =====", id, free_sign);
    for (int i = 0; i < (int)(sizeof(speed_cases) / sizeof(speed_cases[0])); i++) {
        run_probe_with_trace_ex(id, 80 * free_sign, speed_cases[i], 20, 20, 50);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static int step_free_direction_sweep(uint8_t id, int free_sign)
{
    const int abs_deltas[] = { 40, 80, 120, 160, 200, 240, 280, 320 };
    const uint16_t speed = 200;
    const uint8_t acc = 20;
    const int settle_ms = 700;
    const int reach_tolerance = 6;
    motor_profile_t profile = profile_for_id(id);
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint16_t base = 0;
    int max_good_abs_delta = 0;

    if (!read_present_position(id, profile, &base)) {
        ESP_LOGW(TAG, "FREE SWEEP ID=%u read base failed", id);
        return 0;
    }

    ESP_LOGI(TAG,
             "===== FREE SWEEP ID=%u free_sign=%+d base=%u profile=%s =====",
             id, free_sign, base, addrs ? addrs->name : "unknown");

    for (int i = 0; i < (int)(sizeof(abs_deltas) / sizeof(abs_deltas[0])); i++) {
        const int abs_delta = abs_deltas[i];
        const int delta = abs_delta * free_sign;
        const uint16_t goal = clamp_goal(base, delta);
        motor_feedback_t fb = {0};
        uint8_t err_bits = 0;
        esp_err_t err = write_goal_for_profile(id, profile, (int16_t)goal, speed, acc);

        if (goal == base) {
            ESP_LOGI(TAG, "  FREE SWEEP ID=%u abs_delta=%d skipped: goal clamped to base", id, abs_delta);
            break;
        }

        ESP_LOGI(TAG, "  FREE SWEEP ID=%u step abs_delta=%d delta=%+d goal=%u write=%s",
                 id, abs_delta, delta, goal, esp_err_to_name(err));
        if (err != ESP_OK) {
            break;
        }

        log_goal_latch("SWEEP_LATCH", id, profile);
        vTaskDelay(pdMS_TO_TICKS(settle_ms));

        if (!read_feedback_for_profile(id, profile, &fb, &err_bits)) {
            ESP_LOGW(TAG, "  FREE SWEEP ID=%u abs_delta=%d readback failed", id, abs_delta);
            break;
        }

        const int pos_err = abs_diff_u16(fb.position, goal);
        const bool reached = pos_err <= reach_tolerance;
        const bool overload = (err_bits & 0x20) != 0;
        ESP_LOGI(TAG,
                 "  FREE SWEEP ID=%u abs_delta=%d goal=%u pos=%u pos_err=%d speed_raw=%u speed_s16=%d load_raw=%u load10=%u dir10=%u load11=%u dir11=%u err=0x%02X reached=%u overload=%u",
                 id, abs_delta, goal, fb.position, pos_err, fb.speed, speed_raw_to_signed(fb.speed),
                 fb.load, load_mag_10bit(fb.load), load_dir_10bit(fb.load),
                 load_mag_11bit(fb.load), load_dir_11bit(fb.load),
                 err_bits, reached ? 1 : 0, overload ? 1 : 0);

        if (reached && !overload) {
            max_good_abs_delta = abs_delta;
            continue;
        }

        ESP_LOGW(TAG,
                 "  FREE SWEEP ID=%u stop at abs_delta=%d (reached=%u overload=%u)",
                 id, abs_delta, reached ? 1 : 0, overload ? 1 : 0);
        break;
    }

    {
        esp_err_t err = write_goal_for_profile(id, profile, (int16_t)base, speed, acc);
        motor_feedback_t fb = {0};
        uint8_t err_bits = 0;
        ESP_LOGI(TAG, "  FREE SWEEP ID=%u restore base=%u write=%s",
                 id, base, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(900));
        if (read_feedback_for_profile(id, profile, &fb, &err_bits)) {
            ESP_LOGI(TAG,
                     "  FREE SWEEP ID=%u restore pos=%u speed_raw=%u speed_s16=%d load_raw=%u load10=%u dir10=%u load11=%u dir11=%u err=0x%02X",
                     id, fb.position, fb.speed, speed_raw_to_signed(fb.speed),
                     fb.load, load_mag_10bit(fb.load), load_dir_10bit(fb.load),
                     load_mag_11bit(fb.load), load_dir_11bit(fb.load), err_bits);
        }
    }

    ESP_LOGI(TAG, "===== FREE SWEEP RESULT ID=%u max_good_abs_delta=%d =====",
             id, max_good_abs_delta);
    return max_good_abs_delta;
}

static void step_speed_profile_large_stroke(uint8_t id, int free_sign, int max_good_abs_delta)
{
    const uint16_t speed_cases[] = { 80, 200, 400 };
    int test_abs_delta = max_good_abs_delta;

    if (test_abs_delta > 160) {
        test_abs_delta = 160;
    }
    if (test_abs_delta < 120) {
        ESP_LOGW(TAG,
                 "skip large-stroke speed profile ID=%u: max_good_abs_delta=%d too small",
                 id, max_good_abs_delta);
        return;
    }

    ESP_LOGI(TAG,
             "===== LARGE-STROKE SPEED PROFILE ID=%u free_sign=%+d delta=%d =====",
             id, free_sign, test_abs_delta);
    for (int i = 0; i < (int)(sizeof(speed_cases) / sizeof(speed_cases[0])); i++) {
        run_probe_with_trace_ex(id, test_abs_delta * free_sign, speed_cases[i], 20, 24, 50);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static int step_boundary_fine_sweep(uint8_t id, int free_sign)
{
    const int abs_deltas[] = { 90, 100, 110, 120, 130 };
    const uint16_t speed = 200;
    const uint8_t acc = 20;
    const int settle_ms = 900;
    const int reach_tolerance = 6;
    motor_profile_t profile = profile_for_id(id);
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint16_t base = 0;
    int last_good_abs_delta = 0;
    int first_fail_abs_delta = 0;

    if (!read_present_position(id, profile, &base)) {
        ESP_LOGW(TAG, "BOUNDARY FINE SWEEP ID=%u read base failed", id);
        return 0;
    }

    ESP_LOGI(TAG,
             "===== BOUNDARY FINE SWEEP ID=%u free_sign=%+d base=%u profile=%s =====",
             id, free_sign, base, addrs ? addrs->name : "unknown");

    for (int i = 0; i < (int)(sizeof(abs_deltas) / sizeof(abs_deltas[0])); i++) {
        const int abs_delta = abs_deltas[i];
        const int delta = abs_delta * free_sign;
        const uint16_t goal = clamp_goal(base, delta);
        motor_feedback_t fb = {0};
        uint8_t err_bits = 0;
        esp_err_t err = write_goal_for_profile(id, profile, (int16_t)goal, speed, acc);

        if (goal == base) {
            break;
        }

        ESP_LOGI(TAG, "  BOUNDARY FINE ID=%u step abs_delta=%d delta=%+d goal=%u write=%s",
                 id, abs_delta, delta, goal, esp_err_to_name(err));
        if (err != ESP_OK) {
            first_fail_abs_delta = abs_delta;
            break;
        }

        log_goal_latch("BOUNDARY_LATCH", id, profile);
        vTaskDelay(pdMS_TO_TICKS(settle_ms));

        if (!read_feedback_for_profile(id, profile, &fb, &err_bits)) {
            first_fail_abs_delta = abs_delta;
            ESP_LOGW(TAG, "  BOUNDARY FINE ID=%u abs_delta=%d readback failed", id, abs_delta);
            break;
        }

        {
            const int pos_err = abs_diff_u16(fb.position, goal);
            const bool reached = pos_err <= reach_tolerance;
            const bool overload = (err_bits & 0x20) != 0;
            ESP_LOGI(TAG,
                     "  BOUNDARY FINE ID=%u abs_delta=%d goal=%u pos=%u pos_err=%d speed_raw=%u speed_s16=%d load_raw=%u load10=%u dir10=%u load11=%u dir11=%u err=0x%02X reached=%u overload=%u",
                     id, abs_delta, goal, fb.position, pos_err, fb.speed, speed_raw_to_signed(fb.speed),
                     fb.load, load_mag_10bit(fb.load), load_dir_10bit(fb.load),
                     load_mag_11bit(fb.load), load_dir_11bit(fb.load),
                     err_bits, reached ? 1 : 0, overload ? 1 : 0);
            if (reached && !overload) {
                last_good_abs_delta = abs_delta;
            } else {
                first_fail_abs_delta = abs_delta;
                break;
            }
        }
    }

    {
        esp_err_t err = write_goal_for_profile(id, profile, (int16_t)base, speed, acc);
        ESP_LOGI(TAG, "  BOUNDARY FINE ID=%u restore base=%u write=%s",
                 id, base, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(900));
    }

    ESP_LOGI(TAG,
             "===== BOUNDARY FINE RESULT ID=%u last_good_abs_delta=%d first_fail_abs_delta=%d =====",
             id, last_good_abs_delta, first_fail_abs_delta);
    return first_fail_abs_delta ? first_fail_abs_delta : last_good_abs_delta;
}

static void step_boundary_trace(uint8_t id, int free_sign, int abs_delta)
{
    if (abs_delta <= 0) {
        return;
    }

    ESP_LOGI(TAG,
             "===== BOUNDARY TRACE ID=%u free_sign=%+d abs_delta=%d =====",
             id, free_sign, abs_delta);
    run_probe_with_trace_ex(id, abs_delta * free_sign, 80, 20, 24, 100);
    vTaskDelay(pdMS_TO_TICKS(300));
    run_probe_with_trace_ex(id, abs_delta * free_sign, 200, 20, 24, 100);
    vTaskDelay(pdMS_TO_TICKS(300));
    run_probe_with_trace_ex(id, abs_delta * free_sign, 400, 20, 24, 100);
}

static int step_precision_boundary_sweep(uint8_t id, int free_sign,
                                         int start_abs_delta, int end_abs_delta, int step_abs)
{
    const uint16_t speed = 200;
    const uint8_t acc = 20;
    const int settle_ms = 1200;
    const int reach_tolerance = 6;
    motor_profile_t profile = profile_for_id(id);
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint16_t base = 0;
    int last_good_abs_delta = 0;
    int first_fail_abs_delta = 0;

    if (step_abs <= 0) {
        return 0;
    }
    if (!read_present_position(id, profile, &base)) {
        ESP_LOGW(TAG, "PRECISION SWEEP ID=%u read base failed", id);
        return 0;
    }

    ESP_LOGI(TAG,
             "===== PRECISION SWEEP ID=%u free_sign=%+d base=%u range=%d..%d step=%d profile=%s =====",
             id, free_sign, base, start_abs_delta, end_abs_delta, step_abs,
             addrs ? addrs->name : "unknown");

    for (int abs_delta = start_abs_delta; abs_delta <= end_abs_delta; abs_delta += step_abs) {
        const int delta = abs_delta * free_sign;
        const uint16_t goal = clamp_goal(base, delta);
        motor_feedback_t fb = {0};
        uint8_t err_bits = 0;
        esp_err_t err = write_goal_for_profile(id, profile, (int16_t)goal, speed, acc);

        if (goal == base) {
            break;
        }

        ESP_LOGI(TAG, "  PRECISION ID=%u step abs_delta=%d delta=%+d goal=%u write=%s",
                 id, abs_delta, delta, goal, esp_err_to_name(err));
        if (err != ESP_OK) {
            first_fail_abs_delta = abs_delta;
            break;
        }

        log_goal_latch("PRECISION_LATCH", id, profile);
        vTaskDelay(pdMS_TO_TICKS(settle_ms));

        if (!read_feedback_for_profile(id, profile, &fb, &err_bits)) {
            first_fail_abs_delta = abs_delta;
            ESP_LOGW(TAG, "  PRECISION ID=%u abs_delta=%d readback failed", id, abs_delta);
            break;
        }

        {
            const int pos_err = abs_diff_u16(fb.position, goal);
            const bool reached = pos_err <= reach_tolerance;
            const bool overload = (err_bits & 0x20) != 0;
            ESP_LOGI(TAG,
                     "  PRECISION ID=%u abs_delta=%d goal=%u pos=%u pos_err=%d speed_raw=%u speed_s16=%d load_raw=%u load10=%u dir10=%u load11=%u dir11=%u err=0x%02X reached=%u overload=%u",
                     id, abs_delta, goal, fb.position, pos_err, fb.speed, speed_raw_to_signed(fb.speed),
                     fb.load, load_mag_10bit(fb.load), load_dir_10bit(fb.load),
                     load_mag_11bit(fb.load), load_dir_11bit(fb.load),
                     err_bits, reached ? 1 : 0, overload ? 1 : 0);
            if (reached && !overload) {
                last_good_abs_delta = abs_delta;
            } else {
                first_fail_abs_delta = abs_delta;
                break;
            }
        }
    }

    {
        esp_err_t err = write_goal_for_profile(id, profile, (int16_t)base, speed, acc);
        ESP_LOGI(TAG, "  PRECISION ID=%u restore base=%u write=%s",
                 id, base, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(900));
    }

    ESP_LOGI(TAG,
             "===== PRECISION RESULT ID=%u last_good_abs_delta=%d first_fail_abs_delta=%d =====",
             id, last_good_abs_delta, first_fail_abs_delta);
    return first_fail_abs_delta ? first_fail_abs_delta : last_good_abs_delta;
}

// ============================================================
// 测试主流程入口
//
// 根据编译宏选择运行模式：
//   - 完整诊断模式：4 步全面测试
//   - 简单演示模式：快速往复运动验证
// ============================================================
void motor_test_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor bus task");
    ESP_LOGI(TAG, "  IDs: %u, %u", TARGET_IDS[0], TARGET_IDS[1]);
    ESP_LOGI(TAG, "  Baud: 1000000");
    ESP_LOGI(TAG, "  Diagnostic on boot: %s", MOTOR_RUN_FULL_DIAGNOSTIC ? "ON" : "OFF");
    ESP_LOGI(TAG, "======================================");

    // 初始化电机总线（UART2, 1Mbps）
    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // 等待舵机上电稳定
    ESP_LOGI(TAG, "  Attach window: waiting 8000 ms before diagnostics");
    vTaskDelay(pdMS_TO_TICKS(8000));

#if !MOTOR_RUN_FULL_DIAGNOSTIC
    // ---- 非诊断模式：运行简单演示后进入空闲 ----
#if MOTOR_RUN_SIMPLE_DEMO
    run_simple_demo();
#endif
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
#else
    // ---- 完整诊断模式 ----

    // Step 1: PING 探测 — 检查所有舵机是否在线
    step_ping_all();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Step 2: 逐字节寄存器扫描 — 最重要的诊断步骤
    // 生成完整寄存器映射表，自动识别舵机型号
    for (int i = 0; i < TARGET_COUNT; i++) {
        step_register_scan(TARGET_IDS[i]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Step 3: 多字节 READ 测试 — 与单字节扫描结果对比验证
    for (int i = 0; i < TARGET_COUNT; i++) {
        step_read_multi(TARGET_IDS[i]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

#if MOTOR_RUN_FOCUSED_BOUNDARY
    ESP_LOGI(TAG, "===== FOCUSED BOUNDARY MODE =====");

    step_torque_all(0);
    vTaskDelay(pdMS_TO_TICKS(500));
    step_torque_all(1);
    vTaskDelay(pdMS_TO_TICKS(500));

    {
        int precise_12 = step_precision_boundary_sweep(TARGET_IDS[0], -1, 100, 110, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        int precise_22 = step_precision_boundary_sweep(TARGET_IDS[1], +1, 110, 120, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        step_boundary_trace(TARGET_IDS[0], -1, precise_12);
        vTaskDelay(pdMS_TO_TICKS(500));
        step_boundary_trace(TARGET_IDS[1], +1, precise_22);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    step_torque_all(0);

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Focused boundary test COMPLETE. Torque OFF.");
    ESP_LOGI(TAG, "======================================");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "--- periodic read ---");
        for (int i = 0; i < TARGET_COUNT; i++) {
            uint8_t id = TARGET_IDS[i];
            motor_profile_t profile = profile_for_id(id);
            const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
            uint8_t pos_addr = addrs ? addrs->present_position_l : SCS_ADDR_PRESENT_POSITION_L;
            scs_status_t st = {0};
            esp_err_t read_err = scs_read(id, pos_addr, 2, &st);
            if (read_err == ESP_OK) {
                print_status("POS", id, &st);
            } else {
                ESP_LOGW(TAG, "POS ID=%u read failed: %s", id, esp_err_to_name(read_err));
            }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }
#endif

    // Step 4: 安全运动测试（仅在确认寄存器布局正确后执行）
    ESP_LOGI(TAG, "===== WRITE tests (safe) =====");

    // 先卸力（确保安全）
    step_torque_all(0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 上力
    step_torque_all(1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 逐个小步位置测试
    step_write_and_verify(TARGET_IDS[0], +40, 200, 20);
    vTaskDelay(pdMS_TO_TICKS(500));
    step_write_and_verify(TARGET_IDS[1], -40, 200, 20);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 对 Legacy/DXL 风格寄存器做一次双电机同步写验证
    step_sync_write_legacy_pair(180);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Step 5: 细化单电机驱动特性
    step_characterize_motor(TARGET_IDS[0], +1);
    vTaskDelay(pdMS_TO_TICKS(500));
    step_characterize_motor(TARGET_IDS[1], -1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Step 6: 针对已观察到的“好使方向”补一轮更高时间分辨率的速度特性测试
    step_speed_profile_free_direction(TARGET_IDS[0], -1);
    vTaskDelay(pdMS_TO_TICKS(500));
    step_speed_profile_free_direction(TARGET_IDS[1], +1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Step 7: 沿自由方向继续扫程，确认机械可用范围与更大步长下的速度特性
    {
        int max_good_12 = step_free_direction_sweep(TARGET_IDS[0], -1);
        vTaskDelay(pdMS_TO_TICKS(500));
        int max_good_22 = step_free_direction_sweep(TARGET_IDS[1], +1);
        vTaskDelay(pdMS_TO_TICKS(500));

        step_speed_profile_large_stroke(TARGET_IDS[0], -1, max_good_12);
        vTaskDelay(pdMS_TO_TICKS(500));
        step_speed_profile_large_stroke(TARGET_IDS[1], +1, max_good_22);
        vTaskDelay(pdMS_TO_TICKS(500));

        if (max_good_12 < 120) {
            int boundary_12 = step_boundary_fine_sweep(TARGET_IDS[0], -1);
            vTaskDelay(pdMS_TO_TICKS(500));
            step_boundary_trace(TARGET_IDS[0], -1, boundary_12);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (max_good_22 < 120) {
            int boundary_22 = step_boundary_fine_sweep(TARGET_IDS[1], +1);
            vTaskDelay(pdMS_TO_TICKS(500));
            step_boundary_trace(TARGET_IDS[1], +1, boundary_22);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // 卸力结束
    step_torque_all(0);

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Diagnostic test COMPLETE. Torque OFF.");
    ESP_LOGI(TAG, "======================================");

    // 持续监控：每 10 秒读取一次所有电机的当前位置
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "--- periodic read ---");
        for (int i = 0; i < TARGET_COUNT; i++) {
            uint8_t id = TARGET_IDS[i];
            motor_profile_t profile = profile_for_id(id);
            const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
            uint8_t pos_addr = addrs ? addrs->present_position_l : SCS_ADDR_PRESENT_POSITION_L;
            scs_status_t st = {0};
            esp_err_t r = scs_read(id, pos_addr, 2, &st);
            if (r == ESP_OK) {
                print_status("POS", id, &st);
            }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }
#endif
}

static void log_read_u8(uint8_t id, uint8_t addr, const char *label)
{
    uint8_t value = 0;
    esp_err_t err = scs_read_byte(id, addr, &value);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "EXP ID=%u %-18s addr=0x%02X -> 0x%02X (%u)", id, label, addr, value, value);
    } else {
        ESP_LOGW(TAG, "EXP ID=%u %-18s addr=0x%02X read failed: %s",
                 id, label, addr, esp_err_to_name(err));
    }
}

static void log_read_u16(uint8_t id, uint8_t addr, const char *label)
{
    uint16_t value = 0;
    esp_err_t err = scs_read_word(id, addr, &value);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "EXP ID=%u %-18s addr=0x%02X -> 0x%04X (%u)", id, label, addr, value, value);
    } else {
        ESP_LOGW(TAG, "EXP ID=%u %-18s addr=0x%02X read failed: %s",
                 id, label, addr, esp_err_to_name(err));
    }
}

static void explore_read_candidates(uint8_t id)
{
    ESP_LOGI(TAG, "===== LOW-RISK READ EXPLORATION ID=%u =====", id);
    log_read_u8(id, SCS_ADDR_ID, "ID");
    log_read_u8(id, SCS_ADDR_BAUD_RATE, "BaudCode");
    log_read_u8(id, SCS_ADDR_RETURN_DELAY, "ReturnDelay");
    log_read_u16(id, SCS_ADDR_MIN_ANGLE_L, "MinAngle");
    log_read_u16(id, SCS_ADDR_MAX_ANGLE_L, "MaxAngle");
    log_read_u8(id, 0x0B, "TempLimit");
    log_read_u8(id, 0x0C, "MinVoltRaw");
    log_read_u8(id, 0x0D, "MaxVoltRaw");
    log_read_u16(id, 0x0E, "MaxTorque");
    log_read_u8(id, 0x10, "StatusRetLvl");
    log_read_u8(id, 0x11, "AlarmLED");
    log_read_u8(id, 0x12, "Shutdown");
    log_read_u8(id, 0x19, "LED");
    log_read_u8(id, 0x1A, "CWMargin");
    log_read_u8(id, 0x1B, "CCWMargin");
    log_read_u8(id, 0x1C, "CWSlope");
    log_read_u8(id, 0x1D, "CCWSlope");
    log_read_u16(id, 0x22, "TorqueLimit");
    log_read_u16(id, 0x2C, "CurrentLike");
    log_read_u8(id, 0x2E, "Moving");
    log_read_u8(id, 0x2F, "Lock");
    log_read_u16(id, 0x30, "PunchLike");
}

static bool read_byte_expect(uint8_t id, uint8_t addr, uint8_t *out, const char *label)
{
    esp_err_t err = scs_read_byte(id, addr, out);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "EXP ID=%u %s read failed: %s", id, label, esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "EXP ID=%u %s -> 0x%02X (%u)", id, label, *out, *out);
    return true;
}

static void explore_led_write(uint8_t id)
{
    uint8_t led_before = 0;
    uint8_t led_after = 0;

    ESP_LOGI(TAG, "===== LOW-RISK WRITE EXPLORATION ID=%u LED direct write =====", id);
    if (!read_byte_expect(id, 0x19, &led_before, "LED before")) {
        return;
    }

    ESP_LOGI(TAG, "EXP ID=%u LED direct write -> 1", id);
    if (scs_write_byte(id, 0x19, 1) != ESP_OK) {
        ESP_LOGW(TAG, "EXP ID=%u LED write(1) failed", id);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(EXPLORATION_SETTLE_MS));
    (void)read_byte_expect(id, 0x19, &led_after, "LED after write=1");

    ESP_LOGI(TAG, "EXP ID=%u LED direct write restore -> %u", id, led_before);
    if (scs_write_byte(id, 0x19, led_before) != ESP_OK) {
        ESP_LOGW(TAG, "EXP ID=%u LED restore failed", id);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(EXPLORATION_SETTLE_MS));
    (void)read_byte_expect(id, 0x19, &led_after, "LED after restore");
}

static void explore_reg_write_action_pair(void)
{
    uint8_t led_before[TARGET_COUNT] = {0};
    uint8_t led_after_reg[TARGET_COUNT] = {0};
    uint8_t led_after_action[TARGET_COUNT] = {0};
    uint8_t reg_before[TARGET_COUNT] = {0};
    uint8_t reg_after_reg[TARGET_COUNT] = {0};
    uint8_t reg_after_action[TARGET_COUNT] = {0};

    ESP_LOGI(TAG, "===== LOW-RISK PROTOCOL EXPLORATION REG_WRITE + ACTION =====");
    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        (void)read_byte_expect(id, 0x19, &led_before[i], "LED baseline");
        (void)read_byte_expect(id, 0x2C, &reg_before[i], "Registered baseline");
    }

    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        uint8_t params[2] = { 0x19, 1 };
        scs_status_t st = {0};
        esp_err_t err = scs_txrx(id, SCS_INST_REG_WRITE, params, sizeof(params), &st, 30);
        ESP_LOGI(TAG, "EXP ID=%u REG_WRITE LED=1 -> %s err_bits=0x%02X",
                 id, esp_err_to_name(err), (err == ESP_OK) ? st.error : 0xFF);
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        (void)read_byte_expect(id, 0x19, &led_after_reg[i], "LED after REG_WRITE");
        (void)read_byte_expect(id, 0x2C, &reg_after_reg[i], "Registered after REG_WRITE");
    }

    {
        esp_err_t err = scs_txrx(SCS_BROADCAST_ID, SCS_INST_ACTION, NULL, 0, NULL, 30);
        ESP_LOGI(TAG, "EXP ACTION broadcast -> %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(EXPLORATION_SETTLE_MS));

    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        (void)read_byte_expect(id, 0x19, &led_after_action[i], "LED after ACTION");
        (void)read_byte_expect(id, 0x2C, &reg_after_action[i], "Registered after ACTION");
    }

    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        ESP_LOGI(TAG,
                 "EXP SUMMARY ID=%u regwrite_action led:%u->%u->%u registered:%u->%u->%u",
                 id,
                 led_before[i], led_after_reg[i], led_after_action[i],
                 reg_before[i], reg_after_reg[i], reg_after_action[i]);
        (void)scs_write_byte(id, 0x19, led_before[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(EXPLORATION_SETTLE_MS));
}

static void explore_torque_limit_write(uint8_t id)
{
    uint16_t baseline = 0;
    uint16_t restored = 0;
    uint16_t after_test = 0;
    uint16_t test_value = 0x0200;

    ESP_LOGI(TAG, "===== MEDIUM-RISK WRITE EXPLORATION ID=%u TorqueLimit =====", id);
    if (scs_read_word(id, 0x22, &baseline) != ESP_OK) {
        ESP_LOGW(TAG, "EXP ID=%u TorqueLimit baseline read failed", id);
        return;
    }
    if (baseline == test_value) {
        test_value = 0x0180;
    }
    ESP_LOGI(TAG, "EXP ID=%u TorqueLimit baseline=%u test_value=%u", id, baseline, test_value);

    if (scs_write_word(id, 0x22, test_value) != ESP_OK) {
        ESP_LOGW(TAG, "EXP ID=%u TorqueLimit write(%u) failed", id, test_value);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(EXPLORATION_SETTLE_MS));
    if (scs_read_word(id, 0x22, &after_test) == ESP_OK) {
        ESP_LOGI(TAG, "EXP ID=%u TorqueLimit after write -> %u", id, after_test);
    } else {
        ESP_LOGW(TAG, "EXP ID=%u TorqueLimit read after write failed", id);
    }

    if (scs_write_word(id, 0x22, baseline) != ESP_OK) {
        ESP_LOGW(TAG, "EXP ID=%u TorqueLimit restore(%u) failed", id, baseline);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(EXPLORATION_SETTLE_MS));
    if (scs_read_word(id, 0x22, &restored) == ESP_OK) {
        ESP_LOGI(TAG, "EXP ID=%u TorqueLimit after restore -> %u", id, restored);
    } else {
        ESP_LOGW(TAG, "EXP ID=%u TorqueLimit read after restore failed", id);
    }
}

static void explore_compliance_write(uint8_t id)
{
    uint8_t baseline_margin_cw = 0;
    uint8_t baseline_margin_ccw = 0;
    uint8_t baseline_slope_cw = 0;
    uint8_t baseline_slope_ccw = 0;
    uint8_t readback = 0;

    ESP_LOGI(TAG, "===== MEDIUM-RISK WRITE EXPLORATION ID=%u Compliance =====", id);
    if (!read_byte_expect(id, 0x1A, &baseline_margin_cw, "CWMargin baseline") ||
        !read_byte_expect(id, 0x1B, &baseline_margin_ccw, "CCWMargin baseline") ||
        !read_byte_expect(id, 0x1C, &baseline_slope_cw, "CWSlope baseline") ||
        !read_byte_expect(id, 0x1D, &baseline_slope_ccw, "CCWSlope baseline")) {
        return;
    }

    (void)scs_write_byte(id, 0x1A, 2);
    (void)scs_write_byte(id, 0x1B, 2);
    (void)scs_write_byte(id, 0x1C, 16);
    (void)scs_write_byte(id, 0x1D, 16);
    vTaskDelay(pdMS_TO_TICKS(EXPLORATION_SETTLE_MS));
    (void)read_byte_expect(id, 0x1A, &readback, "CWMargin after write");
    (void)read_byte_expect(id, 0x1B, &readback, "CCWMargin after write");
    (void)read_byte_expect(id, 0x1C, &readback, "CWSlope after write");
    (void)read_byte_expect(id, 0x1D, &readback, "CCWSlope after write");

    (void)scs_write_byte(id, 0x1A, baseline_margin_cw);
    (void)scs_write_byte(id, 0x1B, baseline_margin_ccw);
    (void)scs_write_byte(id, 0x1C, baseline_slope_cw);
    (void)scs_write_byte(id, 0x1D, baseline_slope_ccw);
    vTaskDelay(pdMS_TO_TICKS(EXPLORATION_SETTLE_MS));
    (void)read_byte_expect(id, 0x1A, &readback, "CWMargin restored");
    (void)read_byte_expect(id, 0x1B, &readback, "CCWMargin restored");
    (void)read_byte_expect(id, 0x1C, &readback, "CWSlope restored");
    (void)read_byte_expect(id, 0x1D, &readback, "CCWSlope restored");
}

void motor_explore_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor low-risk exploration task");
    ESP_LOGI(TAG, "  IDs: %u, %u", TARGET_IDS[0], TARGET_IDS[1]);
    ESP_LOGI(TAG, "======================================");

    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1500));

    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        scs_status_t st = {0};
        err = scs_ping(id, &st);
        ESP_LOGI(TAG, "EXP PING ID=%u -> %s err_bits=0x%02X",
                 id, esp_err_to_name(err), (err == ESP_OK) ? st.error : 0xFF);
        vTaskDelay(pdMS_TO_TICKS(80));
        explore_read_candidates(id);
        vTaskDelay(pdMS_TO_TICKS(120));
        explore_led_write(id);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    explore_reg_write_action_pair();
    vTaskDelay(pdMS_TO_TICKS(120));

    for (int i = 0; i < TARGET_COUNT; i++) {
        explore_torque_limit_write(TARGET_IDS[i]);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    for (int i = 0; i < TARGET_COUNT; i++) {
        explore_compliance_write(TARGET_IDS[i]);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor low-risk exploration COMPLETE");
    ESP_LOGI(TAG, "======================================");
    vTaskDelete(NULL);
}

void motor_identify_four_nodes_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor four-node identify task");
    ESP_LOGI(TAG, "  IDs: 11, 12, 21, 22");
    ESP_LOGI(TAG, "  Policy: servo safe-open first, then read-only identify");
    ESP_LOGI(TAG, "======================================");

    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1200));
    move_servo_pair_to_safe_open_20();
    vTaskDelay(pdMS_TO_TICKS(400));

    for (int i = 0; i < IDENTIFY_COUNT; i++) {
        uint8_t id = IDENTIFY_IDS[i];
        scs_status_t st = {0};
        motor_profile_t profile = MOTOR_PROFILE_UNKNOWN;

        ESP_LOGI(TAG, "===== IDENTIFY NODE ID=%u =====", id);
        err = scs_ping(id, &st);
        if (err == ESP_OK) {
            print_status("IDENTIFY_PING", id, &st);
        } else {
            ESP_LOGW(TAG, "IDENTIFY_PING ID=%u failed: %s", id, esp_err_to_name(err));
            continue;
        }

        profile = step_register_scan(id);
        vTaskDelay(pdMS_TO_TICKS(200));
        step_read_multi_with_profile(id, profile);
        vTaskDelay(pdMS_TO_TICKS(120));
        log_read_u8(id, 33, "Mode");
        log_read_u16(id, 69, "CurrentLike");
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor four-node identify COMPLETE");
    ESP_LOGI(TAG, "======================================");
    vTaskDelete(NULL);
}

void motor_probe_wheels_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel probe task");
    ESP_LOGI(TAG, "  Safety: open 12/22 to 20%% first");
    ESP_LOGI(TAG, "  Targets: 11, 21");
    ESP_LOGI(TAG, "======================================");

    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1200));
    move_servo_pair_to_safe_open_20();
    vTaskDelay(pdMS_TO_TICKS(400));

    for (int i = 0; i < WHEEL_COUNT; i++) {
        uint8_t id = WHEEL_IDS[i];
        scs_status_t st = {0};
        motor_profile_t profile = MOTOR_PROFILE_UNKNOWN;

        ESP_LOGI(TAG, "===== WHEEL IDENTIFY ID=%u =====", id);
        err = scs_ping(id, &st);
        if (err == ESP_OK) {
            print_status("WHEEL_PING", id, &st);
        } else {
            ESP_LOGW(TAG, "WHEEL_PING ID=%u failed: %s", id, esp_err_to_name(err));
            continue;
        }

        profile = step_register_scan(id);
        vTaskDelay(pdMS_TO_TICKS(150));
        step_read_multi_with_profile(id, profile);
        vTaskDelay(pdMS_TO_TICKS(150));
        wheel_torque_probe(id, profile);
        vTaskDelay(pdMS_TO_TICKS(200));
        wheel_position_probe(id, profile, +16);
        vTaskDelay(pdMS_TO_TICKS(250));
        wheel_position_probe(id, profile, -16);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel probe COMPLETE");
    ESP_LOGI(TAG, "======================================");
    vTaskDelete(NULL);
}

void motor_probe_wheel_drive_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel drive semantics task");
    ESP_LOGI(TAG, "  Safety: open 12/22 to 20%% first");
    ESP_LOGI(TAG, "  Targets: 11, 21");
    ESP_LOGI(TAG, "  Experiments: hold / speed-only / goal+speed");
    ESP_LOGI(TAG, "======================================");

    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1200));
    move_servo_pair_to_safe_open_20();
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < WHEEL_COUNT; i++) {
        uint8_t id = WHEEL_IDS[i];
        scs_status_t st = {0};
        motor_profile_t profile = MOTOR_PROFILE_UNKNOWN;

        ESP_LOGI(TAG, "===== WHEEL DRIVE IDENTIFY ID=%u =====", id);
        err = scs_ping(id, &st);
        if (err == ESP_OK) {
            print_status("WHEEL_DRIVE_PING", id, &st);
        } else {
            ESP_LOGW(TAG, "WHEEL_DRIVE_PING ID=%u failed: %s", id, esp_err_to_name(err));
            continue;
        }

        profile = step_register_scan(id);
        vTaskDelay(pdMS_TO_TICKS(150));
        step_read_multi_with_profile(id, profile);
        vTaskDelay(pdMS_TO_TICKS(200));
        wheel_drive_semantics_probe(id, profile);
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel drive semantics COMPLETE");
    ESP_LOGI(TAG, "======================================");
    vTaskDelete(NULL);
}

void motor_probe_wheel_direction_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel direction semantics task");
    ESP_LOGI(TAG, "  Safety: open 12/22 to 20%% first");
    ESP_LOGI(TAG, "  Targets: 11, 21");
    ESP_LOGI(TAG, "  Experiments: goal +/-128 / speed dir bit");
    ESP_LOGI(TAG, "======================================");

    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1200));
    move_servo_pair_to_safe_open_20();
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < WHEEL_COUNT; i++) {
        uint8_t id = WHEEL_IDS[i];
        scs_status_t st = {0};
        motor_profile_t profile = MOTOR_PROFILE_UNKNOWN;

        ESP_LOGI(TAG, "===== WHEEL DIRECTION IDENTIFY ID=%u =====", id);
        err = scs_ping(id, &st);
        if (err == ESP_OK) {
            print_status("WHEEL_DIRECTION_PING", id, &st);
        } else {
            ESP_LOGW(TAG, "WHEEL_DIRECTION_PING ID=%u failed: %s", id, esp_err_to_name(err));
            continue;
        }

        profile = step_register_scan(id);
        vTaskDelay(pdMS_TO_TICKS(150));
        step_read_multi_with_profile(id, profile);
        vTaskDelay(pdMS_TO_TICKS(200));
        wheel_direction_focus_probe(id, profile);
        vTaskDelay(pdMS_TO_TICKS(500));

        {
            const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
            if (addrs) {
                ESP_LOGI(TAG, "WHEEL DIRECTION ID=%u torque off -> %s",
                         id, esp_err_to_name(scs_write_byte(id, addrs->torque_enable, 0)));
                vTaskDelay(pdMS_TO_TICKS(120));
                log_feedback_snapshot("WHEEL_DIRECTION_AFTER_TORQUE_OFF", id, profile);
            }
        }
    }

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel direction semantics COMPLETE");
    ESP_LOGI(TAG, "======================================");
    vTaskDelete(NULL);
}

void motor_probe_wheel_control_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel control semantics task");
    ESP_LOGI(TAG, "  Safety: open 12/22 to 20%% first");
    ESP_LOGI(TAG, "  Targets: 11, 21");
    ESP_LOGI(TAG, "  Experiments: speed deadband / stop semantics");
    ESP_LOGI(TAG, "======================================");

    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1200));
    move_servo_pair_to_safe_open_20();
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < WHEEL_COUNT; i++) {
        uint8_t id = WHEEL_IDS[i];
        scs_status_t st = {0};
        motor_profile_t profile = MOTOR_PROFILE_UNKNOWN;

        ESP_LOGI(TAG, "===== WHEEL CONTROL IDENTIFY ID=%u =====", id);
        err = scs_ping(id, &st);
        if (err == ESP_OK) {
            print_status("WHEEL_CONTROL_PING", id, &st);
        } else {
            ESP_LOGW(TAG, "WHEEL_CONTROL_PING ID=%u failed: %s", id, esp_err_to_name(err));
            continue;
        }

        profile = step_register_scan(id);
        vTaskDelay(pdMS_TO_TICKS(150));
        step_read_multi_with_profile(id, profile);
        vTaskDelay(pdMS_TO_TICKS(220));
        wheel_speed_deadband_probe(id, profile);
        vTaskDelay(pdMS_TO_TICKS(400));
        wheel_stop_semantics_probe(id, profile);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel control semantics COMPLETE");
    ESP_LOGI(TAG, "======================================");
    vTaskDelete(NULL);
}

void motor_probe_wheel_midrange_task(void *arg)
{
    static const uint16_t speeds[] = { 20, 25, 30, 40 };
    const uint8_t id = 21;

    (void)arg;

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel midrange follow-up task");
    ESP_LOGI(TAG, "  Safety: open 12/22 to 20%% first");
    ESP_LOGI(TAG, "  Target: 21");
    ESP_LOGI(TAG, "  Experiments: speed 20 / 25 / 30 / 40");
    ESP_LOGI(TAG, "======================================");

    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1200));
    move_servo_pair_to_safe_open_20();
    vTaskDelay(pdMS_TO_TICKS(500));

    scs_status_t st = {0};
    motor_profile_t profile = MOTOR_PROFILE_UNKNOWN;
    err = scs_ping(id, &st);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WHEEL_MIDRANGE_PING ID=%u failed: %s", id, esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    print_status("WHEEL_MIDRANGE_PING", id, &st);
    profile = step_register_scan(id);
    vTaskDelay(pdMS_TO_TICKS(150));
    step_read_multi_with_profile(id, profile);
    vTaskDelay(pdMS_TO_TICKS(220));

    for (size_t i = 0; i < sizeof(speeds) / sizeof(speeds[0]); i++) {
        wheel_hold_probe(id, profile, 10);
        vTaskDelay(pdMS_TO_TICKS(220));
        wheel_speed_only_probe_tagged("WHEEL_MIDRANGE_SWEEP", id, profile, speeds[i], 6, 100);
        vTaskDelay(pdMS_TO_TICKS(260));
    }

    wheel_hold_probe(id, profile, 10);
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Motor wheel midrange follow-up COMPLETE");
    ESP_LOGI(TAG, "======================================");
    vTaskDelete(NULL);
}
