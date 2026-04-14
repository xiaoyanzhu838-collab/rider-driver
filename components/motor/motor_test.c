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
#define MOTOR_RUN_FULL_DIAGNOSTIC 0
#endif

#ifndef MOTOR_RUN_SIMPLE_DEMO
#define MOTOR_RUN_SIMPLE_DEMO 0
#endif

// 需要测试的舵机 ID 列表
static const uint8_t TARGET_IDS[] = { 11, 12 };
#define TARGET_COUNT (sizeof(TARGET_IDS) / sizeof(TARGET_IDS[0]))

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
static void step_register_scan(uint8_t id)
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
}

// ============================================================
// Step 3: 多字节 READ 测试
//
// 与 Step 2 的逐字节扫描结果进行对比验证。
// 使用 READ(id, addr, N) 一次读取 N 个连续字节，
// 验证单字节扫描和多字节读取的结果是否一致。
// ============================================================
static void step_read_multi(uint8_t id)
{
    motor_profile_t profile = profile_for_id(id);
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
static void step_write_and_verify(uint8_t id, uint16_t speed, uint8_t acc)
{
    motor_profile_t profile = profile_for_id(id);
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint8_t pos_addr = addrs ? addrs->present_position_l : SCS_ADDR_PRESENT_POSITION_L;
    uint16_t before = 0;
    bool before_valid = false;

    ESP_LOGI(TAG, "===== WritePos ID=%u speed=%u acc=%u profile=%s =====",
             id, speed, acc, addrs ? addrs->name : "unknown");

    // 读取运动前的位置
    {
        scs_status_t st = {0};
        scs_read(id, pos_addr, 2, &st);
        if (st.param_count >= 2) {
            before = st.params[0] | ((uint16_t)st.params[1] << 8);
            before_valid = true;
            ESP_LOGI(TAG, "  before: pos=%u (0x%04X) from addr=%u", before, before, pos_addr);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    // 计算目标位置：在当前位置基础上偏移 +50
    int16_t goal_pos = before_valid ? (int16_t)(before + 50) : 600;
    if (profile == MOTOR_PROFILE_LEGACY_DXL) {
        // Legacy DXL 位置范围较小（0~1023），需要额外安全检查
        if (goal_pos > 900) {
            goal_pos = (int16_t)(before - 50);
        }
        if (goal_pos < 100) {
            goal_pos = 100;
        }
    }

    ESP_LOGI(TAG, "  target goal=%d", goal_pos);

    // 写入目标位置
    esp_err_t err = write_goal_for_profile(id, profile, goal_pos, speed, acc);
    ESP_LOGI(TAG, "  write_pos: %s", esp_err_to_name(err));

    // 等待运动完成
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 读回运动后的位置，验证是否到达
    {
        scs_status_t st = {0};
        scs_read(id, pos_addr, 2, &st);
        if (st.param_count >= 2) {
            uint16_t after = st.params[0] | ((uint16_t)st.params[1] << 8);
            ESP_LOGI(TAG, "  after:  pos=%u (0x%04X) from addr=%u", after, after, pos_addr);
        }
    }
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
    ESP_LOGI(TAG, "  IDs: 11, 12");
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

    // Step 4: 安全运动测试（仅在确认寄存器布局正确后执行）
    ESP_LOGI(TAG, "===== WRITE tests (safe) =====");

    // 先卸力（确保安全）
    step_torque_all(0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 上力
    step_torque_all(1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 小步位置测试（仅 ID 11）
    step_write_and_verify(TARGET_IDS[0], 200, 20);
    vTaskDelay(pdMS_TO_TICKS(500));

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
