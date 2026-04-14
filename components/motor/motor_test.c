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

#ifndef MOTOR_RUN_FULL_DIAGNOSTIC
#define MOTOR_RUN_FULL_DIAGNOSTIC 0
#endif

#ifndef MOTOR_RUN_SIMPLE_DEMO
#define MOTOR_RUN_SIMPLE_DEMO 1
#endif

// 已确认在线的目标 ID
static const uint8_t TARGET_IDS[] = { 11, 12 };
#define TARGET_COUNT (sizeof(TARGET_IDS) / sizeof(TARGET_IDS[0]))

typedef enum {
    MOTOR_PROFILE_UNKNOWN = 0,
    MOTOR_PROFILE_SCSCL,
    MOTOR_PROFILE_LEGACY_DXL,
} motor_profile_t;

typedef struct {
    uint8_t torque_enable;
    uint8_t goal_position_l;
    uint8_t goal_speed_l;
    uint8_t present_position_l;
    bool    uses_acc;
    uint8_t acc_addr;
    const char *name;
} motor_profile_addrs_t;

typedef struct {
    bool valid;
    uint8_t reg_map[72];
    bool reg_ok[72];
    motor_profile_t profile;
} motor_scan_result_t;

static motor_scan_result_t s_scan_results[TARGET_COUNT];

static uint16_t read_word_from_map(const uint8_t *reg_map, const bool *reg_ok, uint8_t addr_l)
{
    if (!reg_ok[addr_l] || !reg_ok[addr_l + 1]) {
        return 0;
    }
    return reg_map[addr_l] | ((uint16_t)reg_map[addr_l + 1] << 8);
}

static const motor_profile_addrs_t *get_profile_addrs(motor_profile_t profile)
{
    static const motor_profile_addrs_t scscl = {
        .torque_enable = 40,
        .goal_position_l = 42,
        .goal_speed_l = 46,
        .present_position_l = 56,
        .uses_acc = true,
        .acc_addr = 41,
        .name = "SCSCL/STS",
    };
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

static motor_profile_t detect_profile(uint8_t id, const uint8_t *reg_map, const bool *reg_ok)
{
    int scscl_score = 0;
    int legacy_score = 0;

    if (reg_ok[5] && reg_map[5] == id) {
        scscl_score += 4;
    }
    if (reg_ok[40] && reg_map[40] <= 1) {
        scscl_score += 2;
    }
    if (reg_ok[62] && reg_map[62] >= 40 && reg_map[62] <= 140) {
        scscl_score += 2;
    }
    if (reg_ok[56] && reg_ok[57]) {
        uint16_t pos = read_word_from_map(reg_map, reg_ok, 56);
        if (pos <= 4095) {
            scscl_score += 1;
        }
    }

    if (reg_ok[3] && reg_map[3] == id) {
        legacy_score += 4;
    }
    if (reg_ok[24] && reg_map[24] <= 1) {
        legacy_score += 2;
    }
    if (reg_ok[42] && reg_map[42] >= 40 && reg_map[42] <= 140) {
        legacy_score += 1;
    }
    if (reg_ok[30] && reg_ok[31]) {
        uint16_t goal = read_word_from_map(reg_map, reg_ok, 30);
        if (goal <= 1023) {
            legacy_score += 1;
        }
    }
    if (reg_ok[36] && reg_ok[37]) {
        uint16_t pos = read_word_from_map(reg_map, reg_ok, 36);
        if (pos <= 1023) {
            legacy_score += 2;
        }
    }

    ESP_LOGI(TAG, "  profile score: SCSCL=%d legacy=%d", scscl_score, legacy_score);

    if (legacy_score > scscl_score) {
        return MOTOR_PROFILE_LEGACY_DXL;
    }
    if (scscl_score > legacy_score) {
        return MOTOR_PROFILE_SCSCL;
    }
    return MOTOR_PROFILE_UNKNOWN;
}

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

static int target_index_for_id(uint8_t id)
{
    for (int i = 0; i < TARGET_COUNT; i++) {
        if (TARGET_IDS[i] == id) {
            return i;
        }
    }
    return -1;
}

static motor_profile_t profile_for_id(uint8_t id)
{
    int idx = target_index_for_id(id);
    if (idx >= 0 && s_scan_results[idx].valid) {
        return s_scan_results[idx].profile;
    }
    return MOTOR_PROFILE_SCSCL;
}

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

static void run_simple_demo(void)
{
    const uint16_t speed = 120;
    const int deltas[MOTOR_COUNT] = { 60, -60 };

    ESP_LOGI(TAG, "===== SIMPLE MOTOR DEMO =====");

    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        esp_err_t err = motor_set_torque(id, true);
        ESP_LOGI(TAG, "  ID %u torque on: %s", id, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    vTaskDelay(pdMS_TO_TICKS(300));

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

        err = motor_set_position(id, target, speed);
        ESP_LOGI(TAG, "  ID %u move out: %s", id, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1200));

        err = motor_read_feedback(id, &fb);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ID %u mid pos=%u load=%u moving=%u", id, fb.position, fb.load, fb.moving);
        }

        err = motor_set_position(id, start, speed);
        ESP_LOGI(TAG, "  ID %u move back: %s", id, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(1200));

        err = motor_read_feedback(id, &fb);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ID %u final pos=%u load=%u moving=%u", id, fb.position, fb.load, fb.moving);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    for (int i = 0; i < TARGET_COUNT; i++) {
        uint8_t id = TARGET_IDS[i];
        esp_err_t err = motor_set_torque(id, false);
        ESP_LOGI(TAG, "  ID %u torque off: %s", id, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    ESP_LOGI(TAG, "===== SIMPLE MOTOR DEMO DONE =====");
}

// ============================================================
// 辅助：打印 status 包
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
// Step 2: 逐字节寄存器扫描 (核心调试)
//   逐个读取 addr 0~70，每次只读 1 字节
//   输出完整的寄存器 map
// ============================================================
static void step_register_scan(uint8_t id)
{
    ESP_LOGI(TAG, "===== REGISTER SCAN ID=%u (addr 0..70, 1 byte each) =====", id);

    uint8_t reg_map[72];
    bool    reg_ok[72];
    memset(reg_map, 0x00, sizeof(reg_map));
    memset(reg_ok, 0, sizeof(reg_ok));

    int ok_count = 0;
    int fail_count = 0;

    for (uint8_t addr = 0; addr <= 70; addr++) {
        scs_status_t st = {0};
        esp_err_t err = scs_read(id, addr, 1, &st);
        if (err == ESP_OK && st.param_count >= 1) {
            reg_map[addr] = st.params[0];
            reg_ok[addr] = true;
            ok_count++;
        } else {
            fail_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // 间隔，给总线恢复时间
    }

    ESP_LOGI(TAG, "  scan complete: %d OK, %d FAIL", ok_count, fail_count);

    // 格式化打印寄存器表 (每行 16 个)
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
                pos += sprintf(line + pos, "-- ");
            }
        }
        ESP_LOGI(TAG, "%s", line);
    }

    // 打印关键地址的解读
    ESP_LOGI(TAG, "  --- Key register interpretation ---");

    // EPROM 区
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

    // STS 特有
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

    // 反馈区
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

    // 也检查 Dynamixel 地址看是否更像 DXL
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

    print_legacy_interpretation(reg_map, reg_ok);

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
// Step 3: READ 多字节测试（与单字节扫描结果对比）
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

    // 按当前判定 profile 读取 present position/speed/load
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

    // 按当前判定 profile 读取 torque enable
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
// Step 4: WRITE torque + 小步测试
// ============================================================
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

static esp_err_t write_goal_for_profile(uint8_t id, motor_profile_t profile,
                                        int16_t goal_pos, uint16_t speed, uint8_t acc)
{
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    if (!addrs) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (addrs->uses_acc) {
        return scs_write_pos(id, goal_pos, speed, acc);
    }

    esp_err_t err = scs_write_word(id, addrs->goal_position_l, (uint16_t)goal_pos);
    if (err != ESP_OK) {
        return err;
    }
    return scs_write_word(id, addrs->goal_speed_l, speed);
}

static void step_write_and_verify(uint8_t id, uint16_t speed, uint8_t acc)
{
    motor_profile_t profile = profile_for_id(id);
    const motor_profile_addrs_t *addrs = get_profile_addrs(profile);
    uint8_t pos_addr = addrs ? addrs->present_position_l : SCS_ADDR_PRESENT_POSITION_L;
    uint16_t before = 0;
    bool before_valid = false;

    ESP_LOGI(TAG, "===== WritePos ID=%u speed=%u acc=%u profile=%s =====",
             id, speed, acc, addrs ? addrs->name : "unknown");

    // 读取写之前的位置
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

    int16_t goal_pos = before_valid ? (int16_t)(before + 50) : 600;
    if (profile == MOTOR_PROFILE_LEGACY_DXL) {
        if (goal_pos > 900) {
            goal_pos = (int16_t)(before - 50);
        }
        if (goal_pos < 100) {
            goal_pos = 100;
        }
    }

    ESP_LOGI(TAG, "  target goal=%d", goal_pos);

    // 写目标位置
    esp_err_t err = write_goal_for_profile(id, profile, goal_pos, speed, acc);
    ESP_LOGI(TAG, "  write_pos: %s", esp_err_to_name(err));

    // 等待运动
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 读取写之后的位置
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
// 测试主流程
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

    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // 等设备上电稳定

#if !MOTOR_RUN_FULL_DIAGNOSTIC
#if MOTOR_RUN_SIMPLE_DEMO
    run_simple_demo();
#endif
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
#else

    // ---- Step 1: PING ----
    step_ping_all();
    vTaskDelay(pdMS_TO_TICKS(500));

    // ---- Step 2: 逐字节寄存器扫描（最重要的诊断） ----
    for (int i = 0; i < TARGET_COUNT; i++) {
        step_register_scan(TARGET_IDS[i]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // ---- Step 3: 多字节 READ 测试 (对比单字节扫描) ----
    for (int i = 0; i < TARGET_COUNT; i++) {
        step_read_multi(TARGET_IDS[i]);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // ---- Step 4: 安全测试 (仅在确认寄存器正确后启用) ----
    ESP_LOGI(TAG, "===== WRITE tests (safe) =====");

    // 先卸力
    step_torque_all(0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 上力
    step_torque_all(1);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 小步测试 (仅 ID 11)
    step_write_and_verify(TARGET_IDS[0], 200, 20);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 卸力结束
    step_torque_all(0);

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Diagnostic test COMPLETE. Torque OFF.");
    ESP_LOGI(TAG, "======================================");

    // 持续监控
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
