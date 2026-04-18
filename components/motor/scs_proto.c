#include "scs_proto.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "scs";

/* 标记 UART 是否已初始化（防止重复初始化） */
static bool s_scs_initialized = false;
/* 记录当前波特率（初始化后不可更改） */
static int s_scs_baud_rate = 0;
/* 保护 UART2 半双工总线，避免多个任务交错发包/收包 */
static SemaphoreHandle_t s_scs_bus_mutex = NULL;

// ============================================================
// UART 硬件配置
// 使用 ESP32 的 UART2，TX=GPIO14，RX=GPIO13
// 通过 BC807 + SN74 硬件电路自动切换收发方向（半双工）
// ============================================================
#define SCS_UART_NUM    UART_NUM_2
#define SCS_TX_GPIO     GPIO_NUM_14
#define SCS_RX_GPIO     GPIO_NUM_13
#define SCS_RX_BUF_SIZE 256

/*
 * TX 回环清除延时（毫秒）
 * 半双工总线上，主控发出的字节会回环到 RX 端。
 * 发送完成后需等待一段时间，确保所有回环字节到达 RX FIFO，
 * 然后一次性丢弃，才能干净地接收舵机的真实回复。
 */
#define SCS_ECHO_FLUSH_DELAY_MS  3
#define SCS_BUS_LOCK_TIMEOUT_MS  100

// ============================================================
// 调试开关：打印原始收发字节（1=开启，0=关闭）
// 开启后可以在串口日志中看到每一帧的完整十六进制内容，方便排查通信问题
// ============================================================
#ifndef SCS_DEBUG_RAW
#define SCS_DEBUG_RAW 0
#endif

/**
 * @brief 十六进制数据打印辅助函数
 * 将一段原始字节数据格式化为十六进制字符串输出到日志
 * 用于调试通信过程中的实际收发内容
 */
static void hex_dump(const char *prefix, const uint8_t *data, size_t len)
{
#if SCS_DEBUG_RAW
    char hex[3 * 64 + 1];
    size_t dump_len = (len > 64) ? 64 : len;
    for (size_t i = 0; i < dump_len; i++) {
        sprintf(hex + i * 3, "%02X ", data[i]);
    }
    hex[dump_len * 3] = '\0';
    ESP_LOGI(TAG, "%s (%u): %s%s", prefix, (unsigned)len, hex, (len > dump_len) ? "..." : "");
#endif
}

// ============================================================
// 状态包载荷规范化（兼容性处理）
//
// 部分舵机在回复中会额外回显请求指令的内容（即"请求前缀"），
// 导致返回的参数中混入了不属于寄存器数据的多余字节。
// 本函数检测并剥离这些前缀，确保上层始终拿到纯净的寄存器数据。
//
// 观察到的异常模式：
//   PING 回复: [0x02, 0x01, 0x00] — 3 字节请求回显
//   READ 回复: [0x04, 0x02, addr] + data... — 回显 LEN/INST/ADDR 后跟实际数据
//   WRITE 回复: [0x05, 0x03, addr] — 3 字节请求回显
// ============================================================

/**
 * @brief 根据指令类型计算期望的参数数量
 * 目前仅 READ 指令需要校验（期望返回请求读取的字节数）
 */
static uint8_t scs_expected_param_count(uint8_t instruction, const uint8_t *params, size_t param_len)
{
    if (instruction == SCS_INST_READ && param_len >= 2) {
        return params[1];   // READ 的第二个参数就是请求读取的字节数
    }
    return 0;
}

/**
 * @brief 规范化状态包的载荷数据，剥离舵机异常回显的请求前缀
 *
 * 某些舵机固件实现在状态包中回显了请求指令的部分内容，
 * 导致 params[] 中前几个字节是请求回显而非实际的寄存器数据。
 * 此函数检测这些模式并跳过前缀，使 params 始终包含纯净的寄存器值。
 */
static void scs_normalize_status_payload(uint8_t instruction,
                                         const uint8_t *req_params,
                                         size_t req_param_len,
                                         const uint8_t **payload,
                                         uint8_t *param_count)
{
    const uint8_t *p = *payload;
    uint8_t count = *param_count;
    uint8_t expected = scs_expected_param_count(instruction, req_params, req_param_len);

    // --- 处理 PING 回复中的请求回显 ---
    // 异常模式: [0x02, 0x01, 0x00]，共 3 字节，与 PING 请求内容对应
    if (instruction == SCS_INST_PING &&
        count == 3 &&
        p[0] == 0x02 &&
        p[1] == SCS_INST_PING) {
        *payload = p + 3;       // 跳过 3 字节前缀
        *param_count = 0;       // PING 本身不期望返回参数
        return;
    }

    // --- 处理 READ 回复中的请求回显 ---
    // 异常模式: [0x04, 0x02, addr] + 实际数据
    // 其中 0x04 是 LEN，0x02 是 INST(READ)，addr 是读取的起始地址
    if (instruction == SCS_INST_READ &&
        req_param_len >= 1 &&
        count == (uint8_t)(expected + 3) &&  // 期望字节数 + 3 字节前缀
        p[0] == 0x04 &&
        p[1] == SCS_INST_READ &&
        p[2] == req_params[0]) {
        *payload = p + 3;           // 跳过 3 字节前缀
        *param_count = expected;    // 保留期望的字节数
        return;
    }

    // --- READ 回复参数数量不匹配时记录警告 ---
    if (instruction == SCS_INST_READ && count != expected) {
        ESP_LOGW(TAG,
                 "READ reply param count mismatch: expect=%u got=%u (addr=0x%02X)",
                 expected, count, (req_param_len >= 1) ? req_params[0] : 0x00);
    }

    // --- 处理 WRITE 回复中的请求回显 ---
    // 异常模式: [0x05, 0x03, addr]，共 3 字节
    if (instruction == SCS_INST_WRITE &&
        req_param_len >= 1 &&
        count == 3 &&
        p[0] == 0x05 &&
        p[1] == SCS_INST_WRITE &&
        p[2] == req_params[0]) {
        *payload = p + 3;
        *param_count = 0;   // WRITE 不期望返回参数
    }
}

// ============================================================
// 校验和计算
// 算法与 Dynamixel Protocol 1.0 完全一致
//
// 原理：从 ID 字节开始（跳过两个 0xFF 帧头），
//       到最后一个参数字节为止，按字节逐个累加，
//       最后取反并保留低 8 位。
//       即: checksum = ~(ID + LEN + INST + PARAM[0..N]) & 0xFF
// ============================================================
static uint8_t scs_checksum(const uint8_t *pkt, size_t pkt_len)
{
    // pkt 结构: [0xFF, 0xFF, ID, LEN, INST, PARAM0, ..., PARAMn, CHECKSUM]
    // 累加范围: 从索引 2 (ID) 到索引 pkt_len-2 (最后一个参数)
    // 不包含: 帧头(0xFF,0xFF) 和 校验和本身
    uint8_t sum = 0;
    for (size_t i = 2; i < pkt_len - 1; i++) {
        sum += pkt[i];
    }
    return ~sum;
}

// ============================================================
// 初始化 SCS 串行总线
//
// 配置 ESP32 的 UART2 外设：
//   - 8 数据位、无校验、1 停止位（标准 8N1）
//   - 无硬件流控（半双工单线不需要 RTS/CTS）
//   - TX=GPIO14，RX=GPIO13
//   - 接收缓冲区 256 字节
// ============================================================
esp_err_t scs_init(int baud_rate)
{
    if (s_scs_bus_mutex == NULL) {
        s_scs_bus_mutex = xSemaphoreCreateMutex();
        if (s_scs_bus_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    // 防止重复初始化：如果已经初始化且波特率一致，直接返回成功
    if (s_scs_initialized) {
        if (baud_rate == s_scs_baud_rate) {
            return ESP_OK;
        }
        return ESP_ERR_INVALID_STATE;
    }

    // UART 配置：标准 8N1 格式
    uart_config_t uart_cfg = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装 UART 驱动（分配接收缓冲区，不使用发送缓冲区和事件队列）
    ESP_RETURN_ON_ERROR(uart_driver_install(SCS_UART_NUM, SCS_RX_BUF_SIZE, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");
    // 配置 UART 参数（波特率、数据位等）
    ESP_RETURN_ON_ERROR(uart_param_config(SCS_UART_NUM, &uart_cfg),
                        TAG, "uart_param_config failed");
    // 绑定 UART 到 GPIO 引脚
    ESP_RETURN_ON_ERROR(uart_set_pin(SCS_UART_NUM, SCS_TX_GPIO, SCS_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");

    ESP_LOGI(TAG, "SCS bus init OK, UART%d baud=%d TX=GPIO%d RX=GPIO%d",
             SCS_UART_NUM, baud_rate, SCS_TX_GPIO, SCS_RX_GPIO);
    s_scs_initialized = true;
    s_scs_baud_rate = baud_rate;
    return ESP_OK;
}

// ============================================================
// 核心通信函数：发送指令包 + 接收状态包
//
// 【半双工通信时序】
//   1. 组装协议帧 (FF FF ID LEN INST PARAMS... CHECKSUM)
//   2. 清空 UART RX 缓冲区（丢弃之前可能残留的数据）
//   3. 通过 UART TX 发送指令帧
//   4. 等待 TX 完成确保所有字节已发出
//   5. 延时后丢弃 TX 回环字节（半双工总线上发出的数据会回到 RX 端）
//   6. 如果不是广播地址，接收舵机回复的状态包
//      a. 先收 4 字节 (FF FF ID LEN)
//      b. 扫描并定位帧头
//      c. 根据 LEN 字段接收剩余字节
//      d. 校验和验证
//      e. 解析并填充 status 结构体
// ============================================================
esp_err_t scs_txrx(uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t param_len,
                    scs_status_t *status, uint32_t timeout_ms)
{
    bool bus_locked = false;

    // 懒初始化：首次调用时自动初始化总线
    ESP_RETURN_ON_ERROR(scs_init(SCS_DEFAULT_BAUD_RATE), TAG, "lazy init failed");

    if (xSemaphoreTake(s_scs_bus_mutex, pdMS_TO_TICKS(SCS_BUS_LOCK_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "bus lock timeout: id=%u inst=0x%02X", id, instruction);
        return ESP_ERR_TIMEOUT;
    }
    bus_locked = true;

    // --- 第一步：组装指令包 ---
    // 帧结构: FF FF ID LEN INST [PARAMS...] CHECKSUM
    // 总长度 = 帧头(2) + ID(1) + LEN(1) + INST(1) + 参数(N) + 校验和(1)
    size_t pkt_len = 4 + 1 + param_len + 1;
    if (pkt_len > SCS_MAX_PKT_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t pkt[SCS_MAX_PKT_LEN];
    pkt[0] = SCS_HEADER;                       // 帧头第 1 字节: 0xFF
    pkt[1] = SCS_HEADER;                       // 帧头第 2 字节: 0xFF
    pkt[2] = id;                               // 目标舵机 ID
    pkt[3] = (uint8_t)(param_len + 2);         // LEN = 参数长度 + 指令码(1) + 校验和(1)
    pkt[4] = instruction;                      // 指令码 (PING/READ/WRITE 等)
    if (param_len > 0 && params) {
        memcpy(&pkt[5], params, param_len);    // 拷贝参数数据
    }
    pkt[5 + param_len] = scs_checksum(pkt, 5 + param_len + 1);  // 计算并填入校验和

    size_t tx_len = 6 + param_len;  // 实际发送字节数 = 帧头(2)+ID(1)+LEN(1)+INST(1)+参数(N)+校验和(1)

    hex_dump("UART2_TX", pkt, tx_len);

    // --- 第二步：清空 RX 缓冲区，丢弃之前可能残留的数据 ---
    uart_flush_input(SCS_UART_NUM);

    // --- 第三步：发送指令帧 ---
    int written = uart_write_bytes(SCS_UART_NUM, pkt, tx_len);
    if (written != (int)tx_len) {
        ESP_LOGW(TAG, "tx: wrote %d/%u", written, (unsigned)tx_len);
        goto cleanup_fail;
    }

    // --- 第四步：等待 UART 发送完成 ---
    // 半双工模式下，必须等 TX 彻底结束才能切换到 RX 接收模式
    {
        esp_err_t err = uart_wait_tx_done(SCS_UART_NUM, pdMS_TO_TICKS(20));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "tx_done timeout");
            goto cleanup_fail;
        }
    }

    // --- 第五步：丢弃 TX 回环字节 ---
    // 在半双工单线总线上，主控发出的每个字节都会通过总线回环到 RX 端。
    // 如果不清除这些回环字节，它们会污染后续对舵机回复的接收。
    // 等待 SCS_ECHO_FLUSH_DELAY_MS 毫秒确保所有回环字节已到达 RX FIFO，
    // 然后一次性读出并丢弃。
    vTaskDelay(pdMS_TO_TICKS(SCS_ECHO_FLUSH_DELAY_MS));
    {
        uint8_t echo_buf[64];
        int echo_n = uart_read_bytes(SCS_UART_NUM, echo_buf, sizeof(echo_buf), 0);  // 非阻塞读取
        if (echo_n > 0) {
            hex_dump("UART2_ECHO_FLUSH", echo_buf, echo_n);
        } else {
            ESP_LOGD(TAG, "UART2_ECHO_FLUSH: 0 bytes (no echo)");
        }
    }

    // 广播地址（0xFE）不等待回复
    if (id == SCS_BROADCAST_ID) {
        xSemaphoreGive(s_scs_bus_mutex);
        return ESP_OK;
    }

    // --- 第六步：接收状态包 ---
    // 接收策略：先读取一段原始数据，手动扫描 FF FF 帧头，再按 LEN 字段接收完整帧
    uint8_t rxbuf[SCS_MAX_PKT_LEN];
    int rx_total = 0;

    // 先尝试接收至少 4 字节（帧头 FF FF + ID + LEN）
    int n = uart_read_bytes(SCS_UART_NUM, rxbuf, 4, pdMS_TO_TICKS(timeout_ms));
    if (n < 4) {
        ESP_LOGD(TAG, "rx header timeout (got %d bytes)", n);
        goto cleanup_timeout;
    }
    rx_total = n;

    // 在已接收的数据中扫描 FF FF 帧头
    // （正常情况下帧头在 offset 0，但如果回环清除不彻底，可能有残留字节在前面）
    int hdr_pos = -1;
    for (int i = 0; i <= rx_total - 2; i++) {
        if (rxbuf[i] == 0xFF && rxbuf[i + 1] == 0xFF) {
            hdr_pos = i;
            break;
        }
    }

    // 如果第一次没找到帧头，再读一些字节继续搜索
    if (hdr_pos < 0) {
        int extra = uart_read_bytes(SCS_UART_NUM, &rxbuf[rx_total],
                                     SCS_MAX_PKT_LEN - rx_total, pdMS_TO_TICKS(timeout_ms));
        if (extra > 0) rx_total += extra;

        for (int i = 0; i <= rx_total - 2; i++) {
            if (rxbuf[i] == 0xFF && rxbuf[i + 1] == 0xFF) {
                hdr_pos = i;
                break;
            }
        }
    }

    // 始终找不到帧头，通信异常
    if (hdr_pos < 0) {
        hex_dump("UART2_RX_NO_HDR", rxbuf, rx_total);
        goto cleanup_bad_response;
    }

    // 如果帧头不在 offset 0，说明前面有残留的回环字节泄漏
    // 将数据前移使帧头对齐到 offset 0
    if (hdr_pos > 0) {
        hex_dump("UART2_RX_ECHO_LEAK", rxbuf, hdr_pos);
        memmove(rxbuf, &rxbuf[hdr_pos], rx_total - hdr_pos);
        rx_total -= hdr_pos;
    }

    // 确保至少有 4 字节（FF FF ID LEN），不够则继续读取
    while (rx_total < 4) {
        n = uart_read_bytes(SCS_UART_NUM, &rxbuf[rx_total], 4 - rx_total,
                             pdMS_TO_TICKS(timeout_ms));
        if (n <= 0) {
            ESP_LOGW(TAG, "rx hdr incomplete after align (%d bytes)", rx_total);
            goto cleanup_timeout;
        }
        rx_total += n;
    }

    // 读取 LEN 字段：表示帧头之后剩余的字节数 = error(1) + params(N) + checksum(1)
    uint8_t rx_len_field = rxbuf[3];
    if (rx_len_field < 2 || rx_len_field > SCS_MAX_PKT_LEN - 4) {
        ESP_LOGW(TAG, "bad rx LEN=%u", rx_len_field);
        hex_dump("UART2_RX_BAD", rxbuf, rx_total);
        goto cleanup_bad_response;
    }

    // 接收完整的帧体：需要总字节数 = 4 (header) + rx_len_field (body)
    int need_total = 4 + rx_len_field;
    while (rx_total < need_total) {
        n = uart_read_bytes(SCS_UART_NUM, &rxbuf[rx_total], need_total - rx_total,
                             pdMS_TO_TICKS(timeout_ms));
        if (n <= 0) {
            ESP_LOGW(TAG, "rx body incomplete: got %d, expect %d", rx_total, need_total);
            goto cleanup_timeout;
        }
        rx_total += n;
    }

    hex_dump("UART2_RX", rxbuf, need_total);

    // --- 第七步：校验和验证 ---
    uint8_t calc_chk = scs_checksum(rxbuf, need_total);    // 本地计算的校验和
    uint8_t got_chk = rxbuf[need_total - 1];               // 包中携带的校验和
    if (calc_chk != got_chk) {
        ESP_LOGW(TAG, "rx checksum mismatch: calc=0x%02X got=0x%02X", calc_chk, got_chk);
        goto cleanup_bad_crc;
    }

    // 校验回包 ID 是否与请求的目标 ID 匹配（防止总线上其他舵机的回复混淆）
    if (rxbuf[2] != id) {
        ESP_LOGW(TAG, "rx ID mismatch: expect=0x%02X got=0x%02X", id, rxbuf[2]);
    }

    // --- 第八步：解析状态包，填充输出结构体 ---
    if (status) {
        const uint8_t *payload = &rxbuf[5];                    // 参数数据起始位置
        uint8_t param_count = rx_len_field - 2;                 // 参数字节数 = LEN - error(1) - checksum(1)

        // 规范化载荷：剥离某些舵机异常回显的请求前缀
        scs_normalize_status_payload(instruction, params, param_len, &payload, &param_count);

        status->id = rxbuf[2];                                  // 回复的舵机 ID
        status->error = rxbuf[4];                               // 错误码（0=正常）
        status->param_count = param_count;                      // 有效参数字节数
        if (status->param_count > 0) {
            memcpy(status->params, payload, status->param_count);  // 拷贝参数数据
        }
    }

    xSemaphoreGive(s_scs_bus_mutex);
    return ESP_OK;

cleanup_timeout:
    if (bus_locked) {
        xSemaphoreGive(s_scs_bus_mutex);
    }
    return ESP_ERR_TIMEOUT;

cleanup_bad_response:
    if (bus_locked) {
        xSemaphoreGive(s_scs_bus_mutex);
    }
    return ESP_ERR_INVALID_RESPONSE;

cleanup_bad_crc:
    if (bus_locked) {
        xSemaphoreGive(s_scs_bus_mutex);
    }
    return ESP_ERR_INVALID_CRC;

cleanup_fail:
    if (bus_locked) {
        xSemaphoreGive(s_scs_bus_mutex);
    }
    return ESP_FAIL;
}

// ============================================================
// 便捷 API 封装
// 每个 API 内部调用 scs_txrx 完成实际通信
// ============================================================

/** PING：探测舵机在线状态，超时 50ms */
esp_err_t scs_ping(uint8_t id, scs_status_t *status)
{
    return scs_txrx(id, SCS_INST_PING, NULL, 0, status, 50);
}

/** READ：读取舵机控制表，参数为 [起始地址, 读取长度] */
esp_err_t scs_read(uint8_t id, uint8_t addr, uint8_t len, scs_status_t *status)
{
    uint8_t params[2] = { addr, len };
    return scs_txrx(id, SCS_INST_READ, params, 2, status, 50);
}

/** WRITE：向舵机控制表写入数据，参数为 [地址, data0, data1, ...] */
esp_err_t scs_write(uint8_t id, uint8_t addr, const uint8_t *data, size_t data_len)
{
    uint8_t params[SCS_MAX_PKT_LEN];
    params[0] = addr;
    if (data_len > 0 && data) {
        memcpy(&params[1], data, data_len);
    }
    scs_status_t st;
    return scs_txrx(id, SCS_INST_WRITE, params, 1 + data_len, &st, 50);
}

/** WRITE_BYTE：写入 1 字节（封装 scs_write） */
esp_err_t scs_write_byte(uint8_t id, uint8_t addr, uint8_t val)
{
    return scs_write(id, addr, &val, 1);
}

/** WRITE_WORD：写入 2 字节（Little-Endian 格式，低字节在前） */
esp_err_t scs_write_word(uint8_t id, uint8_t addr, uint16_t val)
{
    // SCS 协议规定多字节数据采用小端序（低字节在前）
    uint8_t buf[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return scs_write(id, addr, buf, 2);
}

/** READ_BYTE：读取 1 字节，结果写入 *out */
esp_err_t scs_read_byte(uint8_t id, uint8_t addr, uint8_t *out)
{
    scs_status_t st = {0};
    esp_err_t err = scs_read(id, addr, 1, &st);
    if (err != ESP_OK) return err;

    // 严格检查返回的参数数量（应为 1 字节）
    if (st.param_count != 1) {
        ESP_LOGW(TAG, "read_byte: expect 1 param, got %u (addr=0x%02X id=%u)",
                 st.param_count, addr, id);
    }

    if (st.param_count >= 1 && out) {
        *out = st.params[0];
    }
    return err;
}

/** READ_WORD：读取 2 字节（Little-Endian），组合为 uint16_t */
esp_err_t scs_read_word(uint8_t id, uint8_t addr, uint16_t *out)
{
    scs_status_t st = {0};
    esp_err_t err = scs_read(id, addr, 2, &st);
    if (err != ESP_OK) return err;

    if (st.param_count != 2) {
        ESP_LOGW(TAG, "read_word: expect 2 params, got %u (addr=0x%02X id=%u)",
                 st.param_count, addr, id);
    }

    if (st.param_count >= 2 && out) {
        // 小端序：低字节在前，高字节在后
        *out = st.params[0] | ((uint16_t)st.params[1] << 8);
    }
    return err;
}

/**
 * SYNC_WRITE：广播同步写
 * 在一帧内同时向多个舵机写入相同地址的寄存器数据
 *
 * 帧格式: FF FF FE LEN 0x83 ADDR DATA_LEN [ID1 D1..DN] [ID2 D1..DN] ... CHECKSUM
 * 参数格式: id_data_pairs = [ID1, D1, D2, ..., ID2, D1, D2, ...]
 */
esp_err_t scs_sync_write(uint8_t addr, uint8_t data_len,
                          const uint8_t *id_data_pairs, size_t pair_count)
{
    size_t params_len = 2 + pair_count * (1 + data_len);
    uint8_t params[SCS_MAX_PKT_LEN];
    params[0] = addr;
    params[1] = data_len;
    memcpy(&params[2], id_data_pairs, pair_count * (1 + data_len));

    // 使用广播地址发送，不需要等待回复
    return scs_txrx(SCS_BROADCAST_ID, SCS_INST_SYNC_WRITE, params, params_len, NULL, 0);
}

/**
 * WRITE_POS：设置舵机目标位置和速度
 *
 * 实现方式：依次写入两个寄存器
 *   1. GOAL_POSITION (0x1E, 2字节) — 目标位置
 *   2. GOAL_SPEED (0x20, 2字节) — 运动速度
 *
 * @param id        舵机 ID
 * @param position  目标位置（舵机控制表中的位置值）
 * @param speed     运动速度
 * @param acc       加速度（本实现未使用，保留参数）
 */
esp_err_t scs_write_pos(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
    (void)acc;   // 未使用的加速度参数

    ESP_RETURN_ON_ERROR(scs_write_word(id, SCS_ADDR_GOAL_POSITION_L, (uint16_t)position),
                        TAG, "write goal position failed");
    return scs_write_word(id, SCS_ADDR_GOAL_SPEED_L, speed);
}
