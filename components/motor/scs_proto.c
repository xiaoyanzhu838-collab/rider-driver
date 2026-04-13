#include "scs_proto.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "scs";
static bool s_scs_initialized = false;
static int s_scs_baud_rate = 0;

// UART 硬件配置 (与原 dxl_proto 相同)
#define SCS_UART_NUM    UART_NUM_2
#define SCS_TX_GPIO     GPIO_NUM_14
#define SCS_RX_GPIO     GPIO_NUM_13
#define SCS_RX_BUF_SIZE 256

// TX echo 处理延时 (ms) — 半双工，需要等待回环字节全部到达后再 flush
#define SCS_ECHO_FLUSH_DELAY_MS  3

// ============================================================
// 调试开关：打印原始收发字节
// ============================================================
#ifndef SCS_DEBUG_RAW
#define SCS_DEBUG_RAW 1
#endif

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

static uint8_t scs_expected_param_count(uint8_t instruction, const uint8_t *params, size_t param_len)
{
    if (instruction == SCS_INST_READ && param_len >= 2) {
        return params[1];
    }
    return 0;
}

static void scs_normalize_status_payload(uint8_t instruction,
                                         const uint8_t *req_params,
                                         size_t req_param_len,
                                         const uint8_t **payload,
                                         uint8_t *param_count)
{
    const uint8_t *p = *payload;
    uint8_t count = *param_count;
    uint8_t expected = scs_expected_param_count(instruction, req_params, req_param_len);

    // Some servos on this bus prepend a small copy of the request body to the
    // reply payload. The observed pattern is:
    //   PING: [0x02, 0x01, 0x00]
    //   READ: [0x04, 0x02, addr] + data...
    // Strip that prefix so higher layers always see the actual register data.
    if (instruction == SCS_INST_PING &&
        count == 3 &&
        p[0] == 0x02 &&
        p[1] == SCS_INST_PING) {
        ESP_LOGW(TAG, "normalizing PING reply payload: stripping %u request bytes", count);
        *payload = p + 3;
        *param_count = 0;
        return;
    }

    if (instruction == SCS_INST_READ &&
        req_param_len >= 1 &&
        count == (uint8_t)(expected + 3) &&
        p[0] == 0x04 &&
        p[1] == SCS_INST_READ &&
        p[2] == req_params[0]) {
        ESP_LOGW(TAG,
                 "normalizing READ reply payload: stripping request prefix 04 02 %02X",
                 req_params[0]);
        *payload = p + 3;
        *param_count = expected;
        return;
    }

    if (instruction == SCS_INST_READ && count != expected) {
        ESP_LOGW(TAG,
                 "READ reply param count mismatch: expect=%u got=%u (addr=0x%02X)",
                 expected, count, (req_param_len >= 1) ? req_params[0] : 0x00);
    }

    // WRITE replies on this bus may also echo back LEN/INST/ADDR before the
    // actual empty status payload. Strip it for consistency if we ever inspect
    // the returned params in diagnostics.
    if (instruction == SCS_INST_WRITE &&
        req_param_len >= 1 &&
        count == 3 &&
        p[0] == 0x05 &&
        p[1] == SCS_INST_WRITE &&
        p[2] == req_params[0]) {
        ESP_LOGW(TAG,
                 "normalizing WRITE reply payload: stripping request prefix 05 03 %02X",
                 req_params[0]);
        *payload = p + 3;
        *param_count = 0;
    }
}

// ============================================================
// 校验和: ~(ID + LEN + INST + PARAMS...) & 0xFF
// 与 Dynamixel Protocol 1.0 完全一致
// ============================================================
static uint8_t scs_checksum(const uint8_t *pkt, size_t pkt_len)
{
    // pkt[0]=FF, pkt[1]=FF, pkt[2]=ID, pkt[3]=LEN, pkt[4]=INST, ...
    // sum = ID + LEN + INST + PARAM[0..N]  (不含 header 和 checksum 本身)
    uint8_t sum = 0;
    for (size_t i = 2; i < pkt_len - 1; i++) {
        sum += pkt[i];
    }
    return ~sum;
}

// ============================================================
// 初始化
// ============================================================
esp_err_t scs_init(int baud_rate)
{
    if (s_scs_initialized) {
        if (baud_rate == s_scs_baud_rate) {
            return ESP_OK;
        }
        return ESP_ERR_INVALID_STATE;
    }

    uart_config_t uart_cfg = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(uart_driver_install(SCS_UART_NUM, SCS_RX_BUF_SIZE, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(SCS_UART_NUM, &uart_cfg),
                        TAG, "uart_param_config failed");
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
// 发送指令包 + 接收状态包
// ============================================================
esp_err_t scs_txrx(uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t param_len,
                    scs_status_t *status, uint32_t timeout_ms)
{
    ESP_RETURN_ON_ERROR(scs_init(SCS_DEFAULT_BAUD_RATE), TAG, "lazy init failed");

    // --- 构建指令包 ---
    // FF FF ID LEN INST PARAM... CHECKSUM
    size_t pkt_len = 4 + 1 + param_len + 1;  // header(2)+ID(1)+LEN(1)+INST(1)+params+checksum(1)
    if (pkt_len > SCS_MAX_PKT_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t pkt[SCS_MAX_PKT_LEN];
    pkt[0] = SCS_HEADER;
    pkt[1] = SCS_HEADER;
    pkt[2] = id;
    pkt[3] = (uint8_t)(param_len + 2);  // LEN = params + instruction + checksum
    pkt[4] = instruction;
    if (param_len > 0 && params) {
        memcpy(&pkt[5], params, param_len);
    }
    pkt[5 + param_len] = scs_checksum(pkt, 5 + param_len + 1);

    size_t tx_len = 6 + param_len;

    hex_dump("TX", pkt, tx_len);

    // --- 清空 RX 缓冲（丢弃之前残留数据） ---
    uart_flush_input(SCS_UART_NUM);

    // --- 发送 ---
    int written = uart_write_bytes(SCS_UART_NUM, pkt, tx_len);
    if (written != (int)tx_len) {
        ESP_LOGW(TAG, "tx: wrote %d/%u", written, (unsigned)tx_len);
        return ESP_FAIL;
    }
    // 等待发送完毕（半双工切换需要 TX 完成后才能收）
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(SCS_UART_NUM, pdMS_TO_TICKS(20)),
                        TAG, "tx_done timeout");

    // 丢弃 TX 回环字节：
    //   半双工单线总线, TX 出去的字节会原路回到 RX 端.
    //   等待足够时间让所有回环字节到达 RX FIFO, 然后一次性 flush.
    vTaskDelay(pdMS_TO_TICKS(SCS_ECHO_FLUSH_DELAY_MS));

    // 读出并丢弃回环字节，同时打印出来看实际收到多少
    {
        uint8_t echo_buf[64];
        int echo_n = uart_read_bytes(SCS_UART_NUM, echo_buf, sizeof(echo_buf), 0);  // non-blocking
        if (echo_n > 0) {
            hex_dump("ECHO_FLUSH", echo_buf, echo_n);
        } else {
            ESP_LOGD(TAG, "ECHO_FLUSH: 0 bytes (no echo)");
        }
    }

    // 广播不等回包
    if (id == SCS_BROADCAST_ID) {
        return ESP_OK;
    }

    // --- 接收状态包 ---
    // 策略: 先读取一段原始数据, 手动扫描 FF FF 帧头, 解析完整包
    uint8_t rxbuf[SCS_MAX_PKT_LEN];
    int rx_total = 0;

    // 读取尽可能多的字节 (最多 SCS_MAX_PKT_LEN)
    // 先等 4 字节 (最少需要 header)
    int n = uart_read_bytes(SCS_UART_NUM, rxbuf, 4, pdMS_TO_TICKS(timeout_ms));
    if (n < 4) {
        ESP_LOGD(TAG, "rx header timeout (got %d bytes)", n);
        return ESP_ERR_TIMEOUT;
    }
    rx_total = n;

    // 在已收字节中扫描 FF FF 帧头
    int hdr_pos = -1;
    for (int i = 0; i <= rx_total - 2; i++) {
        if (rxbuf[i] == 0xFF && rxbuf[i + 1] == 0xFF) {
            hdr_pos = i;
            break;
        }
    }

    if (hdr_pos < 0) {
        // 没找到帧头, 再读一些字节继续搜索
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

    if (hdr_pos < 0) {
        hex_dump("RX_NO_HDR", rxbuf, rx_total);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 如果帧头不在 offset 0, 有残留 echo 字节泄漏
    if (hdr_pos > 0) {
        hex_dump("RX_ECHO_LEAK", rxbuf, hdr_pos);
        // 移动数据使帧头对齐到 offset 0
        memmove(rxbuf, &rxbuf[hdr_pos], rx_total - hdr_pos);
        rx_total -= hdr_pos;
    }

    // 确保至少有 4 字节 (FF FF ID LEN)
    while (rx_total < 4) {
        n = uart_read_bytes(SCS_UART_NUM, &rxbuf[rx_total], 4 - rx_total,
                             pdMS_TO_TICKS(timeout_ms));
        if (n <= 0) {
            ESP_LOGW(TAG, "rx hdr incomplete after align (%d bytes)", rx_total);
            return ESP_ERR_TIMEOUT;
        }
        rx_total += n;
    }

    uint8_t rx_len_field = rxbuf[3];  // 剩余字节数 (error + params + checksum)
    if (rx_len_field < 2 || rx_len_field > SCS_MAX_PKT_LEN - 4) {
        ESP_LOGW(TAG, "bad rx LEN=%u", rx_len_field);
        hex_dump("RX_BAD", rxbuf, rx_total);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 需要的总字节数 = 4 (header) + rx_len_field (body)
    int need_total = 4 + rx_len_field;
    while (rx_total < need_total) {
        n = uart_read_bytes(SCS_UART_NUM, &rxbuf[rx_total], need_total - rx_total,
                             pdMS_TO_TICKS(timeout_ms));
        if (n <= 0) {
            ESP_LOGW(TAG, "rx body incomplete: got %d, expect %d", rx_total, need_total);
            return ESP_ERR_TIMEOUT;
        }
        rx_total += n;
    }

    hex_dump("RX", rxbuf, need_total);

    // 校验
    uint8_t calc_chk = scs_checksum(rxbuf, need_total);
    uint8_t got_chk = rxbuf[need_total - 1];
    if (calc_chk != got_chk) {
        ESP_LOGW(TAG, "rx checksum mismatch: calc=0x%02X got=0x%02X", calc_chk, got_chk);
        return ESP_ERR_INVALID_CRC;
    }

    // 校验回包 ID 与请求 ID 是否匹配
    if (rxbuf[2] != id) {
        ESP_LOGW(TAG, "rx ID mismatch: expect=0x%02X got=0x%02X", id, rxbuf[2]);
    }

    // 填充 status
    if (status) {
        const uint8_t *payload = &rxbuf[5];
        uint8_t param_count = rx_len_field - 2;  // 减去 error + checksum

        scs_normalize_status_payload(instruction, params, param_len, &payload, &param_count);

        status->id = rxbuf[2];
        status->error = rxbuf[4];
        status->param_count = param_count;
        if (status->param_count > 0) {
            memcpy(status->params, payload, status->param_count);
        }
    }

    return ESP_OK;
}

// ============================================================
// 便捷 API
// ============================================================
esp_err_t scs_ping(uint8_t id, scs_status_t *status)
{
    return scs_txrx(id, SCS_INST_PING, NULL, 0, status, 50);
}

esp_err_t scs_read(uint8_t id, uint8_t addr, uint8_t len, scs_status_t *status)
{
    uint8_t params[2] = { addr, len };
    return scs_txrx(id, SCS_INST_READ, params, 2, status, 50);
}

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

esp_err_t scs_write_byte(uint8_t id, uint8_t addr, uint8_t val)
{
    return scs_write(id, addr, &val, 1);
}

esp_err_t scs_write_word(uint8_t id, uint8_t addr, uint16_t val)
{
    // little-endian: LO byte first (SCS standard)
    uint8_t buf[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return scs_write(id, addr, buf, 2);
}

esp_err_t scs_read_byte(uint8_t id, uint8_t addr, uint8_t *out)
{
    scs_status_t st = {0};
    esp_err_t err = scs_read(id, addr, 1, &st);
    if (err != ESP_OK) return err;

    // 严格检查返回的参数数量是否符合预期
    if (st.param_count != 1) {
        ESP_LOGW(TAG, "read_byte: expect 1 param, got %u (addr=0x%02X id=%u)",
                 st.param_count, addr, id);
    }

    if (st.param_count >= 1 && out) {
        // 如果返回了多余字节，严重的地址偏移问题，先记录
        *out = st.params[0];
    }
    return err;
}

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
        *out = st.params[0] | ((uint16_t)st.params[1] << 8);
    }
    return err;
}

esp_err_t scs_sync_write(uint8_t addr, uint8_t data_len,
                          const uint8_t *id_data_pairs, size_t pair_count)
{
    // SYNC_WRITE: FF FF FE LEN 83 ADDR DATA_LEN [ID1 D1..DN] [ID2 D1..DN] ... CHECKSUM
    // params = ADDR(1) + DATA_LEN(1) + pair_count*(1+data_len)
    size_t params_len = 2 + pair_count * (1 + data_len);
    uint8_t params[SCS_MAX_PKT_LEN];
    params[0] = addr;
    params[1] = data_len;
    memcpy(&params[2], id_data_pairs, pair_count * (1 + data_len));

    return scs_txrx(SCS_BROADCAST_ID, SCS_INST_SYNC_WRITE, params, params_len, NULL, 0);
}

esp_err_t scs_write_pos(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
    (void)acc;

    ESP_RETURN_ON_ERROR(scs_write_word(id, SCS_ADDR_GOAL_POSITION_L, (uint16_t)position),
                        TAG, "write goal position failed");
    return scs_write_word(id, SCS_ADDR_GOAL_SPEED_L, speed);
}
