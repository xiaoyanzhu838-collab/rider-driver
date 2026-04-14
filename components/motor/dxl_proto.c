#include "dxl_proto.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "dxl";

// ============================================================
// UART 硬件配置（与 scs_proto 共用同一套硬件资源）
// 使用 ESP32 的 UART2，TX=GPIO14，RX=GPIO13
// ============================================================
#define DXL_UART_NUM    UART_NUM_2
#define DXL_TX_GPIO     GPIO_NUM_14
#define DXL_RX_GPIO     GPIO_NUM_13
#define DXL_RX_BUF_SIZE 256

// ============================================================
// 校验和计算（与 Dynamixel Protocol 1.0 / SCS 协议一致）
//
// 算法: ~(ID + LEN + INST + PARAM[0..N]) & 0xFF
// 即从 ID 开始到最后一字节参数逐字节累加，取反后取低 8 位
// ============================================================
static uint8_t dxl_checksum(const uint8_t *pkt, size_t pkt_len)
{
    // pkt 结构: [0xFF, 0xFF, ID, LEN, INST, PARAM0, ..., PARAMn, CHECKSUM]
    // 累加从索引 2 (ID) 到 pkt_len-2 (最后一个参数)，跳过帧头和校验和
    uint8_t sum = 0;
    for (size_t i = 2; i < pkt_len - 1; i++) {
        sum += pkt[i];
    }
    return ~sum;
}

// ============================================================
// 初始化 Dynamixel UART 总线
// 配置 8N1 格式、指定波特率、绑定 GPIO 引脚
// ============================================================
esp_err_t dxl_init(int baud_rate)
{
    uart_config_t uart_cfg = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装 UART 驱动
    ESP_RETURN_ON_ERROR(uart_driver_install(DXL_UART_NUM, DXL_RX_BUF_SIZE, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");
    // 配置 UART 参数
    ESP_RETURN_ON_ERROR(uart_param_config(DXL_UART_NUM, &uart_cfg),
                        TAG, "uart_param_config failed");
    // 绑定 GPIO 引脚
    ESP_RETURN_ON_ERROR(uart_set_pin(DXL_UART_NUM, DXL_TX_GPIO, DXL_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");

    ESP_LOGI(TAG, "init OK, UART%d baud=%d TX=GPIO%d RX=GPIO%d",
             DXL_UART_NUM, baud_rate, DXL_TX_GPIO, DXL_RX_GPIO);
    return ESP_OK;
}

// ============================================================
// 核心收发函数：发送指令包 + 接收状态包
//
// 【半双工通信时序】
//   1. 组装指令帧: FF FF ID LEN INST PARAMS... CHECKSUM
//   2. 清空 RX 缓冲区
//   3. 发送指令帧
//   4. 等待 TX 完成
//   5. 短延时后丢弃 TX 回环字节
//   6. 接收舵机回复：先收 4 字节帧头，再按 LEN 收完整帧
//   7. 校验和验证 + 解析状态包
// ============================================================
esp_err_t dxl_txrx(uint8_t id, uint8_t instruction,
                    const uint8_t *params, size_t param_len,
                    dxl_status_t *status, uint32_t timeout_ms)
{
    // --- 组装指令包 ---
    size_t pkt_len = 4 + 1 + param_len + 1;  // header(2)+ID(1)+LEN(1)+INST(1)+params+checksum(1)
    if (pkt_len > DXL_MAX_PKT_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t pkt[DXL_MAX_PKT_LEN];
    pkt[0] = DXL_HEADER;                       // 帧头 0xFF
    pkt[1] = DXL_HEADER;                       // 帧头 0xFF
    pkt[2] = id;                               // 目标 ID
    pkt[3] = (uint8_t)(param_len + 2);         // LEN = 参数数 + 指令码(1) + 校验和(1)
    pkt[4] = instruction;                      // 指令码
    if (param_len > 0 && params) {
        memcpy(&pkt[5], params, param_len);    // 参数
    }
    pkt[5 + param_len] = dxl_checksum(pkt, 5 + param_len + 1);  // 校验和

    size_t tx_len = 6 + param_len;

    // --- 清空 RX 缓冲（丢弃残留数据） ---
    uart_flush_input(DXL_UART_NUM);

    // --- 发送指令帧 ---
    int written = uart_write_bytes(DXL_UART_NUM, pkt, tx_len);
    if (written != (int)tx_len) {
        ESP_LOGW(TAG, "tx: wrote %d/%u", written, (unsigned)tx_len);
        return ESP_FAIL;
    }
    // 等待发送完毕（半双工模式下必须等 TX 彻底结束才能切到 RX）
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(DXL_UART_NUM, pdMS_TO_TICKS(20)),
                        TAG, "tx_done timeout");

    // 丢弃 TX 回环字节（半双工总线上发出的数据会回环到 RX 端）
    vTaskDelay(pdMS_TO_TICKS(1));
    uart_flush_input(DXL_UART_NUM);

    // 广播地址不等待回复
    if (id == DXL_BROADCAST_ID) {
        return ESP_OK;
    }

    // --- 接收状态包 ---
    // 最短状态包: FF FF ID LEN ERR CHECKSUM = 6 字节
    uint8_t rxbuf[DXL_MAX_PKT_LEN];
    int rx_total = 0;

    // 先收 4 字节: FF FF ID LEN
    int n = uart_read_bytes(DXL_UART_NUM, rxbuf, 4, pdMS_TO_TICKS(timeout_ms));
    if (n < 4) {
        ESP_LOGD(TAG, "rx header timeout (got %d bytes)", n);
        return ESP_ERR_TIMEOUT;
    }
    rx_total = 4;

    // 验证帧头
    if (rxbuf[0] != 0xFF || rxbuf[1] != 0xFF) {
        ESP_LOGW(TAG, "bad rx header: %02X %02X", rxbuf[0], rxbuf[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 读取 LEN 字段：表示帧头之后剩余字节数 (error + params + checksum)
    uint8_t rx_len_field = rxbuf[3];
    if (rx_len_field < 2 || rx_len_field > DXL_MAX_PKT_LEN - 4) {
        ESP_LOGW(TAG, "bad rx LEN=%u", rx_len_field);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 收取剩余字节 (error + params + checksum)
    n = uart_read_bytes(DXL_UART_NUM, &rxbuf[4], rx_len_field, pdMS_TO_TICKS(timeout_ms));
    if (n < (int)rx_len_field) {
        ESP_LOGW(TAG, "rx body incomplete: got %d, expect %u", n, rx_len_field);
        return ESP_ERR_TIMEOUT;
    }
    rx_total += n;

    // 校验和验证
    uint8_t calc_chk = dxl_checksum(rxbuf, rx_total);
    uint8_t got_chk = rxbuf[rx_total - 1];
    if (calc_chk != got_chk) {
        ESP_LOGW(TAG, "rx checksum mismatch: calc=0x%02X got=0x%02X", calc_chk, got_chk);
        return ESP_ERR_INVALID_CRC;
    }

    // 填充状态包结构体
    if (status) {
        status->id = rxbuf[2];                                  // 设备 ID
        status->error = rxbuf[4];                               // 错误码
        status->param_count = rx_len_field - 2;                 // 参数数 = LEN - error(1) - checksum(1)
        if (status->param_count > 0) {
            memcpy(status->params, &rxbuf[5], status->param_count);  // 参数数据
        }
    }

    return ESP_OK;
}

// ============================================================
// 便捷 API
// ============================================================

/** PING：探测舵机是否在线，超时 50ms */
esp_err_t dxl_ping(uint8_t id, dxl_status_t *status)
{
    return dxl_txrx(id, DXL_INST_PING, NULL, 0, status, 50);
}

/** READ：读取舵机控制表指定地址的数据 */
esp_err_t dxl_read(uint8_t id, uint8_t addr, uint8_t len, dxl_status_t *status)
{
    uint8_t params[2] = { addr, len };
    return dxl_txrx(id, DXL_INST_READ, params, 2, status, 50);
}

/** WRITE：向舵机控制表写入数据，参数格式为 [地址, data0, data1, ...] */
esp_err_t dxl_write(uint8_t id, uint8_t addr, const uint8_t *data, size_t data_len)
{
    uint8_t params[DXL_MAX_PKT_LEN];
    params[0] = addr;
    if (data_len > 0 && data) {
        memcpy(&params[1], data, data_len);
    }
    dxl_status_t st;
    return dxl_txrx(id, DXL_INST_WRITE, params, 1 + data_len, &st, 50);
}

/**
 * SYNC_WRITE：广播同步写
 * 参数 id_data_pairs 格式: [ID1, D1..DN, ID2, D1..DN, ...]
 * 帧格式: FF FF FE LEN 83 ADDR DATA_LEN [ID1 D1..DN] ... CHECKSUM
 */
esp_err_t dxl_sync_write(uint8_t addr, uint8_t data_len,
                          const uint8_t *id_data_pairs, size_t pair_count)
{
    size_t params_len = 2 + pair_count * (1 + data_len);
    uint8_t params[DXL_MAX_PKT_LEN];
    params[0] = addr;
    params[1] = data_len;
    memcpy(&params[2], id_data_pairs, pair_count * (1 + data_len));

    return dxl_txrx(DXL_BROADCAST_ID, DXL_INST_SYNC_WRITE, params, params_len, NULL, 0);
}
