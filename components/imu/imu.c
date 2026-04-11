#include "imu.h"

#include <string.h>
#include <stdatomic.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "driver/i2c_master.h"

#include "board_config.h"
#include "icm42670.h"

static const char *TAG = "imu";

static i2c_master_bus_handle_t s_i2c_bus;
static icm42670_handle_t s_imu;

static portMUX_TYPE s_latest_mux = portMUX_INITIALIZER_UNLOCKED;
static imu_sample_t s_latest;
static atomic_bool s_has_latest;

icm42670_handle_t imu_get_handle(void)
{
    return s_imu;
}

static esp_err_t imu_i2c_init(void)
{
    if (s_i2c_bus) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1,
    };

    return i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
}

static esp_err_t imu_sensor_init(void)
{
    if (s_imu) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(icm42670_create(s_i2c_bus, BOARD_ICM42670_ADDR, &s_imu), TAG, "icm42670_create failed");

    uint8_t devid = 0;
    ESP_RETURN_ON_ERROR(icm42670_get_deviceid(s_imu, &devid), TAG, "get device id failed");
    ESP_LOGI(TAG, "ICM42670 device id: 0x%02X", devid);

    icm42670_cfg_t cfg = {
        .acce_fs = ACCE_FS_4G,
        .acce_odr = ACCE_ODR_200HZ,
        .gyro_fs = GYRO_FS_500DPS,
        .gyro_odr = GYRO_ODR_200HZ,
    };

    ESP_RETURN_ON_ERROR(icm42670_config(s_imu, &cfg), TAG, "icm42670_config failed");
    ESP_RETURN_ON_ERROR(icm42670_acce_set_pwr(s_imu, ACCE_PWR_LOWNOISE), TAG, "acce pwr failed");
    ESP_RETURN_ON_ERROR(icm42670_gyro_set_pwr(s_imu, GYRO_PWR_LOWNOISE), TAG, "gyro pwr failed");

    return ESP_OK;
}

static void imu_store_latest(const imu_sample_t *s)
{
    portENTER_CRITICAL(&s_latest_mux);
    s_latest = *s;
    portEXIT_CRITICAL(&s_latest_mux);
    atomic_store(&s_has_latest, true);
}

bool imu_get_latest(imu_sample_t *out)
{
    if (!out) return false;
    if (!atomic_load(&s_has_latest)) return false;

    portENTER_CRITICAL(&s_latest_mux);
    *out = s_latest;
    portEXIT_CRITICAL(&s_latest_mux);
    return true;
}

void imu_set_latest(const imu_sample_t *s)
{
    if (!s) return;
    imu_store_latest(s);
}

esp_err_t imu_init(void)
{
    ESP_RETURN_ON_ERROR(imu_i2c_init(), TAG, "i2c init failed");
    ESP_RETURN_ON_ERROR(imu_sensor_init(), TAG, "sensor init failed");
    return ESP_OK;
}
