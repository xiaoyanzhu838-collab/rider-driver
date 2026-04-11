#include "button.h"

#include <stdint.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "button";

static gpio_num_t s_gpio = GPIO_NUM_NC;
static bool s_active_low;
static button_callback_t s_cb;
static void *s_cb_ctx;
static TaskHandle_t s_task;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t hp_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &hp_task_woken);
    if (hp_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static bool read_pressed(void)
{
    int level = gpio_get_level(s_gpio);
    return s_active_low ? (level == 0) : (level == 1);
}

static void button_task(void *arg)
{
    (void)arg;

    bool last = read_pressed();

    // 上电先回调一次当前状态，保证“默认关闭/默认开启”逻辑可控
    if (s_cb) {
        s_cb(last, s_cb_ctx);
    }

    while (1) {
        // 等待中断通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 简单去抖：等待 30ms 再读一次
        vTaskDelay(pdMS_TO_TICKS(30));

        bool now = read_pressed();
        if (now != last) {
            last = now;
            if (s_cb) {
                s_cb(now, s_cb_ctx);
            }
        }
    }
}

esp_err_t button_init(gpio_num_t gpio_num, bool active_low)
{
    s_gpio = gpio_num;
    s_active_low = active_low;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << s_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio_config failed");

    if (s_task == NULL) {
        BaseType_t ok = xTaskCreate(button_task, "button", 2048, NULL, tskIDLE_PRIORITY + 2, &s_task);
        ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create button task failed");
    }

    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "gpio_install_isr_service failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s_gpio, gpio_isr_handler, NULL), TAG, "gpio_isr_handler_add failed");

    ESP_LOGI(TAG, "inited gpio=%d active_low=%d", (int)s_gpio, (int)s_active_low);
    return ESP_OK;
}

esp_err_t button_register_callback(button_callback_t cb, void *user_ctx)
{
    s_cb = cb;
    s_cb_ctx = user_ctx;
    return ESP_OK;
}

