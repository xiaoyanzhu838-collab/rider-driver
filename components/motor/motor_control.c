#include "motor_control.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "scs_proto.h"

static const char *TAG = "motor";
static const uint8_t MOTOR_IDS[MOTOR_COUNT] = { 11, 12 };

esp_err_t motor_init(void)
{
    return scs_init(SCS_DEFAULT_BAUD_RATE);
}

size_t motor_get_ids(const uint8_t **ids_out)
{
    if (ids_out) {
        *ids_out = MOTOR_IDS;
    }
    return MOTOR_COUNT;
}

esp_err_t motor_set_torque(uint8_t id, bool enable)
{
    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor init failed");
    return scs_write_byte(id, SCS_ADDR_TORQUE_ENABLE, enable ? 1 : 0);
}

esp_err_t motor_set_position(uint8_t id, uint16_t position, uint16_t speed)
{
    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor init failed");
    return scs_write_pos(id, (int16_t)position, speed, 0);
}

esp_err_t motor_read_feedback(uint8_t id, motor_feedback_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->id = id;

    ESP_RETURN_ON_ERROR(motor_init(), TAG, "motor init failed");

    scs_status_t st = {0};
    ESP_RETURN_ON_ERROR(scs_read(id, SCS_ADDR_PRESENT_POSITION_L, 6, &st),
                        TAG, "read present block failed");
    if (st.param_count < 6) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    out->position = st.params[0] | ((uint16_t)st.params[1] << 8);
    out->speed = st.params[2] | ((uint16_t)st.params[3] << 8);
    out->load = st.params[4] | ((uint16_t)st.params[5] << 8);

    ESP_RETURN_ON_ERROR(scs_read_byte(id, SCS_ADDR_PRESENT_VOLTAGE, &out->voltage),
                        TAG, "read voltage failed");
    ESP_RETURN_ON_ERROR(scs_read_byte(id, SCS_ADDR_PRESENT_TEMPERATURE, &out->temperature),
                        TAG, "read temperature failed");
    ESP_RETURN_ON_ERROR(scs_read_byte(id, SCS_ADDR_MOVING, &out->moving),
                        TAG, "read moving failed");
    return ESP_OK;
}

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
