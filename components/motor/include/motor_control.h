#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_COUNT 2

typedef struct {
    uint8_t id;
    uint16_t position;
    uint16_t speed;
    uint16_t load;
    uint8_t voltage;
    uint8_t temperature;
    uint8_t moving;
} motor_feedback_t;

esp_err_t motor_init(void);
size_t motor_get_ids(const uint8_t **ids_out);
esp_err_t motor_set_torque(uint8_t id, bool enable);
esp_err_t motor_set_position(uint8_t id, uint16_t position, uint16_t speed);
esp_err_t motor_read_feedback(uint8_t id, motor_feedback_t *out);
esp_err_t motor_read_positions(uint16_t *positions, size_t capacity, size_t *written);

#ifdef __cplusplus
}
#endif
