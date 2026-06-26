#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "ir_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t position;
    bool line_detected;
    bool all_white;
    bool all_black;
    uint8_t black_count;
    uint16_t last_valid_position;
} line_status_t;

void line_follower_init(void);
line_status_t line_follower_compute(const ir_sensor_reading_t *reading);

#ifdef __cplusplus
}
#endif
