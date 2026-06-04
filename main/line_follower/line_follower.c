#include "line_follower.h"
#include "esp_log.h"

static const char *TAG = "line";
static uint16_t s_last_valid_position = 3500;

void line_follower_init(void) {
    s_last_valid_position = 3500;
    ESP_LOGI(TAG, "Seguidor de linha inicializado");
}

line_status_t line_follower_compute(const qtr8rc_reading_t *reading) {
    line_status_t status = {0};
    status.last_valid_position = s_last_valid_position;
    
    if (!reading) return status;

    uint32_t weighted_sum = 0;
    uint32_t sum = 0;
    uint8_t black_count = 0;
    
    for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
        uint16_t val = reading->norm[i];
        if (val >= 600) { // black threshold
            black_count++;
        }
        weighted_sum += (uint32_t)val * i * 1000;
        sum += val;
    }
    
    status.black_count = black_count;
    status.all_white = (black_count == 0);
    status.all_black = (black_count == QTR8RC_NUM_SENSORS);
    status.line_detected = (black_count >= 1);
    
    if (sum > 0) {
        status.position = (uint16_t)(weighted_sum / sum);
        s_last_valid_position = status.position;
        status.last_valid_position = s_last_valid_position;
    } else {
        status.position = s_last_valid_position;
    }
    
    return status;
}
