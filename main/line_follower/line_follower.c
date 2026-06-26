#include "line_follower.h"
#include "esp_log.h"

static const char *TAG = "line";
static uint16_t s_last_valid_position = 1000;

void line_follower_init(void) {
    s_last_valid_position = 1000;
    ESP_LOGI(TAG, "Seguidor de linha inicializado (3 sensores E18-D80NK)");
}

line_status_t line_follower_compute(const ir_sensor_reading_t *reading) {
    line_status_t status = {0};
    status.last_valid_position = s_last_valid_position;
    
    if (!reading) return status;

    bool left   = reading->detected[IR_SENSOR_LEFT];
    bool center = reading->detected[IR_SENSOR_CENTER];
    bool right  = reading->detected[IR_SENSOR_RIGHT];

    uint8_t black_count = (uint8_t)left + (uint8_t)center + (uint8_t)right;

    status.black_count = black_count;
    status.all_white = (black_count == 0);
    status.all_black = (black_count == IR_SENSOR_NUM_SENSORS);
    status.line_detected = (black_count >= 1);

    /*
     * Cálculo de posição com 3 sensores digitais:
     *   Esquerdo=0, Centro=1000, Direito=2000
     *   Setpoint do PID = 1000 (centro)
     *
     * Combinações:
     *   Apenas E      → 0
     *   E + C         → 500
     *   Apenas C      → 1000
     *   C + D         → 1500
     *   Apenas D      → 2000
     *   E + C + D     → 1000  (tudo preto, assume centro)
     *   E + D (sem C) → 1000  (incomum, assume centro)
     *   Nenhum        → última posição válida
     */
    if (black_count == 0) {
        /* Perdeu a linha — manter última posição conhecida. */
        status.position = s_last_valid_position;
    } else {
        uint32_t weighted = 0;
        uint32_t count = 0;

        if (left)   { weighted += 0;    count++; }
        if (center) { weighted += 1000; count++; }
        if (right)  { weighted += 2000; count++; }

        status.position = (uint16_t)(weighted / count);
        s_last_valid_position = status.position;
        status.last_valid_position = s_last_valid_position;
    }
    
    return status;
}
