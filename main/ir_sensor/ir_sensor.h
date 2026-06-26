#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Quantidade de sensores E18-D80NK. */
#define IR_SENSOR_NUM_SENSORS 3

/** Índices dos sensores para legibilidade. */
#define IR_SENSOR_LEFT   0
#define IR_SENSOR_CENTER 1
#define IR_SENSOR_RIGHT  2

/** Leitura dos 3 sensores digitais. */
typedef struct {
    bool detected[IR_SENSOR_NUM_SENSORS];  /**< true se o sensor detecta a linha (preto). */
    uint8_t black_mask;                    /**< Bit i = 1 se sensor i vê preto. */
    bool line_detected;                    /**< true se pelo menos 1 sensor detecta linha. */
} ir_sensor_reading_t;

/** Inicializa GPIOs dos 3 sensores como entrada com pull-up. */
esp_err_t ir_sensor_init(void);

/** Inicia tarefa que lê periodicamente os sensores. */
esp_err_t ir_sensor_start_read_task(uint32_t read_period_ms);

/** Copia a última leitura (thread-safe). */
esp_err_t ir_sensor_get_last_reading(ir_sensor_reading_t *out);

/** Sensores digitais não precisam de calibração, mas mantemos compatibilidade. */
bool ir_sensor_is_calibrated(void);

#ifdef __cplusplus
}
#endif
