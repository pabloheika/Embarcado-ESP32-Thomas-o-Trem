#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Quantidade de sensores do QTR-8RC (D1..D8). */
#define QTR8RC_NUM_SENSORS 8

/** Leitura processada (bruta, normalizada e detecção de linha). */
typedef struct {
    uint32_t raw_us[QTR8RC_NUM_SENSORS];
    uint16_t norm[QTR8RC_NUM_SENSORS];
    uint8_t black_mask; /**< Bit i = 1 se o sensor i considerar “preto”. */
    bool line_detected;
} qtr8rc_reading_t;

/**
 * Inicializa GPIOs: pino IR (LED dos emissores) e sensores D1..D8.
 * Liga os emissores IR (nível alto no pino IR, se usado pelo módulo).
 *
 * Ajuste os pinos em `qtr8rc.c` conforme sua ligação física.
 */
esp_err_t qtr8rc_init(void);

/**
 * Calibração bloqueante: mova o sensor sobre branco e preto durante o tempo indicado.
 * Deve ser chamado após `qtr8rc_init()` e antes de confiar em valores `norm[]`.
 */
void qtr8rc_calibrate_blocking(uint32_t duration_ms);

/**
 * Define limiar e quantidade mínima de sensores “pretos” para considerar linha detectada.
 * \param black_threshold Valores normalizados 0..1000; típico ~600.
 * \param min_sensors Mínimo de sensores acima do limiar (1 ou 2 para reduzir falso positivo).
 */
void qtr8rc_set_line_detection(uint16_t black_threshold, int min_sensors);

/** Lê tempos de descarga RC em microsegundos (uma passagem completa). */
esp_err_t qtr8rc_read_raw(uint32_t values[QTR8RC_NUM_SENSORS]);

/**
 * Normaliza um canal usando min/máx da última calibração.
 * Sem calibração válida, o comportamento segue a lógica interna (normalmente 0 ou extremos).
 */
uint16_t qtr8rc_normalize_sensor(int sensor_index, uint32_t raw_value);

/**
 * Inicia tarefa que lê periodicamente, atualiza última leitura com mutex e opcionalmente loga.
 *
 * \param read_period_ms Intervalo entre leituras (ex.: 50–100 ms para seguidor de linha).
 */
esp_err_t qtr8rc_start_read_task(uint32_t read_period_ms);

/** Copia a última leitura feita pela tarefa (thread-safe). */
esp_err_t qtr8rc_get_last_reading(qtr8rc_reading_t *out);

/** Retorna verdadeiro se a calibração já foi feita. */
bool qtr8rc_is_calibrated(void);

#ifdef __cplusplus
}
#endif
