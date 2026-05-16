#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Amostra bruta do MPU6050 (accel/temp/gyro), endian do sensor já convertido. */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_sample_t;

/**
 * Inicializa o barramento I2C e o dispositivo, acorda o sensor e valida WHO_AM_I.
 */
esp_err_t mpu6050_init(void);

/**
 * Inicia tarefa em background que lê o sensor periodicamente e atualiza a última amostra.
 *
 * @param read_period_ms Intervalo entre leituras (por exemplo 20 ms ≈ 50 Hz).
 */
esp_err_t mpu6050_start_read_task(uint32_t read_period_ms);

/** Copia a última leitura feita pela tarefa (thread-safe). */
esp_err_t mpu6050_get_last_sample(mpu6050_sample_t *out);

#ifdef __cplusplus
}
#endif
