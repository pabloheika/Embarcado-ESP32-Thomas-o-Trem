#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Inicializa LEDC PWM (10 kHz, 8-bit), GPIO Enable (D12), e ADC para sensores de corrente.
 * Motores começam DESABILITADOS (Enable=LOW) por segurança.
 */
esp_err_t motor_driver_init(void);

/** Habilita todos os módulos BTS7960 (D12=HIGH). Chamar antes de mover. */
void motor_driver_enable(void);

/** Desabilita todos os módulos BTS7960 (D12=LOW). Motores em repouso. */
void motor_driver_disable(void);

/** Verifica se módulos estão habilitados. */
bool motor_driver_is_enabled(void);

/**
 * Controla Motor Esquerdo.
 * @param duty 0-255
 * @param forward true=frente, false=trás
 */
void motor_left_set(uint8_t duty, bool forward);
void motor_left_brake(void);

/**
 * Controla Motor Direito.
 * @param duty 0-255
 * @param forward true=frente, false=trás
 */
void motor_right_set(uint8_t duty, bool forward);
void motor_right_brake(void);

/**
 * Controla Atuador Linear (acoplamento de carga).
 * @param duty 0-255
 * @param forward true=subir, false=descer
 */
void actuator_set(uint8_t duty, bool forward);
void actuator_brake(void);

/** Parada de emergência: para tudo e desabilita Enable. */
void motor_emergency_stop(void);

/** Reset de latch térmico do BTS7960 (pulso LOW de 5us no Enable). */
void motor_reset_thermal_latch(void);

/** Inicia task FreeRTOS que monitora corrente via ADC a cada 50ms. */
esp_err_t motor_start_current_monitor(void);

/** Retorna duty atual do motor esquerdo (0-255, negativo=trás). */
int16_t motor_get_left_duty(void);
/** Retorna duty atual do motor direito. */
int16_t motor_get_right_duty(void);

#ifdef __cplusplus
}
#endif
