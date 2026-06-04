#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODE_IDLE,          // Parado, aguardando comando
    MODE_CALIBRATING,   // Calibração do QTR-8RC
    MODE_AUTONOMOUS,    // Seguidor de linha com PID
    MODE_MANUAL,        // Controle manual via web
    MODE_EMERGENCY,     // Parada de emergência
} car_mode_t;

typedef struct {
    car_mode_t mode;
    uint8_t base_speed;     // Velocidade base PWM (0-255)
    pid_state_t pid;
    bool motors_enabled;
} car_state_t;

esp_err_t mode_manager_init(void);
car_mode_t mode_manager_get_mode(void);
esp_err_t mode_manager_set_mode(car_mode_t new_mode);
car_state_t *mode_manager_get_state(void);
esp_err_t mode_manager_start_control_loop(void);

#ifdef __cplusplus
}
#endif
