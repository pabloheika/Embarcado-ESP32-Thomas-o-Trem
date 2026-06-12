#include "mode_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys_logger.h"
#include "motor_driver.h"
#include "qtr8rc.h"
#include "line_follower.h"

static const char *TAG = "mode_mgr";

static car_state_t s_state;

esp_err_t mode_manager_init(void) {
    s_state.mode = MODE_IDLE;
    s_state.base_speed = 150;
    s_state.motors_enabled = false;
    
    pid_init(&s_state.pid, 0.2f, 0.0f, 1.0f);
    line_follower_init();
    
    sys_log(SYS_LOG_INFO, TAG, "Mode manager inicializado");
    return ESP_OK;
}

car_mode_t mode_manager_get_mode(void) {
    return s_state.mode;
}

car_state_t *mode_manager_get_state(void) {
    return &s_state;
}

esp_err_t mode_manager_set_mode(car_mode_t new_mode) {
    if (s_state.mode == MODE_EMERGENCY && new_mode != MODE_IDLE) {
        sys_log(SYS_LOG_WARN, TAG, "Em emergência! Volte para IDLE primeiro.");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (new_mode == s_state.mode) return ESP_OK;

    // Safety first: stop motors on mode change
    motor_left_brake();
    motor_right_brake();
    if (new_mode == MODE_IDLE || new_mode == MODE_CALIBRATING || new_mode == MODE_EMERGENCY) {
        motor_driver_disable();
        s_state.motors_enabled = false;
    } else {
        motor_driver_enable();
        s_state.motors_enabled = true;
    }

    if (new_mode == MODE_AUTONOMOUS) {
        if (!qtr8rc_is_calibrated()) {
            sys_log(SYS_LOG_WARN, TAG, "Sensores não calibrados! Não é possível iniciar autônomo.");
            return ESP_ERR_INVALID_STATE;
        }
        pid_reset(&s_state.pid);
    }

    s_state.mode = new_mode;
    
    const char *mode_strs[] = {"IDLE", "CALIBRATING", "AUTONOMOUS", "MANUAL", "EMERGENCY"};
    sys_log(SYS_LOG_INFO, TAG, "Modo alterado para %s", mode_strs[new_mode]);

    return ESP_OK;
}

static void control_loop_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        switch (s_state.mode) {
            case MODE_AUTONOMOUS: {
                qtr8rc_reading_t reading;
                if (qtr8rc_get_last_reading(&reading) == ESP_OK) {
                    line_status_t line = line_follower_compute(&reading);
                    
                    if (line.line_detected) {
                        float correction = pid_compute(&s_state.pid, (float)line.position, 0.01f);
                        
                        int16_t left_duty = s_state.base_speed + (int16_t)correction;
                        int16_t right_duty = s_state.base_speed - (int16_t)correction;
                        
                        // Limit to 0-255
                        if (left_duty > 255) left_duty = 255;
                        if (left_duty < 0) left_duty = 0;
                        if (right_duty > 255) right_duty = 255;
                        if (right_duty < 0) right_duty = 0;
                        
                        motor_left_set((uint8_t)left_duty, true);
                        motor_right_set((uint8_t)right_duty, true);
                    } else {
                        // Perdeu a linha, para ou tenta recuperar com last_valid_position?
                        // Por segurança, vamos parar e esperar a linha voltar
                        motor_left_brake();
                        motor_right_brake();
                    }
                }
                break;
            }
            case MODE_CALIBRATING:
                qtr8rc_calibrate_blocking(20000); // 20 seconds calibration
                mode_manager_set_mode(MODE_IDLE);
                break;
            case MODE_MANUAL:
            case MODE_IDLE:
            case MODE_EMERGENCY:
            default:
                // Control is done via API in manual mode
                // Or just chilling in IDLE
                break;
        }
    }
}

esp_err_t mode_manager_start_control_loop(void) {
    BaseType_t res = xTaskCreatePinnedToCore(
        control_loop_task, 
        "control_loop", 
        4096, 
        NULL, 
        7, 
        NULL, 
        1 // Core 1
    );
    return res == pdPASS ? ESP_OK : ESP_FAIL;
}
