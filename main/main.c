#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "sys_logger.h"
#include "wifi_station.h"
#include "mpu6050.h"
#include "qtr8rc.h"
#include "motor_driver.h"
#include "mode_manager.h"
#include "web_api.h"
#include "test_modules.h"

// =========================================================================
// MODO DE TESTE (Altere para 1 para rodar o teste isolado)
// =========================================================================
#define RUN_UNIT_TESTS   0 // Testes matemáticos em C puro via Unity
#define TEST_MODE_MOTOR  0 // Testa aceleração e freio dos 3 drivers físicos

static const char *TAG = "main";

static void rotina_teste_motor(void) {
    ESP_LOGI(TAG, "=== INICIANDO ROTINA DE TESTE DOS MOTORES ===");
    ESP_ERROR_CHECK(motor_driver_init());
    motor_driver_enable();

    while (1) {
        ESP_LOGI(TAG, "Motor Esquerdo: FRENTE");
        motor_left_set(150, true);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Motor Esquerdo: FREIO");
        motor_left_brake();
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Motor Esquerdo: TRÁS");
        motor_left_set(150, false);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Motor Esquerdo: FREIO");
        motor_left_brake();
        vTaskDelay(pdMS_TO_TICKS(2000));

        // ... Faria o mesmo para direito e atuador ...
        ESP_LOGI(TAG, "Motor Direito: FRENTE");
        motor_right_set(150, true);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Motor Direito: FREIO");
        motor_right_brake();
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Motor Direito: TRÁS");
        motor_right_set(150, false);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Motor Direito: FREIO");
        motor_right_brake();
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI(TAG, "Atuador: SUBIR");
        actuator_set(255, true);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Atuador: PARAR");
        actuator_brake();
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Atuador: DESCER");
        actuator_set(255, false);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        ESP_LOGI(TAG, "Atuador: PARAR");
        actuator_brake();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    // 1. Iniciar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Sistema de logs
    sys_logger_init();
    sys_log(SYS_LOG_INFO, TAG, "=== Thomas, o Trem — Inicializando ===");

    // Verificação de Testes
    if (RUN_UNIT_TESTS) {
        run_all_unit_tests();
        sys_log(SYS_LOG_INFO, TAG, "Testes Unitários finalizados. Sistema em loop infinito seguro.");
        while(1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    if (TEST_MODE_MOTOR) {
        rotina_teste_motor();
        // Não retorna (loop infinito no teste)
    }

    // 3. WiFi (STA)
    sys_log(SYS_LOG_INFO, TAG, "Iniciando WiFi...");
    ESP_ERROR_CHECK(wifi_station_init());
    
    sys_log(SYS_LOG_INFO, TAG, "Aguardando IP do Roteador...");
    wifi_station_wait_connected(pdMS_TO_TICKS(120000));
    
    // 4. Sensores
    sys_log(SYS_LOG_INFO, TAG, "Iniciando Sensores...");
    if (mpu6050_init() == ESP_OK) {
        ESP_ERROR_CHECK(mpu6050_start_read_task(20));  // 50 Hz
    } else {
        sys_log(SYS_LOG_ERROR, TAG, "Falha no MPU6050! Fios soltos? Operando sem IMU.");
    }
    
    if (qtr8rc_init() == ESP_OK) {
        qtr8rc_set_line_detection(600, 1);
        ESP_ERROR_CHECK(qtr8rc_start_read_task(50));   // 20 Hz
    } else {
        sys_log(SYS_LOG_ERROR, TAG, "Falha no QTR-8RC! Fios soltos? Carro cego.");
    }
    
    // 5. Motores
    sys_log(SYS_LOG_INFO, TAG, "Iniciando Motores...");
    ESP_ERROR_CHECK(motor_driver_init());
    motor_start_current_monitor();
    
    // 6. Gerenciador de modos
    sys_log(SYS_LOG_INFO, TAG, "Iniciando Mode Manager...");
    ESP_ERROR_CHECK(mode_manager_init());
    ESP_ERROR_CHECK(mode_manager_start_control_loop());
    
    // 7. HTTP API
    sys_log(SYS_LOG_INFO, TAG, "Iniciando Web API...");
    ESP_ERROR_CHECK(web_api_start());
    
    sys_log(SYS_LOG_INFO, TAG, "=== Sistema Pronto — Aguardando comandos ===");
}
