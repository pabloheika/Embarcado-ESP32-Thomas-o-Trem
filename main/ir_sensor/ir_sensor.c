#include "ir_sensor.h"

#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "ir_sensor";

/*
 * Pinos — adapte ao hardware.
 *
 * E18-D80NK: sensores digitais de proximidade infravermelha.
 *   - Sensor original:  LOW  = detectado (preto), HIGH = não detectado (branco).
 *   - Sensor alternativo (fornecedor diferente): lógica invertida
 *     HIGH = detectado (preto), LOW = não detectado (branco).
 *
 * GPIO 25: sensor esquerdo  (fornecedor alternativo — invertido)
 * GPIO 26: sensor central   (fornecedor original)
 * GPIO 27: sensor direito   (fornecedor alternativo — invertido)
 */
static const gpio_num_t s_ir_pins[IR_SENSOR_NUM_SENSORS] = {
    GPIO_NUM_25,  /* Esquerdo */
    GPIO_NUM_26,  /* Centro   */
    GPIO_NUM_27,  /* Direito  */
};

/*
 * Máscara de inversão por sensor.
 * true  = sensor com lógica invertida (HIGH = detectado, fornecedor alternativo).
 * false = sensor com lógica padrão    (LOW  = detectado, fornecedor original).
 *
 * Ajuste conforme o fornecedor de cada sensor instalado.
 */
static const bool s_ir_inverted[IR_SENSOR_NUM_SENSORS] = {
    true,   /* Esquerdo — fornecedor alternativo (invertido) */
    false,  /* Centro   — fornecedor original */
    true,   /* Direito  — fornecedor alternativo (invertido) */
};

#define IR_SENSOR_READ_STACK_WORDS 2048
#define IR_SENSOR_READ_TASK_PRIO   5
#define IR_SENSOR_LOG_EVERY_N      10

/* Mutex para acesso thread-safe à última leitura consolidada. */
static SemaphoreHandle_t s_reading_mutex = NULL;
static ir_sensor_reading_t s_last_reading;
static TaskHandle_t s_read_task = NULL;
static uint32_t s_read_period_ms = 40;

/**
 * Lê os 3 sensores digitais e preenche a struct de leitura.
 * E18-D80NK: gpio_get_level() == 0 → linha detectada (preto).
 */
static void ir_sensor_read(ir_sensor_reading_t *reading)
{
    uint8_t mask = 0;
    bool any = false;

    for (int i = 0; i < IR_SENSOR_NUM_SENSORS; i++) {
        /*
         * Leitura com compensação de polaridade:
         *   - Sensor padrão (s_ir_inverted[i] == false): LOW  = detectado.
         *   - Sensor invertido (s_ir_inverted[i] == true):  HIGH = detectado.
         *
         * O XOR (!=) entre o nível bruto e o flag de inversão resolve
         * ambas as polaridades com uma única expressão.
         */
        int raw_level = gpio_get_level(s_ir_pins[i]);
        bool det = (raw_level == 0) != s_ir_inverted[i];
        reading->detected[i] = det;
        if (det) {
            mask |= (uint8_t)(1U << i);
            any = true;
        }
    }

    reading->black_mask = mask;
    reading->line_detected = any;
}

static void ir_sensor_read_task(void *arg)
{
    (void)arg;
    uint32_t n = 0;

    for (;;) {
        ir_sensor_reading_t reading;
        ir_sensor_read(&reading);

        if (xSemaphoreTake(s_reading_mutex, portMAX_DELAY) == pdTRUE) {
            s_last_reading = reading;
            xSemaphoreGive(s_reading_mutex);
        }

        n++;
        if (n % IR_SENSOR_LOG_EVERY_N == 0) {
            ESP_LOGI(TAG, "sensores [ E=%d C=%d D=%d ] mask=0x%02X linha=%d",
                     (int)reading.detected[IR_SENSOR_LEFT],
                     (int)reading.detected[IR_SENSOR_CENTER],
                     (int)reading.detected[IR_SENSOR_RIGHT],
                     (unsigned)reading.black_mask,
                     (int)reading.line_detected);
        }

        vTaskDelay(pdMS_TO_TICKS(s_read_period_ms ? s_read_period_ms : 100));
    }
}

esp_err_t ir_sensor_init(void)
{
    if (s_reading_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_reading_mutex = xSemaphoreCreateMutex();
    if (s_reading_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /*
     * Configurar GPIOs dos sensores como entrada com pull-up interno.
     * O pull-up garante nível HIGH quando o sensor não está ativo,
     * evitando leituras flutuantes.
     */
    for (int i = 0; i < IR_SENSOR_NUM_SENSORS; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << s_ir_pins[i],
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&io_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Erro ao configurar GPIO %d: %s",
                     (int)s_ir_pins[i], esp_err_to_name(err));
            return err;
        }
    }

    memset(&s_last_reading, 0, sizeof(s_last_reading));

    ESP_LOGI(TAG, "E18-D80NK inicializado: E=%d C=%d D=%d",
             (int)s_ir_pins[IR_SENSOR_LEFT],
             (int)s_ir_pins[IR_SENSOR_CENTER],
             (int)s_ir_pins[IR_SENSOR_RIGHT]);
    return ESP_OK;
}

esp_err_t ir_sensor_start_read_task(uint32_t read_period_ms)
{
    if (s_reading_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_read_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_read_period_ms = read_period_ms ? read_period_ms : 100;

    BaseType_t ok = xTaskCreatePinnedToCore(
        ir_sensor_read_task, "ir_sensor_read",
        IR_SENSOR_READ_STACK_WORDS, NULL,
        IR_SENSOR_READ_TASK_PRIO, &s_read_task,
        0 /* Core 0 — I/O */);
    if (ok != pdPASS) {
        s_read_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "tarefa de leitura iniciada (%lu ms)", (unsigned long)s_read_period_ms);
    return ESP_OK;
}

esp_err_t ir_sensor_get_last_reading(ir_sensor_reading_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_reading_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_reading_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out = s_last_reading;
    xSemaphoreGive(s_reading_mutex);
    return ESP_OK;
}

bool ir_sensor_is_calibrated(void)
{
    /* Sensores digitais não precisam de calibração — sempre prontos. */
    return true;
}
