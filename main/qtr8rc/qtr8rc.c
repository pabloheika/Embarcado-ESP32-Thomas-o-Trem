#include "qtr8rc.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "qtr8rc";

/*
 * Pinos — adapte ao hardware.
 *
 * IR:       GPIO 5  (controla LEDs infravermelhos do módulo QTR-8RC)
 * Sensores: GPIO 15, 33, 25, 26, 27, 14, 13, 23  (D1..D8)
 *
 * Restrições observadas:
 *   - GPIO 34/35/36/39: somente entrada, não servem para leitura RC.
 *   - GPIO 21/22: reservados ao I2C do MPU6050.
 *   - GPIO 32: reservado ao PIN_ENABLE dos drivers BTS7960.
 *   - GPIO 12: strapping pin com pull-down (usado pelo enable dos motores).
 */
#define QTR_IR_PIN GPIO_NUM_5

static const gpio_num_t s_qtr_pins[QTR8RC_NUM_SENSORS] = {
    GPIO_NUM_NC,  // D1 (Desativado, era GPIO_NUM_15)
    GPIO_NUM_33,  // D2
    GPIO_NUM_25,  // D3
    GPIO_NUM_26,  // D4
    GPIO_NUM_27,  // D5
    GPIO_NUM_14,  // D6
    GPIO_NUM_13,  // D7
    GPIO_NUM_23,  // D8
};

/*
 * Timeout máximo de leitura em microssegundos.
 * Reduzido para 3000 us para evitar bloqueios excessivos de CPU e
 * inanição do watchdog. Leituras capacitivas de superfícies pretas
 * tipicamente ocorrem entre 2000 us e 3000 us.
 */
#define QTR_TIMEOUT_US 3000

/* Tempo para carregar o capacitor interno do circuito RC (us). */
#define QTR_CHARGE_TIME_US 10

#define QTR8RC_READ_STACK_WORDS 4096
#define QTR8RC_READ_TASK_PRIO 5
#define QTR8RC_LOG_EVERY_N_READS 10

static uint32_t s_calib_min[QTR8RC_NUM_SENSORS];
static uint32_t s_calib_max[QTR8RC_NUM_SENSORS];
static bool s_calibrated = false;

static uint16_t s_black_threshold = 600;
static int s_min_black_sensors = 1;

/* Mutex para acesso exclusivo ao hardware GPIO durante leitura RC. */
static SemaphoreHandle_t s_hardware_mutex;
/* Mutex para acesso thread-safe à última leitura consolidada. */
static SemaphoreHandle_t s_reading_mutex;
static qtr8rc_reading_t s_last_reading;
static TaskHandle_t s_read_task = NULL;
static uint32_t s_read_period_ms = 100;

static void qtr_ir_on(void)
{
    gpio_set_level(QTR_IR_PIN, 1);
}

/*
 * Leitura bruta dos 8 sensores QTR-8RC (thread-safe via hardware mutex).
 *
 * Etapa 1: Carrega os capacitores internos colocando GPIOs como saída HIGH.
 * Etapa 2: Muda para entrada (floating) e mede o tempo de descarga.
 *
 * Superfícies brancas refletem mais IR → descarga rápida (tempo menor).
 * Superfícies pretas absorvem IR → descarga lenta (tempo maior / timeout).
 */
static void qtr_read_raw_impl(uint32_t values[QTR8RC_NUM_SENSORS])
{
    if (xSemaphoreTake(s_hardware_mutex, portMAX_DELAY) != pdTRUE) {
        for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
            values[i] = QTR_TIMEOUT_US;
        }
        return;
    }

    /* Pré-inicializar com timeout (caso algum sensor não responda). */
    for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
        values[i] = QTR_TIMEOUT_US;
    }

    /* Etapa 1: Carregar capacitores (GPIO como saída em HIGH). */
    for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
        if (s_qtr_pins[i] != GPIO_NUM_NC) {
            gpio_set_direction(s_qtr_pins[i], GPIO_MODE_OUTPUT);
            gpio_set_level(s_qtr_pins[i], 1);
        }
    }

    esp_rom_delay_us(QTR_CHARGE_TIME_US);

    /* Etapa 2: Mudar para entrada flutuante e cronometrar descarga. */
    int64_t start = esp_timer_get_time();

    bool finished[QTR8RC_NUM_SENSORS] = { false };
    int finished_count = 0;

    for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
        if (s_qtr_pins[i] != GPIO_NUM_NC) {
            gpio_set_direction(s_qtr_pins[i], GPIO_MODE_INPUT);
            gpio_set_pull_mode(s_qtr_pins[i], GPIO_FLOATING);
        } else {
            values[i] = 0;
            finished[i] = true;
            finished_count++;
        }
    }

    while (finished_count < QTR8RC_NUM_SENSORS) {
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);

        if (elapsed >= QTR_TIMEOUT_US) {
            break;
        }

        for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
            if (!finished[i] && gpio_get_level(s_qtr_pins[i]) == 0) {
                values[i] = elapsed;
                finished[i] = true;
                finished_count++;
            }
        }
    }

    xSemaphoreGive(s_hardware_mutex);
}

static uint16_t normalize_one(int index, uint32_t raw_value)
{
    uint32_t min_v = s_calib_min[index];
    uint32_t max_v = s_calib_max[index];

    if (max_v <= min_v + 1) {
        return 0;
    }

    if (raw_value <= min_v) {
        return 0;
    }

    if (raw_value >= max_v) {
        return 1000;
    }

    return (uint16_t)(((raw_value - min_v) * 1000UL) / (max_v - min_v));
}

static void qtr8rc_read_task(void *arg)
{
    (void)arg;
    uint32_t n = 0;

    for (;;) {
        uint32_t raw[QTR8RC_NUM_SENSORS];
        qtr_read_raw_impl(raw);

        qtr8rc_reading_t reading;
        for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
            reading.raw_us[i] = raw[i];
            reading.norm[i] = normalize_one(i, raw[i]);
        }

        int black_count = 0;
        uint8_t black_mask = 0;
        for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
            if (reading.norm[i] >= s_black_threshold) {
                black_count++;
                black_mask |= (uint8_t)(1U << i);
            }
        }
        reading.black_mask = black_mask;
        reading.line_detected = (black_count >= s_min_black_sensors);

        if (xSemaphoreTake(s_reading_mutex, portMAX_DELAY) == pdTRUE) {
            s_last_reading = reading;
            xSemaphoreGive(s_reading_mutex);
        }

        n++;
        if (n % QTR8RC_LOG_EVERY_N_READS == 0) {
            ESP_LOGI(TAG,
                     "raw [ %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32
                     " | %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " ]",
                     raw[0], raw[1], raw[2], raw[3],
                     raw[4], raw[5], raw[6], raw[7]);
            ESP_LOGI(TAG,
                     "norm [ %u %u %u %u | %u %u %u %u ] linha=%d cal=%d",
                     (unsigned)reading.norm[0],
                     (unsigned)reading.norm[1],
                     (unsigned)reading.norm[2],
                     (unsigned)reading.norm[3],
                     (unsigned)reading.norm[4],
                     (unsigned)reading.norm[5],
                     (unsigned)reading.norm[6],
                     (unsigned)reading.norm[7],
                     (int)reading.line_detected,
                     (int)s_calibrated);
        }

        vTaskDelay(pdMS_TO_TICKS(s_read_period_ms ? s_read_period_ms : 100));
    }
}

esp_err_t qtr8rc_init(void)
{
    if (s_reading_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_reading_mutex = xSemaphoreCreateMutex();
    if (s_reading_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_hardware_mutex = xSemaphoreCreateMutex();
    if (s_hardware_mutex == NULL) {
        vSemaphoreDelete(s_reading_mutex);
        s_reading_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Configurar pino IR como saída e ligar emissores. */
    gpio_config_t ir_conf = {
        .pin_bit_mask = 1ULL << QTR_IR_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ir_conf));
    qtr_ir_on();

    /*
     * Inicializar GPIOs dos sensores com gpio_reset_pin() para
     * limpar qualquer estado anterior (strapping, boot) e configurar
     * como entrada flutuante — necessário para leitura RC confiável.
     */
    for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
        if (s_qtr_pins[i] != GPIO_NUM_NC) {
            gpio_reset_pin(s_qtr_pins[i]);
            gpio_set_direction(s_qtr_pins[i], GPIO_MODE_INPUT);
            gpio_set_pull_mode(s_qtr_pins[i], GPIO_FLOATING);
        }
    }

    memset(&s_last_reading, 0, sizeof(s_last_reading));
    for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
        s_calib_min[i] = UINT32_MAX;
        s_calib_max[i] = 0;
    }
    s_calibrated = false;

    ESP_LOGI(TAG,
             "GPIO IR=%d; D1..D8 = %d,%d,%d,%d,%d,%d,%d,%d",
             (int)QTR_IR_PIN,
             (int)s_qtr_pins[0],
             (int)s_qtr_pins[1],
             (int)s_qtr_pins[2],
             (int)s_qtr_pins[3],
             (int)s_qtr_pins[4],
             (int)s_qtr_pins[5],
             (int)s_qtr_pins[6],
             (int)s_qtr_pins[7]);
    return ESP_OK;
}

void qtr8rc_calibrate_blocking(uint32_t duration_ms)
{
    for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
        s_calib_min[i] = UINT32_MAX;
        s_calib_max[i] = 0;
    }

    ESP_LOGI(TAG, "Calibração por %" PRIu32 " ms — mova sobre branco e preto.", duration_ms);

    int64_t end_time = esp_timer_get_time() + ((int64_t)duration_ms * 1000);
    uint32_t raw[QTR8RC_NUM_SENSORS];

    while (esp_timer_get_time() < end_time) {
        qtr_read_raw_impl(raw);

        for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
            if (raw[i] < s_calib_min[i]) {
                s_calib_min[i] = raw[i];
            }
            if (raw[i] > s_calib_max[i]) {
                s_calib_max[i] = raw[i];
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    s_calibrated = true;
    ESP_LOGI(TAG, "Calibração concluída.");

    for (int i = 0; i < QTR8RC_NUM_SENSORS; i++) {
        uint32_t delta = s_calib_max[i] - s_calib_min[i];
        ESP_LOGI(TAG, "S%d: min=%" PRIu32 " us  max=%" PRIu32 " us  delta=%" PRIu32 " us",
                 i + 1, s_calib_min[i], s_calib_max[i], delta);
        if (delta < 100) {
            ESP_LOGW(TAG, "S%d pouca variação — verifique altura, IR e trilha.", i + 1);
        }
    }
}

void qtr8rc_set_line_detection(uint16_t black_threshold, int min_sensors)
{
    if (min_sensors < 1) {
        min_sensors = 1;
    }
    if (min_sensors > QTR8RC_NUM_SENSORS) {
        min_sensors = QTR8RC_NUM_SENSORS;
    }
    s_black_threshold = black_threshold;
    s_min_black_sensors = min_sensors;
}

esp_err_t qtr8rc_read_raw(uint32_t values[QTR8RC_NUM_SENSORS])
{
    if (values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    qtr_read_raw_impl(values);
    return ESP_OK;
}

uint16_t qtr8rc_normalize_sensor(int sensor_index, uint32_t raw_value)
{
    if (sensor_index < 0 || sensor_index >= QTR8RC_NUM_SENSORS) {
        return 0;
    }
    return normalize_one(sensor_index, raw_value);
}

esp_err_t qtr8rc_start_read_task(uint32_t read_period_ms)
{
    if (s_reading_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_read_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_read_period_ms = read_period_ms ? read_period_ms : 100;

    BaseType_t ok = xTaskCreatePinnedToCore(qtr8rc_read_task, "qtr8rc_read", QTR8RC_READ_STACK_WORDS, NULL,
                              QTR8RC_READ_TASK_PRIO, &s_read_task, 0 /* Core 0 — I/O */);
    if (ok != pdPASS) {
        s_read_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "tarefa de leitura iniciada (%lu ms)", (unsigned long)s_read_period_ms);
    return ESP_OK;
}

esp_err_t qtr8rc_get_last_reading(qtr8rc_reading_t *out)
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

bool qtr8rc_is_calibrated(void)
{
    return s_calibrated;
}
