#include "motor_driver.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "motor";

#define PIN_ENABLE          32
#define PIN_LEFT_R_PWM      18
#define PIN_LEFT_L_PWM      19
#define PIN_RIGHT_R_PWM     17
#define PIN_RIGHT_L_PWM     16
#define PIN_ACT_R_PWM        4
#define PIN_ACT_L_PWM        2

#define ADC_LEFT_R_IS       ADC_CHANNEL_0   // GPIO36
#define ADC_LEFT_L_IS       ADC_CHANNEL_3   // GPIO39
#define ADC_RIGHT_R_IS      ADC_CHANNEL_6   // GPIO34
#define ADC_RIGHT_L_IS      ADC_CHANNEL_7   // GPIO35

#define PWM_FREQUENCY_HZ    10000
#define PWM_RESOLUTION      LEDC_TIMER_8_BIT
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE

#define CH_LEFT_R           LEDC_CHANNEL_0
#define CH_LEFT_L           LEDC_CHANNEL_1
#define CH_RIGHT_R          LEDC_CHANNEL_2
#define CH_RIGHT_L          LEDC_CHANNEL_3
#define CH_ACT_R            LEDC_CHANNEL_4
#define CH_ACT_L            LEDC_CHANNEL_5

#define CURRENT_THRESHOLD   3500
#define CURRENT_SAMPLES     5

static adc_oneshot_unit_handle_t adc1_handle;
static bool s_enabled = false;
static int16_t s_left_duty = 0;
static int16_t s_right_duty = 0;

static void set_channel_duty(ledc_channel_t channel, uint32_t duty) {
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

static uint32_t read_adc_average(adc_channel_t channel, uint8_t samples) {
    int val = 0;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        if (adc_oneshot_read(adc1_handle, channel, &val) == ESP_OK) {
            sum += (uint32_t)val;
        }
    }
    return sum / samples;
}

esp_err_t motor_driver_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_ENABLE),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_ENABLE, 0);

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = PWM_FREQUENCY_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    const struct {
        ledc_channel_t channel;
        int            gpio;
    } ch_map[] = {
        { CH_LEFT_R,  PIN_LEFT_R_PWM  },
        { CH_LEFT_L,  PIN_LEFT_L_PWM  },
        { CH_RIGHT_R, PIN_RIGHT_R_PWM },
        { CH_RIGHT_L, PIN_RIGHT_L_PWM },
        { CH_ACT_R,   PIN_ACT_R_PWM   },
        { CH_ACT_L,   PIN_ACT_L_PWM   },
    };

    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_MODE,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0,
    };

    for (int i = 0; i < 6; i++) {
        ch_cfg.channel = ch_map[i].channel;
        ch_cfg.gpio_num = ch_map[i].gpio;
        ledc_channel_config(&ch_cfg);
    }

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_LEFT_R_IS, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_LEFT_L_IS, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_RIGHT_R_IS, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_RIGHT_L_IS, &config));

    ESP_LOGI(TAG, "Motor driver inicializado");
    return ESP_OK;
}

void motor_driver_enable(void) {
    gpio_set_level(PIN_ENABLE, 1);
    s_enabled = true;
    ESP_LOGI(TAG, "Motores habilitados");
}

void motor_driver_disable(void) {
    gpio_set_level(PIN_ENABLE, 0);
    s_enabled = false;
    ESP_LOGI(TAG, "Motores desabilitados");
}

bool motor_driver_is_enabled(void) {
    return s_enabled;
}

void motor_left_set(uint8_t duty, bool forward) {
    if (forward) {
        set_channel_duty(CH_LEFT_R, duty);
        set_channel_duty(CH_LEFT_L, 0);
        s_left_duty = duty;
    } else {
        set_channel_duty(CH_LEFT_R, 0);
        set_channel_duty(CH_LEFT_L, duty);
        s_left_duty = -duty;
    }
}

void motor_left_brake(void) {
    set_channel_duty(CH_LEFT_R, 0);
    set_channel_duty(CH_LEFT_L, 0);
    s_left_duty = 0;
}

void motor_right_set(uint8_t duty, bool forward) {
    if (forward) {
        set_channel_duty(CH_RIGHT_R, duty);
        set_channel_duty(CH_RIGHT_L, 0);
        s_right_duty = duty;
    } else {
        set_channel_duty(CH_RIGHT_R, 0);
        set_channel_duty(CH_RIGHT_L, duty);
        s_right_duty = -duty;
    }
}

void motor_right_brake(void) {
    set_channel_duty(CH_RIGHT_R, 0);
    set_channel_duty(CH_RIGHT_L, 0);
    s_right_duty = 0;
}

void actuator_set(uint8_t duty, bool forward) {
    ESP_LOGI(TAG, "Atuador acionado: duty=%u, direcao=%s", duty, forward ? "SUBINDO" : "DESCENDO");
    if (forward) {
        set_channel_duty(CH_ACT_R, duty);
        set_channel_duty(CH_ACT_L, 0);
    } else {
        set_channel_duty(CH_ACT_R, 0);
        set_channel_duty(CH_ACT_L, duty);
    }
}

void actuator_brake(void) {
    ESP_LOGI(TAG, "Atuador parado (brake)");
    set_channel_duty(CH_ACT_R, 0);
    set_channel_duty(CH_ACT_L, 0);
}

void motor_emergency_stop(void) {
    ESP_LOGE(TAG, "PARADA DE EMERGÊNCIA ACIONADA");
    motor_left_brake();
    motor_right_brake();
    actuator_brake();
    motor_driver_disable();
}

void motor_reset_thermal_latch(void) {
    ESP_LOGW(TAG, "Reset térmico BTS7960");
    gpio_set_level(PIN_ENABLE, 0);
    ets_delay_us(5);
    gpio_set_level(PIN_ENABLE, 1);
}

int16_t motor_get_left_duty(void) {
    return s_left_duty;
}

int16_t motor_get_right_duty(void) {
    return s_right_duty;
}

static void current_monitor_task(void *arg) {
    uint8_t overcurrent_count_left = 0;
    uint8_t overcurrent_count_right = 0;

    for (;;) {
        uint32_t left_r  = read_adc_average(ADC_LEFT_R_IS, CURRENT_SAMPLES);
        uint32_t left_l  = read_adc_average(ADC_LEFT_L_IS, CURRENT_SAMPLES);
        uint32_t right_r = read_adc_average(ADC_RIGHT_R_IS, CURRENT_SAMPLES);
        uint32_t right_l = read_adc_average(ADC_RIGHT_L_IS, CURRENT_SAMPLES);

        uint32_t left_max  = (left_r > left_l) ? left_r : left_l;
        uint32_t right_max = (right_r > right_l) ? right_r : right_l;

        if (left_max >= CURRENT_THRESHOLD) {
            overcurrent_count_left++;
            if (overcurrent_count_left >= CURRENT_SAMPLES) {
                ESP_LOGE(TAG, "Overcurrent Motor Esquerdo: %lu", left_max);
                motor_emergency_stop();
                overcurrent_count_left = 0;
            }
        } else {
            overcurrent_count_left = 0;
        }

        if (right_max >= CURRENT_THRESHOLD) {
            overcurrent_count_right++;
            if (overcurrent_count_right >= CURRENT_SAMPLES) {
                ESP_LOGE(TAG, "Overcurrent Motor Direito: %lu", right_max);
                motor_emergency_stop();
                overcurrent_count_right = 0;
            }
        } else {
            overcurrent_count_right = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t motor_start_current_monitor(void) {
    BaseType_t res = xTaskCreate(current_monitor_task, "current_mon", 2048, NULL, 6, NULL);
    return res == pdPASS ? ESP_OK : ESP_FAIL;
}
