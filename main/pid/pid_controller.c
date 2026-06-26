#include "pid_controller.h"
#include "esp_log.h"

static const char *TAG = "pid";

void pid_init(pid_state_t *pid, float kp, float ki, float kd) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 1000.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = 10000.0f;
    pid->output_min = -255.0f;
    pid->output_max = 255.0f;
    pid->last_output = 0.0f;
    pid->last_error = 0.0f;
    ESP_LOGI(TAG, "PID inicializado com Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
}

void pid_set_params(pid_state_t *pid, float kp, float ki, float kd) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    ESP_LOGI(TAG, "Parâmetros PID atualizados: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
}

void pid_reset(pid_state_t *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_output = 0.0f;
    pid->last_error = 0.0f;
}

float pid_compute(pid_state_t *pid, float measurement, float dt) {
    if (!pid || dt <= 0.0f) return 0.0f;

    float error = pid->setpoint - measurement;
    pid->last_error = error;

    float p_out = pid->kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_out = pid->ki * pid->integral;

    float derivative = (error - pid->prev_error) / dt;
    float d_out = pid->kd * derivative;

    float output = p_out + i_out + d_out;

    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    pid->last_output = output;
    pid->prev_error = error;

    return output;
}
