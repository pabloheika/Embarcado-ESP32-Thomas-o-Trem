#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_error;
    float integral_limit;
    float output_min;
    float output_max;
    float last_output;
    float last_error;
} pid_state_t;

void pid_init(pid_state_t *pid, float kp, float ki, float kd);
void pid_set_params(pid_state_t *pid, float kp, float ki, float kd);
void pid_reset(pid_state_t *pid);
float pid_compute(pid_state_t *pid, float measurement, float dt);

#ifdef __cplusplus
}
#endif
