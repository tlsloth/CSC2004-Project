#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"

void pid_init(void);
float pid_compute_speed(float target_speed, float measured_speed);
float pid_compute_heading(float heading_error);
void pid_get_heading_gains(float *kp, float *ki, float *kd);

#endif
