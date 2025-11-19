#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

// ===== Motor Driver API for RoboPico =====

// Initialize motor driver (PWM + direction pins)
void motor_init(void);

// Set motor speed
// left, right: range -1.0 (full reverse) to 1.0 (full forward)
void motor_set_speed(float left, float right);

// Stop both motors immediately
void motor_stop(void);

#endif // MOTOR_H
