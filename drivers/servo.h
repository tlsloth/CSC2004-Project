#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdint.h>

#define SERVO_PIN 15
#define SERVO_SCAN_START_ANGLE 30
#define SERVO_SCAN_END_ANGLE 150

void servo_init();

void servo_set_angle(uint servoPin, int angle);

// RTOS-friendly API
// Start the servo task which processes async angle commands
void servo_start_task(void);
void servo_pause_task(void);
void servo_resume_task(void);
void servo_stop_task(void);
// Send an async angle command to the servo task. Timeout_ms is in milliseconds.
// Returns true if the command was queued.
bool servo_set_angle_async(int angle, uint32_t timeout_ms);

#endif // SERVO_H