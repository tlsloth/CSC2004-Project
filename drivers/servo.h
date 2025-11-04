#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"

void servo_init(uint servo_pin);

void servo_set_angle(uint servoPin, int angle);

#endif // SERVO_H