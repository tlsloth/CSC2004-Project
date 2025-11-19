#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

#define ENC_LEFT_PIN 2
#define ENC_RIGHT_PIN 16

void encoder_irq(uint gpio, uint32_t events);
void encoder_init(void);
float encoder_get_left_rpm(void);
float encoder_get_right_rpm(void);
float encoder_get_distance_m(void);
void encoder_get_ticks(uint32_t *left, uint32_t *right); 

#endif
