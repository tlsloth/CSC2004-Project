#ifndef ULTRASONIC_H
#define ULTRASONIC_H


#include "pico/stdlib.h"


static void gpio_callback(uint gpio, uint32_t events);
void ultrasonic_init_pins(uint trig_pin, uint echo_pin);   // call once at startup
double ultrasonic_get_distance_cm(void);                   // blocking single measurement (median)
//void ultrasonic_start_monitor_task(uint32_t poll_ms, double stop_cm, double clear_cm); // creates RTOS task
double ultrasonic_get_stable_distance_cm(int samples, int delay_ms);

#endif