#ifndef ULTRASONIC_H
#define ULTRASONIC_H


#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>     // ← added for qsort
#include <stdint.h>     // ← added for uintptr_t
#include <stdbool.h>

// if you need motor API:
#include "motor.h" // adjust if different

static void gpio_callback(uint gpio, uint32_t events);
void ultrasonic_init_pins(uint trig_pin, uint echo_pin);   // call once at startup
double ultrasonic_get_distance_cm(void);                   // blocking single measurement (median)
void ultrasonic_start_monitor_task(uint32_t poll_ms, double stop_cm, double clear_cm); // creates RTOS task

#endif