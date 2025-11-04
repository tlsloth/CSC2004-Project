#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
//#include "FreeRTOS.h"
//#include "task.h"
#include <stdint.h>     // ← added for uintptr_t
#include <stdbool.h>
#include <stdlib.h>  

// if you need motor API:
#include "motor.h" // adjust if different

// pins (set by init)
static uint ULTRA_TRIG_PIN = 4;
static uint ULTRA_ECHO_PIN = 5;

// ISR-safe 32-bit timestamps
volatile static uint32_t echo_start_us = 0;
volatile static uint32_t pulse_dur_us = 0;
volatile static bool echo_completed = false;
volatile static bool echo_rising = false;

#define BASE_SPEED 0.30f  

// basic init: set pins and enable callback
void ultrasonic_init_pins(uint trig_pin, uint echo_pin) {
    ULTRA_TRIG_PIN = trig_pin;
    ULTRA_ECHO_PIN = echo_pin;
    gpio_init(ULTRA_TRIG_PIN);
    gpio_set_dir(ULTRA_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRA_TRIG_PIN, 0);

    gpio_init(ULTRA_ECHO_PIN);
    gpio_set_dir(ULTRA_ECHO_PIN, GPIO_IN);
    // enable both edges, provide our callback function (was NULL)
    gpio_set_irq_enabled_with_callback(ULTRA_ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// GPIO callback (the SDK will call this; keep it short)
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio != ULTRA_ECHO_PIN) return;
    if (events & GPIO_IRQ_EDGE_RISE) {
        echo_start_us = time_us_32();
        echo_completed = false;
        echo_rising = true;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        if (echo_rising) {
            uint32_t end = time_us_32();
            pulse_dur_us = end - echo_start_us; // wrap-safe
            echo_completed = true;
            echo_rising = false;
        }
    }
}

// send trigger pulse (blocking micro delays ok)
static void trigger_pulse(void) {
    gpio_put(ULTRA_TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(ULTRA_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRA_TRIG_PIN, 0);
}

// single measurement (blocks up to timeout_ms)
double ultrasonic_get_distance_cm(void) {
    // clear previous flags
    echo_completed = false;
    pulse_dur_us = 0;
    trigger_pulse();

    uint32_t start = time_us_32();
    const uint32_t timeout_us = 50000u;
    while (!echo_completed) {
        if ((uint32_t)(time_us_32() - start) > timeout_us) return 0.0; // timeout -> invalid
        sleep_us(100);
    }

    uint32_t dur = pulse_dur_us;
    if (dur == 0 || dur > 30000u) return 0.0;
    // precise conversion: (us * 0.0343) / 2
    return (dur * 0.0343) / 2.0;
}

// add comparator for qsort
static int compare_double(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}




double ultrasonic_get_stable_distance_cm(int num_samples,int delay_ms) {
    if (num_samples % 2 == 0){
        num_samples++; 
    }

    double samples[num_samples];

    for (int i = 0; i < num_samples; i++) {
        samples[i] = ultrasonic_get_distance_cm();
        sleep_ms(30); 
    }

    qsort(samples, num_samples, sizeof(double), compare_double);

    return samples[num_samples / 2];
}







/*
// Monitor task: poll distance and stop/resume motors
static void ultrasonic_monitor_task(void *pv) {
    // interpret pv as poll_ms passed from xTaskCreate (cast via uintptr_t)
    uint32_t poll_ms = 100;
    if (pv) poll_ms = (uint32_t)(uintptr_t)pv;
    const TickType_t poll = pdMS_TO_TICKS(poll_ms);

    for (;;) {
        double dist = get_stable_distance(3, 60); // 3 samples, 60ms spacing
        // ignore invalid reading
        if (dist > 0.0) {
            // use external motor API: motor_stop() / motor_resume() or adapt names
            static bool stopped = false;
            const double STOP_CM = 20.0;
            const double CLEAR_CM = 25.0;
            if (!stopped && dist <= STOP_CM) {
                stopped = true;
                motor_stop();    // implement in drivers/motor.c
            } else if (stopped && dist >= CLEAR_CM) {
                stopped = false;
                motor_set_speed(BASE_SPEED, BASE_SPEED);  // implement or call motor_set_speed(...)
            }
        }
        vTaskDelay(poll); // use configured poll period
    }
}

// start the monitor task (call after RTOS is up)
void ultrasonic_start_monitor_task(uint32_t poll_ms, double stop_cm, double clear_cm) {
    // you can store poll/thresholds in static variables if you want; using constants above is fine
    BaseType_t rc = xTaskCreate(ultrasonic_monitor_task, "UltraMon", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    (void)rc;
}

*/
