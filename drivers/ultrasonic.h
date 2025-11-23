#ifndef ULTRASONIC_H
#define ULTRASONIC_H


#include "pico/stdlib.h"
#define ULTRASONIC_TRIG_PIN 4 // for send ultrasonic pulse
#define ULTRASONIC_ECHO_PIN 5 // for receive ultrasonic pulse

typedef struct
{
    bool obstacle_found;
    double width_cm;
    double distance_cm;
    double short_distance_cm;
    int right_edge_angle;
    int left_edge_angle;
    double right_edge_dist;
    double left_edge_dist;
    int middle_angle;
} ScanResult;


static void gpio_callback(uint gpio, uint32_t events);
void ultrasonic_init_pins();   // call once at startup
double ultrasonic_get_distance_cm(void);                   // blocking single measurement (median)
double ultrasonic_get_stable_distance_cm(int samples, int delay_ms);

// --- RTOS-friendly APIs ---
// Create and run the ultrasonic sampling FreeRTOS task. Pass desired poll period in ms.
void ultrasonic_start_task(uint32_t poll_ms);
void ultrasonic_pause(void);
void ultrasonic_resume(void);
// Retrieve last sampled distance (thread-safe). Returns 0.0 for invalid/timeout.
double ultrasonic_get_last_distance_cm(void);
// Stop the ultrasonic sampling task (if running). Safe to call before using blocking APIs.
void ultrasonic_stop_task(void);

#endif