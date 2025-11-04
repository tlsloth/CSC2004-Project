#include "encoder.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"

#define TICKS_PER_REV 25.0f   // Encoder ticks per wheel revolution
#define WHEEL_CIRC 0.21f       // Wheel circumference in meters
#define RPM_INTERVAL_MS 50      // Compute RPM every 50 ms
#define MAX_TICKS_HISTORY 5     // Moving average for RPM smoothing

volatile int32_t left_ticks = 0;
volatile int32_t right_ticks = 0;

// Distance traveled
static float dist_left = 0.0f;
static float dist_right = 0.0f;

// RPM smoothing arrays
static float rpm_left_history[MAX_TICKS_HISTORY] = {0};
static float rpm_right_history[MAX_TICKS_HISTORY] = {0};
static int rpm_index = 0;

// Last tick counts for RPM calculation
static int32_t last_left_ticks = 0;
static int32_t last_right_ticks = 0;

// RPM values
static float rpm_left = 0.0f;
static float rpm_right = 0.0f;

// Timestamp for last RPM calculation
static absolute_time_t last_rpm_time;

// =========================
// Encoder IRQ Handler
// =========================
void encoder_irq(uint gpio, uint32_t events) {
    if (gpio == ENC_LEFT_PIN) {
        left_ticks++;
        dist_left = (left_ticks / TICKS_PER_REV) * WHEEL_CIRC;
    }
    if (gpio == ENC_RIGHT_PIN) {
        right_ticks++;
        dist_right = (right_ticks / TICKS_PER_REV) * WHEEL_CIRC;
    }
}

// =========================
// Initialize Encoder Pins
// =========================
void encoder_init(void) {
    gpio_init(ENC_LEFT_PIN);
    gpio_set_dir(ENC_LEFT_PIN, GPIO_IN);
    gpio_pull_up(ENC_LEFT_PIN);
    gpio_set_irq_enabled_with_callback(ENC_LEFT_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_irq);

    gpio_init(ENC_RIGHT_PIN);
    gpio_set_dir(ENC_RIGHT_PIN, GPIO_IN);
    gpio_pull_up(ENC_RIGHT_PIN);
    gpio_set_irq_enabled(ENC_RIGHT_PIN, GPIO_IRQ_EDGE_RISE, true);

    last_rpm_time = get_absolute_time();
}

// =========================
// Update RPM (call periodically)
// =========================
void encoder_update_rpm(void) {
    absolute_time_t now = get_absolute_time();
    int64_t dt_us = absolute_time_diff_us(last_rpm_time, now);

    if (dt_us >= RPM_INTERVAL_MS * 1000) {
        // Compute ticks in interval
        int32_t delta_left = left_ticks - last_left_ticks;
        int32_t delta_right = right_ticks - last_right_ticks;

        // RPM = (ticks / ticks_per_rev) * (60 s / interval in seconds)
        float interval_sec = dt_us / 1e6f;
        float rpm_l = (delta_left / TICKS_PER_REV) * (60.0f / interval_sec);
        float rpm_r = (delta_right / TICKS_PER_REV) * (60.0f / interval_sec);

        // Store in moving average
        rpm_left_history[rpm_index] = rpm_l;
        rpm_right_history[rpm_index] = rpm_r;
        rpm_index = (rpm_index + 1) % MAX_TICKS_HISTORY;

        // Compute average
        float sum_l = 0, sum_r = 0;
        for (int i = 0; i < MAX_TICKS_HISTORY; i++) {
            sum_l += rpm_left_history[i];
            sum_r += rpm_right_history[i];
        }
        rpm_left = sum_l / MAX_TICKS_HISTORY;
        rpm_right = sum_r / MAX_TICKS_HISTORY;

        last_left_ticks = left_ticks;
        last_right_ticks = right_ticks;
        last_rpm_time = now;
    }
}

// =========================
// Get Current RPM
// =========================
float encoder_get_left_rpm(void) {
    encoder_update_rpm();
    return rpm_left;
}

float encoder_get_right_rpm(void) {
    encoder_update_rpm();
    return rpm_right;
}

// =========================
// Get Average Distance (m)
// =========================
float encoder_get_distance_m(void) {
    return (dist_left + dist_right) / 2.0f;
}

// =========================
// Get Raw Tick Counts
// =========================
void encoder_get_ticks(uint32_t *left, uint32_t *right) {
    if (left)  *left = left_ticks;
    if (right) *right = right_ticks;
}
