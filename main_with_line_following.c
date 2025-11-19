#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"

// Drivers
#include "drivers/barcode.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/pid.h"
#include "drivers/ir_sensor.h"

// ===== Configuration (from main_reference.c) =====
#define BASE_SPEED              0.30f   // Base speed (reduced for stability)
#define MAX_CORRECTION          0.20f   // Maximum correction
#define SENSOR_OFFSET           0.0f    // No offset
#define ERROR_DEADBAND          0.08f   // Reduce oscillation on edge
#define LOOP_DELAY_MS           10      // ~100 Hz control
#define TELEMETRY_INTERVAL_MS   200     // Unused here (no prints)
#define CALIBRATION_MODE        0       // Keep 0 (no calibration loop/prints)
#define STRAIGHT_MODE           0       // 1 = drive straight, 0 = PID line following

// PID Gains for line following
#define KP 0.20f
#define KI 0.0f
#define KD 0.02f

// New: scanning behavior
#define SCAN_SPEED              0.10f   // Slow speed while scanning (0..1)

// Remove the task handle (no more suspend/resume)
// static TaskHandle_t g_line_follow_task = NULL;

// Use a flag the line-follow task will check each cycle (ISR-safe)
static volatile bool g_barcode_scanning = false;
static volatile bool g_barcode_log_start_pending = false;
static volatile bool g_barcode_log_end_pending = false;

// Expose these for the barcode driver to call at scan start/finish (ISR-safe)
// They only set volatile flags; actual logging happens in task context.
// void barcode_scan_started(void) {
//     g_barcode_scanning = true;
//     g_barcode_log_start_pending = true;   // request a log print from a task
// }

// void barcode_scan_finished(void) {
//     g_barcode_scanning = false;
//     g_barcode_log_end_pending = true;     // request a log print from a task
// }

// Print ONLY the decoded barcode value; also mark scan finished on successful decode
static void on_barcode_detected_callback(const char *decoded_str, barcode_command_t cmd) {
    (void)cmd;
    printf("%s\n", decoded_str);
    fflush(stdout);
    // mark finished (driver may already call barcode_scan_finished; keep idempotent)
    barcode_scan_finished();
}

// Periodic barcode maintenance (also handles pending log prints)
static void barcode_timeout_task(void *params) {
    (void)params;
    while (true) {
        // Do barcode driver periodic work
        barcode_update_local();                  // process timeouts/buffers

        // Handle pending log requests (must be printed from task context)
        if (g_barcode_log_start_pending) {
            g_barcode_log_start_pending = false;
            printf("[BARCODE] Scan started - slowing robot for scan\n");
            fflush(stdout);
        }
        if (g_barcode_log_end_pending) {
            g_barcode_log_end_pending = false;
            printf("[BARCODE] Scan finished - resuming normal line-follow speed\n");
            fflush(stdout);
        }

        vTaskDelay(pdMS_TO_TICKS(100));          // 10 Hz
    }
}

// Line follow or straight-drive task (pauses/slow during scan; no printing)
static void line_follow_task(void *params) {
    (void)params;

    // PID state
    float previous_error = 0.0f;
    float integral = 0.0f;

    while (true) {
        // If a scan is active, pause PID and drive slowly straight
        if (g_barcode_scanning) {
            // motor_set_speed(SCAN_SPEED, SCAN_SPEED);
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            continue;
        }

        // Read raw sensor value
        uint16_t raw_value = line_sensor_read_raw();

#if STRAIGHT_MODE
        // Drive straight (no PID)
        motor_set_speed(BASE_SPEED, BASE_SPEED);
#else
        // Compute error relative to center threshold (~1350)
        float error = (float)((int16_t)raw_value - 1350) / 1350.0f;
        if (error > 1.0f) error = 1.0f;
        if (error < -1.0f) error = -1.0f;

        // Apply offset and deadband
        error += SENSOR_OFFSET;
        if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) {
            error = 0.0f;
        }

        // PID (discrete)
        const float dt = (float)LOOP_DELAY_MS / 1000.0f;
        float P = KP * error;
        integral += error * dt;
        float I = KI * integral;
        float D = KD * ((error - previous_error) / dt);
        previous_error = error;

        // Anti-windup
        if (integral > 100.0f) integral = 100.0f;
        if (integral < -100.0f) integral = -100.0f;

        float correction = P + I + D;
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

        float left_speed  = BASE_SPEED - correction;
        float right_speed = BASE_SPEED + correction;

        // Clamp
        if (left_speed  < 0.0f) left_speed  = 0.0f;
        if (left_speed  > 1.0f) left_speed  = 1.0f;
        if (right_speed < 0.0f) right_speed = 0.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;

        if (!CALIBRATION_MODE) {
            motor_set_speed(left_speed, right_speed);
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1500); // Allow USB CDC to enumerate

    // Hardware init once
    motor_init();
    encoder_init();
    pid_init();
    line_sensor_init();

    // Barcode init + callback
    barcode_init_local();
    barcode_set_callback_local(on_barcode_detected_callback);
    barcode_start_scanning_local();

    // Start tasks
    (void)xTaskCreate(
        line_follow_task, "LineFollow",
        1024, NULL, 2, NULL);

    (void)xTaskCreate(
        barcode_timeout_task, "BarcodeTimeout",
        1024, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) { }
}