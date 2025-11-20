#include "straight_mode.h"
#include <stdio.h>
#include "pico/stdlib.h"

// Hardware drivers
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/ir_sensor.h"
#include "drivers/imu.h"

// Configuration from main_reference.c
#define BASE_SPEED 0.25f
#define LOOP_DELAY_MS 10
#define TELEMETRY_INTERVAL_MS 200
#define HEADING_SMOOTH 5
#define KP_BEARING 0.1f
#define KI_BEARING 0.0f
#define KD_BEARING 0.05f
#define MAX_CORRECTION 0.10f

void drive_straight_task(float target_heading) {
    uint32_t telemetry_timer = 0;
    float heading_history[HEADING_SMOOTH] = {0};
    int heading_idx = 0;
    int startup_loops = 2; // Number of loops to run motors straight before applying PID
    int loop_counter = 0;
    float previous_error = 0.0f;
    float integral = 0.0f;
    IMU_Data imu_data;

    printf("\n*** STRAIGHT MODE - DRIVING FORWARD USING IMU HEADING PID ***\n");
    printf("Target heading: %.1f°\n", target_heading);

    while (1) {
        float left_speed, right_speed;

        if (loop_counter < startup_loops) {
            // 1. Soft-start phase: drive straight at BASE_SPEED
            left_speed  = BASE_SPEED;
            right_speed = BASE_SPEED;
            motor_set_speed(left_speed, right_speed);
        } else {
            // Update IMU readings
            imu_update(&imu_data);

            // --- Moving average for heading ---
            heading_history[heading_idx] = imu_data.heading;
            heading_idx = (heading_idx + 1) % HEADING_SMOOTH;

            float smoothed_heading = 0.0f;
            for (int i = 0; i < HEADING_SMOOTH; i++) {
                smoothed_heading += heading_history[i];
            }
            smoothed_heading /= HEADING_SMOOTH;

            // --- Heading PID ---
            float heading_error = smoothed_heading - target_heading;

            // Wrap-around correction
            if (heading_error > 180.0f) heading_error -= 360.0f;
            if (heading_error < -180.0f) heading_error += 360.0f;

            // PID calculation
            float dt = LOOP_DELAY_MS / 1000.0f;
            integral += heading_error * dt;
            float derivative = (heading_error - previous_error) / dt;
            float correction = KP_BEARING * heading_error + KI_BEARING * integral + KD_BEARING * derivative;

            // Clamp correction
            if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
            if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

            previous_error = heading_error;

            // ----- Apply correction to motors -----
            left_speed  = BASE_SPEED - correction;
            right_speed = BASE_SPEED + correction;

            // Clamp speeds
            if (left_speed < 0.0f) left_speed = 0.0f;
            if (left_speed > 1.0f) left_speed = 1.0f;
            if (right_speed < 0.0f) right_speed = 0.0f;
            if (right_speed > 1.0f) right_speed = 1.0f;

            motor_set_speed(BASE_SPEED, BASE_SPEED);

            // ----- Telemetry -----
            if (to_ms_since_boot(get_absolute_time()) - telemetry_timer >= TELEMETRY_INTERVAL_MS) {
                telemetry_timer = to_ms_since_boot(get_absolute_time());
                float distance = encoder_get_distance_m();

                printf("\n=== IMU STRAIGHT MODE ===\n");
                printf("Target Heading: %.1f° | Smoothed: %.1f° | Error: %.2f°\n", 
                    target_heading, smoothed_heading, heading_error);
                printf("PID Correction: %.3f\n", correction);
                printf("Motors: L=%.2f  R=%.2f\n", left_speed, right_speed);
                printf("Distance: %.2f m\n", distance);
                printf("============================\n");
            }
        }

        loop_counter++;
        sleep_ms(LOOP_DELAY_MS);
    }
}
