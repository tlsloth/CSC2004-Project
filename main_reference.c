#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"

// Hardware drivers
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/pid.h"
#include "drivers/ir_sensor.h"
#include "drivers/imu.h"

#include "FreeRTOS.h"
#include "task.h"


// ===== Configuration =====
#define BASE_SPEED 0.20f            // Base speed (reduced for stability)
#define MAX_CORRECTION 0.35f        // Maximum correction (reduced to prevent wild turns)
#define SENSOR_OFFSET 0.0f          // No offset - aim for true center-line tracking
#define ERROR_DEADBAND 0.08f        // Increased deadband to reduce oscillation on line edge
#define LOOP_DELAY_MS 10            // Control loop period (~100Hz)
#define TELEMETRY_INTERVAL_MS 200   // Telemetry reporting interval (~5Hz)
#define STRAIGHT_MODE 0             // Set to 1 to drive straight (no PID), 0 for PID line following



int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial

    printf("\n=== PID Line Following Robot (Analog IR) ===\n");

    // Initialize hardware
    motor_init();
    encoder_init();
    pid_init();
    line_sensor_init();
    imu_init();

    printf("\n=== INITIAL SENSOR CHECK ===\n");
    sleep_ms(500);
    for (int i = 0; i < 5; i++) {
        uint16_t raw = line_sensor_read_raw();
        line_state_t state = line_sensor_read();
        printf("Reading %d: Raw=%u, State=%s\n", i+1, raw, state == LINE_BLACK ? "BLACK" : "WHITE");
        sleep_ms(200);
    }
    printf("Starting line following in 2 seconds...\n");
    sleep_ms(2000);

    uint32_t telemetry_timer = to_ms_since_boot(get_absolute_time());
    

    // IMU state for turn detection
    IMU_Data imu_data;
    float bounded_heading = 0.0f;
    imu_update(&imu_data);
    const char* EDGE_TO_CHECK = "LEFT"; // We always check the left edge for line following
    bounded_heading = imu_data.heading;
    
    const float BEARING_CHANGE_THRESHOLD = 15.0f; // Threshold to detect significant turn

    while (1) {
        // Update IMU data at the start of the loop
        imu_update(&imu_data);
        
        // Calculate heading change to determine turn direction
        float heading_change = imu_data.heading - bounded_heading;
        if (heading_change > 180.0f) heading_change -= 360.0f;
        if (heading_change < -180.0f) heading_change += 360.0f;

        // Read sensor and calculate error
        uint16_t raw_value = line_sensor_read_raw();
        float error = line_sensor_get_error();
        
        // Apply sensor offset compensation
        error += SENSOR_OFFSET;

        // Apply deadband filter - ignore small errors on straight lines
        if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) {
            error = 0.0f;
        }
        
        // PID calculation using the PID driver
        float correction = pid_compute_heading(error, (LOOP_DELAY_MS / 1000.0f));
        
        // Override correction based on IMU turn direction
        if (heading_change < -BEARING_CHANGE_THRESHOLD) {
            // Turning LEFT - bias correction to the left (negative)
            if(strcmp(EDGE_TO_CHECK, "RIGHT") != 0) {
                printf("Switching to check RIGHT edge due to LEFT turn\n");
                EDGE_TO_CHECK = "RIGHT";
            }
            bounded_heading = imu_data.heading;
        } else if (heading_change > BEARING_CHANGE_THRESHOLD) {
            // Turning RIGHT - bias correction to the right (positive)
            if(strcmp(EDGE_TO_CHECK, "LEFT") != 0) {
                printf("Switching to check LEFT edge due to RIGHT turn\n");
                EDGE_TO_CHECK = "LEFT";
            }
            bounded_heading = imu_data.heading;
        } 

        // apply edge correction based on turn direction
        if (strcmp(EDGE_TO_CHECK, "RIGHT") == 0) {
            correction = -correction; // check opposite
        }
           
        // STRAIGHT - use PID correction, but clamp it
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;  
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
        
        
        // Apply differential steering
        float left_speed = BASE_SPEED - correction;
        float right_speed = BASE_SPEED + correction;
        
        // Ensure speeds stay in valid range
        if (left_speed < 0.0f) left_speed = 0.0f;
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (right_speed < 0.0f) right_speed = 0.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;

        // Apply motor speeds
        motor_set_speed(left_speed, right_speed);

        // ----- Telemetry -----
        if (to_ms_since_boot(get_absolute_time()) - telemetry_timer >= TELEMETRY_INTERVAL_MS) {
            telemetry_timer = to_ms_since_boot(get_absolute_time());
            float distance = encoder_get_distance_m();
            float p, i, d;
            pid_get_heading_gains(&p, &i, &d);

            printf("\n=== PID LINE FOLLOWING (Analog IR) ===\n");
            printf("Raw ADC: %u | Error: %.3f | Heading: %.1f | BOUNDED_HEADING: %.1f | EDGE_TO_CHECK: %s\n", raw_value, error, imu_data.heading, bounded_heading, EDGE_TO_CHECK);
            printf("P=%.2f  I=%.2f  D=%.2f\n", p, i, d);
            printf("Correction: %.3f\n", correction);
            printf("Motors: L=%.2f  R=%.2f\n", left_speed, right_speed);
            printf("Distance: %.2f m\n", distance);
            printf("==========================\n");
        }


        sleep_ms(LOOP_DELAY_MS);
    }

    return 0;
}