#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

// Hardware drivers
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/pid.h"
#include "drivers/imu.h"


// ===== Configuration =====
#define BASE_SPEED 0.30f            // Base speed (reduced for stability)
#define MAX_CORRECTION 0.20f        // Maximum correction (reduced to prevent wild turns)
#define SENSOR_OFFSET 0.0f          // No offset - aim for true center-line tracking
#define ERROR_DEADBAND 0.08f        // Increased deadband to reduce oscillation on line edge
#define LOOP_DELAY_MS 10            // Control loop period (~100Hz)
#define TELEMETRY_INTERVAL_MS 200   // Telemetry reporting interval (~5Hz)
#define CALIBRATION_MODE 0          // Set to 1 to enable calibration mode (motors off)
#define STRAIGHT_MODE 1        // Set to 1 to drive straight (no PID), 0 for line following

// PID Gains for line following
#define KP 0.20f                    // Proportional gain (reduced further for smoother response)
#define KI 0.0f                     // Integral gain (disabled to prevent windup during oscillations)
#define KD 0.02f                    // Derivative gain (dramatically reduced - was causing wild swings)

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial

    printf("\n=== PID Line Following Robot ===\n");

    // Initialize hardware
    motor_init();
    encoder_init();
    pid_init();
    imu_init();
    
    IMU_Data imu_data;

    printf("\n=== INITIAL SENSOR CHECK ===\n");
    sleep_ms(2000);

#if CALIBRATION_MODE
    printf("\n*** CALIBRATION MODE ***\n");
    printf("Place sensor over WHITE surface, then BLACK surface\n");
    printf("Watch the RAW ADC values to set the threshold\n\n");
    
    uint16_t min_value = 4095, max_value = 0;
    
    while (1) {
        uint16_t raw = line_sensor_read_raw();
        
        if (raw < min_value) min_value = raw;
        if (raw > max_value) max_value = raw;
        
        printf("RAW: %4u  |  MIN: %4u  |  MAX: %4u  |  SUGGESTED THRESHOLD: %4u\n", 
               raw, min_value, max_value, (min_value + max_value) / 2);
        
        sleep_ms(100);
    }
#endif

    uint32_t telemetry_timer = 0;
    
    // PID variables
    float previous_error = 0.0f;
    float integral = 0.0f;

#if STRAIGHT_MODE

    // PID constants - tune these for your robot
    #define HEADING_SMOOTH 5
    #define KP_BEARING 0.5f
    #define KI_BEARING 0.0f
    #define KD_BEARING 0.0f
    //#define MAX_CORRECTION 0.12f   // Tighter correction limit for straight driving

    float heading_history[HEADING_SMOOTH] = {0};
    int heading_idx = 0;
    int startup_loops = 2; // Number of loops to run motors straight before applying PID
    int loop_counter = 0;

    printf("\n*** STRAIGHT MODE - DRIVING FORWARD USING IMU HEADING PID ***\n");

    // Read initial heading as target
    imu_update(&imu_data);
    float target_heading = imu_data.heading;
    printf("Initial heading: %.1f° (target for straight drive)\n", target_heading);

    while (1) {

        float left_speed, right_speed;

        if (loop_counter < startup_loops) {
            // 1. Soft-start phase: drive straight at BASE_SPEED
            left_speed  = BASE_SPEED;
            right_speed = BASE_SPEED;
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
            float left_speed  = BASE_SPEED - correction;
            float right_speed = BASE_SPEED + correction;

            // Clamp speeds
        if (left_speed < 0.0f) left_speed = 0.0f;
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (right_speed < 0.0f) right_speed = 0.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;

        motor_set_speed(left_speed, right_speed);

        // ----- Telemetry -----
        telemetry_timer += LOOP_DELAY_MS;
        if (telemetry_timer >= TELEMETRY_INTERVAL_MS) {
            telemetry_timer = 0;
            float distance = encoder_get_distance_m();

            printf("\n=== IMU STRAIGHT MODE ===\n");
            printf("Target Heading: %.1f° | Smoothed: %.1f° | Error: %.2f°\n", 
                target_heading, smoothed_heading, heading_error);
            printf("PID Correction: %.3f\n", correction);
            printf("Motors: L=%.2f  R=%.2f\n", left_speed, right_speed);
            printf("Distance: %.2f m\n", distance);
            printf("*** IMU RAW DATA ***\n");
            printf("Raw Accel: %.2f %.2f %.2f | Raw Mag: %.2f %.2f %.2f\n",
                imu_data.raw_ax, imu_data.raw_ay, imu_data.raw_az,
                imu_data.raw_mx, imu_data.raw_my, imu_data.raw_mz);
            printf("Filtered Accel: %.2f %.2f %.2f | Filtered Mag: %.2f %.2f %.2f | Heading: %.1f°\n",
                imu_data.ax, imu_data.ay, imu_data.az,
                imu_data.mx, imu_data.my, imu_data.mz,
                imu_data.heading);
            printf("============================\n");
        }
        
        }

        loop_counter++;

        sleep_ms(LOOP_DELAY_MS);
    }

#else
    while (1) {
        // Read raw sensor value
        uint16_t raw_value = line_sensor_read_raw();
        
        // Calculate error from threshold (centerline)
        // Center-line tracking: threshold set to middle of black line (~1350)
        // error > 0 means sensor is deeper into black (turn left to get back to center)
        // error < 0 means sensor is on white/edge (turn right to get back to center)
        float error = (float)((int16_t)raw_value - 1350) / 1350.0f;
        
        // Clamp error to reasonable range
        if (error > 1.0f) error = 1.0f;
        if (error < -1.0f) error = -1.0f;
        
        // Apply sensor offset compensation
        error += SENSOR_OFFSET;
        
        // Apply deadband filter - ignore small errors on straight lines
        if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) {
            error = 0.0f;
        }
        
        // PID calculation
        float P = KP * error;
        integral += error * (LOOP_DELAY_MS / 1000.0f);
        float I = KI * integral;
        float derivative = (error - previous_error) / (LOOP_DELAY_MS / 1000.0f);
        float D = KD * derivative;
        
        previous_error = error;
        
        // Anti-windup for integral
        if (integral > 100.0f) integral = 100.0f;
        if (integral < -100.0f) integral = -100.0f;
        
        // Calculate correction
        float correction = P + I + D;
        
        // Clamp correction
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
        
        // Apply differential steering
        // Positive correction = turn left (slow down left motor)
        // Negative correction = turn right (slow down right motor)
        float left_speed = BASE_SPEED - correction;
        float right_speed = BASE_SPEED + correction;
        
        // Ensure speeds stay in valid range
        if (left_speed < 0.0f) left_speed = 0.0f;
        if (left_speed > 1.0f) left_speed = 1.0f;
        if (right_speed < 0.0f) right_speed = 0.0f;
        if (right_speed > 1.0f) right_speed = 1.0f;

        // Apply motor speeds
        if (!CALIBRATION_MODE) {
            motor_set_speed(left_speed, right_speed);
        }

        // ----- Telemetry -----
        telemetry_timer += LOOP_DELAY_MS;
        if (telemetry_timer >= TELEMETRY_INTERVAL_MS) {
            telemetry_timer = 0;
            float distance = encoder_get_distance_m();

            printf("\n=== PID LINE FOLLOWING ===\n");
            printf("Raw ADC: %u  |  Error: %.3f (with offset)\n", raw_value, error);
            printf("P=%.3f  I=%.3f  D=%.3f  |  Corr=%.3f\n", P, I, D, correction);
            printf("Motors: L=%.2f  R=%.2f\n", left_speed, right_speed);
            printf("Distance: %.2f m\n", distance);
            printf("*** PID-ONLY FIRMWARE - NO SEARCH MODE ***\n");
            printf("==========================\n");
        }

        sleep_ms(LOOP_DELAY_MS);
    }
#endif

    return 0;
}
