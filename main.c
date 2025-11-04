#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

// Hardware drivers
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/pid.h"
#include "drivers/ir_sensor.h"
#include "drivers/ultrasonic.h"

// ===== Configuration =====
#define BASE_SPEED 0.30f            // Base speed (reduced for stability)
#define MAX_CORRECTION 0.20f        // Maximum correction (reduced to prevent wild turns)
#define SENSOR_OFFSET 0.0f          // No offset - aim for true center-line tracking
#define ERROR_DEADBAND 0.08f        // Increased deadband to reduce oscillation on line edge
#define LOOP_DELAY_MS 10            // Control loop period (~100Hz)
#define TELEMETRY_INTERVAL_MS 200   // Telemetry reporting interval (~5Hz)
#define CALIBRATION_MODE 0          // Set to 1 to enable calibration mode (motors off)
#define STRAIGHT_MODE 1             // Set to 1 to drive straight (no PID), 0 for line following

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
    line_sensor_init();
    ultrasonic_init_pins(4, 5);
    ultrasonic_start_monitor_task(100 /*poll ms*/, 20.0 /*stop_cm*/, 25.0 /*clear_cm*/);
    
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
    printf("\n*** STRAIGHT MODE - DRIVING FORWARD (NO PID) ***\n");
    printf("Both motors running at BASE_SPEED=%.2f\n", BASE_SPEED);
    printf("Use this mode for testing other sensors\n\n");
    uint32_t last_ultra_ms = to_ms_since_boot(get_absolute_time());
    bool obstacle_active = false;
    const double STOP_CM = 20.0;
    const double CLEAR_CM = 25.0;
    const int POLL_MS = 100;
    while (1) {
        // Read sensor for telemetry only
        uint16_t raw_value = line_sensor_read_raw();
        
        // Drive straight - both motors same speed
        motor_set_speed(BASE_SPEED, BASE_SPEED);
        
        // Telemetry
        if (to_ms_since_boot(get_absolute_time()) - telemetry_timer >= TELEMETRY_INTERVAL_MS) {
            telemetry_timer = to_ms_since_boot(get_absolute_time());
            
            float distance = encoder_get_distance_m();
            
            printf("\n=== STRAIGHT MODE ===\n");
            printf("Raw ADC: %u\n", raw_value);
            printf("Motors: L=%.2f  R=%.2f (equal speeds)\n", BASE_SPEED, BASE_SPEED);
            printf("Distance: %.2f m\n", distance);
            printf("*** DRIVING STRAIGHT - NO LINE FOLLOWING ***\n");
            printf("==========================\n");
        }

        if ((int)(to_ms_since_boot(get_absolute_time()) - last_ultra_ms) >= POLL_MS) {
            last_ultra_ms = to_ms_since_boot(get_absolute_time());
            double d = ultrasonic_get_distance_cm(); // from drivers/ultrasonic.c
            if (d > 0.0 && !obstacle_active && d <= STOP_CM) {
                obstacle_active = true;
                motor_stop();            // or motor_set_speed(0,0)
            } else if (obstacle_active && d >= CLEAR_CM) {
                obstacle_active = false;
                motor_set_speed(BASE_SPEED, BASE_SPEED);
            }
        }

        // apply motors only if not obstacle_active
        if (!obstacle_active) motor_set_speed(BASE_SPEED, BASE_SPEED);
        else motor_set_speed(0.0f, 0.0f);
        
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
