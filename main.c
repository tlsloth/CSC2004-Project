#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"

// Hardware drivers
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/pid.h"
#include "drivers/ir_sensor.h"
#include "drivers/ultrasonic.h"
#include "drivers/servo.h"

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

#define ULTRASONIC_TRIG_PIN 4 //for send ultrasonic pulse
#define ULTRASONIC_ECHO_PIN 5 //for receive ultrasonic pulse 

#define SERVO_PIN 15

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    bool obstacle_found;
    double width_cm;
    double distance_cm;
    int left_edge_angle;
    int right_edge_angle;
} ScanResult;

double angle_to_radians(int angle) {
    return angle * M_PI / 180.0;
}

ScanResult measure_obstacle_width(double obstacle_cm) {
    // --- Initialize the result struct to a "failure" state ---
    ScanResult result = {
        .obstacle_found = false,
        .width_cm = 0.0,
        .distance_cm = obstacle_cm, // Store the initial distance
        .left_edge_angle = -1,
        .right_edge_angle = -1
    };

    const int SERVO_SCAN_START_ANGLE = 30; 
    const int SERVO_SCAN_END_ANGLE = 150;
    const int SERVO_SCAN_STEP = 3; 
    const int SCAN_OBSTACLE_THRESHOLD = 30;

    int left_edge_angle = -1;
    int right_edge_angle = -1;
    double dist_cm;

    printf("\n--- Starting Scan (30-150 deg) ---\n");

    for (int angle = SERVO_SCAN_START_ANGLE; angle <= SERVO_SCAN_END_ANGLE; angle += SERVO_SCAN_STEP) {
        servo_set_angle(SERVO_PIN,angle);
        sleep_ms(50);

        // Use the ultrasonic driver's "stable distance" function
        dist_cm = ultrasonic_get_stable_distance_cm(3, 20);
        
        printf("Angle: %3d, Dist: %5.1f cm\n", angle, dist_cm);

        if (dist_cm > 0 && dist_cm < SCAN_OBSTACLE_THRESHOLD) {
            // --- Obstacle IS seen ---
            if (left_edge_angle == -1) {
                left_edge_angle = angle; // First detection
                printf("  -> Found left edge at %d deg\n", left_edge_angle);
            }
            right_edge_angle = angle; // Always update right edge
        } 
    }
    
    // Point servo back to center after scan
    servo_set_angle(SERVO_PIN,90);
    printf("--- Scan Complete. Returning to 90 deg ---\n");

    // --- Calculate width ---
    if (left_edge_angle != -1 && right_edge_angle > left_edge_angle) {
        printf("  -> Found right edge at %d deg\n", right_edge_angle);
        printf("  -> Using distance: %.1f cm\n", obstacle_cm);
        
        double angle_diff_deg = (double)(right_edge_angle - left_edge_angle);
        double angle_half_rad = (angle_diff_deg / 2.0) * M_PI / 180.0;
        double width = 2.0 * obstacle_cm * tan(angle_half_rad);

        printf("  -> Calculated width: %.2f cm\n", width);
        result.obstacle_found = true;
        result.width_cm = width;
        result.left_edge_angle = left_edge_angle;
        result.right_edge_angle = right_edge_angle;
        return result;
    }
    
    printf("  -> Could not determine obstacle edges.\n");
    return result;
}


int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial

    printf("\n=== PID Line Following Robot ===\n");

    // Initialize hardware
    motor_init();
    encoder_init();
    pid_init();
    line_sensor_init();
    ultrasonic_init_pins(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    //ultrasonic_start_monitor_task(100 /*poll ms*/, 20.0 /*stop_cm*/, 25.0 /*clear_cm*/);
    servo_init(SERVO_PIN);

    servo_set_angle(SERVO_PIN, 90);

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
    printf("Use this mode for testing other sensors\n\n");
    uint32_t last_ultra_ms = to_ms_since_boot(get_absolute_time());
    const double STOP_CM = 20.0;
    const int POLL_MS = 20;
    double last_obstacle_distance = 0.0;

    enum RobotState {
        STATE_DRIVING,     // Looking for obstacles
        STATE_SCANNING,    // Obstacle found, stopped, running scan
    };

    enum RobotState state = STATE_DRIVING;
    
    while (1) {

        if (state == STATE_DRIVING){
            // Read sensor for telemetry only
            uint16_t raw_value = line_sensor_read_raw();
            // Telemetry
            if (to_ms_since_boot(get_absolute_time()) - telemetry_timer >= TELEMETRY_INTERVAL_MS) {
                telemetry_timer = to_ms_since_boot(get_absolute_time());
                
                float distance = encoder_get_distance_m();
                /*
                printf("\n=== STRAIGHT MODE ===\n");
                printf("Raw ADC: %u\n", raw_value);
                printf("Motors: L=%.2f  R=%.2f (equal speeds)\n", BASE_SPEED, BASE_SPEED);
                printf("Distance: %.2f m\n", distance);
                printf("*** DRIVING STRAIGHT - NO LINE FOLLOWING ***\n");
                printf("==========================\n");
                */
                
            }

            if ((int)(to_ms_since_boot(get_absolute_time()) - last_ultra_ms) >= POLL_MS) {
                last_ultra_ms = to_ms_since_boot(get_absolute_time());
                
                double d = ultrasonic_get_stable_distance_cm(3, 10);
                printf("Dist: %.2f cm\n", d);

                if (d > 0.0 && d <= STOP_CM) {
                    printf("\n!!! Obstacle DETECTED at %.1f cm !!!\n", d);
                    last_obstacle_distance = d;
                    state = STATE_SCANNING;
                }
            }
        }

        if (state==STATE_DRIVING){
            motor_set_speed(BASE_SPEED, BASE_SPEED);
        }
        else if (state == STATE_SCANNING) {
            // Action: Stop, scan, and decide what to do next
            
            motor_set_speed(0.0f, 0.0f); // Ensure we are stopped
            
            // 1. Run the blocking scan
            ScanResult scan = measure_obstacle_width(last_obstacle_distance);
            
            // 2. Print results (you can add your navigation logic here)
            if (scan.obstacle_found) {
                printf("\n+++ RESULT: Obstacle Scan Complete +++\n");
                printf("  -> Width:   %.2f cm\n", scan.width_cm);
                printf("  -> Edges:   %d deg (L) to %d deg (R)\n", 
                       scan.left_edge_angle, scan.right_edge_angle);
            } else {
                printf("\n+++ RESULT: Scan complete. Could not measure width. +++\n");
            }
            
            printf("Post-scan check...\n");
            double d_check = ultrasonic_get_stable_distance_cm(3, 10);
            
            if (d_check >= STOP_CM) {
                printf("  -> Post-scan dist: %.1f cm. Obstacle is CLEAR.\n", d_check);
                state = STATE_DRIVING;
            } else {
                // The obstacle is still there (d_check < 28.0)
                printf("  -> Post-scan dist: %.1f cm. Obstacle still present.\n", d_check);
                last_obstacle_distance = d_check;
                sleep_ms(2000);
            }
        }
        
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
