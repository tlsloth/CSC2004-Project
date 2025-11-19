#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"

// Hardware drivers
#include "drivers/barcode.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/pid.h"
#include "drivers/ir_sensor.h"
#include "drivers/imu.h"

#include "FreeRTOS.h"
#include "task.h"


// Task handle for barcode task
static TaskHandle_t barcode_task_handle = NULL;

// ===== State Machine =====
typedef enum {
    ROBOT_STATE_STRAIGHT,
    ROBOT_STATE_TURNING
} robot_state_t;

// ===== Configuration =====
#define BASE_SPEED 0.25f            // Base speed (reduced for stability)
#define MAX_CORRECTION 0.35f        // Maximum correction (reduced to prevent wild turns)
#define SENSOR_OFFSET 0.0f          // No offset - aim for true center-line tracking
#define ERROR_DEADBAND 0.08f        // Increased deadband to reduce oscillation on line edge
#define LOOP_DELAY_MS 5            // Control loop period (~200Hz)
#define TELEMETRY_INTERVAL_MS 200   // Telemetry reporting interval (~5Hz)
#define STRAIGHT_MODE 0             // Set to 1 to drive straight (no PID), 0 for PID line following

// Scanning/log flags
static volatile bool g_barcode_scanning = false;
static volatile bool g_barcode_log_start_pending = false;
static volatile bool g_barcode_log_end_pending = false;

// Application hooks called by driver (ISR-safe; only set flags)
void barcode_scan_started(void) {
    g_barcode_scanning = true;
    g_barcode_log_start_pending = true;
}
void barcode_scan_finished(void) {
    g_barcode_scanning = false;
    g_barcode_log_end_pending = true;
}

static void on_barcode_detected_callback(const char *decoded_str, barcode_command_t cmd) {
    (void)cmd;
    printf("%s\n", decoded_str);
    fflush(stdout);
    // ensure scan finished flag cleared
    barcode_scan_finished();
}


static void barcode_timeout_task(void *params) {
    (void)params;
    printf("[BARCODE] Task started.\n");
    fflush(stdout);

    while (true) {
        barcode_update_local(); // process timeouts/buffers

        if (g_barcode_log_start_pending) {
            g_barcode_log_start_pending = false;
            printf("[BARCODE] Scan started\n");
            fflush(stdout);
        }
        if (g_barcode_log_end_pending) {
            g_barcode_log_end_pending = false;
            printf("[BARCODE] Scan finished\n");
            fflush(stdout);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}


void line_following_task(void *pvParameters) {
    // Give other tasks a moment to start up
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("Starting line following in 2 seconds...\n");
    sleep_ms(2000);

    uint32_t telemetry_timer = to_ms_since_boot(get_absolute_time());
    

    // IMU state for turn detection
    IMU_Data imu_data;
    float bounded_heading = 0.0f;
    imu_update(&imu_data);
    const char* EDGE_TO_CHECK = "LEFT"; // We always check the left edge for line following
    bounded_heading = imu_data.heading;
    
    const float BEARING_CHANGE_THRESHOLD = 18.0f; // Threshold to detect significant turn

    // State machine variables
    robot_state_t robot_state = ROBOT_STATE_STRAIGHT;
    uint32_t last_significant_turn_time = 0;
    const uint32_t STRAIGHT_TRANSITION_TIMEOUT_MS = 500; // Time before state -> STRAIGHT

    while (1) {
        // Update IMU data at the start of the loop
        imu_update(&imu_data);
        
        // Calculate heading change to determine turn direction
        float heading_change = imu_data.heading - bounded_heading;
        if (heading_change > 180.0f) heading_change -= 360.0f;
        if (heading_change < -180.0f) heading_change += 360.0f;

        // --- State Machine Logic ---
        if (fabsf(heading_change) > BEARING_CHANGE_THRESHOLD) {
            bounded_heading = imu_data.heading; // Update heading anchor on turn
            // A significant turn is detected, enter or stay in TURNING state
            if(robot_state != ROBOT_STATE_TURNING) {
                printf("Significant turn detected. Entering TURNING state.\n");
                robot_state = ROBOT_STATE_TURNING;
                barcode_set_scanning_active_local(false); // Pause scanning
            }
            last_significant_turn_time = to_ms_since_boot(get_absolute_time());
 
            if (heading_change < 0) { // Turning LEFT
                if(strcmp(EDGE_TO_CHECK, "RIGHT") != 0) {
                    printf("Switching to check RIGHT edge due to LEFT turn\n");
                    EDGE_TO_CHECK = "RIGHT";
                }
            } else { // Turning RIGHT
                if(strcmp(EDGE_TO_CHECK, "LEFT") != 0) {
                    printf("Switching to check LEFT edge due to RIGHT turn\n");
                    EDGE_TO_CHECK = "LEFT";
                }
            }
            
        } else {
            // Heading change is not significant, check if we should go back to STRAIGHT
            if (robot_state == ROBOT_STATE_TURNING && 
                (to_ms_since_boot(get_absolute_time()) - last_significant_turn_time > STRAIGHT_TRANSITION_TIMEOUT_MS)) {
                printf("No significant turn detected for a while. Returning to STRAIGHT state.\n");
                robot_state = ROBOT_STATE_STRAIGHT;
                barcode_set_scanning_active_local(true); // Resume scanning
            }
        }

        // Read sensor and calculate error
        float error = line_sensor_get_error();
        
        // Apply sensor offset compensation
        error += SENSOR_OFFSET;

        // Apply deadband filter - ignore small errors on straight lines
        if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) {
            error = 0.0f;
        }
        
        // PID calculation using the PID driver
        float correction = pid_compute_heading(error, (LOOP_DELAY_MS / 1000.0f));
        
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
        // if (to_ms_since_boot(get_absolute_time()) - telemetry_timer >= TELEMETRY_INTERVAL_MS) {
        //     telemetry_timer = to_ms_since_boot(get_absolute_time());
        //     float distance = encoder_get_distance_m();
        //     float p, i, d;
        //     pid_get_heading_gains(&p, &i, &d);

        //     const char* state_str = (robot_state == ROBOT_STATE_TURNING) ? "TURNING" : "STRAIGHT";

        //     printf("\n=== PID LINE FOLLOWING (Analog IR) ===\n");
        //     printf("State: %s | Raw: %u | Error: %.3f | Heading: %.1f | Edge: %s\n", state_str, raw_value, error, imu_data.heading, EDGE_TO_CHECK);
        //     printf("P=%.2f  I=%.2f  D=%.2f\n", p, i, d);
        //     printf("Correction: %.3f\n", correction);
        //     printf("Motors: L=%.2f  R=%.2f\n", left_speed, right_speed);
        //     printf("Distance: %.2f m\n", distance);
        //     printf("==========================\n");
        // }


        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

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
    barcode_init_local();
    barcode_set_callback_local(on_barcode_detected_callback);
    barcode_set_scanning_active_local(true); // Start scanning initially
    
    BaseType_t lf_task_status = xTaskCreate(line_following_task, "LineFollowingTask", 256, NULL, 1, NULL);
    BaseType_t barcode_task_status = xTaskCreate(barcode_timeout_task, "BarcodeTimeoutTask", 256, NULL, 1, &barcode_task_handle);

    if(lf_task_status != pdPASS) {
        printf("[ERROR] Failed to create LineFollowingTask (rc=%ld)\n", (long)lf_task_status);
        fflush(stdout);
    }
    if(barcode_task_status != pdPASS) {    
        printf("[ERROR] Failed to create BarcodeTimeoutTask (rc=%ld)\n", (long)barcode_task_status);
        fflush(stdout);
    }
    vTaskStartScheduler();

    while(1){};
}