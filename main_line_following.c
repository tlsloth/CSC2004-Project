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


#define PRINT_STATEMENTS 0  // Set to 1 to enable printf statements, 0 to disable


// Task handle for barcode task
static TaskHandle_t barcode_task_handle = NULL;
static TaskHandle_t line_following_task_handle = NULL;

// ===== State Machine =====
typedef enum {
    ROBOT_STATE_STRAIGHT,
    ROBOT_STATE_TURNING
} robot_state_t;

// ===== Configuration =====
#define BASE_SPEED 0.30f            // Base speed (reduced for stability)
#define MAX_CORRECTION 0.30f        // Maximum correction (reduced to prevent wild turns)
#define STRAIGHT_MAX_CORRECTION 0.20f   // Max correction during scanning (more stable)
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
    // immediately set motors to base speed
    //motor_set_speed(BASE_SPEED, BASE_SPEED);
}
void barcode_scan_finished(void) {
    g_barcode_scanning = false;
    g_barcode_log_end_pending = true;
}

static void on_barcode_detected_callback(const char *decoded_str, barcode_command_t cmd) {
    (void)cmd;
#if PRINT_STATEMENTS
    printf("%s\n", decoded_str);
    fflush(stdout);
#endif
    // ensure scan finished flag cleared
    barcode_scan_finished();
}


static void barcode_timeout_task(void *params) {
    (void)params;
#if PRINT_STATEMENTS
    printf("[BARCODE] Task started.\n");
    fflush(stdout);
#endif

    while (true) {
        barcode_update_local(); // process timeouts/buffers
        if(g_barcode_scanning) {
            //motor_set_speed(BASE_SPEED, BASE_SPEED);
        }

        if (g_barcode_log_start_pending) {
            g_barcode_log_start_pending = false;
#if PRINT_STATEMENTS
            printf("[BARCODE] Scan started\n");
            fflush(stdout);
#endif
        }
        if (g_barcode_log_end_pending) {
            g_barcode_log_end_pending = false;
#if PRINT_STATEMENTS
            printf("[BARCODE] Scan finished\n");
            fflush(stdout);
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 2 Hz
    }
}


void line_following_task(void *pvParameters) {
    // Give other tasks a moment to start up
    vTaskDelay(pdMS_TO_TICKS(100));

#if PRINT_STATEMENTS
    printf("Starting line following in 2 seconds...\n");
#endif
    sleep_ms(2000);

    uint32_t telemetry_timer = to_ms_since_boot(get_absolute_time());
    

    // IMU state for turn detection
    IMU_Data imu_data;
    float bounded_heading = 0.0f;
    imu_update(&imu_data);
    const char* EDGE_TO_CHECK = "LEFT"; // We always check the left edge for line following
    bounded_heading = imu_data.heading;
    
    const float BEARING_CHANGE_THRESHOLD = 17.0f; // Threshold to detect significant turn

    // State machine variables
    robot_state_t robot_state = ROBOT_STATE_STRAIGHT;
    uint32_t last_significant_turn_time = 0;
    const uint32_t STRAIGHT_TRANSITION_TIMEOUT_MS = 300; // Time before state -> STRAIGHT
    uint32_t loop_start_time = 0;
    while (1) {
        loop_start_time = to_ms_since_boot(get_absolute_time());
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
#if PRINT_STATEMENTS
                printf("Significant turn detected. Entering TURNING state.\n");
#endif
                robot_state = ROBOT_STATE_TURNING;
                barcode_set_scanning_active_local(false); // Pause scanning
            }
            last_significant_turn_time = to_ms_since_boot(get_absolute_time());
 
            if (heading_change < 0) { // Turning LEFT
                if(strcmp(EDGE_TO_CHECK, "RIGHT") != 0) {
#if PRINT_STATEMENTS
                    printf("Switching to check RIGHT edge due to LEFT turn\n");
#endif
                    EDGE_TO_CHECK = "RIGHT";
                }
            } else { // Turning RIGHT
                if(strcmp(EDGE_TO_CHECK, "LEFT") != 0) {
#if PRINT_STATEMENTS
                    printf("Switching to check LEFT edge due to RIGHT turn\n");
#endif
                    EDGE_TO_CHECK = "LEFT";
                }
            }
            
        } else {
            // Heading change is not significant, check if we should go back to STRAIGHT
            if (robot_state == ROBOT_STATE_TURNING && 
                (to_ms_since_boot(get_absolute_time()) - last_significant_turn_time > STRAIGHT_TRANSITION_TIMEOUT_MS)) {
#if PRINT_STATEMENTS
                printf("No significant turn detected for a while. Returning to STRAIGHT state.\n");
#endif
                robot_state = ROBOT_STATE_STRAIGHT;
                barcode_set_scanning_active_local(true); // Resume scanning
            }
        }

        // Read sensor and calculate error
        float error = line_sensor_get_error();
        
        // Apply sensor offset compensation
        error += SENSOR_OFFSET;
        
        // PID calculation using the PID driver
        float correction = pid_compute_heading(error, (LOOP_DELAY_MS / 1000.0f));
        
        // apply edge correction based on turn direction
        if (strcmp(EDGE_TO_CHECK, "RIGHT") == 0) {
            correction = -correction; // check opposite
        }
           
        // STRAIGHT - use PID correction, but clamp it
        if(robot_state == ROBOT_STATE_TURNING) {
            if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;  
            if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
        } else {
            // STRAIGHT mode - clamp more tightly
            if (correction > STRAIGHT_MAX_CORRECTION) correction = STRAIGHT_MAX_CORRECTION;  
            if (correction < -STRAIGHT_MAX_CORRECTION) correction = -STRAIGHT_MAX_CORRECTION;
        }
        
        
        // Apply differential steering
        float left_speed = BASE_SPEED - correction;
        float right_speed = BASE_SPEED + correction;
        if(strcmp(EDGE_TO_CHECK, "RIGHT") == 0) {
            //if edge to check is right,we will slow the left speed
            left_speed = left_speed * 0.95f;
        }
        else{
            right_speed = right_speed * 0.95f;
        }
        
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

            const char* state_str = (robot_state == ROBOT_STATE_TURNING) ? "TURNING" : "STRAIGHT";
            // printf("\n=== PID LINE FOLLOWING (Analog IR) ===\n");
            // printf("State: %s |  Error: %.3f | Heading: %.1f | Edge: %s\n", state_str, error, imu_data.heading, EDGE_TO_CHECK);
            // printf("P=%.2f  I=%.2f  D=%.2f\n", p, i, d);
            // printf("Correction: %.3f\n", correction);
            // printf("Motors: L=%.2f  R=%.2f\n", left_speed, right_speed);
            // printf("Distance: %.2f m\n", distance);
            // uint32_t loop_duration = to_ms_since_boot(get_absolute_time()) - loop_start_time;
            // printf("Loop Duration: %u ms\n", loop_duration);
            // printf("==========================\n");
        }


        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial

#if PRINT_STATEMENTS
    printf("\n=== PID Line Following Robot (Analog IR) ===\n");
#endif

    // Initialize hardware
    motor_init();
    encoder_init();
    pid_init();
    line_sensor_init();
    imu_init();
    barcode_init_local();
    barcode_set_callback_local(on_barcode_detected_callback);
    barcode_set_scanning_active_local(true); // Start scanning initially
    BaseType_t lf_task_status = xTaskCreate(line_following_task, "LineFollowingTask", 256, NULL, 1, &line_following_task_handle);
    BaseType_t barcode_task_status = xTaskCreate(barcode_timeout_task, "BarcodeTimeoutTask", 256, NULL, 1, &barcode_task_handle);

    if(lf_task_status != pdPASS) {
#if PRINT_STATEMENTS
        printf("[ERROR] Failed to create LineFollowingTask (rc=%ld)\n", (long)lf_task_status);
        fflush(stdout);
#endif
    }
    if(barcode_task_status != pdPASS) {    
#if PRINT_STATEMENTS
        printf("[ERROR] Failed to create BarcodeTimeoutTask (rc=%ld)\n", (long)barcode_task_status);
        fflush(stdout);
#endif
    }
    
    vTaskStartScheduler();
    
    while(1){};
}