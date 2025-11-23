#include <stdio.h>
#include <string.h> 
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

// Hardware drivers
#include "drivers/barcode.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/pid.h"
#include "drivers/ir_sensor.h"
#include "drivers/imu.h"
#include "drivers/config.h"
#include "drivers/ultrasonic.h"
#include "drivers/servo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/apps/mqtt.h"
#include "semphr.h"
#include "queue.h"


#define PRINT_STATEMENTS 1  // Set to 1 to enable printf statements, 0 to disable


// ===== State Machine =====
typedef enum {
    ROBOT_STATE_STRAIGHT,
    ROBOT_STATE_TURNING
} robot_state_t;

typedef struct {
    char topic[32];
    char payload[256];      
} mqtt_message_t;

// Task handle for barcode task
static TaskHandle_t barcode_task_handle = NULL;
static TaskHandle_t line_following_task_handle = NULL;

static QueueHandle_t mqtt_queue = NULL;

// ===== Configuration =====
#define BASE_SPEED 0.25f            // Base speed (reduced for stability)
#define MAX_CORRECTION 0.20f        // Maximum correction (reduced to prevent wild turns)
#define STRAIGHT_MAX_CORRECTION 0.10f   // Max correction during scanning (more stable)
#define SENSOR_OFFSET 0.0f          // No offset - aim for true center-line tracking
#define ERROR_DEADBAND 0.08f        // Increased deadband to reduce oscillation on line edge
#define LOOP_DELAY_MS 5           // Control loop period (~200Hz)
#define TELEMETRY_INTERVAL_MS 2000   // Telemetry reporting interval (~5Hz)
#define STRAIGHT_MODE 0             // Set to 1 to drive straight (no PID), 0 for PID line following
#define BEARING_CHANGE_THRESHOLD 25.0f // Threshold to detect significant turn
#define EDGE_SWITCH_DEBOUNCE_MS 300 // Minimum time between edge switches
#define M_PI 3.14159265358979323846
#define ULTRA_POLL_MS 50 // Ultrasonic polling interval in ms


// Scanning/log flags
static volatile bool g_barcode_scanning = false;
static volatile bool g_barcode_log_start_pending = false;
static volatile bool g_barcode_log_end_pending = false;
static volatile bool g_request_line_following_suspend = false;
static volatile float g_target_heading = 0.0f;
static volatile uint32_t g_last_scan_finish_time = 0;
static float g_cardinal_bearings[4]; // N, E, S, W
static volatile bool g_command_received = false;
static volatile barcode_command_t g_received_command = CMD_NONE; // Assuming CMD_NONE is defined in barcode.h




// MQTT client
static mqtt_client_t *mqtt_client = NULL;
static bool mqtt_connected = false;

void mqtt_publisher_task(void *pv) {
    (void) pv;
    mqtt_message_t msg;
    while (1) {
        if (xQueueReceive(mqtt_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (mqtt_connected && mqtt_client != NULL) {
                mqtt_publish(mqtt_client, msg.topic, msg.payload, strlen(msg.payload), 0, 0, NULL, NULL);
            }
        }
    }
}

void safe_mqtt_publish(const char *topic, const char *payload) {
    if(mqtt_queue == NULL) return; 
    //printf("[MQTT] Queueing message to topic: %s, payload: %s\n", topic, payload);
    mqtt_message_t msg;
    strncpy(msg.topic, topic, sizeof(msg.topic) - 1);
    msg.topic[sizeof(msg.topic) - 1] = '\0';
    strncpy(msg.payload, payload, sizeof(msg.payload) - 1);
    msg.payload[sizeof(msg.payload) - 1] = '\0';

    xQueueSend(mqtt_queue, &msg, portMAX_DELAY);
}

void calibrate_cardinal_bearings(float initial_north_bearing) {
    g_cardinal_bearings[0] = initial_north_bearing;                         // North
    g_cardinal_bearings[1] = fmodf(initial_north_bearing + 90.0f, 360.0f);  // East
    g_cardinal_bearings[2] = fmodf(initial_north_bearing + 180.0f, 360.0f); // South
    g_cardinal_bearings[3] = fmodf(initial_north_bearing + 270.0f, 360.0f); // West
}

float get_nearest_cardinal_bearing(float current_bearing) {
    const float* bearings = g_cardinal_bearings;
    float min_diff = 360.0f;
    float nearest_bearing = 0.0f;

    for (int i = 0; i < 4; i++) {
        float diff = fabsf(current_bearing - bearings[i]);
        if (diff > 180.0f) {
            diff = 360.0f - diff; // Handle wrap-around
        }
        if (diff < min_diff) {
            min_diff = diff;
            nearest_bearing = bearings[i];
        }
    }
    // Handle the 0/360 case. If nearest is 0 and current is > 270, target 360 for continuity.
    // Note: The PID logic already handles wrap-around, so returning 0.0f is sufficient.
    return nearest_bearing;
}
// Application hooks called by driver (ISR-safe; only set flags)

size_t FindIndex( const float a[], size_t size, float value )
{
    size_t index = 0;

    while ( index < size && a[index] != value ) ++index;

    return ( index == size ? -1 : index );
}

float get_left_cardinal_bearing(float current_bearing) {
    const float* bearings = g_cardinal_bearings;
    // find index of current bearing
    size_t index = FindIndex( (const float*)bearings, 4, current_bearing );
    index -= 1; // go left

    if((int)index < 0) {
        // not found, return current bearing
        return bearings[3];
    }
    return bearings[index];
}

float get_right_cardinal_bearing(float current_bearing) {
    const float* bearings = g_cardinal_bearings;
    // find index of current bearing
    size_t index = FindIndex( (const float*)bearings, 4, current_bearing );
    
    index += 1; // go right
    if((int)index >= 4) {
        // loop back to first element
        return bearings[0];
    }
    return bearings[index];
}

void barcode_scan_started(void) {
    if(to_ms_since_boot(get_absolute_time()) - g_last_scan_finish_time < 1000) {
        // Ignore scan started if within 1 second of last finish
        return;
    }
    g_barcode_scanning = true; 
    g_barcode_log_start_pending = true;
    g_request_line_following_suspend = true;


}
void barcode_scan_finished(void) {
    if(g_barcode_scanning) {
        // Only act if we were actually scanning    
    g_barcode_scanning = false;
    g_barcode_log_end_pending = true;
    g_last_scan_finish_time = to_ms_since_boot(get_absolute_time());
    g_request_line_following_suspend = false;
    }
}

void drive_straight_to_bearing(float target_heading) {
    IMU_Data imu_data;
    imu_update(&imu_data);
    float current_max_speed;
    float error = imu_data.heading - target_heading;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    if (fabs(error) > 10.0f) {
        current_max_speed = MAX_CORRECTION;
    } else {
        current_max_speed = 0.14f;
    }

    float correction = pid_compute_heading(error,(float)LOOP_DELAY_MS / 1000.0f);
    if(correction > current_max_speed) correction = current_max_speed; 
    if(correction < -current_max_speed) correction = -current_max_speed;
    float left_speed = BASE_SPEED - correction;
    float right_speed = BASE_SPEED + correction;
    left_speed *= 1.1f;
    //printf("[LINE] Driving straight to heading %.1f deg (error=%.1f deg, corr=%.3f), left_speed=%.3f, right_speed=%.3f\n", target_heading, error, correction, left_speed, right_speed);
    
    motor_set_speed(left_speed, right_speed);
}


// MQTT connection callback
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Connected to broker\n");
        mqtt_connected = true;
    } else {
        printf("[MQTT] Connection failed: %d\n", status);
        mqtt_connected = false;
    }
}

static void on_barcode_detected_callback(const char *decoded_str, barcode_command_t cmd) {
    (void)cmd;
#if PRINT_STATEMENTS
    printf("%s\n", decoded_str);
    fflush(stdout);
#endif
    //printf("[BARCODE] Decoded: \"%s\" Command: %d\n", decoded_str, cmd);
    if(cmd == CMD_LEFT || cmd == CMD_RIGHT){
        g_received_command = cmd;
        g_command_received = true;
    }
    // Publish barcode to MQTT
    if (mqtt_connected && mqtt_client != NULL) {

        mqtt_message_t msg;
        snprintf(msg.payload, sizeof(msg.payload), "{\"barcode\": \"%s\", \"command\": %d}", decoded_str, cmd);
        safe_mqtt_publish(MQTT_TOPIC_BARCODE, msg.payload);
    }
}


static void barcode_timeout_task(void *params) {
    (void)params;
#if PRINT_STATEMENTS
    printf("[BARCODE] Task started.\n");
    fflush(stdout);
#endif

    while (true) {
        barcode_update_local(); // process timeouts/buffers
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

ScanResult measure_obstacle_width(double obstacle_cm)
{
    // --- Initialize the result struct to a "failure" state ---
    ScanResult result = {
        .obstacle_found = false,
        .distance_cm = obstacle_cm, // Store the initial distance
        .short_distance_cm = 0.0,
        .right_edge_angle = -1,
        .left_edge_angle = -1,
        .right_edge_dist = 0.0,
        .left_edge_dist = 0.0,
        .middle_angle = -1};

    const int SERVO_SCAN_STEP = 5;
    const int SCAN_OBSTACLE_THRESHOLD = 30;
    const int SETTLE_MS = 150; // allow servo to settle

    int right_edge_angle = -1;
    int left_edge_angle = -1;
    double right_edge_dist = 0.0;
    double left_edge_dist = 0.0;
    int middle_angle = -1;

    int min_start = -1;
    int min_end = -1;

    double min_scan_dist = 1000.0;

    printf("\n--- Starting Scan (30-150 deg) ---\n");
    servo_pause_task();
    ultrasonic_pause();
    for (int angle = SERVO_SCAN_START_ANGLE; angle <= SERVO_SCAN_END_ANGLE; angle += SERVO_SCAN_STEP)
    {
        // Move servo and allow settling
        servo_set_angle(SERVO_PIN, angle);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

        double dist_at_angle = ultrasonic_get_stable_distance_cm(3, 30);

        if (dist_at_angle > 0 && dist_at_angle < SCAN_OBSTACLE_THRESHOLD)
        {
            // --- Obstacle IS seen ---
            if (dist_at_angle < min_scan_dist - 0.2)
            { // new smaller minimum
                min_scan_dist = dist_at_angle;
                min_start = angle;
                min_end = angle;
            }
            else if (fabs(dist_at_angle - min_scan_dist) <= 0.2)
            { // same flat minimum region
                min_end = angle;
            }

            if (right_edge_angle == -1)
            {
                right_edge_angle = angle; // First detection
                right_edge_dist = dist_at_angle;
            }

            left_edge_angle = angle;
            left_edge_dist = dist_at_angle; // Update left edge on every hit
        }
    }

    // Point servo back to center after scan
    servo_set_angle(SERVO_PIN, 90);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
    ultrasonic_resume();
    servo_resume_task();
    //printf("--- Scan Complete. Returning to 90 deg ---\n");
    middle_angle = (min_start + min_end) / 2;

    if (right_edge_angle != -1 && left_edge_angle > right_edge_angle && middle_angle != -1)
    {

        // prefer law of cosines using the actual measured distances
        double d1 = right_edge_dist - 3.0;
        double d2 = min_scan_dist;
        double d3 = left_edge_dist - 3.0;

        // guard invalids
        if (d1 <= 0 || d3 <= 0)
        {
            printf("Invalid edge distance(s): cannot compute width.\n");
        }
        else
        {
            // double width = sqrt(d1*d1 + d2*d2 - 2.0*d1*d2*cos(theta));
            // result.left_width_cm  = sqrt(d2*d2 - min_scan_dist*min_scan_dist);
            // result.right_width_cm  = sqrt(d1*d1 - min_scan_dist*min_scan_dist);
            // result.obstacle_found = true;
            // result.short_distance_cm = min_scan_dist;

            // double right_width = sqrt(d1*d1 + d2*d2 - 2.0*d1*d2*cos(angle_to_radians(middle_angle - right_edge_angle + ANGLE_OFFSET)));
            // double left_width = sqrt(d3*d3 + d2*d2 - 2.0*d3*d2*cos(angle_to_radians(left_edge_angle - middle_angle - ANGLE_OFFSET)));
            // double width = left_width + right_width;
            double angle_radian = ((left_edge_angle - right_edge_angle) * M_PI / 180.0);
            printf("Angle between edges: %2f", angle_radian);
            double width = sqrt(d1 * d1 + d3 * d3 - 2.0 * d1 * d3 * cos(angle_radian));
            printf("Equation: d1=%.2f, d3=%.2f, angle_diff=%.2f deg\n", d1, d3, angle_radian);

            result.obstacle_found = true;
            result.width_cm = width;
            result.short_distance_cm = min_scan_dist;
            // fill edge angles & distances if you want
            result.right_edge_angle = right_edge_angle;
            result.left_edge_angle = left_edge_angle;
            result.right_edge_dist = right_edge_dist - 3.0;
            result.left_edge_dist = left_edge_dist - 3.0;
            result.middle_angle = middle_angle;
            // result.left_width_cm = left_width;
            // result.right_width_cm = right_width;
            result.middle_angle = middle_angle;
        }
    }
    else
    {
        printf(" -> Could not determine obstacle edges.\n");
    }

    return result;
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
    imu_update(&imu_data);
    float bounded_heading = imu_data.heading;
    const char* EDGE_TO_CHECK = "RIGHT"; // We always check the left edge for line following
    

    // State machine variables
    robot_state_t robot_state = ROBOT_STATE_STRAIGHT;
    uint32_t last_significant_turn_time = 0;
    const uint32_t STRAIGHT_TRANSITION_TIMEOUT_MS = 300; // Time before state -> STRAIGHT
    uint32_t loop_start_time = 0;
    static float heading_history[5] = {0};
    for(int i = 0; i < 5; i++) heading_history[i] = bounded_heading;
    static int heading_idx = 0;
    static uint32_t last_edge_switch_time = 0;

    bool is_first_run = true;


    typedef enum {
        STATE_LINE_FOLLOWING, 
        STATE_STRAIGHT_FOR_SCAN,
        STATE_DRIVING_TO_CARDINAL
    } task_motion_state_t;
    task_motion_state_t motion_state = STATE_LINE_FOLLOWING;
    uint32_t cardinal_drive_start_time = 0;
    while (1) {
        if(g_command_received){
            motion_state = STATE_DRIVING_TO_CARDINAL;
            cardinal_drive_start_time = to_ms_since_boot(get_absolute_time());

            // stop scanning
            barcode_set_scanning_active_local(false);
            motor_set_speed(0.0f, 0.0f); // Stop before turning
            vTaskDelay(pdMS_TO_TICKS(500)); // brief pause
            if(g_received_command == CMD_LEFT){
                printf("COMMAND RECEIVED: TURN LEFT to nearest cardinal bearing.\n");
                g_target_heading = get_left_cardinal_bearing(g_target_heading);
                EDGE_TO_CHECK = "LEFT";
            }
            else if(g_received_command == CMD_RIGHT){
                g_target_heading = get_right_cardinal_bearing(g_target_heading);
                printf("CMD_RIGHT received. TURN RIGHT to nearest cardinal bearing.\n");
                EDGE_TO_CHECK = "RIGHT";
            }
            g_command_received = false;
            g_received_command = CMD_NONE;
        }
        else if(g_request_line_following_suspend && motion_state == STATE_LINE_FOLLOWING) {
            motion_state = STATE_STRAIGHT_FOR_SCAN;

            IMU_Data imu_data;
            imu_update(&imu_data);
            g_target_heading = get_nearest_cardinal_bearing(imu_data.heading);
             printf("SCAN START: Transitioning to STRAIGHT_FOR_SCAN. Target: %.1f\n", g_target_heading);
        }
        else if(!g_request_line_following_suspend && motion_state == STATE_STRAIGHT_FOR_SCAN) {
            motion_state = STATE_LINE_FOLLOWING;
             printf("SCAN END: Resuming LINE_FOLLOWING mode.\n");
        }
        if(motion_state == STATE_STRAIGHT_FOR_SCAN) {
            drive_straight_to_bearing(g_target_heading);
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        }

        else if(motion_state == STATE_DRIVING_TO_CARDINAL){
            uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - cardinal_drive_start_time;
            if(elapsed_time < 2000){
                drive_straight_to_bearing(g_target_heading);
            }
            else{
                printf("Reached target cardinal bearing. Stopping.\n");
                motion_state = STATE_LINE_FOLLOWING;
                barcode_set_scanning_active_local(true); // Resume scanning
            }

            // This state is persistent until a new command is received.
            // The robot will continue driving in this direction indefinitely.
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        }

        else{
            loop_start_time = to_ms_since_boot(get_absolute_time());
        // Update IMU data at the start of the loop
            imu_update(&imu_data);
            heading_history[heading_idx] = imu_data.heading;
            heading_idx = (heading_idx + 1) % 5;
            float smoothed_heading = 0.0f;
            for (int i = 0; i < 5; i++) smoothed_heading += heading_history[i];
            smoothed_heading /= 5.0f;
        
        // Calculate heading change to determine turn direction
        if(is_first_run) {
            // On first run, initialize bounded_heading to current heading
            bounded_heading = imu_data.heading;
            is_first_run = false;
        }
        float heading_change = smoothed_heading - bounded_heading;
        if (heading_change > 180.0f) heading_change -= 360.0f;
        if (heading_change < -180.0f) heading_change += 360.0f;

        // --- State Machine Logic ---
        if (fabsf(heading_change) > BEARING_CHANGE_THRESHOLD) {
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if(now_ms - last_edge_switch_time > EDGE_SWITCH_DEBOUNCE_MS) {
                last_edge_switch_time = now_ms;
                printf("Heading change: %.1f degrees, previous heading: %.1f, current heading: %.1f\n", heading_change, bounded_heading, smoothed_heading);
            bounded_heading = imu_data.heading; // Update heading anchor on turn

            if(robot_state != ROBOT_STATE_TURNING) {
#if PRINT_STATEMENTS
                printf("Significant turn detected. Entering TURNING state.\n");
               
#endif
                robot_state = ROBOT_STATE_TURNING;
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
            last_edge_switch_time = now_ms;
          }
        } else {
            // Heading change is not significant, check if we should go back to STRAIGHT
            if (robot_state == ROBOT_STATE_TURNING && 
                (to_ms_since_boot(get_absolute_time()) - last_significant_turn_time > STRAIGHT_TRANSITION_TIMEOUT_MS)) {
#if PRINT_STATEMENTS
                printf("No significant turn detected for a while. Returning to STRAIGHT state.\n");
#endif
                robot_state = ROBOT_STATE_STRAIGHT;
            }
        }

        line_state_t line_state = line_sensor_read();
        float error = (float)line_state;
        float correction = error;
        
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

            const char* state_str = (robot_state == ROBOT_STATE_TURNING) ? "TURNING" : "STRAIGHT";
            
            // Publish MQTT telemetry
            if (mqtt_connected && mqtt_client != NULL) {
                mqtt_message_t msg;
                
                // Publish state
                snprintf(msg.payload, sizeof(msg.payload), "{\"state\": \"%s\", \"heading\": %.1f, \"edge\": \"%s\"}", state_str, imu_data.heading, EDGE_TO_CHECK);
                safe_mqtt_publish(MQTT_TOPIC_STATE, msg.payload);
                
                // Publish position as a JSON object
                snprintf(msg.payload, sizeof(msg.payload), "{\"distance\": %.2f}", distance);
                safe_mqtt_publish(MQTT_TOPIC_POSITION, msg.payload);
                snprintf(msg.payload, sizeof(msg.payload), "{\"left_speed\": %.3f, \"right_speed\": %.3f}", left_speed, right_speed);
                safe_mqtt_publish(MQTT_TOPIC_SPEED, msg.payload);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        }
        


        
    }
}

void control_task(void *pv)
{
    (void)pv;
    double last_obstacle_distance = 0.0;
    float STOP_CM = 20.0f;
    IMU_Data imu_data;

    enum RobotState
    {
        STATE_SCANNING,
        STATE_CORRECTING
    };

    enum RobotState state = STATE_SCANNING;
    uint32_t last_ultra_ms = to_ms_since_boot(get_absolute_time());

    printf("Waiting for hardware to settle...\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    imu_update(&imu_data);
    float drive_distance_cm, current_yaw;
    uint32_t required_time_ms = 0;
    uint32_t start_time_ms;
    float bearings_list [4];
    int bearings_index = 0;


    while (1)
    {

        if (state == STATE_SCANNING)
        {
            double d = ultrasonic_get_last_distance_cm();
            if (d <= 0.0 || d > STOP_CM)
            {
                // fall back to blocking stable read occasionally
                d = ultrasonic_get_stable_distance_cm(3, 10);
            }

            if (d > 0.0 && d <= STOP_CM)
            {
                // if obstacle, suspend tasks
                vTaskSuspend(line_following_task_handle);
                vTaskSuspend(barcode_task_handle);
                printf("\n!!! Obstacle DETECTED at %.1f cm !!!\n", d);
                last_obstacle_distance = d;
                motor_set_speed(0.0f, 0.0f);
                ScanResult scan = measure_obstacle_width(last_obstacle_distance);
                if (scan.obstacle_found)
                {
                    printf("\n+++ RESULT: Obstacle Scan Complete +++\n");
                    // printf("  -> Left Width:   %.2f cm, Right Width:   %.2f cm, Total Width:   %.2f cm\n", scan.left_width_cm, scan.right_width_cm, scan.left_width_cm + scan.right_width_cm);
                    printf("  -> Measured Width:   %.2f cm\n", scan.width_cm);
                    printf("  -> Shortest Distance to obstacle: %.2f cm\n", scan.short_distance_cm);
                    printf("  -> Edges:   %d deg (L) to %d deg (R)\n", scan.right_edge_angle, scan.left_edge_angle);
                    printf("  -> Distances: %.2f cm (L) to %.2f cm (R)\n", scan.left_edge_dist, scan.right_edge_dist);

                    if (mqtt_connected && mqtt_client != NULL) {
                        mqtt_message_t msg;
                        snprintf(msg.payload, sizeof(msg.payload), "{\"detected\": true, \"width\": %.2f, \"distance\": %.2f}", scan.width_cm, scan.short_distance_cm);
                        safe_mqtt_publish(MQTT_TOPIC_OBSTACLE, msg.payload);
                    }


                    // Variables to remember the turn direction and distance
                    drive_distance_cm = (float)scan.width_cm / 2; // add small buffer
                    if (drive_distance_cm > 30.0f)
                    {
                        drive_distance_cm = 30.0f;
                    }
                    if (scan.left_edge_dist < scan.right_edge_dist)
                    {
                        // Left is shorter (turn CCW/Left)
                        printf("Left is shorter. Starting 90 deg turn LEFT.\n");
                        imu_update(&imu_data);
                        g_target_heading = get_left_cardinal_bearing(get_nearest_cardinal_bearing(imu_data.heading));
                        
                        bearings_list[0] = g_target_heading;
                        bearings_list[1] = get_right_cardinal_bearing(bearings_list[0]);
                        bearings_list[2] = get_right_cardinal_bearing(bearings_list[1]);
                        bearings_list[3] = get_left_cardinal_bearing(bearings_list[2]);
                    }
                    else
                    {
                        // Right is shorter (turn CW/Right)
                        printf("Right is shorter. Starting 90 deg turn RIGHT.\n");
                        imu_update(&imu_data);
                        g_target_heading = get_right_cardinal_bearing(get_nearest_cardinal_bearing(imu_data.heading));
                        bearings_list[0] = g_target_heading;
                        bearings_list[1] = get_left_cardinal_bearing(bearings_list[0]);
                        bearings_list[2] = get_left_cardinal_bearing(bearings_list[1]);
                        bearings_list[3] = get_right_cardinal_bearing(bearings_list[2]);

                        
                    }
                    printf("Bearings list for correction: [%.1f, %.1f, %.1f, %.1f]\n", bearings_list[0], bearings_list[1], bearings_list[2], bearings_list[3]);
                    state = STATE_CORRECTING;
                }
                else
                {
                    printf("\n+++ RESULT: Scan complete. Could not measure width. +++\n");
                    if (mqtt_connected && mqtt_client != NULL) {
                        const char* payload = "{\"detected\": false}";
                        safe_mqtt_publish(MQTT_TOPIC_OBSTACLE, payload);
                    }
                    vTaskResume(line_following_task_handle);
                    vTaskResume(barcode_task_handle);
                }
            }
        }

        else if (state == STATE_CORRECTING)
        {
            // set first loop
            if(!required_time_ms){
                    required_time_ms = (uint32_t)((drive_distance_cm / 9.7f) * 1000.0f + 4000.0f);
                    start_time_ms = to_ms_since_boot(get_absolute_time());
                    printf("required_time_ms set to %lu ms for distance %.1f cm\n", required_time_ms, drive_distance_cm);
            }
            // if required time has not elapsed, keep driving straight
            if((to_ms_since_boot(get_absolute_time()) - start_time_ms) < required_time_ms){
                drive_straight_to_bearing(bearings_list[bearings_index]);
            }
            // else if required time has elapsed, check list of instructions
             // LEFT_CORRECT = L R R L or RIGHT_CORRECT = R L L R
            else if(bearings_index < 3){
                bearings_index++;
                printf("Completed segment to bearing %.1f deg. Next bearing: %.1f deg\n", bearings_list[bearings_index - 1], bearings_list[bearings_index]);
                // reset timer for next segment
                start_time_ms = to_ms_since_boot(get_absolute_time());
            }
            else{
                // completed all segments   
                printf("Completed all correction segments. Resuming SCANNING state.\n");
                // reset for next obstacle
                bearings_index = 0;
                required_time_ms = 0;
                state = STATE_SCANNING;
                vTaskResume(line_following_task_handle);
                vTaskResume(barcode_task_handle);
            }
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

    // Initialize Wi-Fi
    printf("[WIFI] Initializing Wi-Fi...\n");
    if (cyw43_arch_init()) {
        printf("[WIFI] Failed to initialize\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[WIFI] Failed to connect\n");
        return 1;
    }
    
    printf("[WIFI] Connected\n");

    // Initialize MQTT client
    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("[MQTT] Failed to create client\n");
        return 1;
    }

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = MQTT_CLIENT_ID;

    ip_addr_t mqtt_server_ip;
    if (!ip4addr_aton(MQTT_BROKER_IP, &mqtt_server_ip)) {
        printf("[MQTT] Invalid broker IP\n");
        return 1;
    }

    printf("[MQTT] Connecting to broker at %s:%d...\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    err_t err = mqtt_client_connect(mqtt_client, &mqtt_server_ip, MQTT_BROKER_PORT, 
                                    mqtt_connection_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("[MQTT] Connection failed: %d\n", err);
    }
        mqtt_queue = xQueueCreate(10, sizeof(mqtt_message_t)); // Queue can hold 10 messages
    if (mqtt_queue == NULL) {
        printf("[ERROR] Failed to create MQTT queue.\n");
    }

    // Initialize hardware
    motor_init();
    encoder_init();
    pid_init();
    line_sensor_init();
    imu_init();
    barcode_init_local();
    ultrasonic_init_pins();
    servo_init();
    servo_set_angle(SERVO_PIN, 90); // Center servo
    sleep_ms(600); 

    IMU_Data imu_data;
    imu_update(&imu_data);
    calibrate_cardinal_bearings(imu_data.heading);
    printf("Initial target heading set to %.1f degrees\n", g_target_heading);
    barcode_set_callback_local(on_barcode_detected_callback);
    barcode_set_scanning_active_local(true); // Start scanning initially


    // tasks
    ultrasonic_start_task(ULTRA_POLL_MS);
    sleep_ms(300);

    servo_start_task();
    BaseType_t rc = xTaskCreate(control_task, "Control", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    (void)rc;
    BaseType_t lf_task_status = xTaskCreate(line_following_task, "LineFollowingTask", 2048, NULL, 1, &line_following_task_handle);
    BaseType_t barcode_task_status = xTaskCreate(barcode_timeout_task, "BarcodeTimeoutTask", 1024, NULL, 1, &barcode_task_handle);
    xTaskCreate(mqtt_publisher_task, "MQTTPublishTask", 2048, NULL, 1, NULL);

    if(lf_task_status != pdPASS) {
        printf("[ERROR] Failed to create LineFollowingTask (rc=%ld)\n", (long)lf_task_status);
        fflush(stdout);
    }
    if(barcode_task_status != pdPASS) {    
        printf("[ERROR] Failed to create BarcodeTimeoutTask (rc=%ld)\n", (long)barcode_task_status);
        fflush(stdout);
    }
    
    vTaskStartScheduler();


    while(1){
        
    };
}