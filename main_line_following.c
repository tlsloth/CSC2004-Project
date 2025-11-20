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

#include "FreeRTOS.h"
#include "task.h"

// MQTT
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"


// Task handle for barcode task
static TaskHandle_t barcode_task_handle = NULL;

// MQTT client and connection state
static mqtt_client_t *mqtt_client = NULL;
static volatile bool mqtt_connected = false;
static ip_addr_t mqtt_broker_ip;

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

// MQTT connection callback
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Connected to broker\n");
        mqtt_connected = true;
    } else {
        printf("[MQTT] Connection failed: %d\n", status);
        mqtt_connected = false;
    }
    fflush(stdout);
}

// MQTT publish complete callback
static void mqtt_pub_request_cb(void *arg, err_t result) {
    (void)arg;
    if (result != ERR_OK) {
        printf("[MQTT] Publish error: %d\n", result);
    }
}

static void on_barcode_detected_callback(const char *decoded_str, barcode_command_t cmd) {
    (void)cmd;
    printf("%s\n", decoded_str);
    fflush(stdout);
    
    // Publish barcode to MQTT
    if (mqtt_connected && mqtt_client) {
        char payload[128];
        snprintf(payload, sizeof(payload), "{\"barcode\":\"%s\"}", decoded_str);
        err_t err = mqtt_publish(mqtt_client, "robot/barcode", payload, strlen(payload), 
                                 0, 0, mqtt_pub_request_cb, NULL);
        if (err != ERR_OK) {
            printf("[MQTT] Failed to publish barcode: %d\n", err);
        }
    }
    
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
        if (to_ms_since_boot(get_absolute_time()) - telemetry_timer >= TELEMETRY_INTERVAL_MS) {
            telemetry_timer = to_ms_since_boot(get_absolute_time());
            float distance = encoder_get_distance_m();
            float p, i, d;
            pid_get_heading_gains(&p, &i, &d);

            const char* state_str = (robot_state == ROBOT_STATE_TURNING) ? "TURNING" : "STRAIGHT";

            // Publish telemetry to MQTT
            if (mqtt_connected && mqtt_client) {
                char payload[256];
                
                // State
                snprintf(payload, sizeof(payload), 
                         "{\"state\":\"%s\",\"edge\":\"%s\",\"heading\":%.1f}",
                         state_str, EDGE_TO_CHECK, imu_data.heading);
                mqtt_publish(mqtt_client, MQTT_TOPIC_STATE, payload, strlen(payload), 0, 0, NULL, NULL);
                
                // Error
                snprintf(payload, sizeof(payload), 
                         "{\"error\":%.3f,\"correction\":%.3f,\"p\":%.2f,\"i\":%.2f,\"d\":%.2f}",
                         error, correction, p, i, d);
                mqtt_publish(mqtt_client, MQTT_TOPIC_ERROR, payload, strlen(payload), 0, 0, NULL, NULL);
                
                // Speed
                snprintf(payload, sizeof(payload), 
                         "{\"left\":%.2f,\"right\":%.2f,\"base\":%.2f}",
                         left_speed, right_speed, BASE_SPEED);
                mqtt_publish(mqtt_client, MQTT_TOPIC_SPEED, payload, strlen(payload), 0, 0, NULL, NULL);
                
                // Position
                snprintf(payload, sizeof(payload), "{\"distance\":%.2f}", distance);
                mqtt_publish(mqtt_client, MQTT_TOPIC_POSITION, payload, strlen(payload), 0, 0, NULL, NULL);
            }
        }


        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial

    printf("\n=== PID Line Following Robot (Analog IR) with MQTT ===\n");

    // Initialize Wi-Fi
    if (cyw43_arch_init()) {
        printf("[ERROR] Failed to initialize Wi-Fi\n");
        return 1;
    }
    
    cyw43_arch_enable_sta_mode();
    printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
    
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[ERROR] Failed to connect to Wi-Fi\n");
        return 1;
    }
    
    printf("[WIFI] Connected successfully\n");
    
    // Initialize MQTT
    mqtt_client = mqtt_client_new();
    if (!mqtt_client) {
        printf("[ERROR] Failed to create MQTT client\n");
        return 1;
    }
    
    // Parse broker IP
    if (!ip4addr_aton(MQTT_BROKER_IP, &mqtt_broker_ip)) {
        printf("[ERROR] Invalid MQTT broker IP\n");
        return 1;
    }
    
    // Connect to MQTT broker
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = MQTT_CLIENT_ID;
    
    err_t err = mqtt_client_connect(mqtt_client, &mqtt_broker_ip, MQTT_BROKER_PORT, 
                                    mqtt_connection_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("[ERROR] Failed to start MQTT connection: %d\n", err);
        return 1;
    }
    
    printf("[MQTT] Connecting to broker at %s:%d...\n", MQTT_BROKER_IP, MQTT_BROKER_PORT);
    sleep_ms(2000); // Give MQTT time to connect

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