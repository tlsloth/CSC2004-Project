#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h" 

void main_task(void *pv) {
    (void)pv;
    printf("Servo Test Starting...\n");
    servo_init();
    
    // --- Synchronous Test ---
    printf("\n--- SYNC Test: 0 to 180 degrees ---\n");
    servo_set_angle(SERVO_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    servo_set_angle(SERVO_PIN, 180);
    vTaskDelay(pdMS_TO_TICKS(1000));
    servo_set_angle(SERVO_PIN, 90);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // --- Asynchronous Test Setup ---
    printf("\n--- ASYNC Test: Starting Servo Task ---\n");
    servo_start_task();
    vTaskDelay(pdMS_TO_TICKS(500)); // Give task time to start

    // --- Asynchronous Sweep Test ---
    int angle;
    printf("ASYNC Sweep from %d to %d...\n", SERVO_SCAN_START_ANGLE, SERVO_SCAN_END_ANGLE);
    for (angle = SERVO_SCAN_START_ANGLE; angle <= SERVO_SCAN_END_ANGLE; angle += 5) {
        // Send command to the queue with a short timeout
        if (servo_set_angle_async(angle, 10) == true) {
            printf("Queued angle: %d\n", angle);
        } else {
            printf("Queue full or task not started!\n");
        }
        // This short delay allows the main task to continue rapidly while the servo task handles movement
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
    
    // Test end-to-end movement
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    servo_set_angle_async(SERVO_SCAN_START_ANGLE, 100);
    
    vTaskDelay(pdMS_TO_TICKS(2000)); 

    printf("\n--- ASYNC Test: Pausing Task ---\n");
    servo_pause_task();
    // Attempt to queue commands while paused (they should queue but not execute)
    servo_set_angle_async(45, 100);
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    printf("\n--- ASYNC Test: Resuming Task ---\n");
    servo_resume_task(); // Should jump to 45 degrees
    
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    printf("\nServo Test Complete. Stopping task.\n");
    servo_stop_task();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main() {
    stdio_init_all();
    xTaskCreate(main_task, "MainTask", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    vTaskStartScheduler();
    
    for (;;) {}
}