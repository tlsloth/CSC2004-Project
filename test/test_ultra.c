#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ultrasonic.h" 

void ultrasonic_test_task(void *pv) {
    (void)pv;
    printf("Ultrasonic Test Starting...\n");
    ultrasonic_init_pins();
    
    // --- Test 1: Blocking Single Measurement ---
    printf("\n--- Test 1: Single Blocking Read ---\n");
    for (int i = 0; i < 5; i++) {
        double dist = ultrasonic_get_distance_cm();
        printf("Distance (Single): %.2f cm\n", dist);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // --- Test 2: Stable (Median) Measurement ---
    printf("\n--- Test 2: Stable (Median) Read (5 samples, 50ms delay) ---\n");
    double stable_dist = ultrasonic_get_stable_distance_cm(5, 50);
    printf("Distance (Stable Median): %.2f cm\n", stable_dist);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // --- Test 3: Asynchronous RTOS Task ---
    printf("\n--- Test 3: Starting RTOS Polling Task (100ms) ---\n");
    ultrasonic_start_task(100);
    
    for (int i = 0; i < 20; i++) {
        double last_dist = ultrasonic_get_last_distance_cm();
        printf("Distance (Async): %.2f cm\n", last_dist);
        vTaskDelay(pdMS_TO_TICKS(250)); // Read every 250ms, task polls every 100ms
    }
    
    printf("\n--- Test 3: Pausing and Resuming Task ---\n");
    ultrasonic_pause();
    printf("Task Paused. Distance should stop updating.\n");
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    ultrasonic_resume();
    printf("Task Resumed. Distance should start updating.\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Cleanup
    ultrasonic_stop_task();
    printf("\nUltrasonic Test Complete. Task stopped.\n");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main() {
    stdio_init_all();
    xTaskCreate(ultrasonic_test_task, "UltraTest", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
    vTaskStartScheduler();
    
    for (;;) {}
}