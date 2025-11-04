#include <stdio.h>
#include <string.h> 
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "barcode.h"

void on_barcode_detected_callback(const char *decoded_str, barcode_command_t cmd)
{
    printf("\n--- BARCODE DETECTED ---\n");
    printf("  Raw String: \"%s\"\n", decoded_str);

    switch (cmd)
    {
        case CMD_LEFT:
            printf("  Command: LEFT\n");
            break;
        case CMD_RIGHT:
            printf("  Command: RIGHT\n");
            break;
        case CMD_STOP:
            printf("  Command: STOP\n");
            break;
        case CMD_FORWARD:
            printf("  Command: FORWARD\n");
            break;
        default:
            printf("  Command: UNKNOWN\n");
            break;
    }
    printf("------------------------\n\n");
    fflush(stdout);
}


void barcode_timeout_task(void *params)
{
    printf("[TIMEOUT TASK] Task started\n");
    while (true)
    {
        // This function checks for scan timeouts
        barcode_update_local();
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


int main() {
    stdio_init_all();
    sleep_ms(2000); 

    printf("--- Pico Barcode Decoder (MERGED Library Version) ---\n");
    fflush(stdout);

    // 1. Initialize the new local library
    barcode_init_local();

    // 2. Tell the library which function to call on success
    barcode_set_callback_local(on_barcode_detected_callback);

    // 3. Start the library's internal timer interrupt
    barcode_start_scanning_local();

    // 4. Create the simple update task
    BaseType_t task_ok = xTaskCreate(
        barcode_timeout_task, 
        "BarcodeTimeoutTask", 
        1024,
        NULL, 
        1, // Low priority
        NULL
    );

    if (task_ok != pdPASS) {
        printf("Failed to create BarcodeTimeoutTask!\n");
        fflush(stdout);
    }

    // Start the scheduler
    vTaskStartScheduler();

    while(1) {
        // Should never reach here
    }
}

