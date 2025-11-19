#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

// Drivers
#include "drivers/barcode.h"
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/pid.h"
#include "drivers/ir_sensor.h"
#include "drivers/state.h"
#include "straight_mode.h"


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

// Only print decoded barcode value
static void on_barcode_detected_callback(const char *decoded_str, barcode_command_t cmd) {
    (void)cmd;
    printf("%s\n", decoded_str);
    fflush(stdout);
    // ensure scan finished flag cleared
    barcode_scan_finished();
}

// Periodic barcode maintenance and log printing task
static void barcode_timeout_task(void *params) {
    (void)params;
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

int main(void) {
    stdio_init_all();
    sleep_ms(1500); // allow USB CDC enumerate

    // Barcode init + callback
    barcode_init_local();
    barcode_set_callback_local(on_barcode_detected_callback);
    barcode_start_scanning_local();

    //Start barcode task only
    BaseType_t rc = xTaskCreate(
        barcode_timeout_task, "BarcodeTimeout",
        1024, NULL, 1, NULL);

    if (rc != pdPASS) {
        printf("[ERROR] Failed to create BarcodeTimeout task (rc=%ld)\n", (long)rc);
        fflush(stdout);
    }

    vTaskStartScheduler();

    while (1) { }
    return 0;
}