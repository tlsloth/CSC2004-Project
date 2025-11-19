#ifndef BARCODE_H
#define BARCODE_H

#include "pico/stdlib.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

// ==========================================================
// --- FIXED DEFINITIONS ---
// ==========================================================
#define BARCODE_IR_DO_PIN 6
#define BARCODE_BAR_IS_LOW 0
#define BARCODE_SAMPLE_MS 1
#define BARCODE_VERIFY_MS 5 
#define BARCODE_RESET_MS 2000
#define BARCODE_MAX_LEN 64

// Barcode commands (for the callback)
typedef enum {
    CMD_NONE = 0,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STOP,
    CMD_FORWARD
} barcode_command_t;

// Callback signature for barcode detection
typedef void (*barcode_callback_t)(const char* decoded_str, barcode_command_t cmd);

// --- Public API Functions used by main.c ---
void barcode_init_local(void);
void barcode_start_scanning_local(void);
void barcode_set_callback_local(barcode_callback_t callback);
void barcode_update_local(void);

// Application hooks (implemented in main.c)
// The barcode driver will call these to notify the app that a scan started/finished.
void barcode_scan_started(void);
void barcode_scan_finished(void);

#endif // BARCODE_H