#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h>
#include "hardware/adc.h"


// ===== Configuration =====
#define LINE_ADC_PIN     28      // GPIO connected to line sensor (example)
#define LINE_ADC_INPUT   2      // ADC input channel for the sensor
#define LINE_THRESHOLD   2081    // Calibrated ADC threshold for line detection

// ===== Line sensor states =====
typedef enum {
    LINE_WHITE = 0,
    LINE_BLACK
} line_state_t;

// ===== Initialization =====
void line_sensor_init(void);

// ===== Read line sensor =====
// Returns LINE_BLACK or LINE_WHITE
line_state_t line_sensor_read(void);

// ===== Read raw ADC value =====
uint16_t line_sensor_read_raw(void);

// ===== Threshold control =====
void line_sensor_set_threshold(uint16_t new_threshold);

float line_sensor_get_error(void);

#endif // IR_SENSOR_H
