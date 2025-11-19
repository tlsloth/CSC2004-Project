// ===============================================
//  Module: Line Sensor Driver
//  Description: Analog ADC-based line detection for PID control
// ===============================================
#include "ir_sensor.h"
#include <stdio.h>
#include "hardware/adc.h"

// Static variable to hold the detection threshold
static uint16_t threshold = LINE_THRESHOLD;

void line_sensor_init(void) {
    adc_init();
    adc_gpio_init(LINE_ADC_PIN);
    adc_select_input(LINE_ADC_INPUT);
    printf("[LINE_SENSOR] ADC Initialized on GPIO %d (ADC%d)\n", LINE_ADC_PIN, LINE_ADC_INPUT);
}

// Read line state based on threshold
line_state_t line_sensor_read(void) {
    uint16_t value = adc_read();
    return (value > threshold) ? LINE_BLACK : LINE_WHITE;
}

// Read raw ADC value
uint16_t line_sensor_read_raw(void) {
    return adc_read();
}

// Set new detection threshold
void line_sensor_set_threshold(uint16_t new_threshold) {
    threshold = new_threshold;
    printf("[LINE_SENSOR] Threshold set to %u\n", threshold);
}

// Calculate a normalized error value based on the raw ADC reading
float line_sensor_get_error(void) {
    uint16_t raw_value = line_sensor_read_raw();
    
    // This logic assumes the sensor is positioned on the edge of the line.
    // The error is proportional to how far from the threshold the reading is.
    // A more sophisticated multi-sensor setup would provide better error values.
    const float max_range = 1500.0f; // The expected range of ADC values around the threshold
    
    float error = (float)((int16_t)raw_value - (int16_t)threshold) / max_range;
    
    // Clamp error to [-1.0, 1.0] range
    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;
    
    return error;
}