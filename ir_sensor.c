// ===============================================
//  Module: Line Sensor Driver
//  Description: ADC-based line detection for PID control
// ===============================================
#include "ir_sensor.h"
#include <stdio.h>

// Current threshold for line detection
static uint16_t threshold = LINE_THRESHOLD;

// Initialize line sensor (ADC)
void line_sensor_init(void) {
    adc_init();
    adc_gpio_init(LINE_ADC_PIN);
    adc_select_input(LINE_ADC_INPUT);
    printf("[LINE_SENSOR] Initialized on GPIO %d (ADC%d)\n", LINE_ADC_PIN, LINE_ADC_INPUT);
}

// Read line sensor state (black or white)
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

// Add this function to ir_sensor.c
float line_sensor_get_error(void) {
    uint16_t raw_value = line_sensor_read_raw();
    
    // For single sensor: treat deviation from ideal line value as error
    // When perfectly on line: raw_value should be around threshold
    // When off to one side: value changes
    
    const uint16_t target_value = threshold;  // Use the threshold as center
    const uint16_t max_error = 1000;          // Maximum expected deviation
    
    float error = (float)((int16_t)raw_value - (int16_t)target_value) / max_error;
    
    // Clamp error to [-1.0, 1.0] range
    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;
    
    return error;
}