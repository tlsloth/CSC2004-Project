#include <stdio.h>
#include "pico/stdlib.h"
#include "ir_sensor.h" 

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("Line Sensor Test Starting...\n");

    // Initialize the line sensor module
    line_sensor_init();

    // Main loop
    while (1) {
        // Read the current line state (digital mode)
        line_state_t state = line_sensor_read();
        
        // Print the result
        if (state == LINE_BLACK) {
            printf("State: **LINE_BLACK** (1)\n");
        } else {
            printf("State: LINE_WHITE (-1)\n");
        }

        sleep_ms(50); // Read every 50ms
    }

    return 0;
}