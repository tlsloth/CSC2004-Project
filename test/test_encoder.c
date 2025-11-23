#include <stdio.h>
#include "pico/stdlib.h"
#include "encoder.h" // Include your header file

int main() {
    // Initialize standard I/O (e.g., for serial printing)
    stdio_init_all();
    
    // Give time for serial connection to establish
    sleep_ms(2000); 
    printf("Encoder Test Starting...\n");

    // Initialize the encoder module
    encoder_init();

    // Main loop
    while (1) {
        // Read values
        float left_rpm = encoder_get_left_rpm();
        float right_rpm = encoder_get_right_rpm();
        float distance = encoder_get_distance_m();
        
        uint32_t left_ticks, right_ticks;
        encoder_get_ticks(&left_ticks, &right_ticks);

        // Print values
        printf("Ticks: L=%lu, R=%lu | ", left_ticks, right_ticks);
        printf("RPM: L=%.2f, R=%.2f | ", left_rpm, right_rpm);
        printf("Distance: %.4f m\n", distance);

        // Wait for a short period before the next reading
        sleep_ms(100); 
    }

    return 0;
}