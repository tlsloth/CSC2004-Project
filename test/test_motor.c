#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h" 

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("Motor Driver Test Starting...\n");

    // Initialize the motor driver
    motor_init();
    printf("Motor PWM and GPIO pins initialized.\n");

    while (1) {
        printf("\n--- Test 1: Full Forward ---\n");
        motor_set_speed(1.0f, 1.0f); // Left 100% FWD, Right 100% FWD
        sleep_ms(2000);

        printf("\n--- Test 2: Half Speed Reverse ---\n");
        motor_set_speed(-0.5f, -0.5f); // Left 50% REV, Right 50% REV
        sleep_ms(2000);

        printf("\n--- Test 3: Pivot Left (Left Reverse, Right Forward) ---\n");
        motor_set_speed(-0.8f, 0.8f); 
        sleep_ms(2000);

        printf("\n--- Test 4: Stop & Rest ---\n");
        motor_stop();
        sleep_ms(3000); 
    }

    return 0;
}