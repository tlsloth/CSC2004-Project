#include <stdio.h>
#include "pico/stdlib.h"
#include "imu.h" // Include your header file

int main() {
    // Initialize standard I/O (e.g., for serial printing)
    stdio_init_all();
    sleep_ms(2000); 
    printf("IMU Test Starting...\n");

    // Initialize the IMU module and I2C
    imu_init();
    
    // --- Calibration Variables ---
    float mx_offset = 0.0f;
    float my_offset = 0.0f;
    float mx_scale = 1.0f;
    float my_scale = 1.0f;

    // --- Magnetometer Calibration ---
    // NOTE: This function requires the user to rotate the IMU board
    imu_calibrate_mag(&mx_offset, &my_offset, &mx_scale, &my_scale);
    printf("Offsets (raw): X=%.2f, Y=%.2f\n", mx_offset, my_offset);
    printf("Scales: X=%.2f, Y=%.2f\n", mx_scale, my_scale);

    // Main loop
    IMU_Data data;
    while (1) {
        imu_update(&data); // Read and process new IMU data

        // Print Accelerometer (g)
        printf("Acc (g): X=%.2f, Y=%.2f, Z=%.2f | ", data.ax, data.ay, data.az);
        
        // Print Magnetometer (uT)
        printf("Mag (uT): X=%.2f, Y=%.2f, Z=%.2f | ", data.mx, data.my, data.mz);

        // Print Heading
        printf("Heading: %.1f deg\n", data.heading);

        // Wait for a short period before the next reading
        sleep_ms(100); 
    }

    return 0;
}