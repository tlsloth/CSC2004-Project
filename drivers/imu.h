#pragma once
#include <stdint.h>
#include <stdbool.h>

// Data structure to hold IMU readings
typedef struct {
    float raw_ax, raw_ay, raw_az; // Raw accelerometer data
    float ax, ay, az;   // Accelerometer (g)
    float raw_mx, raw_my, raw_mz; // Raw magnetometer data
    float mx, my, mz;   // Magnetometer (µT)
    float heading;      // Tilt-compensated heading (degrees)
} IMU_Data;

// Initialize the IMU (LSM303DLHC)
void imu_init(void);

// Update IMU readings (call in loop)
void imu_update(IMU_Data* data);