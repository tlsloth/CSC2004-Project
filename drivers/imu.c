#include "imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>

// ---------------- CONFIG ----------------
#define I2C_PORT i2c1
#define SDA_PIN 26
#define SCL_PIN 27
#define BAUDRATE 100000

#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E

// Accelerometer registers
#define CTRL_REG1_A 0x20
#define OUT_X_L_A 0x28

// Magnetometer registers
#define CRA_REG_M 0x00
#define MR_REG_M  0x02
#define OUT_X_H_M 0x03

#define ALPHA 0.1f
#define MAG_SENS 0.092f  // µT per LSB

// ---------------- INTERNAL STATE ----------------
static float ax_f=0, ay_f=0, az_f=0;
static float mx_f=0, my_f=0, mz_f=0;

// Hard-iron offsets for mag calibration
static float mx_off=0, my_off=0;
static float mx_scale=1.0f, my_scale=1.0f;

// ---------------- I2C HELPERS ----------------
static void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

static void i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t* buf, int len) {
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buf, len, false);
}

static float convert_acc(int16_t raw) { return raw * 0.001f; }
static float convert_mag(int16_t raw) { return raw * MAG_SENS; }

// ---------------- PUBLIC FUNCTIONS ----------------
void imu_init(void) {
    i2c_init(I2C_PORT, BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Enable accelerometer and magnetometer
    i2c_write_byte(ACC_ADDR, CTRL_REG1_A, 0x27); // 50Hz XYZ
    i2c_write_byte(MAG_ADDR, CRA_REG_M, 0x14);   // 15Hz
    i2c_write_byte(MAG_ADDR, MR_REG_M, 0x00);    // Continuous mode

    sleep_ms(50);
    printf("✅ IMU initialized (LSM303DLHC)\n");
}

void imu_calibrate_mag(float* mx_o, float* my_o, float* mx_s, float* my_s) {
    printf("🔁 Rotate the IMU 360° slowly for 10 seconds...\n");

    int16_t mx, my, mz;
    int16_t mx_min = 32767, my_min = 32767;
    int16_t mx_max = -32768, my_max = -32768;
    uint8_t buf[6];

    absolute_time_t start = get_absolute_time();
    while (absolute_time_diff_us(start, get_absolute_time()) < 10e6) {
        i2c_read_bytes(MAG_ADDR, OUT_X_H_M, buf, 6);
        mx = (int16_t)(buf[0]<<8 | buf[1]);
        mz = (int16_t)(buf[2]<<8 | buf[3]);
        my = (int16_t)(buf[4]<<8 | buf[5]);

        if (mx < mx_min) mx_min = mx;
        if (mx > mx_max) mx_max = mx;
        if (my < my_min) my_min = my;
        if (my > my_max) my_max = my;

        sleep_ms(50);
    }

    *mx_o = (mx_max + mx_min)/2.0f;
    *my_o = (my_max + my_min)/2.0f;

    float mx_r = (mx_max - mx_min)/2.0f;
    float my_r = (my_max - my_min)/2.0f;
    float avg = (mx_r + my_r)/2.0f;

    *mx_s = avg / mx_r;
    *my_s = avg / my_r;

    printf("✅ Mag calibration done.\n");
}

void imu_update(IMU_Data* data) {
    uint8_t buf[6];

    // --- Accelerometer ---
    i2c_read_bytes(ACC_ADDR, OUT_X_L_A | 0x80, buf, 6);
    int16_t ax_raw = (int16_t)(buf[1]<<8 | buf[0]);
    int16_t ay_raw = (int16_t)(buf[3]<<8 | buf[2]);
    int16_t az_raw = (int16_t)(buf[5]<<8 | buf[4]);

    data->raw_ax = (float)ax_raw;
    data->raw_ay = (float)ay_raw;
    data->raw_az = (float)az_raw;

    float ax = convert_acc(ax_raw);
    float ay = convert_acc(ay_raw);
    float az = convert_acc(az_raw);

    ax_f = ALPHA*ax + (1-ALPHA)*ax_f;
    ay_f = ALPHA*ay + (1-ALPHA)*ay_f;
    az_f = ALPHA*az + (1-ALPHA)*az_f;

    data->ax = ax_f;
    data->ay = ay_f;
    data->az = az_f;

    // --- Magnetometer ---
    i2c_read_bytes(MAG_ADDR, OUT_X_H_M, buf, 6);
    int16_t mx_raw = (int16_t)(buf[0]<<8 | buf[1]);
    int16_t mz_raw = (int16_t)(buf[2]<<8 | buf[3]);
    int16_t my_raw = (int16_t)(buf[4]<<8 | buf[5]);

    data->raw_mx = (float)mx_raw;
    data->raw_my = (float)my_raw;
    data->raw_mz = (float)mz_raw;

    float mx = convert_mag(mx_raw - mx_off);
    float my = convert_mag(my_raw - my_off);
    float mz = convert_mag(mz_raw);

    mx_f = ALPHA*mx + (1-ALPHA)*mx_f;
    my_f = ALPHA*my + (1-ALPHA)*my_f;
    mz_f = ALPHA*mz + (1-ALPHA)*mz_f;

    // Axis correction for LSM303DLHC
    float mx_corr = my_f;
    float my_corr = -mx_f;
    float mz_corr = mz_f;

    data->mx = mx_corr;
    data->my = my_corr;
    data->mz = mz_corr;

    // Tilt-compensated heading
    float roll  = atan2f(ay_f, az_f);
    float pitch = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f));

    float mx_comp = mx_corr * cosf(pitch) + mz_corr * sinf(pitch);
    float my_comp = mx_corr * sinf(roll)*sinf(pitch) + my_corr*cosf(roll) - mz_corr*sinf(roll)*cosf(pitch);

    float heading = atan2f(my_comp, mx_comp) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;

    data->heading = heading;
}
