#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ---------------- CONFIG ----------------
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1
#define BAUDRATE 400000

#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E

// Accelerometer registers
#define CTRL_REG1_A 0x20
#define OUT_X_L_A 0x28

// Magnetometer registers
#define CRA_REG_M  0x00
#define MR_REG_M   0x02
#define OUT_X_H_M  0x03

#define ALPHA 0.1f
#define MAG_SENS 0.092f  // µT/LSB

// ---------------- HELPERS ----------------
static void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
}

static void i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, int len) {
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buf, len, false);
}

static float convert_acc(int16_t raw) { return raw * 0.001f; }
static float convert_mag(int16_t raw) { return raw * MAG_SENS; }

// ---------------- INIT ----------------
void imu_init(void) {
    i2c_init(I2C_PORT, BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_write_byte(ACC_ADDR, CTRL_REG1_A, 0x27); // 50Hz, enable XYZ
    i2c_write_byte(MAG_ADDR, CRA_REG_M, 0x14);   // 15Hz
    i2c_write_byte(MAG_ADDR, MR_REG_M, 0x00);    // Continuous mode

    sleep_ms(50);
    printf("✅ IMU initialized (LSM303DLHC)\n");
}

// ---------------- CALIBRATION ----------------
void mag_calibration(float *mx_off, float *my_off, float *mx_scale, float *my_scale) {
    printf("🔁 Rotate the IMU 360° slowly for 10 seconds...\n");

    int16_t mx, my, mz;
    int16_t mx_min = 32767, my_min = 32767;
    int16_t mx_max = -32768, my_max = -32768;
    uint8_t buf[6];

    absolute_time_t start = get_absolute_time();
    while (absolute_time_diff_us(start, get_absolute_time()) < 10e6) { // 10 seconds
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

    // --- HARD-IRON OFFSET ---
    *mx_off = (mx_max + mx_min) / 2.0f;
    *my_off = (my_max + my_min) / 2.0f;

    // --- SOFT-IRON SCALE ---
    float mx_range = (mx_max - mx_min) / 2.0f;
    float my_range = (my_max - my_min) / 2.0f;
    float avg_range = (mx_range + my_range) / 2.0f;

    *mx_scale = avg_range / mx_range;
    *my_scale = avg_range / my_range;

    printf("✅ Calibration done.\n");
    printf("   mx_offset = %.2f, my_offset = %.2f\n", *mx_off, *my_off);
    printf("   mx_scale  = %.3f, my_scale  = %.3f\n", *mx_scale, *my_scale);
}

// ---------------- MAIN TEST LOOP ----------------
int main() {
    
    stdio_init_all();
    // wait for usb serial monitor
    while(!stdio_usb_connected()) sleep_ms(100);

    imu_init();

    sleep_ms(2000); // Wait for serial to connect

    float mx_off = 0, my_off = 0;
    //mag_calibration(&mx_off, &my_off);

    uint8_t buf[6];
    float ax_f=0, ay_f=0, az_f=0;
    float mx_f=0, my_f=0, mz_f=0;

    printf("\n--- Live IMU Data Stream ---\n");
    printf("ax\tay\taz\tmx\tmy\tmz\theading\n");

    while (true) {
        // --- ACC ---
        i2c_read_bytes(ACC_ADDR, OUT_X_L_A | 0x80, buf, 6);
        int16_t ax_raw = (int16_t)(buf[1]<<8 | buf[0]);
        int16_t ay_raw = (int16_t)(buf[3]<<8 | buf[2]);
        int16_t az_raw = (int16_t)(buf[5]<<8 | buf[4]);

        float ax = convert_acc(ax_raw);
        float ay = convert_acc(ay_raw);
        float az = convert_acc(az_raw);

        ax_f = ALPHA*ax + (1-ALPHA)*ax_f;
        ay_f = ALPHA*ay + (1-ALPHA)*ay_f;
        az_f = ALPHA*az + (1-ALPHA)*az_f;

        // --- MAG ---
        i2c_read_bytes(MAG_ADDR, OUT_X_H_M, buf, 6);
        int16_t mx_raw = (int16_t)(buf[0]<<8 | buf[1]);
        int16_t mz_raw = (int16_t)(buf[2]<<8 | buf[3]);
        int16_t my_raw = (int16_t)(buf[4]<<8 | buf[5]);

        float mx = convert_mag(mx_raw - mx_off);
        float my = convert_mag(my_raw - my_off);
        float mz = convert_mag(mz_raw);

        mx_f = ALPHA*mx + (1-ALPHA)*mx_f;
        my_f = ALPHA*my + (1-ALPHA)*my_f;
        mz_f = ALPHA*mz + (1-ALPHA)*mz_f;

        // --- AXIS CORRECTION (important for LSM303DLHC)
        float mx_corr = my_f;
        float my_corr = -mx_f;
        float mz_corr = mz_f;

        // --- HEADING COMPUTATION ---
        float roll  = atan2f(ay_f, az_f);
        float pitch = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f));

        float mx_comp = mx_corr * cosf(pitch) + mz_corr * sinf(pitch);
        float my_comp = mx_corr * sinf(roll)*sinf(pitch) + my_corr*cosf(roll) - mz_corr*sinf(roll)*cosf(pitch);

        float heading = atan2f(my_comp, mx_comp) * 180.0f / M_PI;
        if (heading < 0) heading += 360.0f;

        printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.1f°\n",
               ax_f, ay_f, az_f, mx_f, my_f, mz_f, heading);

        sleep_ms(200);
    }
}
