#include "servo.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// --- Servo configuration ---
#define SERVO_PWM_FREQ 50       // 50 Hz, standard for most servos
#define SERVO_MIN_PULSE_US 565  // Pulse for 0 degrees
#define SERVO_MAX_PULSE_US 2565 // Pulse for 180 degrees


void servo_init(uint servo_pin) {
    gpio_set_function(servo_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(servo_pin);

    // Get system clock speed
    uint32_t f_sys = clock_get_hz(clk_sys);
    
    // Calculate divider to get clock down to 1MHz (1us per tick)
    float divider = (float)f_sys / 1000000.0f;
    
    // Calculate wrap value for 50Hz (20,000 us period)
    uint16_t wrap = 20000; 

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, divider);
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_num, &config, true);
}

void servo_set_angle(uint servoPin, int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Map angle (0-180) to pulse width (MIN_PULSE - MAX_PULSE)
    uint32_t pulse_width_us = (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180) + SERVO_MIN_PULSE_US;
    
    // Set the PWM duty cycle
    // The value set is the number of 1us ticks
    pwm_set_gpio_level(servoPin, pulse_width_us);
}