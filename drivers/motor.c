// ===============================================
//  Module: Motor Driver for RoboPico
//  Description: PWM motor control with direction pins
// ===============================================
#include "motor.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// ----- Motor pins -----
#define M1_PWM_PIN  10
#define M1_DIR_PIN  11
#define M2_PWM_PIN  8
#define M2_DIR_PIN  9

// ----- PWM slices & channels -----
static uint slice_l, chan_l;
static uint slice_r, chan_r;

// ----- Convert speed [0.0 - 1.0] to PWM duty -----
static inline uint16_t speed_to_pwm(float speed)
{
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;
    return (uint16_t)(speed * 65535.0f);
}

// ----- Initialize motor driver -----
void motor_init(void) 
{
    // Direction pins
    gpio_init(M1_DIR_PIN);
    gpio_set_dir(M1_DIR_PIN, GPIO_OUT);
    gpio_put(M1_DIR_PIN, 0);

    gpio_init(M2_DIR_PIN);
    gpio_set_dir(M2_DIR_PIN, GPIO_OUT);
    gpio_put(M2_DIR_PIN, 0);

    // PWM pins
    gpio_set_function(M1_PWM_PIN, GPIO_FUNC_PWM);
    gpio_set_function(M2_PWM_PIN, GPIO_FUNC_PWM);

    slice_l = pwm_gpio_to_slice_num(M1_PWM_PIN);
    chan_l  = pwm_gpio_to_channel(M1_PWM_PIN);
    slice_r = pwm_gpio_to_slice_num(M2_PWM_PIN);
    chan_r  = pwm_gpio_to_channel(M2_PWM_PIN);

    pwm_set_wrap(slice_l, 65535);
    pwm_set_wrap(slice_r, 65535);

    pwm_set_enabled(slice_l, true);
    pwm_set_enabled(slice_r, true);
}

// ----- Set motor speed -----
void motor_set_speed(float left, float right) 
{
    // Handle left motor direction
    if (left < 0.0f) {
        gpio_put(M1_DIR_PIN, 1);  // Reverse
        left = -left;
    } else {
        gpio_put(M1_DIR_PIN, 0);  // Forward
    }

    // Handle right motor direction
    if (right < 0.0f) {
        gpio_put(M2_DIR_PIN, 1);  // Reverse
        right = -right;
    } else {
        gpio_put(M2_DIR_PIN, 0);  // Forward
    }

    pwm_set_chan_level(slice_l, chan_l, speed_to_pwm(left));
    pwm_set_chan_level(slice_r, chan_r, speed_to_pwm(right));
}

// ----- Stop motors -----
void motor_stop(void) 
{
    pwm_set_chan_level(slice_l, chan_l, 0);
    pwm_set_chan_level(slice_r, chan_r, 0);
}
