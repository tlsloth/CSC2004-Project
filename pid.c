#include "pid.h"

static float kp_speed   = 0.45f;
static float ki_speed   = 0.05f;
static float kd_speed   = 0.02f;

static float kp_heading = 0.8f;   
static float ki_heading = 0.05f; 
static float kd_heading = 0.25f; 

static float speed_integral = 0.0f;
static float prev_speed_err = 0.0f;

static float heading_integral = 0.0f;
static float prev_heading_err = 0.0f;


void pid_init(void) {
    speed_integral = 0.0f;
    prev_speed_err = 0.0f;
    heading_integral = 0.0f;
    prev_heading_err = 0.0f;
}

float pid_compute_speed(float target, float measured) {
    float error = target - measured;
    speed_integral += error;
    float derivative = error - prev_speed_err;
    prev_speed_err = error;

    float output = (kp_speed * error) + (ki_speed * speed_integral) + (kd_speed * derivative);

    if (speed_integral > 200.0f) speed_integral = 200.0f;
    if (speed_integral < -200.0f) speed_integral = -200.0f;

    return output;
}

float pid_compute_heading(float heading_error)
{
    // Very small deadband for narrow line - we need precision
    if (heading_error > -0.5f && heading_error < 0.5f) {
        heading_error = 0.0f;
    }
    
    heading_integral += heading_error;
    float derivative = heading_error - prev_heading_err;
    prev_heading_err = heading_error;

    float output = (kp_heading * heading_error) +
                   (ki_heading * heading_integral) +
                   (kd_heading * derivative);

    // Anti-windup limit for integral term
    if (heading_integral > 100.0f) heading_integral = 100.0f;
    if (heading_integral < -100.0f) heading_integral = -100.0f;

    // Clamp output for motor correction range (increased from ±30 to ±50)
    if (output > 50.0f) output = 50.0f;
    if (output < -50.0f) output = -50.0f;

    return output;
}
// ---- Get PID gains for debugging/tuning ----
void pid_get_heading_gains(float *kp, float *ki, float *kd)
{
    if (kp) *kp = kp_heading;
    if (ki) *ki = ki_heading;
    if (kd) *kd = kd_heading;
}