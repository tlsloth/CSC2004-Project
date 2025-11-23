#include "servo.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// FreeRTOS for async servo task
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// --- Servo configuration ---
#define SERVO_PWM_FREQ 50       // 50 Hz, standard for most servos
#define SERVO_MIN_PULSE_US 565  // Pulse for 0 degrees
#define SERVO_MAX_PULSE_US 2565 // Pulse for 180 degrees

// stored servo pin for RTOS task
static uint g_servo_pin = 0;

static SemaphoreHandle_t servo_mutex = NULL;


void servo_init() {
    // store pin for RTOS task use
    g_servo_pin = SERVO_PIN;
    gpio_set_function(g_servo_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(g_servo_pin);

    // Get system clock speed
    uint32_t f_sys = clock_get_hz(clk_sys);

    float clk_div = (float)f_sys / 1000000.0f;
    
    
    //uint16_t wrap = (1000000u / SERVO_PWM_FREQ) - 1u;
    uint32_t pwm_clk = f_sys / clk_div;
    uint16_t wrap = (pwm_clk / SERVO_PWM_FREQ) - 1;
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clk_div);
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_num, &config, true);

    if (servo_mutex == NULL) {
        servo_mutex = xSemaphoreCreateMutex();
    }
}

void servo_set_angle(uint servoPin, int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Map angle (0-180) to pulse width (MIN_PULSE - MAX_PULSE)
    uint32_t pulse_width_us = (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180) + SERVO_MIN_PULSE_US;
    
    if (servo_mutex) xSemaphoreTake(servo_mutex, portMAX_DELAY);
    // The PWM config above uses 1us ticks (wrap ~= 20000), so writing microseconds as level is correct
    pwm_set_gpio_level(servoPin, pulse_width_us);
    if (servo_mutex) xSemaphoreGive(servo_mutex);
}

// --- RTOS servo task implementation ---
static QueueHandle_t servo_queue = NULL;
static TaskHandle_t servo_task_handle = NULL;

static void servo_task(void *pv) {
    (void)pv;
    for (;;) {
        int angle = 90; // default center
        if (xQueueReceive(servo_queue, &angle, portMAX_DELAY) == pdTRUE) {
            // set angle synchronously (will block briefly while PWM updated)
            servo_set_angle(g_servo_pin, angle);
            // small delay to allow servo to move to position
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void servo_start_task(void) {
    if (servo_queue == NULL) {
        servo_queue = xQueueCreate(8, sizeof(int));
    }
    if (servo_task_handle == NULL) {
        BaseType_t rc = xTaskCreate(servo_task, "ServoTask", 1024, NULL, tskIDLE_PRIORITY + 1, &servo_task_handle);
        (void)rc;
    } else {
        vTaskResume(servo_task_handle);
    }
}

void servo_pause_task(void) {
    if (servo_task_handle) {
        vTaskSuspend(servo_task_handle);
    }
}

void servo_resume_task(void) {
    if (servo_task_handle) {
        vTaskResume(servo_task_handle);
    } else {
        servo_start_task();
    }
}

void servo_stop_task(void) {
    // If you truly want to delete resources, do so explicitly.
    if (servo_task_handle) {
        vTaskDelete(servo_task_handle);
        servo_task_handle = NULL;
    }
    if (servo_queue) {
        vQueueDelete(servo_queue);
        servo_queue = NULL;
    }
    if (servo_mutex) {
        vSemaphoreDelete(servo_mutex);
        servo_mutex = NULL;
    }
}

bool servo_set_angle_async(int angle, uint32_t timeout_ms) {
    if (servo_queue == NULL) return false;
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    return xQueueSend(servo_queue, &angle, ticks) == pdTRUE;
}