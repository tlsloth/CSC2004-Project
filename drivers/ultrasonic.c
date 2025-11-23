#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"

// FreeRTOS (available in this project)
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// if you need motor API:
#include "motor.h" // adjust if different

// pins (set by init)
static uint ULTRA_TRIG_PIN = 4;
static uint ULTRA_ECHO_PIN = 5;

// ISR-safe 32-bit timestamps
volatile static uint32_t echo_start_us = 0;
volatile static uint32_t pulse_dur_us = 0;
volatile static bool echo_completed = false;
volatile static bool echo_rising = false;

// RTOS primitives
static SemaphoreHandle_t ultrasonic_mutex = NULL; // protects trigger/echo sequence & last_distance_cm
static double last_distance_cm = 0.0;
static TaskHandle_t ultrasonic_task_handle = NULL;


// ----- GPIO callback -----
// Make callback static to avoid external linkage collisions
static void gpio_callback(uint gpio, uint32_t events) {
    if (gpio != ULTRA_ECHO_PIN) return;

    if (events & GPIO_IRQ_EDGE_RISE) {
        echo_start_us = time_us_32();
        echo_completed = false;
        echo_rising = true;
    } 
    else if (events & GPIO_IRQ_EDGE_FALL) {
        if (echo_rising) {
            uint32_t end = time_us_32();
            pulse_dur_us = end - echo_start_us;
            echo_completed = true;
            echo_rising = false;
        }
    }

}

// send trigger pulse (blocking micro delays ok)
static void trigger_pulse(void) {
    gpio_put(ULTRA_TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(ULTRA_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRA_TRIG_PIN, 0);
}

// Initialize pins and register callback
void ultrasonic_init_pins() {
    ULTRA_TRIG_PIN = ULTRASONIC_TRIG_PIN;
    ULTRA_ECHO_PIN = ULTRASONIC_ECHO_PIN;

    if (!ultrasonic_mutex) {
        ultrasonic_mutex = xSemaphoreCreateMutex();
    }

    gpio_init(ULTRA_TRIG_PIN);
    gpio_set_dir(ULTRA_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRA_TRIG_PIN, 0);

    gpio_init(ULTRA_ECHO_PIN);
    gpio_set_dir(ULTRA_ECHO_PIN, GPIO_IN);

    // Register IRQ callback for the GPIO subsystem
    // The SDK expects a pointer to a callback; our callback is static above
    gpio_set_irq_enabled_with_callback(ULTRA_ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

// Helper to check whether FreeRTOS scheduler is running
static inline bool scheduler_running(void) {
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

// single measurement (blocks up to timeout_us)
// NOTE: This function is RTOS-aware: if the scheduler is running it will yield via vTaskDelay(1)
//       otherwise it falls back to sleep_us() (safe for pre-scheduler use in main()).
double ultrasonic_get_distance_cm(void) {
    xSemaphoreTake(ultrasonic_mutex, portMAX_DELAY);
    echo_completed = false;
    pulse_dur_us = 0;
    trigger_pulse();

    uint32_t start = time_us_32();
    const uint32_t timeout_us = 50000u;
    double result = 0.0;

    while (!echo_completed) {
        if ((uint32_t)(time_us_32() - start) > timeout_us) {
            xSemaphoreGive(ultrasonic_mutex);
            return 0.0;
        }

        if (scheduler_running()) {
            vTaskDelay(1); 
        } else {
            sleep_us(100); 
        }
    }

    // we've got an echo; copy duration
    uint32_t dur = pulse_dur_us;

    // release mutex before computing
    xSemaphoreGive(ultrasonic_mutex);


    if (dur == 0 || dur > 30000u) return 0.0;
    // convert microseconds to centimeters: time * (speed_of_sound (cm/us)) / 2
    // speed of sound ≈ 0.0343 cm/us
    result = (dur * 0.0343) / 2.0;
    return result;
}

// comparator for median sorting
static int compare_double(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

double ultrasonic_get_stable_distance_cm(int num_samples, int delay_ms) {
    if (num_samples <= 0) return 0.0;
    if (num_samples % 2 == 0) num_samples++; // make odd to have a single median

    // guard upper size to reasonable stack usage
    if (num_samples > 31) num_samples = 31; // safe cap

    double samples[num_samples];

    for (int i = 0; i < num_samples; i++) {
        samples[i] = ultrasonic_get_distance_cm();
        // use RTOS delay when possible, otherwise fallback to sleep_ms if called pre-scheduler
        if (scheduler_running()) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        } else {
            sleep_ms(delay_ms);
        }
    }

    qsort(samples, num_samples, sizeof(double), compare_double);

    return samples[num_samples / 2];
}

// RTOS: task that periodically triggers a measurement and stores last_distance_cm
static void ultrasonic_task(void *pv) {
    uint32_t poll_ms = 100;
    if (pv) poll_ms = (uint32_t)(uintptr_t)pv;

    const TickType_t delay = pdMS_TO_TICKS(poll_ms);

    for (;;) {
        double d = ultrasonic_get_distance_cm();

        // store result protected by mutex (ultrasonic_get_distance already used mutex to protect trigger/echo,
        // but we take it again to safely update last_distance_cm)
        if (ultrasonic_mutex) xSemaphoreTake(ultrasonic_mutex, portMAX_DELAY);
        last_distance_cm = d;
        if (ultrasonic_mutex) xSemaphoreGive(ultrasonic_mutex);

        vTaskDelay(delay);
    }
}

void ultrasonic_start_task(uint32_t poll_ms) {
    configASSERT(ultrasonic_mutex != NULL);
    if (ultrasonic_task_handle == NULL) {
        BaseType_t rc = xTaskCreate(ultrasonic_task, "UltraTask", 2048, (void *)(uintptr_t)poll_ms, tskIDLE_PRIORITY + 1, &ultrasonic_task_handle);
        (void)rc;
    } else {
        vTaskResume(ultrasonic_task_handle);
    }
}

void ultrasonic_pause(void) {
    if (ultrasonic_task_handle) vTaskSuspend(ultrasonic_task_handle);
}

void ultrasonic_resume(void) {
    if (ultrasonic_task_handle) vTaskResume(ultrasonic_task_handle);
}

double ultrasonic_get_last_distance_cm(void) {
    double d = 0.0;
    if (ultrasonic_mutex) {
        if (xSemaphoreTake(ultrasonic_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            d = last_distance_cm;
            xSemaphoreGive(ultrasonic_mutex);
        } else {
            // timed out taking mutex — return last known value (or 0)
            d = last_distance_cm;
        }
    } else {
        d = last_distance_cm;
    }
    return d;
}

void ultrasonic_stop_task(void) {
    if (ultrasonic_task_handle) {
        vTaskDelete(ultrasonic_task_handle);
        ultrasonic_task_handle = NULL;
    }
    if (ultrasonic_mutex) {
        vSemaphoreDelete(ultrasonic_mutex);
        ultrasonic_mutex = NULL;
    }
}
