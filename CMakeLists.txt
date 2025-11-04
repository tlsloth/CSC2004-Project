include(FREERTOS_Kernel_import.cmake)

add_executable(demo3
        main.c
        # Populate with path to your c files e.g.
        # drivers/ir_sensor/decoding/barcode_decoder.c
        # drivers/ir_sensor/line_detection/line_detection.c
        # drivers/imu/simulated_imu.c

        drivers/motor.c
        drivers/encoder.c
        drivers/pid.c
        drivers/state.c
        drivers/ir_sensor.c
        drivers/ultrasonic.c
)
target_include_directories(demo3 PRIVATE
${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(demo3
        pico_stdlib
        pico_rand
        hardware_gpio
        hardware_pwm
        hardware_i2c
        hardware_adc
        pico_lwip_freertos
        pico_lwip_mqtt
        pico_cyw43_arch_lwip_sys_freertos
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
    )

pico_enable_stdio_usb(demo3 1)
pico_enable_stdio_uart(demo3 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(demo3)

# add url via pico_set_program_url
example_auto_set_url(demo3)



