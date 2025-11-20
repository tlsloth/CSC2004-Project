include(FreeRTOS_Kernel_import.cmake)
add_executable(car_project
    main.c
    drivers/barcode.c
    drivers/encoder.c
    drivers/motor.c
    drivers/pid.c
    drivers/ir_sensor.c
    drivers/imu.c
    straight_mode.c
)

add_executable(car_project_line_following
    main_line_following.c
    drivers/barcode.c
    drivers/encoder.c
    drivers/motor.c
    drivers/pid.c
    drivers/ir_sensor.c
    drivers/imu.c
    straight_mode.c
)

target_include_directories(car_project PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(car_project 
    pico_stdlib 
    hardware_adc 
    hardware_gpio
    hardware_pwm
    hardware_i2c
    FreeRTOS-Kernel-Heap4
    )
pico_add_extra_outputs(car_project)
pico_enable_stdio_uart(car_project 1)
pico_enable_stdio_usb(car_project 1)

# -- Add missing setup for the line_following target so pico headers are found --
target_include_directories(car_project_line_following PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(car_project_line_following
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_pwm
    hardware_i2c
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
    FreeRTOS-Kernel-Heap4
)

pico_add_extra_outputs(car_project_line_following)
pico_enable_stdio_uart(car_project_line_following 1)
pico_enable_stdio_usb(car_project_line_following 1)

