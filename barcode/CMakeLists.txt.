include(FreeRTOS_Kernel_import.cmake)
add_executable(car_project
    main.c
    barcode.c
)

target_include_directories(car_project PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(car_project 
    pico_stdlib 
    hardware_adc 
    hardware_gpio
    FreeRTOS-Kernel-Heap4
    )
pico_add_extra_outputs(car_project)
pico_enable_stdio_uart(car_project 1)
pico_enable_stdio_usb(car_project 1)

