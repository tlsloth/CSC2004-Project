    # ================================
    #  Robotic Car - Main Project
    # ================================
    cmake_minimum_required(VERSION 3.13)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    # Wi-Fi credentials - adjust these as necessary
    set(WIFI_SSID "Muhammad Ridwan")
    set(WIFI_PASSWORD "RIDwan3107")
    set(MQTT_SERVER_IP "192.168.43.1")  # Android hotspot default


    # FreeRTOS kernel path - adjust this path as necessary
    # set(FREERTOS_KERNEL_PATH "C:/FreeRTOS-Kernel-main")


    # --- Wi-Fi + MQTT (FreeRTOS variant) ---
    # set(PICO_LWIP_FREERTOS 1)
    # set(PICO_LWIP_MQTT 1)
    # set(PICO_CYW43_ARCH_DEFAULT lwip_sys_freertos)

    # -----------------------------------------------
    # SDK Initialization
    # -----------------------------------------------


    set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")

    set(sdkVersion 2.2.0)
    set(toolchainVersion 14_2_Rel1)
    set(picotoolVersion 2.2.0)
    set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
    if(EXISTS ${picoVscode})
        include(${picoVscode})
    endif()

    set(PICO_BOARD pico_w CACHE STRING "Board type")

    include(C:/Program\ Files/Raspberry\ Pi/Pico\ SDK\ v1.5.1/pico-sdk/external/pico_sdk_import.cmake)

    # -----------------------------------------------
    # FreeRTOS Import
    # -----------------------------------------------
    set(FREE_RTOS_KERNEL_SMP 0 CACHE BOOL "Disable SMP mode" FORCE)
    if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
        message(FATAL_ERROR "FREERTOS_KERNEL_PATH not defined. Example needs FreeRTOS-Kernel cloned.")
    endif()
    include(C:/FreeRTOS-Kernel-main/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

    # -----------------------------------------------
    # Project Definition
    # -----------------------------------------------
    project(Line-Following-Robo2 C CXX ASM)
    pico_sdk_init()

    # -----------------------------------------------
    # Source Files
    # -----------------------------------------------
    add_executable(Line-Following-Robo2
        src/main.c
        # Populate with path to your c files e.g.
        # drivers/ir_sensor/decoding/barcode_decoder.c
        # drivers/ir_sensor/line_detection/line_detection.c
        # drivers/imu/simulated_imu.c

        drivers/motor.c
        drivers/encoder.c
        drivers/pid.c
        drivers/imu.c
    )

    # -----------------------------------------------
    # Include Directories
    # -----------------------------------------------
    target_include_directories(Line-Following-Robo2 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/drivers

        ${FREERTOS_KERNEL_PATH}/include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include

        # ✅ Core SDK Wi-Fi & networking paths
        ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/include
        ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include
        ${PICO_SDK_PATH}/lib/cyw43-driver/include
        ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
        ${PICO_SDK_PATH}/lib/lwip/src/include
        # ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include/arch 
        ${PICO_SDK_PATH}/lib/cyw43-driver/src
        ${PICO_SDK_PATH}/src/rp2_common/pico_async_context/include
    )

    # -----------------------------------------------
    # Compiler Definitions (Wi-Fi / MQTT)
    # -----------------------------------------------
    target_compile_definitions(Line-Following-Robo2 PRIVATE
        WIFI_SSID="${WIFI_SSID}"
        WIFI_PASS="${WIFI_PASSWORD}"
        MQTT_SERVER_IP="${MQTT_SERVER_IP}"
        CYW43_LWIP=1
    )

    # -----------------------------------------------
    # Link Libraries
    # -----------------------------------------------
    target_link_libraries(Line-Following-Robo2
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

    # -----------------------------------------------
    # I/O Configuration
    # -----------------------------------------------
    pico_enable_stdio_usb(Line-Following-Robo2 1)
    pico_enable_stdio_uart(Line-Following-Robo2 1)

    # -----------------------------------------------
    # Output Formats
    # -----------------------------------------------
    pico_add_extra_outputs(Line-Following-Robo2)