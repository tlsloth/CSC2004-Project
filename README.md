# CSC2004-Project
# 🤖 Intelligent Autonomous Line-Following Robot Project

## Project Description

This project develops an **intelligent autonomous robot** capable of navigating a track and performing advanced tasks under real-time constraints. The core functionality is **line following** using an IR sensor and assisted with IMU, augmented by several key features:
1.  **Encoded Navigation:** Interpreting barcodes to execute complex, IMU-assisted turns (e.g., Left/Right).
2.  **Obstacle Avoidance:** Using an ultrasonic sensor mounted on a servo to detect, measure, and detour around obstacles.
3.  **Real-time Telemetry:** Communicating live data (speed, heading, status) to a remote dashboard via **MQTT over Wi-Fi**.

---

## 📁 Project Structure Summary

The project utilises a modular structure, separating hardware drivers, control logic, and application-level state machines.

| File/Folder | Purpose |
| :--- | :--- |
| **main_line_following.c** | Initialises all hardware components, configures the IMU's cardinal bearings, creates and starts the FreeRTOS tasks (Control, Line Following, Barcode Reading, Obstacle sensing, Obstacle Avoidance), and starts the scheduler. Contains the **`line_following_task`** (lower priority). Implements the core **PID line-following loop**, managing the robot's steering (`STATE_LINE_FOLLOWING`/`STATE_STRAIGHT_FOR_SCAN`/`STATE_STRAIGHT_FOR_SCAN`) and reacting to barcode detection flags set by the driver. |
| **drivers/** | Contains all low-level hardware abstraction layers (HALs). |
| `drivers/imu.c` | Handles communication with the MPU-9250 sensor, provides **raw and filtered** acceleration, gyroscope, and heading data, and includes the critical `imu_get_corrected_yaw()` function. |
| `drivers/motor.c` | Controls the DC motors via the Motor Controller using PWM outputs for speed and direction. |
| `drivers/encoder.c` | Processes pulses from the IR-based Wheel Encoders to calculate real-time **speed, RPM, and distance travelled**. |
| `drivers/ir_sensor.c` | Manages the IR-based 'line' detector for line-following, providing digital/analog line position feedback, and the dedicated sensor for **barcode decoding**. |
| `drivers/pid.c` | Implements the **PID algorithm** (Proportional-Integral-Derivative) used for both line-following steering and IMU-assisted straight-line heading maintenance. |
| `drivers/ultrasonic.c` | Interfaces with the HC-SR04 Ultrasonic Sensor for distance measurement. |
| `drivers/servo.c` | Controls the Servo used to sweep the ultrasonic sensor for **obstacle width measurement**. |
| `drivers/barcode.c` | Controls the logic for reading and decoding barcodes placed on the track |

| **FreeRTOS/** | FreeRTOS Kernel source and configuration files. |
| **lwip/** | Lightweight IP stack for Wi-Fi and MQTT network communication. |

---

## ⚙️ Compilation, Running, and Testing

### Prerequisites
* **Microcontroller:** Raspberry Pi Pico W.
* **OS:** Windows, macOS, or Linux with Pico SDK installed.
* **Toolchain:** ARM GCC compiler.
* **Debugging:** Serial terminal (e.g., PuTTY, CoolTerm) for `printf` output.
* **Networking:** An MQTT broker accessible by the Pico W's Wi-Fi network (e.g., Mosquitto, cloud broker) for telemetry.

### 1. Compile Instructions

1.  **Navigate to the project directory:**
    ```bash
    cd /path/to/your/project
    ```
2.  **Create and enter the build directory:**
    ```bash
    mkdir build
    cd build
    ```
3.  **Run CMake to configure the build:**
    ```bash
    cmake ..
    ```
4.  **Build the project:**
    ```bash
    make
    ```
    This will generate the required `.uf2` file (e.g., `main.uf2`).

### 2. Running Instructions

1.  **Put the Pico W into Bootloader Mode:** Hold the **BOOTSEL** button while connecting the Pico W to your PC via USB.
2.  **Drag and Drop:** Copy the generated `main.uf2` file into the mounted RPI-RP2 drive.
3.  **Reboot:** The Pico W will reboot, load the firmware, and begin executing `main()`.

### 3. Testing and Demo Summary

The project is validated through a series of progressive demos:

| Demo Phase | Key Features to Test | Success Criteria |
| :--- | :--- | :--- |
| **Week 6: Individual Driver** | Motor Encoder (Speed/Distance), IR Sensor (Line/Thickness), Ultrasonic Distance Measurement, IMU (Raw data). | Verified sensor readings and motor control (no PID required yet). |
| **Week 10: Partial Integration** | **PID Control, IMU Filtering & Heading, Line Following, Barcode Command, Obstacle Width Measurement.** | Robot maintains straight path with IMU correction. Correctly follows the line, interprets barcodes, and executes accurate turns. Stops at the obstacle, scans, and measures the width. |
| **Week 13: Final Demo Run** | Full autonomous operation of all integrated features. | Successful completion of the final track, demonstrating reliable barcode navigation, obstacle detour, and seamless line re-acquisition. |
