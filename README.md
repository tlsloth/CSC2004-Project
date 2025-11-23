# CSC2004-Project
# 🤖 Intelligent Autonomous Line-Following Robot Project

## Project Description

[cite_start]This project develops an **intelligent autonomous robot** capable of navigating a track and performing advanced tasks under real-time constraints[cite: 4, 9]. [cite_start]The core functionality is **line following** using a PID controller[cite: 12], augmented by several key features:
1.  [cite_start]**Encoded Navigation:** Interpreting barcodes to execute complex, IMU-assisted turns (e.g., Left/Right)[cite: 13, 284].
2.  [cite_start]**Obstacle Avoidance:** Using an ultrasonic sensor mounted on a servo to detect, measure, and detour around obstacles[cite: 14, 307].
3.  [cite_start]**Real-time Telemetry:** Communicating live data (speed, heading, status) to a remote dashboard via **MQTT over Wi-Fi**[cite: 15, 291, 294].

---

## 📁 Project Structure Summary

The project utilizes a modular structure, separating hardware drivers, control logic, and application-level state machines.

| File/Folder | Purpose |
| :--- | :--- |
| **main.c** | Initializes all hardware components, configures the IMU's cardinal bearings, creates and starts the FreeRTOS tasks (Control, Line Following, Barcode Timeout), and starts the scheduler. |
| **control_task.c** | Contains the top-level **`control_task`** (main execution loop). Manages high-priority state transitions, specifically handling **Obstacle Detection & Avoidance** (`STATE_SCANNING` and `STATE_CORRECTING` transitions), suspending other tasks during maneuvers. |
| **line_following_task.c** | Contains the **`line_following_task`** (lower priority). Implements the core **PID line-following loop**, managing the robot's steering (`ROBOT_STATE_STRAIGHT`/`ROBOT_STATE_TURNING`) and reacting to barcode detection flags set by the driver. |
| **drivers/** | Contains all low-level hardware abstraction layers (HALs). |
| `drivers/imu.c` | [cite_start]Handles communication with the MPU-9250 sensor[cite: 184, 185], provides **raw and filtered** acceleration, gyroscope, and heading data, and includes the critical `imu_get_corrected_yaw()` function. |
| `drivers/motor.c` | [cite_start]Controls the DC motors via the Motor Controller [cite: 178, 218] using PWM outputs for speed and direction. |
| `drivers/encoder.c` | [cite_start]Processes pulses from the IR-based Wheel Encoders [cite: 163, 240] [cite_start]to calculate real-time **speed, RPM, and distance travelled**[cite: 245]. |
| `drivers/ir_sensor.c` | [cite_start]Manages the IR-based 'line' detector [cite: 192] [cite_start]for line-following, providing digital/analog line position feedback, and the dedicated sensor for **barcode decoding**[cite: 260, 267]. |
| `drivers/pid.c` | [cite_start]Implements the **PID algorithm** (Proportional-Integral-Derivative) used for both line-following steering and IMU-assisted straight-line heading maintenance[cite: 217, 220]. |
| `drivers/ultrasonic.c` | [cite_start]Interfaces with the HC-SR04 Ultrasonic Sensor [cite: 196, 199] for distance measurement. |
| `drivers/servo.c` | [cite_start]Controls the TowerPro Servo [cite: 211, 212] [cite_start]used to sweep the ultrasonic sensor for **obstacle width measurement**[cite: 309, 312]. |
| **FreeRTOS/** | FreeRTOS Kernel source and configuration files. |
| **lwip/** | [cite_start]Lightweight IP stack for Wi-Fi and MQTT network communication[cite: 292, 293]. |

---

## ⚙️ Compilation, Running, and Testing

### Prerequisites
* [cite_start]**Microcontroller:** Raspberry Pi Pico W[cite: 109, 292].
* **OS:** Windows, macOS, or Linux with Pico SDK installed.
* **Toolchain:** ARM GCC compiler.
* **Debugging:** Serial terminal (e.g., PuTTY, CoolTerm) for `printf` output.
* [cite_start]**Networking:** An MQTT broker accessible by the Pico W's Wi-Fi network (e.g., Mosquitto, cloud broker) for telemetry[cite: 296].

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
| [cite_start]**Week 6: Individual Driver** [cite: 343] | [cite_start]Motor Encoder (Speed/Distance), IR Sensor (Line/Thickness), Ultrasonic + Servo (Basic distance/sweep), IMU (Raw data)[cite: 344, 346, 349, 352]. | [cite_start]Verified sensor readings and motor control (no PID required yet)[cite: 345, 347, 348]. |
| [cite_start]**Week 10: Partial Integration** [cite: 354] | **PID Control, IMU Filtering & Heading, Line Following, Barcode Command, Obstacle Avoidance.** | [cite_start]Robot maintains straight path with IMU correction[cite: 376]. [cite_start]Correctly follows line, interprets barcodes, executes accurate turns[cite: 380]. [cite_start]Stops at obstacle, scans, navigates around it, and rejoins the line[cite: 384]. |
| [cite_start]**Week 13: Final Demo Run** [cite: 387] | Full autonomous operation of all integrated features. | [cite_start]Successful completion of the final track, demonstrating reliable barcode navigation, obstacle detour, and seamless line re-acquisition[cite: 388]. |
