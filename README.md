# SENTRY – STM32-Based Intelligent Autonomous Mobile Robot

**An embedded autonomous mobile robot featuring multi-sensor obstacle detection, real-time control, voice interaction, and wireless telemetry.**

## Project Overview

SENTRY is a physically developed autonomous mobile robot built around the STM32F407VGT6 microcontroller. It combines embedded firmware, motor control, sensor integration, and wireless communication to perform autonomous movement and obstacle avoidance.

The robot uses ultrasonic sensors to detect nearby obstacles, a servo mechanism to scan the front surroundings, and an MPU6500 inertial measurement unit to obtain motion-related data. An ESP32-C3 provides wireless telemetry, allowing sensor information from the STM32 controller to be viewed through a web dashboard.

The project demonstrates the integration of embedded systems, robotics, real-time operating systems, and wireless communication.

## Key Features

- Autonomous mobile robot movement.
- Front and rear ultrasonic obstacle detection.
- Servo-assisted left and right environmental scanning.
- Differential-drive motor control.
- STM32F407-based embedded firmware.
- FreeRTOS-based firmware architecture.
- MPU6500 IMU integration.
- Quadrature encoder interfaces for wheel feedback.
- Wireless telemetry using ESP32-C3.
- Browser-based live telemetry dashboard.
- Voice interaction using the AI Thinker VC-02 module.

## Hardware Components

| Component | Purpose |
|---|---|
| STM32F407VGT6 | Main embedded controller |
| L298N | Dual DC motor driver |
| N25 geared DC motors | Robot movement |
| Built-in quadrature encoders | Wheel feedback |
| HC-SR04 ultrasonic sensors | Obstacle detection |
| SG90 servo motor | Front sensor scanning |
| MPU6500 | Accelerometer and gyroscope data |
| ESP32-C3 SuperMini | Wireless telemetry gateway |
| AI Thinker VC-02 | Voice interaction |
| 7.4 V battery | Robot power source |

## System Architecture

The robot uses a distributed embedded architecture.

1. The STM32F407 acts as the primary controller.
2. Ultrasonic sensors measure obstacle distances.
3. The servo rotates the front ultrasonic sensor to scan different directions.
4. The motor driver controls the left and right DC motors.
5. Encoder interfaces provide wheel movement feedback.
6. The MPU6500 provides accelerometer and gyroscope measurements.
7. The STM32 sends telemetry through USART2.
8. The ESP32-C3 receives the UART data and connects to a Wi-Fi network.
9. A browser dashboard displays the latest received telemetry.

### Wireless Communication

**STM32F407 → USART2 UART → ESP32-C3 → Wi-Fi → Web Dashboard**

The telemetry connection uses 115200 baud, 8-N-1 serial communication.

## Software and Development Tools

- STM32CubeIDE
- STM32CubeMX
- STM32 HAL Library
- FreeRTOS
- Embedded C
- Arduino IDE
- ESP32 Arduino Core
- HTML, CSS and JavaScript
- Git and GitHub

## Repository Structure

```text
SENTRY-Autonomous-Mobile-Robot/
│
├── Core/
│   ├── Inc/
│   └── Src/
│
├── Drivers/
├── Middlewares/
├── USB_HOST/
│
│
│── ESP32C3_Telemetry.ino
│
├── .settings/
├── Robot_Project.ioc
├── .project
├── .cproject
├── STM32F407VGTX_FLASH.ld
└── STM32F407VGTX_RAM.ld
```

## Telemetry Data

The STM32 transmits sensor and encoder information to the ESP32-C3.

Example telemetry format:

```text
DIST:55.57,ACC:-0.09,-0.02,0.97,
GYRO:0.10,0.07,0.01,YAW:0.45,
ENC_L:3829,ENC_R:62293
```

The ESP32-C3 receives these records and makes the latest telemetry available through its web server. The example below illustrates the reported fields; actual values depend on the running robot.

## Current Implementation

| Feature | Status |
|---|---|
| STM32F407 firmware | Implemented |
| FreeRTOS integration | Implemented |
| Ultrasonic sensing | Implemented |
| Servo-based scanning | Implemented |
| Motor control | Implemented |
| MPU6500 integration | Implemented |
| Encoder interfaces | Implemented |
| ESP32-C3 wireless telemetry | Tested |
| Browser telemetry dashboard | Implemented |
| VC-02 voice interaction | Integrated |
| ESP32-CAM live video | Planned |

## Future Improvements

- Structured telemetry visualization with individual sensor cards.
- Live graphs for IMU and encoder measurements.
- ESP32-CAM video streaming integration.
- Improved motion estimation and encoder-based velocity measurement.
- Enhanced obstacle avoidance and navigation algorithms.
- Improved robot performance monitoring.

## Applications

The technologies demonstrated in SENTRY are relevant to:

- Autonomous mobile robotics.
- Embedded control systems.
- Indoor service robots.
- Sensor-based navigation.
- Industrial automation.
- Wireless robotic monitoring.

## Author

**C. Bhavith Sai**

B.Tech – Mechatronics Engineering

GitHub: [challabhavithsai0123-cell](https://github.com/challabhavithsai0123-cell)

---

*This repository documents the physical SENTRY robot and its embedded hardware implementation. It is separate from the author's ROS 2 and Gazebo simulation projects.*
