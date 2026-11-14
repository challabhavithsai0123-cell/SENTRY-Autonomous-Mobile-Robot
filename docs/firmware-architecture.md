# SENTRY Firmware Architecture

This note documents the behaviour visible in the committed STM32 and ESP32-C3 source. It describes the current implementation, not an idealized future design.

## Hardware/software boundary

- STM32F407 firmware: motor control, ultrasonic ranging, MPU6500 acquisition, encoder sampling, obstacle response and UART telemetry.
- ESP32-C3 firmware: receives newline-terminated UART telemetry, joins Wi-Fi and serves a small browser dashboard.
- UART link: USART2 on the STM32 side and Serial1 on ESP32-C3, configured for 115200 baud, 8-N-1.
- Mechanical CAD assets are stored in the repository's 3D Parts directory.

## STM32 task responsibilities

The firmware creates two CMSIS-RTOS tasks in main.c.

### StartDefaultTask — encoder-based correction

The task samples TIM2 and TIM3 encoder counters, calculates count deltas, estimates per-cycle speed as delta × 10, and computes a proportional correction:

- speed_error = |encoder1_speed| − |encoder2_speed|
- correction = Kp × speed_error
- left/right PWM commands are adjusted around base_pwm and clamped to the 16-bit range.
- The task repeats after osDelay(100).

This is a proportional speed-balancing correction. It is not a complete PID controller, and the current source does not convert encoder counts into calibrated wheel speed in physical units.

### RobotTaskFunction — sensing and obstacle response

The task:

1. Reads the front HC-SR04 sensor.
2. Reads the MPU6500 when available and integrates gyro Z using the measured elapsed time to update Yaw_Angle.
3. Sends a telemetry line over USART2 approximately once per second.
4. Drives forward when the front distance exceeds 20 cm.
5. Stops and scans left/right with the SG90 when an obstacle is detected.
6. Turns toward a side with more than 20 cm clearance; if neither side is clear, it checks the rear sensor and reverses only when rear clearance exceeds 25 cm.

The obstacle response is a fixed-threshold reactive behaviour, not a global path planner.

## Sensor and actuator interfaces

| Function | Interface visible in source |
|---|---|
| Front ultrasonic | Trigger PE7, echo PE8; TIM5 used for pulse timing |
| Rear ultrasonic | Trigger PE12, echo PE13; TIM6 used for pulse timing |
| MPU6500 | I2C1; WHO_AM_I check expects 0x70 |
| Left/right wheel encoders | TIM2 and TIM3 encoder mode |
| Motor PWM | TIM4 channels 1 and 2 |
| Servo PWM | TIM1 channel 1 |
| STM32-to-ESP32 telemetry | USART2, 115200 baud |

The ultrasonic routines return -1.0 when an echo timeout occurs. The front sensor uses a 30 ms timeout while waiting for the echo transition and pulse completion.

## Telemetry format

The STM32 emits a comma-separated text record terminated with CR/LF. The fields currently formatted are:

~~~text
DIST:<cm>,ACC:<x>,<y>,<z>,GYRO:<x>,<y>,<z>,YAW:<angle>,ENC_L:<count>,ENC_R:<count>
~~~

The ESP32-C3 sketch reads Serial1 until newline, stores the latest complete line, and exposes it through the HTTP /data endpoint. The root endpoint serves the embedded dashboard page, which polls /data every 500 ms.

## Important implementation limits to address

These are engineering review items, not claims that the current robot has failed:

1. **Encoder rollover and direction:** verify counter-delta arithmetic for both rotation directions and timer wraparound. The current source calculates speed from count differences without an explicit wrap/direction correction.
2. **Speed calibration:** delta × 10 is a relative count-rate estimate, not metres per second or wheel RPM. Calibrate using encoder CPR, gearbox ratio, wheel diameter and sampling period.
3. **Yaw drift:** integrating gyro Z accumulates bias and drift. The MPU6500 calibration is a starting point; validate bias stability and consider encoder-assisted heading estimation.
4. **Blocking sensor reads:** ultrasonic echo waits and the long servo scan delays run inside the robot task. Consider bounded non-blocking acquisition or a dedicated sensor task if timing becomes unpredictable.
5. **PWM range:** verify TIM4 ARR and the requested compare values together. A compare value greater than ARR will not provide the intended duty cycle.
6. **Telemetry robustness:** consider a structured packet format with a checksum, sequence counter and explicit sensor-validity flags.
7. **Wi-Fi loss handling:** the ESP32-C3 currently waits in setup until Wi-Fi connects. It does not provide a cellular fallback or an offline telemetry buffer.

## Safety and validation checklist

Before demonstrating autonomous operation, test with wheels lifted and a current-limited supply, then use a clear test area and an accessible emergency stop. Verify motor direction, PWM limits, encoder sign, ultrasonic timeout behaviour, servo endpoints, rear-clearance logic and loss-of-telemetry behaviour. Record test conditions and measured results rather than assuming source configuration proves physical performance.

## Source of truth

- STM32 application: Core/Src/main.c
- ESP32-C3 gateway: ESP32C3_Telemetr.ino
- Peripheral initialization: Core/Src/main.c and generated Core/Src peripheral files
