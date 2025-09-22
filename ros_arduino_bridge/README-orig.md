Code Base Overview
==================

This code base is modularized for easy integration and troubleshooting of all robot subsystems. Each hardware component (motors, encoders, servos, stepper, ultrasonic sensors, color sensor, IMU) is managed by its own `.h` and `.cpp` files in the `src/` directory. The main logic is in `ROSArduinoBridge.ino`.

Directory Structure
-------------------
- `src/ROSArduinoBridge.ino` — Main Arduino sketch
- `src/config.h` — All pin definitions and hardware constants
- `src/commands.h` — Single-letter serial command definitions
- `src/servos.h` / `src/servos.ino` — Servo control (camera and tipper)
- `src/stepper_driver.h` / `src/stepper_driver.cpp` — Stepper motor control
- `src/sensors.h` / `src/sensors.cpp` — Ultrasonic and color sensor code
- `src/imu.h` / `src/imu.cpp` — MPU6050 IMU (Z angle)
- `src/README-orig.md` — This documentation

Pinout Summary
--------------

**Motors (TB6612):**
- M1 (Front Left): PWM 7, IN1 22, IN2 23
- M2 (Front Right): PWM 5, IN1 24, IN2 25
- M3 (Rear Left): PWM 6, IN1 26, IN2 27
- M4 (Rear Right): PWM 9, IN1 29, IN2 28
- Standby: 40

**Encoders:**
- ENC1_A: 2, ENC1_B: 30
- ENC2_A: 18, ENC2_B: 31
- ENC3_A: 19, ENC3_B: 32
- ENC4_A: 3, ENC4_B: 31

**Ultrasonic Sensors:**
- Left: TRIG A6, ECHO A4
- Right: TRIG A7, ECHO A5

**Color Sensor (TCS34725):**
- INT: A0
- LED: A3
- I2C: SDA 20, SCL 21

**Servos:**
- Camera: 52
- Tipper: 44

**Stepper Motor (28BYJ-48):**
- IN1: 45, IN2: 47, IN3: 49, IN4: 51

**IMU (MPU6050):**
- I2C: SDA 20, SCL 21

Command Formats
---------------

All commands are single-letter, space-separated, and sent over serial (115200 baud).  
**Examples:**

| Command | Format/Example         | Description                                 |
|---------|-----------------------|---------------------------------------------|
| a       | a 3                   | Analog read pin 3                           |
| b       | b                     | Get baudrate                                |
| c       | c 3 1                 | Set pin 3 to OUTPUT                         |
| d       | d 13                  | Digital read pin 13                         |
| e       | e                     | Read all encoders                           |
| m       | m 100:100:100:100     | Set all 4 motor speeds                      |
| o       | o 255:255255:255     | Set raw PWM for all motors                  |
| p       | p                     | Ping (ultrasonic)                           |
| q       | q 20:300:0            | Stepper: rpm:distance_mm:flag               |
| q       | q 0:0:1               | Stepper: return to zero at 20 RPM           |
| U       | U                     | Ultrasonic: returns "<left_cm> <right_cm>"  |
| v       | v                     | Color sensor: returns "<R> <G> <B>"         |
| s       | s 0 90                | Set servo 0 to 90°                          |
| f       | f 0 90 10             | Set servo 0 to 90° at 10 ms/deg             |
| t       | t 0                   | Read servo 0 angle                          |
| w       | w 13 1                | Digital write pin 13 HIGH                   |
| x       | x 3 128               | Analog write pin 3, value 128               |
| z       | z                     | Get IMU Z (yaw) angle in degrees            |
| r       | r                     | Reset encoders                              |
| u       | u ...                 | Update PID params (see code)                |

**Note:**  
- Servo indices: 0 = camera, 1 = tipper
- Stepper flag: 1 = return to zero, 0 = move

Troubleshooting
---------------

1. **No Response on Serial**
   - Check baudrate (57600).
   - Ensure correct COM port.
   - Use "Both NL & CR" or "Carriage return" in Serial Monitor.

2. **Motors/Servos/Stepper Not Moving**
   - Double-check wiring and power supply.
   - Confirm pin numbers in `config.h` match your hardware.
   - Test with standalone sketches for each subsystem (provided in `/test` or as examples).

3. **Sensors Not Reporting**
   - For I2C devices (IMU, color sensor), check SDA/SCL wiring and address.
   - For ultrasonic, check trigger/echo pin wiring and sensor orientation.

4. **Stepper Motor Issues**
   - Lower speed/acceleration in code if motor stalls.
   - Use `q 0:0:1` to return to home and verify movement.

5. **IMU Angle Not Updating**
   - Ensure DMP is initialized (see serial output).
   - Use `z` command only after DMP is ready.

6. **General Debugging**
   - Use `Serial.println()` statements in code to trace execution.
   - Test each subsystem with its standalone code before integrating.

7. **Pin Conflicts**
   - Make sure no two devices share the same pin in `config.h`.

For further help, see the comments in each `.h`/`.cpp` file and the troubleshooting notes above.

---