# Serial Command Reference - ROS Arduino Bridge

## Connection Settings
- **Baud Rate**: 57600
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Line Ending**: Carriage Return (CR / `\r` / 13)

## Command Format
```
<command> <arg1> <arg2>
```
Commands are single characters. Arguments are separated by spaces or colons depending on the command.

---

## Motor Control Commands

### Set Motor Speeds (PID Controlled)
**Command**: `m <speed1>:<speed2>:<speed3>:<speed4>`

**Example**: `m 60:60:60:60`

**Description**: Sets target speed for each motor in ticks per PID frame (50Hz = 20ms)
- **speed1**: Front-left motor
- **speed2**: Front-right motor  
- **speed3**: Rear-left motor
- **speed4**: Rear-right motor

**Range**: Typically ±200 ticks/frame (depends on MOTOR_MAX_SPEED)

**Alternative**: `m 60 60 60 60` (space-separated also works)

---

### Set Motor Raw PWM (No PID)
**Command**: `o <pwm1>:<pwm2>:<pwm3>:<pwm4>`

**Example**: `o 150:150:150:150`

**Description**: Directly sets PWM values, bypassing PID control
- Resets PID controller
- Sets `moving = 0`

**Range**: -255 to +255

---

### Stop All Motors
**Example**: `m 0:0:0:0`

---

## Encoder Commands

### Read All Encoders
**Command**: `e`

**Response**: `enc1 enc2 enc3 enc4` (space-separated encoder counts)

**Example Response**: `1250 1300 1245 1298`

**Note**: Only sends data when NOT in OFFLOADING state

---

### Reset All Encoders
**Command**: `r`

**Response**: `OK`

**Description**: 
- Resets all encoder counts to zero
- Resets PID controller
- Resets movement detection variables

---

## IMU Commands

### Get Full IMU Data
**Command**: `i`

**Response**: `yaw pitch roll gyro_x gyro_y gyro_z accel_x accel_y accel_z` (space-separated)

All values are floats. Yaw/pitch/roll are in degrees (yaw is inverted so left is negative). Gyroscope and accelerometer are returned as raw integer-converted floats from the MPU6050 registers.

**Example Response**: `-12.34 1.23 0.45 123 -45 8 16384 0 -512`

**Note**: 
- Yaw (CCW left) = negative
- Pitch and Roll follow standard DMP conventions
- Only sends data when NOT in OFFLOADING state
- Returns a line of zeros if IMU failed to initialize or timed out during setup
- IMU initialization has 3-second timeout to prevent blocking if not connected

---

## PID Tuning Commands

### Update PID Constants
**Command**: `u <Kp>:<Kd>:<Ki>:<Ko>`

**Example**: `u 30:1:3:100`

**Response**: `OK`

**Description**: Updates PID controller gains
- **Kp**: Proportional gain (scaled by Ko)
- **Kd**: Derivative gain (scaled by Ko)
- **Ki**: Integral gain (scaled by Ko)
- **Ko**: Scaling factor (typically 100)

**Current Tuned Values**: `u 30:1:3:100`

---

## Sensor Commands

### Read Color Sensor
**Command**: `c [0|1]`

**Examples**:
- `c` - Read with current LED state
- `c 0` - Turn LED off and read
- `c 1` - Turn LED on and read

**Response**: `R:<red> G:<green> B:<blue> C:<clear>`

**Example Response**: `R:145 G:210 B:98 C:453`

**Note**: Only sends data when NOT in OFFLOADING state

---

### Read Ultrasonic Sensors
**Command**: `s`

**Response**: `<left_cm> <right_cm>`

**Example Response**: `25 30` (left sensor: 25cm, right sensor: 30cm)

**Note**: Only sends data when NOT in OFFLOADING state

---

### Read Single Ultrasonic Sensor (Ping)
**Command**: `p <trig_pin> <echo_pin>`

**Example**: `p 2 3`

**Response**: `<distance_cm>`

---

## Stepper Motor (Ball Offloader) Commands

### Move Stepper
**Command**: `q <rpm>:<distance>:<flag>`

**Example**: `q -25:400:0`

**Parameters**:
- **rpm**: Motor speed (negative for reverse)
- **distance**: Distance in motor steps
- **flag**: Reserved for future use (0 = normal operation)

**Response**: `OK`

**Description**: Controls AccelStepper for ball offloading mechanism

---

## State Machine Commands

### Get Current Robot State
**Command**: `t`

**Response**: `<state>` (0, 1, or 2)

**States**:
- `0` = STATE_OFFLOADING (sensors/IMU/encoders disabled)
- `1` = STATE_MOVING (all sensors active)
- `2` = STATE_STATIONARY (all sensors active)

---

### Debug State Info
**Command**: `D`

**Response**: `State:<s> Moving:<m> Stepper:<st>`

**Example Response**: `State:1 Moving:1 Stepper:0`

---

## Servo Commands (if USE_SERVOS enabled)

### Set Servo Position
**Command**: `w <index> <angle>`

**Example**: `w 0 90`

**Response**: `OK`

---

### Read Servo Position
**Command**: `v <index>`

**Example**: `v 0`

**Response**: `<angle>`

---

## General Commands

### Get Baud Rate
**Command**: `b`

**Response**: `57600`

---

### Digital Read
**Command**: `d <pin>`

**Example**: `d 13`

**Response**: `0` or `1`

---

### Digital Write
**Command**: `w <pin> <value>`

**Example**: `w 13 1`

**Response**: `OK`

---

### Analog Read
**Command**: `a <pin>`

**Example**: `a 0`

**Response**: `<value>` (0-1023)

---

### Analog Write (PWM)
**Command**: `x <pin> <value>`

**Example**: `x 9 128`

**Response**: `OK`

---

### Set Pin Mode
**Command**: `c <pin> <mode>`

**Example**: `c 13 1`

**Modes**:
- `0` = INPUT
- `1` = OUTPUT

**Response**: `OK`

---

## State Machine Behavior

### STATE_OFFLOADING (State 0)
- **Triggered by**: Stepper motor activity
- **Duration**: Until 1000ms after stepper stops
- **Disabled Features**:
  - Encoder reading (`e` returns nothing)
  - IMU reading (`a` returns nothing)  
  - Sensor reading (`c`, `s` return nothing)
- **Enabled Features**:
  - Motor control still works
  - PID control still active

### STATE_MOVING (State 1)
- **Triggered by**: Any motor command with non-zero speed
- **Transitions to STATIONARY**: After 500ms of no encoder movement
- **All Features Enabled**

### STATE_STATIONARY (State 2)
- **Triggered by**: 500ms with no encoder movement
- **All Features Enabled**
- **PID Reset**: Integrators cleared to prevent windup

---

## Quick Testing Sequence

```
r                  # Reset encoders
m 60:60:60:60      # Move forward at 60 ticks/frame
e                  # Check encoder values (should show ~60 ticks)
a                  # Check IMU angle (should be near 0 if moving straight)
m 0:0:0:0          # Stop motors
c 1                # Read color sensor with LED on
s                  # Read ultrasonic sensors
t                  # Check robot state
```

---

## Mecanum Drive Examples

### Forward
```
m 60:60:60:60
```

### Backward
```
m -60:-60:-60:-60
```

### Strafe Right
```
m 60:-60:-60:60
```

### Strafe Left
```
m -60:60:60:-60
```

### Rotate Clockwise (CW)
```
m 60:-60:60:-60
```

### Rotate Counter-Clockwise (CCW)
```
m -60:60:-60:60
```

### Stop
```
m 0:0:0:0
```

---

## Troubleshooting

### Motors not moving
- Check `t` command to see current state
- Verify encoder connections (should show counts)
- Try raw PWM: `o 100:100:100:100`

### Wild oscillations
- Reduce Kp: `u 20:1:2:100`
- Check encoder wiring (reversed phases cause instability)

### Drift during straight movement
- Check wheel alignment
- Verify all encoders counting correctly
- Individual motor PID may need trimming

### No sensor data
- Check state with `t` command
- Sensors disabled during OFFLOADING (state 0)
- Wait for stepper to finish

---

## Notes
- Commands are case-sensitive
- Line ending must be Carriage Return (CR)
- Timeout removed - motors run until explicitly stopped
- PID updates at 50Hz (every 20ms)
- State machine auto-manages sensor availability
