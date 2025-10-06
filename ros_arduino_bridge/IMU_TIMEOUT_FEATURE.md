# IMU Initialization Timeout Feature

## Overview
The IMU (MPU6050) initialization has been made non-blocking with timeout protection to prevent the Arduino from hanging during startup if the IMU is not connected or fails to respond.

## Problem Solved
Previously, if the IMU was disconnected or malfunctioning, the initialization code would block indefinitely during:
- I2C communication attempts
- DMP initialization
- Calibration routines (6 iterations each for accelerometer and gyroscope)

This caused the entire robot to freeze during startup, preventing any functionality.

## Solution Implemented

### 1. Connection Test Timeout (1 second)
```cpp
unsigned long testStart = millis();
bool connectionOK = false;

while (millis() - testStart < 1000) {  // 1 second timeout
    if (mpu.testConnection()) {
        connectionOK = true;
        break;
 IMU data command (`i`) returns full IMU data (yaw pitch roll gyro_x gyro_y gyro_z accel_x accel_y accel_z)
    delay(100);
}
```

### 2. Overall Initialization Timeout (3 seconds)
```cpp
#define IMU_INIT_TIMEOUT 3000  // 3 second timeout
```
Wire.setWireTimeout(5000, true);  // 5ms I2C timeout
```
Prevents I2C operations from hanging indefinitely.

### 4. Calibration Skip on Timeout
If the initialization is taking too long, the calibration step (which can take 1-2 seconds) is skipped:
```cpp
if (millis() - startTime < IMU_INIT_TIMEOUT - 1500) {
    Serial.print(F("[IMU] Calibrating..."));
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
} else {
    Serial.println(F("[IMU] Skipping calibration due to timeout"));

## Robot Behavior

### When IMU Initializes Successfully
```
[IMU] Initializing...
[IMU] Testing connection... OK
[IMU] Initializing DMP... OK
[IMU] Calibrating... OK
[Active Offsets Printed]
[IMU] Enabling DMP... OK
[IMU] Initialization complete in XXXms
```
- IMU angle command (`a`) returns actual yaw angle
- All robot functions work normally

### When IMU Fails or Times Out
[IMU] Not detected - continuing without IMU
[WARN] IMU initialization failed or timeout - robot will operate without IMU
```

## Status Check

```

This can be used to check if the IMU is available before attempting to read angles.

## Serial Messages

All IMU status messages are prefixed with `[IMU]` for easy identification:
- `[IMU] Initializing...` - Starting initialization
- `[IMU] Testing connection... OK/FAILED` - Connection test result
- `[IMU] Initializing DMP... OK/FAILED` - DMP initialization result
- `[IMU] Calibrating... OK` - Calibration in progress
- `[IMU] Enabling DMP... OK` - DMP enabled
- `[IMU] Initialization complete in XXXms` - Success with timing
- `[IMU] Not detected - continuing without IMU` - Failed, but continuing
- `[WARN] IMU initialization failed or timeout - robot will operate without IMU` - Main setup warning

## Timeout Values

| Stage | Timeout | Reason |
|-------|---------|--------|
| Connection Test | 1 second | Quick detection of missing IMU |

### Scenario 1: IMU Connected and Working
**Expected**: Full initialization in ~1-2 seconds with calibration
- Connection test times out after 1 second
- Immediate abort with "Not detected" message
- Robot operational in ~1 second

- Robot continues without IMU
- Total time: ~1-1.5 seconds

### Scenario 4: Slow I2C Communication
**Expected**:
- I2C wire timeout catches hanging transactions
- Initialization aborts gracefully
- Robot continues without IMU

## Benefits

1. **Non-Blocking**: Robot always becomes operational, even with IMU issues
2. **Fast Fail**: Detects missing IMU in 1 second instead of indefinite hang
3. **Graceful Degradation**: Robot works without IMU, losing only yaw angle feedback
4. **Clear Feedback**: Serial messages explain exactly what happened
5. **Timing Information**: Reports how long initialization took

## Debugging

If you see repeated timeout messages:
1. **Check I2C connections**: SDA (pin 20), SCL (pin 21) on Arduino Mega
2. **Check power**: MPU6050 requires 3.3V or 5V (check your module)
3. **Check I2C pullup resistors**: Should be 4.7kΩ on SDA and SCL lines
4. **Try slower I2C**: Change `Wire.setClock(400000)` to `Wire.setClock(100000)`
5. **Check module**: Test MPU6050 with i2c_scanner sketch

## Future Enhancements

Possible improvements:
- Retry mechanism with exponential backoff
- Runtime IMU reconnection detection
- Configurable timeout values via serial command
- Status LED to indicate IMU state
- Periodic health checks during operation

## Code Location

- **Header**: `src/imu.h`
- **Implementation**: `src/imu.cpp`
- **Main Setup**: `src/ROSArduinoBridge.ino` (setup function)

## Related Features

This timeout mechanism works in conjunction with:
- State machine (STATE_OFFLOADING, STATE_MOVING, STATE_STATIONARY)
- IMU data transmission control (`shouldSendIMUData()`)
- Serial command interface (command `a` for angle)
- Movement detection and control loops
