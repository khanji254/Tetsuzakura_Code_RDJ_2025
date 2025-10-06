# PID Tuning Summary for ROS Arduino Bridge

## Date
January 2025

## Hardware Configuration
- **Microcontroller**: Arduino Mega 2560
- **Motor Driver**: TB6612 Dual H-Bridge (4 motors)
- **Drive Type**: Mecanum Drive
- **Encoders**: Quadrature, 285 ticks/revolution per motor
- **Communication**: Serial @ 57600 baud

## Tuning Process

### Initial State
- Original PID values:
  - Kp = 20
  - Kd = 12
  - Ki = 0
  - Ko = 50

### Problems Encountered
1. **Wild Oscillations**: Initial auto-tuning with standard Ziegler-Nichols gains caused extreme speed swings (60 → 240 → -200 ticks/frame)
2. **Integral Windup**: Integral term accumulated without bounds, causing overshoot
3. **Fast Motor Response**: Motors were too responsive for aggressive PID gains
4. **Jitter**: Small errors caused constant motor adjustments

### Solutions Implemented

#### 1. Conservative PID Gains
Reduced gains significantly from standard Ziegler-Nichols:
- **Kp = 0.3 × Ku** (instead of 0.6 × Ku)
- **Ki = 0.1 × Kp** (very conservative integral)
- **Kd = 0.05 × Kp** (minimal derivative)

#### 2. Anti-Windup Protection
```cpp
#define MAX_INTEGRAL 5000  // Limit integral accumulation
```
- Only accumulate integral when output is not saturated
- Clamp integral term to ±MAX_INTEGRAL

#### 3. Dead Zone
```cpp
#define MIN_ERROR_THRESHOLD 2  // Dead zone in ticks
```
- Ignore errors smaller than 2 ticks to prevent jitter
- Reduces constant micro-adjustments when near target

#### 4. Output Limiting
- Already present in code: MOTOR_MAX_SPEED (typically 255)
- Prevents output saturation

## Final Tuned Values

### Integer PID Constants (used in diff_controller.h)
```cpp
int Kp = 30;   // Proportional gain (scaled by Ko=100)
int Kd = 1;    // Derivative gain (scaled by Ko=100)
int Ki = 3;    // Integral gain (scaled by Ko=100)
int Ko = 100;  // Scaling factor for integer math
```

### Actual Gains (after scaling)
- Kp_actual = 30/100 = 0.30
- Ki_actual = 3/100 = 0.03
- Kd_actual = 1/100 = 0.01

## Performance Validation

### Test Conditions
- Target: 60 ticks/frame step input
- PID Rate: 50 Hz (20ms interval)
- Duration: 2 seconds

### Expected Performance
- **Overshoot**: < 20% (target < 72 ticks/frame)
- **Steady-State Error**: < 5 ticks/frame
- **Settling Time**: < 1 second (50 PID cycles)
- **Stability**: No wild oscillations

### Grading Criteria
- **Excellent**: Overshoot < 10%, SS Error < 3
- **Good**: Overshoot < 20%, SS Error < 5
- **Acceptable**: Overshoot < 30%, SS Error < 10
- **Poor**: Overshoot > 30% or SS Error > 10

## Code Changes Made

### 1. diff_controller.h
**Updated PID constants:**
- Changed from Kp=20, Kd=12, Ki=0, Ko=50
- To Kp=30, Kd=1, Ki=3, Ko=100

**Added anti-windup:**
- MAX_INTEGRAL = 5000
- MIN_ERROR_THRESHOLD = 2
- Conditional integral accumulation (only when not saturated)
- Integral clamping

**Enhanced PID computation:**
- Dead zone for small errors
- Anti-windup protection
- Improved comments

### 2. ROSArduinoBridge.ino (Previous Changes)
**State Machine Updates:**
- Removed auto-stop timeout (per user request)
- IMU/odometry/sensors active in MOVING and STATIONARY states
- Only disabled during OFFLOADING

**PID Rate:**
- Increased from 30Hz to 50Hz
- PID_INTERVAL = 20ms

**IMU Direction:**
- Modified to return negative yaw (left turn = negative)

## Serial Command Interface

### Verified Working Commands
1. **Motor Speeds**: `m speed1:speed2:speed3:speed4` or `m speed1 speed2 speed3 speed4`
2. **Update PID**: `u Kp:Kd:Ki:Ko` (e.g., `u 30:1:3:100`)
3. **Reset Encoders**: `r`
4. **Read Encoders**: `e`
5. **IMU Data**: `i` (returns yaw pitch roll gx gy gz ax ay az)
6. **Stepper**: `q rpm:distance:flag`
7. **Sensors**: `c`, `s`, `p`

### Command Parsing Features
- Supports both colon-separated and space-separated values
- Handles partial commands gracefully
- Reads remaining characters from serial buffer for long commands
- Resets command buffer after execution

## Recommendations

### For Testing
1. Start with low speeds to verify PID stability
2. Test each motor individually before full mecanum control
3. Monitor encoder feedback vs target speeds
4. Watch for oscillations or overshoot

### For Tuning Adjustments
If system still oscillates:
- Reduce Kp further (try 20 or 25)
- Keep Ki very low (< 5)
- Kd can help dampen but keep minimal

If system is too sluggish:
- Increase Kp slightly (try 35 or 40)
- Keep Ki proportional to Kp (Ki ≈ 0.1 × Kp)
- Don't exceed Kp = 50 without retesting

### For Different Hardware
If encoders have different resolution:
- Scale PID gains proportionally
- Higher resolution → higher gains needed
- Lower resolution → lower gains needed

## Testing Procedure

### Quick Validation Test
1. Upload code to Arduino
2. Open Serial Monitor @ 57600 baud
3. Send: `r` (reset encoders)
4. Send: `m 60:60:60:60` (set all motors to 60 ticks/frame)
5. Send: `e` (read encoders repeatedly)
6. Verify encoders approach ~60 ticks/frame without wild swings
7. Send: `m 0:0:0:0` (stop motors)

### Full System Test
1. Test individual motor control
2. Test mecanum drive movements (forward, strafe, rotate)
3. Monitor IMU feedback during rotation
4. Verify sensors work in all states
5. Test stepper for ball offloading
6. Confirm state transitions work correctly

## Notes
- These values are conservative for safety
- Fine-tuning may be needed for your specific robot mass/friction
- Always test in a safe environment with ability to emergency stop
- Monitor motor temperature during extended operation

## References
- Original ROS Arduino Bridge: http://vanadium-ros-pkg.googlecode.com
- Ziegler-Nichols tuning method (modified for fast motors)
- PID anti-windup techniques: Brett Beauregard's blog
- State machine design for robot control
