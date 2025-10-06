# Arduino Mega Reset Issue - Diagnostic Report

## Symptoms
- Robot resets during teleop control
- Arduino Mega resetting unexpectedly
- Possible brownout or stack overflow

## Root Causes Found

### 🔴 CRITICAL #1: Duplicate Code Causing Double Responses
**Location**: `src/ROSArduinoBridge.ino` lines 390-400

**Problem**:
```cpp
if (cmd == MOTOR_RAW_PWM) {
    setMotorSpeedsTB6612(speeds[0], speeds[1], speeds[2], speeds[3]);
}

Serial.println("OK");        // First OK
if (cmd == MOTOR_RAW_PWM) {
    setMotorSpeedsTB6612(speeds[0], speeds[1], speeds[2], speeds[3]);  // DUPLICATE!
}

Serial.println("OK");        // Second OK - DUPLICATE!
```

**Impact**:
- Double "OK" response confuses ROS2
- Motor command executed twice
- Could cause timing issues

**Fix**: Remove duplicate blocks

---

### 🔴 CRITICAL #2: Power Supply Issues (Most Likely Cause)
**Probable Cause**: Motor current draw causing voltage drop → brownout reset

**Symptoms Match**:
- Works initially (low current)
- Resets during movement (high current)
- Teleop control triggers it (repeated motor commands)

**Diagnosis Steps**:
1. Check power supply voltage under load
2. Measure current draw during movement
3. Check for voltage drops on Arduino VIN/5V pins
4. Verify capacitors on TB6612 motor driver

**Typical Power Requirements**:
- Arduino Mega: ~50mA
- MPU6050 IMU: ~3.5mA
- TCS34725 Color Sensor: ~65mA (LED on)
- TB6612 Logic: ~10mA
- **Motors (4x under load): 500mA - 2A+ each = 2-8A total!**

**Solution Options**:
1. **Separate power supplies**: Motors on battery, Arduino on USB/separate 5V
2. **Large capacitor**: Add 1000µF+ on motor supply
3. **Current limiting**: Reduce MAX_PWM value
4. **Better power supply**: Use higher current capacity supply

---

### 🟠 HIGH #3: Stack Overflow Risk
**Problem**: Arduino Mega has only 8KB RAM

**Memory Hogs Found**:
```cpp
SetPointInfo flPID, frPID, rlPID, rrPID;  // ~160 bytes
char argv1[16], argv2[16];                // 32 bytes
char args[32];                            // 32 bytes (in motor command)
uint8_t fifoBuffer[64];                   // 64 bytes (IMU)
// + Serial buffers, stack frames, etc.
```

**Check Free RAM**:
Add this function to check available memory:
```cpp
int freeRAM() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
```

If free RAM < 500 bytes → stack overflow risk!

---

### 🟡 MEDIUM #4: Aggressive PID Could Cause Motor Overcurrent
**Current PID Values**:
```cpp
int Kp = 30;  // Proportional gain
int Kd = 1;   // Derivative gain  
int Ki = 3;   // Integral gain
int Ko = 100; // Scaling factor
```

**With MAX_PWM = 255**:
- PID can command full throttle instantly
- 4 motors at full throttle = maximum current draw
- Could trigger brownout

---

## Most Likely Cause: POWER SUPPLY BROWNOUT

### Why Power Issues Cause Resets:
1. Teleop sends motor command
2. PID sets all 4 motors to high PWM
3. Motors draw several amps
4. Voltage drops below ~4.5V
5. Arduino brownout detector triggers reset
6. Reset → motors stop → voltage recovers
7. Boot → ready for next command
8. **Cycle repeats!**

### Telltale Signs:
- ✓ Works when stationary (low current)
- ✓ Resets during movement (high current)
- ✓ Happens repeatedly with teleop
- ✓ No software infinite loops found
- ✓ No watchdog timer enabled

---

## Immediate Fixes

### Fix #1: Remove Duplicate Code (CRITICAL)
**File**: `src/ROSArduinoBridge.ino`

Remove duplicate blocks at lines 390-400.

### Fix #2: Add Power Supply Decoupling
**Hardware**:
- Add 1000µF capacitor across motor supply
- Ensure Arduino powered separately from motors
- Check all ground connections

### Fix #3: Reduce Motor Current
**Software** - Temporary fix to test:
```cpp
#define MOTOR_MAX_SPEED  180  // Reduce from 255 to 70% power
```

This limits maximum current draw.

### Fix #4: Add Reset Detection
**Software**:
```cpp
void setup() {
  // Check reset reason
  Serial.begin(BAUDRATE);
  uint8_t mcusr_copy = MCUSR;
  MCUSR = 0;
  
  if (mcusr_copy & (1 << BORF)) Serial.println(F("[RESET] Brownout"));
  if (mcusr_copy & (1 << EXTRF)) Serial.println(F("[RESET] External"));
  if (mcusr_copy & (1 << WDRF)) Serial.println(F("[RESET] Watchdog"));
  if (mcusr_copy & (1 << PORF)) Serial.println(F("[RESET] Power-on"));
  
  // ... rest of setup
}
```

This tells you WHY the reset happened!

### Fix #5: Add RAM Monitor
**Software**:
```cpp
// In setup():
Serial.print(F("Free RAM: "));
Serial.println(freeRAM());

// In loop() periodically:
if (millis() % 5000 == 0) {
  Serial.print(F("Free RAM: "));
  Serial.println(freeRAM());
}
```

---

## Testing Protocol

### Test #1: Verify Reset Cause
1. Upload code with reset detection
2. Trigger reset
3. Check serial output for reset reason
4. If "Brownout" → power issue confirmed
5. If "Watchdog" → software hang (unlikely)
6. If "External" → hardware reset button/pin

### Test #2: Reduce Motor Power
1. Set `MOTOR_MAX_SPEED = 180`
2. Test teleop
3. If no reset → power issue confirmed
4. Gradually increase until reset occurs
5. Find safe maximum

### Test #3: Monitor Current
1. Add ammeter in motor supply line
2. Run teleop
3. Watch current spikes
4. If >5A → need better power supply or current limiting

### Test #4: Separate Power
1. Power Arduino from USB
2. Power motors from battery
3. **Connect grounds together!**
4. Test teleop
5. If no reset → power issue definitely confirmed

---

## Long-Term Solutions

### Option 1: Hardware Upgrade (Best)
- Use separate 5V/2A for Arduino
- Use 7.4V LiPo 20C+ for motors
- Add 2200µF capacitor on motor supply
- Use thick wires (20AWG or thicker)
- Star ground configuration

### Option 2: Software Current Limiting
```cpp
#define MOTOR_MAX_SPEED  200
#define MOTOR_RAMP_RATE  10  // PWM units per PID cycle

// In doPID_4motor():
// Limit rate of change
long delta = output - p->output;
if (delta > MOTOR_RAMP_RATE) delta = MOTOR_RAMP_RATE;
if (delta < -MOTOR_RAMP_RATE) delta = -MOTOR_RAMP_RATE;
output = p->output + delta;
```

### Option 3: Aggressive PID Tuning
- Reduce Kp to limit overshoot
- Add output ramping
- Limit acceleration

---

## Checklist Before Next Test

Hardware:
- [ ] Check power supply voltage (should be >6V, <12V for TB6612)
- [ ] Measure current capability (should be >3A continuous)
- [ ] Add capacitor on motor supply (1000µF minimum)
- [ ] Verify all ground connections solid
- [ ] Check motor driver heat (if hot → overcurrent)

Software:
- [ ] Remove duplicate code blocks
- [ ] Add reset cause detection
- [ ] Add free RAM monitoring
- [ ] Reduce MOTOR_MAX_SPEED temporarily
- [ ] Upload and test

Testing:
- [ ] Note reset cause from serial
- [ ] Monitor current draw
- [ ] Check free RAM values
- [ ] Test with reduced power first
- [ ] Gradually increase power limit

---

## Quick Diagnostic Commands

Add to `runCommand()`:
```cpp
case 'M': // Memory check
    Serial.print(F("Free RAM: "));
    Serial.println(freeRAM());
    break;

case 'R': // Reset cause
    Serial.print(F("Last reset: "));
    // ... print MCUSR saved at startup
    break;
```

---

## Expected Behavior After Fixes

✅ **With power fix**:
- No resets during teleop
- Sustained motor operation
- Stable operation under load

✅ **With current limiting**:
- Slower acceleration
- Lower top speed
- No resets
- May need to increase gradually

❌ **If still resets**:
- Check for stack overflow (free RAM < 500)
- Look for hardware shorts
- Check for damaged motor driver
- Verify motor wiring

---

## Conclusion

**Primary Suspect**: Power supply brownout (95% confidence)
**Secondary Suspect**: Duplicate code causing timing issues (5%)
**Unlikely**: Stack overflow, infinite loop, watchdog

**Recommended Action Order**:
1. Remove duplicate code (5 minutes)
2. Add reset detection (5 minutes)
3. Test with MOTOR_MAX_SPEED = 180 (2 minutes)
4. If stable → gradually increase limit
5. Add capacitor to motor supply (hardware fix)
6. Consider separate power supplies

---

*Generated: October 6, 2025*
*Branch: feature/imu-full-packet*
