# Quick Test Guide - Reset Issue Fix

## What Was Fixed

### 1. ✅ Removed Duplicate Code
- Fixed double "OK" responses
- Fixed duplicate motor commands
- Cleaned up MOTOR_RAW_PWM handling

### 2. ✅ Added Reset Cause Detection
- Shows WHY Arduino reset on boot
- Key reset types:
  - **Brownout** = Power supply voltage dropped too low
  - **Watchdog** = Software hung (unlikely with our code)
  - **External** = Reset button pressed
  - **Power-on** = Normal startup

### 3. ✅ Added Memory Monitor
- Shows free RAM on boot
- New command `M` to check RAM anytime
- Warns if memory getting low

### 4. ✅ Reduced Motor Power (Temporary)
- Changed `MOTOR_MAX_SPEED` from 255 → 180 (70% power)
- Reduces current draw to prevent brownout
- Can increase gradually once power supply confirmed stable

---

## Testing Steps

### Step 1: Upload and Monitor Serial Output

```bash
# Upload the fixed code
pio run --target upload

# Open serial monitor
pio device monitor --baud 57600
```

**Look for these messages on boot:**
```
=== ROSArduinoBridge Starting ===
[RESET] Brownout detected!        <-- If you see this, power issue confirmed!
[MEM] Free RAM: XXXX bytes        <-- Should be > 1000 bytes
[IMU] Initializing...
...
```

### Step 2: Check Reset Cause

**First boot after upload:**
- Should show `[RESET] External reset` or `[RESET] Power-on reset` ✅

**If robot resets during operation:**
- Look for `[RESET] Brownout detected!` 
- This CONFIRMS power supply issue
- See fix options below

### Step 3: Test Memory

Send command via serial:
```
M<CR>
```

Response:
```
Free RAM: 5234 bytes  <-- Good! (if > 1000)
Free RAM: 423 bytes   <-- WARNING! Stack overflow risk
```

**If Free RAM < 500 bytes**: Contact me for memory optimization

### Step 4: Test Teleop with Reduced Power

**Current limit: 70% power (MOTOR_MAX_SPEED = 180)**

Test teleop:
- If no resets → Power issue was the cause! ✅
- If still resets → Check reset cause message
- Robot will be slower but should be stable

### Step 5: Gradually Increase Power (Optional)

If stable at 180, try increasing:

**Edit `src/config.h`:**
```cpp
#define MOTOR_MAX_SPEED  200  // Test 78% power
// If stable, try 220, then 240, then 255
```

**Stop increasing if you see:**
- Resets with `[RESET] Brownout detected!`
- That's your power supply limit
- Stay 10-20 points below that value

---

## Interpreting Results

### Scenario A: No More Resets at 180 Power
**Diagnosis**: Power supply brownout confirmed ✅  
**Solution**: 
- Keep MOTOR_MAX_SPEED at 180 (works but slower)
- OR improve power supply (see options below)

### Scenario B: Still Resets, Shows "Brownout"
**Diagnosis**: Power issue even at reduced power  
**Solution**:
- Reduce MOTOR_MAX_SPEED to 150
- Check power supply connections
- Verify motor driver not overheating
- Consider separate power supplies

### Scenario C: Still Resets, Shows "Watchdog"
**Diagnosis**: Software hang (unexpected!)  
**Solution**:
- Check free RAM (command `M`)
- Contact me with serial output
- May need stack overflow fix

### Scenario D: No Resets, But Sluggish
**Diagnosis**: Working correctly, just limited power  
**Solution**:
- Gradually increase MOTOR_MAX_SPEED
- Find sweet spot between speed and stability

---

## Power Supply Upgrade Options

### Option 1: Add Large Capacitor (Easiest)
**Parts**: 1000µF-2200µF electrolytic capacitor, 16V+ rating

**Installation**:
1. Solder capacitor across motor power supply (+/-)
2. **Watch polarity!** Stripe = negative
3. Place close to TB6612 motor driver
4. This smooths voltage spikes

**Cost**: $2-5  
**Difficulty**: Easy (5 minutes)  
**Effectiveness**: Medium (may help 50-80%)

### Option 2: Separate Power Supplies (Best)
**Setup**:
1. Arduino powered by USB or 7-12V adapter
2. Motors powered by 7.4V LiPo battery (2S, 20C+ rating)
3. **CRITICAL: Connect grounds together!**

**Cost**: $10-30 (if need battery)  
**Difficulty**: Medium (wire management)  
**Effectiveness**: High (90%+ success rate)

### Option 3: Upgrade Motor Power Supply
**Requirements**:
- Voltage: 7.4V-12V (TB6612 limit)
- Current: 5A+ continuous (10A+ preferred)
- If using wall adapter: 12V 5A minimum

**Cost**: $15-25  
**Difficulty**: Easy  
**Effectiveness**: High

### Option 4: Add Voltage Regulator
**Parts**: 7805 or 7812 voltage regulator + heatsink

**Setup**:
1. Regulator between battery and Arduino
2. Motors connect directly to battery
3. Grounds connected

**Cost**: $5-10  
**Difficulty**: Medium (soldering)  
**Effectiveness**: High

---

## Quick Diagnostic Commands

| Command | Purpose | Expected Response |
|---------|---------|-------------------|
| `M<CR>` | Check free RAM | `Free RAM: XXXX bytes` |
| `D<CR>` | Debug state | `State:X Moving:X Stepper:X` |
| `e<CR>` | Read encoders | `1234 5678 9012 3456` |
| `m 0:0:0:0<CR>` | Stop motors | `OK` |
| `m 50:50:50:50<CR>` | Test motors (low power) | `OK` |

---

## Next Steps Based on Results

### If Brownout Confirmed:
1. ✅ Confirm by checking reset message
2. Choose power upgrade option (1, 2, 3, or 4)
3. Implement hardware fix
4. Gradually increase MOTOR_MAX_SPEED
5. Test stability at each level

### If Memory Issue Found:
1. Check free RAM with `M` command
2. If < 500 bytes, contact me
3. May need to optimize memory usage
4. Disable unused features

### If Different Reset Cause:
1. Note exact reset message
2. Check hardware connections
3. Verify motor driver not damaged
4. Test motors individually

---

## Success Criteria

✅ **System is healthy when:**
- No resets during 5+ minutes of teleop
- Free RAM stays > 1000 bytes
- Motors respond smoothly to commands
- Serial communication clean (no garbage)

⚠️ **Warning signs:**
- Frequent brownout resets
- Free RAM < 500 bytes
- Motors jerky or unresponsive
- Serial output corrupted

---

## Emergency Commands

If robot misbehaving:

```bash
# Stop all motors immediately
m 0:0:0:0

# Force state reset
F

# Check system health
M
D
```

---

## Contact Info for Issues

If you see unexpected behavior, capture:
1. Reset cause message (on boot)
2. Free RAM value (command `M`)
3. What command triggered reset
4. Power supply specs (voltage/current)

This helps diagnose the root cause!

---

*Generated: October 6, 2025*  
*Priority: HIGH - Test immediately after upload*
