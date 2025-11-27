# ALFR Thick Line Controller - Setup Guide

## LAM Research Challenge 2025 - Hardware Hustle

---

## 📋 Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [Wiring Connections](#wiring-connections)
3. [Software Setup](#software-setup)
4. [Calibration Process](#calibration-process)
5. [Tuning PID Parameters](#tuning-pid-parameters)
6. [Troubleshooting](#troubleshooting)
7. [Serial Commands Reference](#serial-commands-reference)

---

## 🔧 Hardware Requirements

Based on your LAM Research Challenge kit:

- ✅ Arduino Nano R3 (CH340 chip)
- ✅ Smart Elex RLS-08 Analog Line Sensor Array (8 sensors)
- ✅ L298N Motor Driver Module (5V-35V 2A)
- ✅ 2x N20 Gear Motors (600 RPM, 12V)
- ✅ 3 or 4 Battery Holder
- ✅ Jumper wires
- ✅ Robot chassis (3D printed ALFR)

---

## 🔌 Wiring Connections

### 1. RLS-08 Sensor Array to Arduino Nano

The RLS-08 has 8 analog outputs. Connect them as follows:

```
RLS-08 Pin  →  Arduino Nano Pin
─────────────────────────────────
Sensor 1    →  A0  (Leftmost)
Sensor 2    →  A1
Sensor 3    →  A2
Sensor 4    →  A3
Sensor 5    →  A4
Sensor 6    →  A5
Sensor 7    →  A6
Sensor 8    →  A7  (Rightmost)
VCC         →  5V
GND         →  GND
```

**IMPORTANT:** Verify your sensor array's pinout! Some RLS-08 boards have sensors numbered differently.

### 2. L298N Motor Driver to Arduino Nano

```
L298N Pin   →  Arduino Nano Pin
─────────────────────────────────
ENA (PWM)   →  D5  (Left motor speed)
IN1         →  D6  (Left motor direction 1)
IN2         →  D7  (Left motor direction 2)
ENB (PWM)   →  D9  (Right motor speed)
IN3         →  D10 (Right motor direction 1)
IN4         →  D11 (Right motor direction 2)

12V Input   →  Battery +ve (11.1V or 12V)
GND         →  Battery -ve AND Arduino GND
5V Output   →  Arduino Nano VIN (if powering Arduino from L298N)
```

**POWER NOTES:**
- Remove the L298N's 5V regulator jumper if your motors draw high current
- Use a separate 5V buck converter to power Arduino if needed
- Ensure common ground between Arduino and L298N

### 3. Motor Connections

```
L298N Motor Terminals:
OUT1 & OUT2  →  Left Motor
OUT3 & OUT4  →  Right Motor
```

**If motors run in wrong direction:**
- Simply swap the two wires of that motor
- OR modify the code (easier to swap wires)

### 4. Optional Calibration Button

```
Push Button  →  D2 (with internal pull-up)
Other pin    →  GND
```

### 5. Status LED (Built-in)

```
Arduino Nano built-in LED on Pin 13 is used
No external wiring needed
```

---

## 💻 Software Setup

### Step 1: Install Arduino IDE
1. Download from: https://www.arduino.cc/en/software
2. Install for your operating system

### Step 2: Setup Arduino Nano Board
1. Open Arduino IDE
2. Go to **Tools** → **Board** → **Arduino Nano**
3. Go to **Tools** → **Processor** → **ATmega328P (Old Bootloader)**
   - If upload fails, try **ATmega328P** (without "Old Bootloader")
4. Select correct **Port** (COM port on Windows, /dev/ttyUSB0 on Linux)

### Step 3: Upload the Code
1. Open `ALFR_ThickLine_Controller.ino`
2. Click **Verify** (✓) button to compile
3. Click **Upload** (→) button to upload to Arduino
4. Wait for "Done uploading" message

### Step 4: Open Serial Monitor
1. Click **Tools** → **Serial Monitor**
2. Set baud rate to **115200** (bottom right)
3. You should see the startup message

---

## 🎯 Calibration Process

**Calibration is CRITICAL for thick line following!**

### Why Calibrate?
- Different surfaces reflect different amounts of light
- Thick black lines may have varying darkness
- Calibration helps Arduino distinguish black from white

### How to Calibrate:

1. **Place robot partially on the line**
   - Position robot so some sensors are on black, some on white
   - For thick lines: place robot ON the black line

2. **Trigger calibration**
   - **Method 1:** Press the calibration button (if wired to D2)
   - **Method 2:** Send `C` via Serial Monitor

3. **Move the robot for 5 seconds**
   - Slide robot left and right across the line
   - Make sure EACH sensor sees both black and white
   - For thick lines: move robot so sensors see:
     - Full black (on thick line)
     - White (off line)
     - Edge transition

4. **Calibration complete**
   - LED will blink 5 times rapidly
   - Serial Monitor shows calibration results
   - Robot will start in 3 seconds

### Calibration Tips:
- ✅ DO move slowly and smoothly
- ✅ DO ensure all sensors see both black and white
- ✅ DO calibrate in the same lighting as competition
- ❌ DON'T move too fast (sensors need time to read)
- ❌ DON'T skip any sensor
- ❌ DON'T calibrate in different lighting conditions

---

## ⚙️ Tuning PID Parameters

PID tuning is an **iterative process**. Start with default values and adjust based on robot behavior.

### Default Values (in code):
```cpp
float Kp = 25.0;  // Proportional
float Ki = 0.0;   // Integral
float Kd = 15.0;  // Derivative
```

### Tuning Table:

| Problem | Solution |
|---------|----------|
| **Robot oscillates/wobbles too much** | • Decrease `Kp` by 5-10<br>• Increase `Kd` by 5<br>• Try: Kp=15, Kd=20 |
| **Robot responds slowly to turns** | • Increase `Kp` by 5-10<br>• Try: Kp=30-35 |
| **Robot gradually drifts off line** | • Increase `Ki` slightly (0.1-0.5)<br>• Warning: Too much Ki causes oscillation |
| **Robot overshoots turns** | • Decrease `TURN_SPEED`<br>• Increase `Kd` |
| **Robot is too slow** | • Increase `BASE_SPEED`<br>• Increase `MAX_SPEED` |
| **Robot is too fast (unstable)** | • Decrease `BASE_SPEED`<br>• Decrease `MAX_SPEED` |

### Step-by-Step Tuning Process:

#### Step 1: Tune Kp (Proportional)
1. Set Ki=0, Kd=0
2. Increase Kp until robot oscillates
3. Reduce Kp to 50-70% of oscillation point
4. Example: If oscillates at Kp=40, set Kp=25

#### Step 2: Tune Kd (Derivative)
1. Keep Kp from Step 1
2. Increase Kd to reduce oscillations
3. Too much Kd makes robot sluggish
4. Sweet spot: smooth curves without wobble

#### Step 3: Tune Ki (Integral) - Optional
1. Only if robot drifts on long straights
2. Start with very small value (Ki=0.1)
3. Increase slowly if drift persists
4. **Most LFRs work fine with Ki=0**

#### Step 4: Tune Speeds
1. Adjust `BASE_SPEED` for normal operation
2. `MAX_SPEED` for straight sections
3. `TURN_SPEED` for sharp curves
4. `MIN_SPEED` to prevent motor stall

### Thick Line Specific Tuning:
- Lower Kp values work better (15-20 range)
- Edge following requires less aggressive corrections
- Slower speeds improve stability on thick lines

---

## 🐛 Troubleshooting

### Problem 1: Robot doesn't move
**Possible causes:**
- ✓ Check battery voltage (should be 11-12V)
- ✓ Verify motor driver connections
- ✓ Test motors directly with L298N
- ✓ Check if code uploaded successfully
- ✓ Look for error messages in Serial Monitor

### Problem 2: Motors run in wrong direction
**Solution:**
- Swap motor wire connections on L298N
- OR change motor pins in code

### Problem 3: Sensors not detecting line
**Check:**
- ✓ Sensor height above ground (optimal: 3-8mm)
- ✓ Calibration was done correctly
- ✓ Sensor wiring to correct Arduino pins
- ✓ Send `V` command to view raw sensor values
- ✓ Black line should give LOW values (~0-200)
- ✓ White surface should give HIGH values (~700-1023)

### Problem 4: Robot loses line on turns
**Solutions:**
- Reduce speed (`BASE_SPEED`, `TURN_SPEED`)
- Increase Kp for faster response
- Ensure all 8 sensors are working
- Check sensor array is centered on robot

### Problem 5: Robot works on thin lines but not thick
**Solutions:**
- Verify `followMode = FOLLOW_MODE_EDGE` (line 59)
- Recalibrate with robot ON the thick line
- Reduce Kp (try 15-20)
- Position sensor array to track one edge of thick line

### Problem 6: Erratic behavior
**Check:**
- ✓ Loose connections (especially GND)
- ✓ Battery voltage (low battery causes issues)
- ✓ EMI from motors (add capacitors to motors)
- ✓ Sensor array ribbon cable not loose
- ✓ Code uploaded completely without errors

### Problem 7: Won't upload to Arduino
**Solutions:**
- Select correct board: **Tools** → **Board** → **Arduino Nano**
- Try **ATmega328P (Old Bootloader)** for CH340 chip
- Check USB cable (must be data cable, not charge-only)
- Install CH340 drivers if needed
- Try different USB port
- Press reset button on Nano just before upload

---

## 📡 Serial Commands Reference

Open Serial Monitor (115200 baud) and send these commands:

| Command | Function |
|---------|----------|
| `C` or `c` | Start calibration routine |
| `S` or `s` | **STOP** motors (emergency stop) |
| `G` or `g` | **GO** - Resume from stop |
| `V` or `v` | **View** raw sensor values |
| `P` or `p` | **Print** current PID values |
| `+` | Increase base speed |
| `-` | Decrease base speed |
| `H` or `h` | Show help menu |

### Using Serial Monitor for Debugging:

The code automatically prints debug info every 200ms:

```
Sensors: [□|□|■|■|■|□|□|□] Pos:2500 Err:-1000 Corr:25 Motors L:145 R:95 Time:12s
```

**Legend:**
- `■` = Sensor detects BLACK
- `□` = Sensor detects WHITE
- `Pos` = Calculated line position (0-7000, center=3500)
- `Err` = Error from center (negative=left, positive=right)
- `Corr` = PID correction value
- `Motors L/R` = Left and right motor speeds
- `Time` = Time since start

---

## 🚀 Quick Start Checklist

1. ✅ Wire all connections as per diagram
2. ✅ Upload code to Arduino Nano
3. ✅ Open Serial Monitor (115200 baud)
4. ✅ Place robot on track (partially on line)
5. ✅ Send `C` to calibrate (or press button)
6. ✅ Move robot across line for 5 seconds
7. ✅ Wait for 3-second countdown
8. ✅ Robot starts following line!
9. ✅ Observe Serial Monitor for debugging
10. ✅ Tune PID if needed (see tuning section)

---

## 🎓 Understanding Thick Line Following

### Why Thick Lines Are Different:

**Normal LFR (thin line):**
```
  White  [S1][S2][S3][S4][S5][S6]  White
            ↑  Black Line ↑
```
Robot straddles the line, sensors detect edges.

**Thick Line LFR:**
```
       [S1][S2][S3][S4][S5][S6]
     Black  Black  Black  Black
       Thick Line (wider than robot)
```
Multiple sensors are on black simultaneously!

### Our Solution: EDGE FOLLOWING

Instead of following the CENTER, we follow one EDGE of the thick line:

```
[S1][S2][S3][S4][S5][S6][S7][S8]
 ↓   ↓   ↓   ↓   ↓   ←───────── Track this edge!
Black Black Black Black White
```

The code detects when multiple sensors see black and switches to edge-following mode automatically.

---

## 📞 Support & Tips

### Competition Day Tips:
1. **Arrive early** to calibrate in competition lighting
2. **Test on actual arena** before competition run
3. **Bring backup batteries** (fully charged)
4. **Have USB cable ready** for last-minute tuning
5. **Note your best PID values** on paper
6. **Check all connections** before each run

### Common Mistakes to Avoid:
- ❌ Not calibrating before each run
- ❌ Calibrating in wrong lighting conditions
- ❌ Sensors too high/low above ground
- ❌ Weak battery (always use fresh/charged)
- ❌ Loose wires (use hot glue or tape)
- ❌ Skipping Serial Monitor debugging

### Pro Tips:
- 💡 Mark your best PID values with tape on robot
- 💡 Keep a notebook of successful configurations
- 💡 Test in same lighting as competition
- 💡 Lower speeds are more reliable than faster speeds
- 💡 Watch the Serial Monitor to understand robot behavior
- 💡 Calibrate multiple times to find best position

---

## 📈 Expected Performance

With proper tuning, your ALFR should:
- ✅ Follow thick black lines smoothly
- ✅ Navigate sharp curves without losing line
- ✅ Recover if briefly losing line
- ✅ Complete complex tracks with multiple turns
- ✅ Work reliably in varying lighting conditions

**Typical speeds:**
- Straight sections: 150-200 (out of 255)
- Curved sections: 100-120
- Sharp turns: 80-100

**Competition time:**
- Target: Under 10 minutes
- Good: 7-8 minutes
- Excellent: 5-6 minutes

---

## 🔄 Version History

**v1.0** - Initial release
- Edge following for thick lines
- Auto-calibration
- PID control
- Serial debugging
- Runtime commands

---

## 📄 License

Created for LAM Research Challenge 2025
Free to use and modify for educational purposes

---

**Good luck with your competition! 🏆**

For questions or issues, check Serial Monitor debug output and refer to troubleshooting section.
