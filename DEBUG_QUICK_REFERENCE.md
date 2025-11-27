# 🔧 ALFR Quick Debug Reference Card

## 🚨 Emergency Commands
```
S  →  STOP motors immediately
G  →  GO (resume)
C  →  Calibrate sensors
```

---

## 📊 Reading Debug Output

### Example Debug Line:
```
Sensors: [□|□|■|■|■|□|□|□] Pos:2500 Err:-1000 Corr:25 Motors L:145 R:95 Time:12s
```

| Symbol | Meaning |
|--------|---------|
| `■` | Sensor ON black line |
| `□` | Sensor ON white surface |
| `Pos` | Line position (0=far left, 7000=far right, 3500=center) |
| `Err` | Error value (negative=line left, positive=line right) |
| `Corr` | Correction being applied |
| `L` | Left motor speed |
| `R` | Right motor speed |

---

## 🎯 What Should You See?

### Robot on Line (Centered):
```
Sensors: [□|□|□|■|■|□|□|□]   ← Middle sensors detect line
Pos: 3000-4000                ← Near center
Err: -500 to +500             ← Small error
Motors L:120 R:120            ← Similar speeds
```

### Robot Turning Left (line is on left):
```
Sensors: [■|■|□|□|□|□|□|□]   ← Left sensors detect line
Pos: 0-1500                   ← Position far left
Err: -2000 to -3500           ← Large negative error
Motors L:80 R:160             ← Left slower, right faster
```

### Robot Turning Right (line is on right):
```
Sensors: [□|□|□|□|□|■|■|□]   ← Right sensors detect line
Pos: 5500-7000                ← Position far right
Err: +2000 to +3500           ← Large positive error
Motors L:160 R:80             ← Left faster, right slower
```

### Robot on THICK Line:
```
Sensors: [□|□|■|■|■|■|■|□]   ← Many sensors detect black
Pos: 4000-5000                ← Follows right edge
Err: +500 to +1500            ← Slight right bias (normal)
Motors L:130 R:110            ← Tracking edge
```

### Line LOST:
```
Sensors: [□|□|□|□|□|□|□|□]   ← All white!
Pos: 0 or 7000                ← Goes to last known position
Err: ±3500                    ← Maximum error
Motors: Searching pattern     ← Trying to find line
```

---

## ⚠️ Problem Patterns & Fixes

### Pattern 1: Constant Wobbling
```
Debug shows:
Err: -800 → +900 → -700 → +850 → ...  (rapidly changing)
Motors: L:90 R:150 → L:150 R:90 → ... (rapidly changing)
```
**FIX:**
- Reduce `Kp` by 5-10
- Increase `Kd` by 5
- Try: Kp=15, Kd=20

---

### Pattern 2: Slow Response to Turns
```
Debug shows:
Line moves left but robot doesn't turn quickly
Sensors: [■|■|□|□|□|□|□|□]
Err: -2500
Motors: L:110 R:130  ← Not enough difference!
```
**FIX:**
- Increase `Kp` by 5-10
- Try: Kp=30-35

---

### Pattern 3: Gradual Drift
```
Debug shows:
On straight, robot slowly drifts left/right
Err: -100 → -200 → -300 → -400 (slowly increasing)
```
**FIX:**
- Add small `Ki` value (0.1-0.5)
- Or recalibrate sensors

---

### Pattern 4: Overshooting Turns
```
Debug shows:
Robot turns too far past line, then corrects back
Sensors: □□■■□□□□ → □□□□□□■■ → □□■■□□□□
```
**FIX:**
- Reduce `TURN_SPEED` in code
- Increase `Kd` for damping
- Lower `BASE_SPEED`

---

### Pattern 5: All Sensors Show White (Lost Line)
```
Sensors: [□|□|□|□|□|□|□|□]
Pos: 0 or 7000
Err: ±3500
```
**CAUSES:**
1. Sensors too high above ground
2. Poor calibration
3. Robot too fast
4. Line break/gap in track

**FIX:**
- Send `S` to stop
- Send `V` to check raw values
- Recalibrate with `C`
- Lower sensor array height
- Reduce speed

---

### Pattern 6: All Sensors Show Black
```
Sensors: [■|■|■|■|■|■|■|■]
```
**CAUSES:**
1. Sensors too low (touching surface)
2. Very thick line (normal for your track!)
3. Incorrect calibration

**FIX:**
- If on thick line: This is NORMAL
- Code should use edge-following mode
- Verify `followMode = FOLLOW_MODE_EDGE` in code
- Recalibrate

---

## 🔍 Sensor Value Check

### Send `V` command to see raw values:

**Good calibration example:**
```
S0: 850 (WHITE)   ← High value on white
S1: 820 (WHITE)
S2: 150 (BLACK)   ← Low value on black
S3: 120 (BLACK)
S4: 140 (BLACK)
S5: 800 (WHITE)
S6: 830 (WHITE)
S7: 810 (WHITE)
```

**Bad calibration example:**
```
S0: 500 (WHITE)   ← Should be higher!
S1: 520 (WHITE)
S2: 480 (BLACK)   ← Should be lower!
S3: 490 (BLACK)   ← Not enough difference
...
```
**FIX:** Recalibrate! Difference should be >300 between black and white.

---

## 🎚️ PID Values Quick Reference

### Conservative (Slow & Stable):
```cpp
Kp = 15.0
Ki = 0.0
Kd = 20.0
BASE_SPEED = 100
```
**Use for:** Testing, complex tracks, thick lines

### Moderate (Balanced):
```cpp
Kp = 25.0
Ki = 0.0
Kd = 15.0
BASE_SPEED = 120
```
**Use for:** Normal operation, competition

### Aggressive (Fast & Responsive):
```cpp
Kp = 35.0
Ki = 0.1
Kd = 10.0
BASE_SPEED = 150
```
**Use for:** Simple tracks, speed runs
**Warning:** May oscillate on thick lines!

---

## 📝 Before Each Run Checklist

```
□ Battery fully charged (>11V)
□ All wires secure (wiggle test)
□ Sensors clean (no dust/dirt)
□ Sensor height correct (3-8mm)
□ Upload latest code
□ Open Serial Monitor (115200 baud)
□ Calibrate on actual track
□ Test run and check debug output
□ Note successful PID values
```

---

## 🧪 Testing Procedure

### Step 1: Static Test
1. Place robot on line (not moving)
2. Check debug output
3. Should see sensors detecting line
4. Position should be reasonable (1000-6000)

### Step 2: Manual Movement Test
1. Keep USB connected
2. Slide robot left and right
3. Watch debug output change
4. Verify sensors respond correctly

### Step 3: Slow Speed Test
1. Set `BASE_SPEED = 80`
2. Upload and run
3. Should follow line smoothly (even if slow)
4. If fails at slow speed, check calibration

### Step 4: Increase Speed Gradually
1. Increase `BASE_SPEED` by 10 each test
2. Test: 80 → 100 → 120 → 140 → 160
3. Find maximum stable speed
4. Use that speed -20 for competition

---

## 💡 Pro Tips

### Tip 1: Use Serial Plotter
- Tools → Serial Plotter
- Shows error, correction as graphs
- Easier to see oscillation patterns

### Tip 2: Mark Good Values
- When you find good PID values
- Write them on tape and stick on robot
- Take photo for backup

### Tip 3: Test Sections Individually
- Test straight section first
- Then gentle curves
- Then sharp turns
- Tune for each section

### Tip 4: Emergency Stop is Your Friend
- Press `S` anytime to stop
- Analyze debug output
- Adjust code
- Press `G` to continue testing

### Tip 5: Lighting Matters
- Calibrate in competition lighting
- Recalibrate if moved to different room
- Indoor vs outdoor makes huge difference

---

## 🔧 Common Code Changes

### Change 1: Swap Motor Direction
```cpp
// In setMotorSpeed() function, swap these lines:
// For left motor:
digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);  // Change LOW to HIGH
digitalWrite(MOTOR_LEFT_ENABLE, LOW);     // Change HIGH to LOW
```

### Change 2: Adjust Speeds in Code
```cpp
// At top of file (around line 35-40):
#define BASE_SPEED 120   ← Change this value
#define MAX_SPEED  200   ← Change this value
#define TURN_SPEED 100   ← Change this value
```

### Change 3: Change PID Values
```cpp
// Around line 43-45:
float Kp = 25.0;  ← Change this
float Ki = 0.0;   ← Change this
float Kd = 15.0;  ← Change this
```

### Change 4: Switch Follow Mode
```cpp
// Line 59:
int followMode = FOLLOW_MODE_EDGE;  // For THICK lines
// OR
int followMode = FOLLOW_MODE_CENTER; // For THIN lines
```

---

## 📞 Quick Fault Diagnosis

| Symptom | Most Likely Cause | Quick Fix |
|---------|------------------|-----------|
| Motors don't move | Power/wiring | Check battery, L298N connections |
| Random jerky motion | Loose wire | Secure all connections, check GND |
| Can't find line | Bad calibration | Recalibrate (`C` command) |
| Wobbles constantly | Kp too high | Reduce Kp, increase Kd |
| Slow to turn | Kp too low | Increase Kp |
| Overshoots turns | Speed too high | Reduce TURN_SPEED |
| Drifts on straights | Misalignment or Ki needed | Add Ki=0.1-0.5 |
| All sensors white | Lost line | Slow down, check sensor height |
| All sensors black | On thick line OR too low | Normal for thick line if edge following |
| Works sometimes | Inconsistent calibration | Recalibrate before each run |

---

## 📐 Sensor Array Positioning

### Correct Height:
```
        [Sensor Array]
           |||||||
         3-8mm gap  ← Optimal
      ═════════════
      Black Line
```

### Too High:
```
        [Sensor Array]
           |||||||
         >10mm gap  ← Too far!
      ═════════════
      Can't detect line
```

### Too Low:
```
    [Sensor Array]
       |||||||
      ~ ~ ~ ~ ~  ← Touching!
      ═════════════
      False readings
```

---

## 🎯 Success Criteria

Your ALFR is properly tuned when:
- ✅ Follows straight lines without wobbling
- ✅ Smoothly navigates gentle curves
- ✅ Can handle sharp 90° turns
- ✅ Recovers if briefly losing line
- ✅ Debug shows stable error values
- ✅ Motor speeds adjust appropriately
- ✅ Completes full track consistently

---

**Print this guide and keep it with your robot during testing!** 📋

