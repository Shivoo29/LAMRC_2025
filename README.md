# LAM Research Challenge 2025 - ALFR Code

## Advanced Line Follower Robot (Thick Line Edition)

![Competition](https://img.shields.io/badge/Competition-LAM_Research_2025-blue)
![Status](https://img.shields.io/badge/Status-Ready_for_Testing-green)
![Arduino](https://img.shields.io/badge/Arduino-Nano_R3-00979D)

---

## 🎯 Project Overview

This repository contains **production-ready Arduino code** for an Advanced Line Follower Robot (ALFR) specifically optimized for **THICK BLACK LINE** tracks, as used in the LAM Research Challenge 2025 - Hardware Hustle competition.

### Key Features

- ✅ **Thick Line Optimized** - Edge-following algorithm for wide black lines
- ✅ **Robust PID Control** - Smooth, stable line following
- ✅ **Auto-Calibration** - Easy sensor calibration routine
- ✅ **Comprehensive Debugging** - Real-time Serial Monitor output
- ✅ **Runtime Commands** - Adjust parameters without re-uploading
- ✅ **Well Documented** - Extensive comments and guides

---

## 📁 Repository Contents

| File | Description |
|------|-------------|
| `ALFR_ThickLine_Controller.ino` | **Main Arduino code** - Upload this to your Arduino Nano |
| `ALFR_SETUP_GUIDE.md` | **Complete setup guide** - Wiring, calibration, tuning |
| `DEBUG_QUICK_REFERENCE.md` | **Debugging cheat sheet** - Quick problem diagnosis |
| `WIRING_DIAGRAM.txt` | **Detailed wiring diagram** - ASCII art connections |
| `README.md` | This file - Project overview |
| `Game_Rule_Book_R2_Hardware_Hustle (1).pdf` | Competition rulebook |

---

## 🚀 Quick Start (5 Minutes)

### 1. Hardware Setup
- Wire your robot according to `WIRING_DIAGRAM.txt`
- Key components: Arduino Nano, RLS-08 sensor, L298N driver, N20 motors

### 2. Upload Code
```bash
1. Open ALFR_ThickLine_Controller.ino in Arduino IDE
2. Select: Tools → Board → Arduino Nano
3. Select: Tools → Processor → ATmega328P (Old Bootloader)
4. Select correct COM Port
5. Click Upload
```

### 3. Calibrate
```bash
1. Open Serial Monitor (115200 baud)
2. Place robot partially on black line
3. Send 'C' command (or press button)
4. Move robot left/right for 5 seconds
5. Done! Robot starts in 3 seconds
```

### 4. Test & Tune
- Robot follows line automatically
- Watch Serial Monitor for debug info
- Adjust PID values if needed (see tuning guide)

---

## 📚 Documentation

### For First-Time Setup
👉 **Start here:** `ALFR_SETUP_GUIDE.md`
- Complete wiring instructions
- Step-by-step calibration
- PID tuning guide
- Troubleshooting section

### For Quick Debugging
👉 **Quick reference:** `DEBUG_QUICK_REFERENCE.md`
- Understanding debug output
- Common problem patterns
- Fault diagnosis table
- PID value presets

### For Wiring Reference
👉 **Wiring details:** `WIRING_DIAGRAM.txt`
- Pin-by-pin connections
- Power distribution diagram
- Component layout
- Safety warnings

---

## 🔧 Hardware Requirements

Based on LAM Research Challenge 2025 kit:

- Arduino Nano R3 (CH340 chip)
- Smart Elex RLS-08 Line Sensor Array (8 IR sensors)
- L298N Motor Driver Module
- 2× N20 Gear Motors (600 RPM, 12V)
- Battery (11.1-12V, 3S LiPo or 3×18650)
- Jumper wires
- 3D printed chassis

---

## ⚙️ Configuration

### Default PID Values
```cpp
Kp = 25.0  // Proportional gain
Ki = 0.0   // Integral gain (usually not needed)
Kd = 15.0  // Derivative gain
```

### Speed Settings
```cpp
BASE_SPEED = 120  // Normal speed (0-255)
MAX_SPEED  = 200  // Straight sections
TURN_SPEED = 100  // Sharp turns
MIN_SPEED  = 80   // Minimum to prevent stall
```

### Line Following Mode
```cpp
followMode = FOLLOW_MODE_EDGE  // For THICK lines (recommended)
// OR
followMode = FOLLOW_MODE_CENTER  // For thin lines
```

All these can be changed in the `.ino` file header section.

---

## 🎮 Serial Commands

Open Serial Monitor at **115200 baud** and use these commands:

| Command | Function |
|---------|----------|
| `C` | Calibrate sensors |
| `S` | **STOP** motors (emergency) |
| `G` | **GO** - Resume |
| `V` | View raw sensor values |
| `P` | Print current PID values |
| `+` | Increase base speed |
| `-` | Decrease base speed |
| `H` | Show help |

---

## 📊 Debug Output Example

```
Sensors: [□|□|■|■|■|□|□|□] Pos:2500 Err:-1000 Corr:25 Motors L:145 R:95 Time:12s
```

**Meaning:**
- `■` = Black detected
- `□` = White detected
- `Pos` = Line position (0-7000)
- `Err` = Error from center
- `Corr` = PID correction
- `Motors` = Individual motor speeds
- `Time` = Runtime

---

## 🎯 Why This Code Works for Thick Lines

### The Problem
Traditional line followers assume the robot **straddles** a thin line with sensors on either side:

```
White  [sensor array]  White
         ↑ thin line ↑
```

But with thick lines, **multiple sensors** detect black simultaneously:

```
     [sensor array]
Black Black Black Black
   ← thick line →
```

### Our Solution: Edge Following

Instead of following the center, we follow **one edge** of the thick line:

```
[S1][S2][S3][S4][S5][S6][S7][S8]
 ↓   ↓   ↓   ↓   ↓   ←── Track this edge
Black Black Black Black White
```

The algorithm automatically:
1. Detects when >4 sensors see black (thick line)
2. Switches to edge-following mode
3. Tracks the right edge of the line
4. Provides smooth, stable control

---

## 🔧 Tuning Guide (Quick Reference)

| Problem | Solution |
|---------|----------|
| Wobbles too much | ↓ Decrease Kp by 5-10<br>↑ Increase Kd by 5 |
| Slow to respond | ↑ Increase Kp by 5-10 |
| Drifts on straights | ↑ Add small Ki (0.1-0.5) |
| Overshoots turns | ↓ Reduce TURN_SPEED<br>↑ Increase Kd |
| Too slow overall | ↑ Increase BASE_SPEED and MAX_SPEED |
| Too fast/unstable | ↓ Decrease speeds |

For detailed tuning, see `ALFR_SETUP_GUIDE.md` section 5.

---

## 🐛 Common Issues & Fixes

### Motors don't move
- Check battery voltage (should be >11V)
- Verify L298N connections
- Ensure code uploaded successfully

### Sensors not working
- Check sensor height (optimal: 3-8mm)
- Verify wiring (A0-A7)
- Recalibrate with `C` command
- Send `V` to view raw values

### Robot loses line
- Reduce speed
- Recalibrate sensors
- Ensure all 8 sensors working
- Check for loose connections

### Works on thin lines but not thick
- Verify `followMode = FOLLOW_MODE_EDGE`
- Recalibrate ON the thick line
- Reduce Kp to 15-20

For complete troubleshooting, see `DEBUG_QUICK_REFERENCE.md`.

---

## 📦 What Makes This Code Special

### 1. **Thick Line Detection**
Automatically detects thick lines and switches to edge-following mode.

### 2. **Robust Calibration**
Easy 5-second calibration process with visual feedback.

### 3. **Adaptive Speed Control**
Automatically slows down for curves, speeds up on straights.

### 4. **Anti-Windup Protection**
Prevents integral term from causing instability.

### 5. **Line Loss Recovery**
Remembers last position and searches intelligently.

### 6. **Real-Time Debugging**
Comprehensive Serial Monitor output for easy troubleshooting.

### 7. **Runtime Control**
Change parameters without re-uploading code.

### 8. **Well Commented**
Every section explained with inline comments.

---

## 🏆 Competition Performance

**Expected Results:**
- ✅ Smooth navigation of complex tracks
- ✅ Handles sharp turns and S-curves
- ✅ Reliable performance on thick line tracks
- ✅ Consistent lap times
- ✅ Minimal line loss

**Optimization Tips:**
- Start conservative (slower speeds)
- Calibrate in competition lighting
- Test on actual arena before competition
- Note successful PID values for backup
- Bring extra charged batteries

---

## 📝 Testing Checklist

Before Competition:
- [ ] Code uploads without errors
- [ ] Serial Monitor shows debug output
- [ ] All 8 sensors detect black/white
- [ ] Motors run in correct direction
- [ ] Calibration completes successfully
- [ ] Robot follows straight lines
- [ ] Robot handles 90° turns
- [ ] Robot handles S-curves
- [ ] PID values optimized
- [ ] Battery fully charged
- [ ] All wires secured

---

## 🤝 Support

### If you encounter issues:

1. **Check Serial Monitor** - Most issues show up in debug output
2. **Review Documentation** - See setup guide and debug reference
3. **Verify Wiring** - Compare with wiring diagram
4. **Test Sensors** - Use `V` command to view raw values
5. **Recalibrate** - Many issues fixed by recalibration

### Documentation Files:
- `ALFR_SETUP_GUIDE.md` - Complete setup instructions
- `DEBUG_QUICK_REFERENCE.md` - Quick problem diagnosis
- `WIRING_DIAGRAM.txt` - Detailed wiring

---

## 📄 License

Created for LAM Research Challenge 2025 - Hardware Hustle
Free to use and modify for educational and competition purposes.

---

## 🎓 Code Structure

```
ALFR_ThickLine_Controller.ino
│
├── Configuration Section
│   ├── Pin definitions
│   ├── PID parameters
│   ├── Speed settings
│   └── Follow mode selection
│
├── Setup Function
│   ├── Pin initialization
│   ├── Serial communication
│   ├── Calibration trigger
│   └── Startup countdown
│
├── Main Loop
│   ├── Read sensors
│   ├── Calculate position
│   ├── Calculate PID
│   ├── Apply motor control
│   ├── Debug output
│   └── Check serial commands
│
└── Helper Functions
    ├── Sensor reading & thresholding
    ├── Position calculation (edge following)
    ├── PID calculation
    ├── Motor control
    ├── Calibration routine
    ├── Debug printing
    └── Command handling
```

---

## 🚀 Next Steps

1. **Upload the code** to your Arduino Nano
2. **Follow setup guide** for complete wiring
3. **Calibrate sensors** on your track
4. **Test and tune** PID parameters
5. **Practice runs** on competition arena
6. **Optimize for speed** (if time permits)

---

## ⭐ Key Takeaways

- **This code is specifically designed for THICK BLACK LINES**
- **Calibration is critical** - do it before every run
- **Start slow and stable** - speed up gradually
- **Use Serial Monitor** - it's your best debugging tool
- **Read the docs** - everything is explained in detail

---

## 📞 Quick Help

**Problem?** → Check `DEBUG_QUICK_REFERENCE.md`
**Setup?** → Read `ALFR_SETUP_GUIDE.md`
**Wiring?** → See `WIRING_DIAGRAM.txt`
**Tuning?** → Section 5 of setup guide

---

**Good luck with the LAM Research Challenge 2025! 🏆**

May your robot follow every line flawlessly!

---

*Last Updated: 2025-11-27*
*Code Version: 1.0*
*Compatible with: Arduino Nano R3, RLS-08 Sensor Array, L298N Driver*
