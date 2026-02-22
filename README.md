# EN2533 Robotics Project – Autonomous Competition Robot (Arduino Firmware)

![Robot photo](assets/robot.jpg)
<!-- ✅ Add your image to: assets/robot.jpg (recommended: 1200px wide). -->

Arduino firmware for our **EN2533 Robotics Project / competition robot**.  
The codebase is split into small `.ino` modules (motion, line following, task logic, filtering) and combined by the Arduino build system.

---

## 🚀 What the robot can do (High-Level)
This robot is designed to autonomously complete multiple arena tasks using:
- **Closed-loop drive** using **quadrature encoders**
- **Line following + line-cross detection** using an **8-sensor QTR array**
- **Wall following** using **VL53L0X Time-of-Flight (ToF)** distance sensors
- **Special task behaviors** (grid traversal, dotted line, ramp, arrow alignment + direction decision)
- **Sensor filtering** to improve stability (spike removal + smoothing)

---

## ✅ Implemented Tasks / Behaviors (from code)
### Task 1 – Grid Navigation + Cell Traversal
Files: `task1_grid.ino`, `task1_mapping.ino`
- Detects **line crossings** reliably using edge timing (`lt`, `rt`) + encoder position.
- Aligns at intersections by reversing/forward corrections.
- Traverses a **CELL_COUNT × CELL_COUNT grid** (currently set to `4 × 4`).
- Includes a mapping/task hook on each intersection via:
  - `onLineCross()` → update `(nx, ny)` and apply turning logic at boundaries.
  - Placeholders for object checking/picking hooks: `checkLeft()`, `checkRight()`, `pickObjectInLeft()`, `pickObjectInRight()`.

### Task 2 – Dotted / Broken Line Following
File: `task2_dotline_follow.ino`
- Handles interrupted line patterns (dotted / segmented) with logic to recover line direction.
- Uses IR/QTR-based detection and movement strategies to avoid getting lost at gaps.

### Task 3 – Ramp Handling
File: `task3_ramp.ino`
- Contains ramp-specific movement logic (tuned drive + stability control for climbing/descending).

### Task 4 – Wall Following
File: `task4_wall_follow.ino`
- Uses **two ToF sensors (front-left + back-left)** to keep a target distance from the wall.
- Combines:
  - **Distance control**: keep average distance near target
  - **Angle control**: keep FL and BL distances similar (robot parallel to the wall)
- Output correction is applied as differential PWM to left/right motors.

### Task 6 – Arrow Detection + Shooting Alignment (Arena Arrow Logic)
File: `task6_arrow_shooting.ino` (+ rotation helpers in `rotational.ino`)
- Drives forward while reading line sensors to detect **arrow direction** using weighted error sign.
- Executes:
  - `arrow_shoot(dist, threshold)` → approach + determine arrow direction
  - `arrow_following()` → follow line patterns until exit condition
  - `arrow_find(dir)` → rotate/search until arrow is found (or timeout by encoder arc)

---

## 🧠 Core Control Algorithms
### 1) Wheel Synchronization (Straight driving)
File: `translational.ino`
- `wheelSyncIter(...)` keeps **left and right encoder counts matched** (minimizes drift).
- Used heavily in distance moves (`moveForwardDist()`).

### 2) Line Following (QTR weighted centroid)
File: `line_follow.ino`
- Converts 8 sensor readings into a **weighted error** using:
  `{-16,-9,-4,-1, +1,+4,+9,+16}`
- PID-like controller updates correction:
  - `lineFollowIter(kp, ki, kd, ...)`

### 3) Line PID Mode (separate line PID routine)
File: `line_pid.ino`
- Used during specific sections (ex: after enabling with encoder distance).
- Helps keep tracking stable at higher speeds.

### 4) Filtering / Robustness
Files: `filters.ino`, `ir_functions.ino`
- `removeSpikes()` rejects invalid ToF values + sudden jumps.
- Helper checks:
  - `allSensorsBlack()`, `allSensorsWhite()`, `anySensorWhite()`

---

## 🧩 Hardware & Specifications (as used in code)
### Controller + Interfaces
- **Arduino board** (uses many high-numbered digital pins + multiple serial ports; typical for **Mega**-class boards)
- **I2C** (`Wire`) for sensors
- **Serial**
  - `Serial` @ 115200 (debug)
  - `Serial2` @ 115200 (secondary link / external module integration)

### Drive System
- 2 × DC motors + H-bridge control (direction + PWM)
- 2 × quadrature encoders

**Physical parameters (from `main.ino`):**
- Encoder CPR: **896**
- Wheel diameter: **68 mm**
- Wheel-to-wheel distance: **162 mm**

### Sensors
- **QTR 8-channel analog line sensor array**
  - Pins: `A0..A7`
- **VL53L0X ToF sensors (3 units)**
  - Enabled via XSHUT pins:
    - `XSHUT_FL = 33`
    - `XSHUT_BL = 32`
    - `XSHUT_FLT = 24`
  - Assigned I2C addresses:
    - `FL  = 0x30`
    - `BL  = 0x31`
    - `FLT = 0x32`
- **TCS34725 Color Sensor**
  - LED pin: `TCS_LED_PIN = 46`
- Extra analog IR references (used in arrow routine):
  - `A12`, `A13` (threshold-based)

### Motor Pin Mapping (from `main.ino`)
**Left Motor**
- Direction pins: `42`, `40`
- PWM pin: `12`
- Encoder pins: `2`, `3`

**Right Motor**
- Direction pins: `38`, `36`
- PWM pin: `11`
- Encoder pins: `18`, `19`

---

## 📁 Repository Structure
| File | Purpose |
|------|---------|
| `main.ino` | Setup, sensor init, motor structs, global state |
| `motor_instructions.ino` | `setPWM()`, `brakeHIGH()`, `brakeLOW()` |
| `translational.ino` | Straight motion + wheel synchronization |
| `rotational.ino` | Turning / rotation behaviors (includes `arrow_find`) |
| `line_follow.ino` | Line-following controller and helpers |
| `line_pid.ino` | Dedicated line PID routine (enable/disable logic) |
| `filters.ino` | ToF spike removal utility |
| `ir_functions.ino` | all-black/all-white/any-white QTR helpers |
| `task1_grid.ino` | Grid movement + intersection alignment |
| `task1_mapping.ino` | Grid indexing + traversal logic (mapping hooks) |
| `task2_dotline_follow.ino` | Dotted/broken line logic |
| `task3_ramp.ino` | Ramp handling |
| `task4_wall_follow.ino` | Wall following (ToF-based) |
| `task6_arrow_shooting.ino` | Arrow task logic |

---

## 🛠️ Build & Upload
### Arduino IDE / PlatformIO Requirements
Install these libraries:
- `QTRSensors`
- `Adafruit VL53L0X`
- `Adafruit TCS34725`
- `Wire` (built-in)

### Upload
1. Open `main.ino` (Arduino will load the other `.ino` tabs automatically).
2. Select board + correct COM port.
3. Upload.
4. Open Serial Monitor at **115200** for debug logs.

---

