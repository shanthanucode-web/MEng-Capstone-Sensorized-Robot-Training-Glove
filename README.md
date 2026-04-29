# Sensorized Robot Training Glove — ESP32-S3 Firmware & Toolchain

A wearable data glove that captures hand orientation, finger curl, and pressure contact in real time, streams them over USB serial, and renders them as a live 3D robot-hand visualization. Designed for hand-pose capture in robot training environments where magnetic interference makes traditional IMU fusion unreliable.

---

## Table of Contents

1. [Goal](#goal)
2. [Outcome](#outcome)
3. [Hardware](#hardware)
   - [Component List](#component-list)
   - [Hardware Tree](#hardware-tree)
   - [Circuit — Flex Sensor Voltage Divider](#circuit--flex-sensor-voltage-divider)
   - [Pin Assignment Table](#pin-assignment-table)
4. [Software](#software)
   - [Software Tree](#software-tree)
   - [Firmware (`src/main.cpp`)](#firmware-srcmaincpp)
   - [Serial Protocol](#serial-protocol)
   - [Python Tools](#python-tools)
5. [Data Flow](#data-flow)
6. [Setup & Installation](#setup--installation)
   - [Firmware](#firmware-setup)
   - [Python Environment](#python-environment)
7. [Build & Flash](#build--flash)
8. [Running the Tools](#running-the-tools)
   - [Full Glove Visualizer](#full-glove-visualizer)
   - [Flex Sensor Test Tool](#flex-sensor-test-tool)
   - [Session Recorder](#session-recorder)
   - [Flex Calibration Analyzer](#flex-calibration-analyzer)
   - [Legacy IMU Visualizer](#legacy-imu-visualizer)
9. [Flex Calibration Workflow](#flex-calibration-workflow)
10. [Recorded Data Format](#recorded-data-format)
11. [Known Issues & TODOs](#known-issues--todos)

---

## Goal

Capture and visualize hand pose data from a sensorized glove for use in robot teleoperation training datasets. The system must:

- Track full hand orientation in 3D space using an IMU
- Track individual finger curl for the thumb, index, and middle fingers
- Track three pressure/FSR contact points
- Stream data continuously at a consistent rate (~10 Hz)
- Remain stable in environments with motors, metal, and magnetic interference
- Provide real-time visualization and offline calibration tooling

---

## Outcome

The glove streams three compact serial lines every 100ms — a quaternion (`Q:`), normalized flex values (`F:`), and normalized pressure values (`P:`) — which Python tools parse live to render a 3D hand that rotates with the glove, curls fingers as they bend, and shows pressure contact. Sessions can be recorded to timestamped CSVs, and a calibration tool analyzes recordings to recommend per-sensor tuning constants for the firmware.

```
Q:0.9997,0.0067,-0.0211,-0.0097
F:1.000,0.000,0.000,0.000,0.015
P:0.000,0.438,0.912
```

---

## Hardware

### Component List

| Component | Part | Notes |
|---|---|---|
| Microcontroller | ESP32-S3-N8R8 DevKitC-1 | 8MB flash, 8MB OPI PSRAM, native USB CDC |
| IMU | Adafruit BNO085 | 9-DOF, I2C, address `0x4A` |
| Flex sensors | 5x generic resistive flex sensors | ~10–25kΩ flat, ~50–100kΩ fully bent |
| Pressure sensors | 3x FSR sensors | Routed through CD4051BE Y5-Y7 |
| Resistors | 8x 10kΩ | Voltage divider pull-ups, one per analog sensor |
| Power | USB 5V (from host) → 3.3V via onboard LDO | |
| Connection | USB-C cable | Upload + serial monitor + power |

---

### Hardware Tree

```
Glove Hardware
├── ESP32-S3-N8R8 DevKitC-1
│   ├── USB-C (native CDC)
│   │   ├── Serial data out (Q:, F:, and P: lines at 115200 baud)
│   │   ├── Firmware upload (esp-builtin JTAG over USB)
│   │   └── Power input (5V → 3.3V onboard LDO)
│   │
│   ├── I2C Bus (SDA=GPIO8, SCL=GPIO9)
│   │   └── Adafruit BNO085 IMU (addr 0x4A)
│   │       ├── Accelerometer
│   │       ├── Gyroscope
│   │       └── Magnetometer (unused — Game Rotation Vector mode)
│   │
│   └── Analog Sensor Mux (CD4051BE)
│       ├── GPIO4  — ADC1 CH3, mux Z/common output
│       ├── GPIO5  — mux A / select bit 0
│       ├── GPIO6  — mux B / select bit 1
│       ├── GPIO7  — mux C / select bit 2
│       ├── Y0     — Thumb flex sensor
│       ├── Y1     — Upper index flex sensor
│       ├── Y2     — Lower index flex sensor
│       ├── Y3     — Lower middle flex sensor
│       ├── Y4     — Upper middle flex sensor
│       ├── Y5     — FSR pressure sensor 1
│       ├── Y6     — FSR pressure sensor 2
│       └── Y7     — FSR pressure sensor 3
│
└── Analog Sensor Voltage Dividers (per sensor)
    ├── 3.3V rail
    ├── 10kΩ fixed resistor
    ├── CD4051BE mux channel (reads midpoint voltage)
    ├── Flex/FSR sensor (variable resistance)
    └── GND
```

---

### Circuit — Flex Sensor Voltage Divider

Each flex sensor is wired as a voltage divider against a fixed 10kΩ resistor:

```
3.3V
 |
[10kΩ fixed]
 |
 +——— CD4051BE Y channel
 |
[flex sensor]   ~10–25kΩ flat  /  ~50–100kΩ fully bent
 |
GND
```

**Reading behavior:**
- **Finger extended (flat):** flex resistance ~10–25kΩ → lower midpoint voltage → lower ADC value (~2800)
- **Finger curled (bent):** flex resistance ~50–100kΩ → higher midpoint voltage → higher ADC value (~3700)

The firmware maps this range to [0.0 = open, 1.0 = closed] using per-sensor `FLEX_MIN`/`FLEX_MAX` constants. FSR pressure sensors use the same divider orientation, but their normalization is inverted: no pressure reads high and maps to 0.0, while firm pressure reads lower and maps to 1.0 using `PRESSURE_IDLE`/`PRESSURE_PRESS`.

---

### Pin Assignment Table

| GPIO / Mux | ADC Channel | Function | Status |
|---|---|---|---|
| GPIO4 | ADC1 CH3 | CD4051BE Z/common output | Active |
| GPIO5 | — | CD4051BE A / select bit 0 | Active |
| GPIO6 | — | CD4051BE B / select bit 1 | Active |
| GPIO7 | — | CD4051BE C / select bit 2 | Active |
| CD4051BE Y0 | — | Thumb flex sensor | Active |
| CD4051BE Y1 | — | Upper index flex sensor | Active |
| CD4051BE Y2 | — | Lower index flex sensor | Active |
| CD4051BE Y3 | — | Lower middle flex sensor | Active |
| CD4051BE Y4 | — | Upper middle flex sensor | Active |
| CD4051BE Y5 | — | FSR pressure sensor 1 | Active |
| CD4051BE Y6 | — | FSR pressure sensor 2 | Active |
| CD4051BE Y7 | — | FSR pressure sensor 3 | Active |
| GPIO8 | — | I2C SDA (BNO085) | Active |
| GPIO9 | — | I2C SCL (BNO085) | Active |

---

## Software

### Software Tree

```
Glove_IMUBNO055_ESP32/
├── src/
│   └── main.cpp                  Firmware — IMU + flex + pressure read loop, serial output
│
├── platformio.ini                PlatformIO build config (board, libs, upload protocol)
│
├── Python Tools
│   ├── glove_visualization.py    Full visualizer: 3D robot hand + flex/pressure bars + HUD
│   ├── flex_visualization.py     Flex-only test tool: fixed camera, no IMU
│   ├── pressure_visualization.py Pressure-only test tool for the 3 FSR channels
│   ├── record_session.py         Records quaternion + flex + pressure to timestamped CSV
│   ├── calibrate_flex.py         Analyzes CSV and recommends FLEX_MIN/FLEX_MAX constants
│   └── visualize_imu.py          Legacy: solid mesh hand, IMU-only (no flex bars)
│
├── session_YYYYMMDD_HHMMSS.csv   Recorded sessions (generated by record_session.py)
│
├── boards/
│   └── lolin_s3_mini.json        Custom board definition (not currently used)
│
├── .pio/
│   └── libdeps/                  Auto-downloaded library dependencies
│       └── esp32-s3-n8r8/
│           ├── Adafruit BNO08x   IMU driver (SH2 protocol)
│           ├── Adafruit BusIO    I2C/SPI abstraction layer
│           └── Adafruit Unified Sensor
│
├── CLAUDE.md                     Claude Code guidance and project notes
└── README.md                     This file
```

---

### Firmware (`src/main.cpp`)

The firmware runs a single 100ms loop that reads the IMU, flex sensors, and pressure sensors, then emits three lines of serial output.

#### `setup()`

```
1. Serial.begin(115200)
   └── delay(500) + while(!Serial) — waits for USB CDC to enumerate on macOS

2. analogReadResolution(12)
   └── Sets ADC to 12-bit (0–4095) instead of default 10-bit (0–1023)

3. Wire.begin(8, 9)
   └── I2C on custom pins SDA=GPIO8, SCL=GPIO9

4. bno.begin_I2C(0x4A)
   └── Initializes BNO085 — halts with error message if not found

5. bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)
   └── Enables quaternion output at 10ms intervals (100Hz internal, 10Hz output)
```

#### `loop()` — every 100ms

```
1. Drain BNO085 FIFO
   while (bno.getSensorEvent(&sensorValue))
   └── Updates qw, qx, qy, qz from the freshest SH2_GAME_ROTATION_VECTOR event

2. Read flex sensors
   select CD4051BE channels Y0/Y1/Y2/Y4/Y3 and analogRead(GPIO4)
   └── 12-bit raw values (0–4095)

3. Normalize flex
   normalizeFlex(raw, idx) = clamp((raw - FLEX_MIN[idx]) / (FLEX_MAX[idx] - FLEX_MIN[idx]), 0.0, 1.0)

4. Read and normalize pressure sensors
   select CD4051BE channels Y5/Y6/Y7 and analogRead(GPIO4)
   normalizePressure(raw, idx) = clamp((PRESSURE_IDLE[idx] - raw) / (PRESSURE_IDLE[idx] - PRESSURE_PRESS[idx]), 0.0, 1.0)

5. Machine-readable output
   Serial.println("Q:w,x,y,z")      — quaternion (4 decimal places)
   Serial.println("F:t,ui,li,um,lm") — normalized flex (3 decimal places)
   Serial.println("P:p1,p2,p3")      — normalized pressure (3 decimal places)

6. Human-readable output
   "Quat | W:... X:... Y:... Z:... Acc:..."
   "Flex | T:raw(norm) UI:raw(norm) ..."
   "Press | P1:raw(norm) P2:raw(norm) P3:raw(norm)"
   "---"
```

#### Why Game Rotation Vector?

The BNO085 offers two quaternion modes:

| Mode | Fuses | Pros | Cons |
|---|---|---|---|
| Rotation Vector | Gyro + Accel + Magnetometer | Absolute heading (north-referenced) | Sensitive to motors, metal, magnetic interference in robot environments |
| **Game Rotation Vector** | **Gyro + Accel only** | **Stable near motors and metal** | Heading drifts slowly over minutes |

For hand-pose capture sessions (seconds to tens of seconds), heading drift is negligible. Stability is far more useful.

---

### Serial Protocol

Three machine-readable lines are emitted every 100ms. All Python tools parse only these — all other lines (`Quat |`, `Flex |`, `Press |`, `---`) are ignored.

```
Q:w,x,y,z
```
- BNO085 Game Rotation Vector quaternion components
- `w` = real part, `x y z` = imaginary vector (i, j, k)
- 4 decimal places, signed floats
- Example: `Q:0.9997,0.0067,-0.0211,-0.0097`

```
F:t,ui,li,um,lm
```
- Normalized flex values: thumb, upper index, lower index, upper middle, lower middle
- Range: `0.000` = finger fully open, `1.000` = finger fully closed
- 3 decimal places
- Example: `F:1.000,0.312,0.448,0.000,0.021`

```
P:p1,p2,p3
```
- Normalized pressure values from FSR channels Y5, Y6, Y7
- Range: `0.000` = no pressure, `1.000` = firm calibrated pressure
- 3 decimal places
- Example: `P:0.000,0.438,0.912`

**Field mapping:**

| Position | Field | Sensor | GPIO |
|---|---|---|---|
| `t` | Thumb | Thumb curl | CD4051BE Y0 |
| `ui` | Upper Index | Proximal index segment | CD4051BE Y1 |
| `li` | Lower Index | Distal index segment | CD4051BE Y2 |
| `um` | Upper Middle | Proximal middle segment | CD4051BE Y4 |
| `lm` | Lower Middle | Distal middle segment | CD4051BE Y3 |

| Position | Field | Sensor | GPIO |
|---|---|---|---|
| `p1` | Pressure 1 | FSR 1 | CD4051BE Y5 |
| `p2` | Pressure 2 | FSR 2 | CD4051BE Y6 |
| `p3` | Pressure 3 | FSR 3 | CD4051BE Y7 |

---

### Python Tools

All tools auto-detect the current glove serial device, preferring `/dev/cu.usbmodem*` and then `/dev/cu.usbserial*`, at 115200 baud. Set `GLOVE_SERIAL_PORT` to override this if needed. They open serial with `dsrdtr=False, rtscts=False` to prevent DTR/RTS from resetting the ESP32, and auto-reconnect if the port drops.

#### `glove_visualization.py` — Full Real-Time Visualizer

The primary visualization tool. Renders a solid 3D robot hand that tracks IMU orientation, curls fingers with flex sensor data, and shows pressure values from the FSR channels.

**Architecture:**

```
glove_visualization.py
├── Section 1: Constants
│   ├── Window size (1280×800)
│   ├── Colors (RGBA numpy arrays per hand part, joint, flex bar, pressure bar)
│   ├── Hand geometry (palm nodes, wrist anchors, segment lengths)
│   └── FINGER_DEFS — (name, palm_node, splay_deg, upper_flex_idx, lower_flex_idx, color)
│
├── Section 2: Shared State (_state dict + threading.Lock)
│   ├── 'q'     — latest quaternion [w, x, y, z]
│   ├── 'f'     — latest flex values [0..1] × 5
│   ├── 'p'     — latest pressure values [0..1] × 3
│   ├── 'acc'   — BNO085 accuracy level (0–3)
│   └── 'times' — deque of F: arrival timestamps (for Hz calculation)
│
├── Section 3: Serial Thread (daemon)
│   ├── Parses Q: → updates _state['q']
│   ├── Parses F: → updates _state['f'] + timestamps
│   ├── Parses P: → updates _state['p']
│   └── Auto-reconnects on SerialException
│
├── Section 4: Quaternion Math
│   ├── _qmul(a, b)    — Hamilton product
│   ├── _qrel(ref, q)  — relative quaternion (removes calibration offset)
│   └── _qmat(w,x,y,z) — quaternion → 3×3 rotation matrix
│
├── Section 5: Robot Hand Geometry
│   ├── compute_hand_pose(flex)
│   │   └── Calculates thumb/index/middle live joints plus passive ring/pinky
│   └── build_hand_mesh(flex, pressure, R_imu)
│       ├── Builds palm shell, wrist mount, finger plates, hinges, and rivets
│       ├── Adds pressure pads to the live fingertips
│       └── Applies R_imu to all mesh vertices
│
├── Section 6: Flex/Pressure Bar Geometry (2D panel)
│   ├── _bar_quad()       — single bar filled quad
│   ├── build_fill_mesh() — all 5 fill bars as one Mesh
│   ├── build_track_mesh()— static dark background tracks
│   └── build_pressure_fill_mesh() — all 3 pressure bars as one Mesh
│
├── Section 7: Serial Thread Start
│
├── Section 8: VisPy Scene
│   ├── view3d (col_span=3) — TurntableCamera, 3D hand + XYZ axes
│   └── view2d (col_span=1) — PanZoomCamera, flex bars + labels + values
│
├── Section 9: Timer Callback (30fps)
│   ├── Reads _state under lock
│   ├── _qrel → _qmat → build_hand_mesh → update hand_mesh
│   ├── build_fill_mesh → update flex bars + value_texts
│   ├── build_pressure_fill_mesh → update pressure bars
│   └── Update HUD (quaternion, accuracy, sample rate)
│
├── Section 10: Keyboard Handler
│   └── C — captures _ref_q = current quaternion (sets neutral pose)
│
└── Section 11: Main (app.run())
```

**Key design decisions:**
- **Chained rotation model:** Each live finger has two segments. The lower segment's curl is applied in the local frame of the upper segment (`R_splay @ R_upper @ R_lower`), producing a realistic closed-fist shape at full flex.
- **Calibration reference:** `_ref_q` stores the quaternion at the moment `C` is pressed. `_qrel` subtracts this from all subsequent readings so the hand always shows its pose relative to the neutral reference.
- **Lock discipline:** The serial thread holds `_lock` only while writing to `_state`. The timer callback holds it only while copying values out. The lock is never held across rendering.

---

#### `flex_visualization.py` — Flex Sensor Test Tool

Simplified version of the full visualizer. Uses the same skeleton geometry and bar panel but with no IMU — the hand is fixed at a diagonal view angle using a constant rotation matrix (`R_FIXED`). Use this to verify flex sensor wiring and calibration without needing IMU data to be working.

**Differences from `glove_visualization.py`:**

| Feature | `glove_visualization.py` | `flex_visualization.py` |
|---|---|---|
| IMU quaternion | Live from serial | Fixed matrix (no rotation) |
| Serial lines parsed | `Q:`, `F:`, and `P:` | `F:` only |
| Calibration (`C` key) | Yes | No |
| Accuracy HUD | Yes | No |
| Camera preset | Slight elevation, angled | Front-facing |

---

#### `pressure_visualization.py` — Pressure Sensor Test Tool

Pressure-only test tool for the three FSR channels. It parses only `P:p1,p2,p3`
lines and shows large live force pads plus horizontal bars. Use this when tuning
`PRESSURE_IDLE` and `PRESSURE_PRESS` without needing flex or IMU data.

---

#### `record_session.py` — Session Recorder

Records a complete session to a timestamped CSV. Each row pairs the freshest quaternion, flex reading, and pressure reading from that frame.

**Flow:**

```
1. Serial thread starts (background daemon)
   └── Parses Q: → latest_q
       Parses F: → latest_flex
       Parses P: → writes CSV row (if recording=True)

2. Main thread blocks on input("Press Enter to start")

3. On Enter:
   ├── Opens session_YYYYMMDD_HHMMSS.csv
   ├── Writes header row
   └── Sets recording=True, start_time_s = time.monotonic()

4. Serial thread writes rows:
   [timestamp_ms] + [qw, qx, qy, qz] + [flex × 5] + [pressure × 3]
   Prints live feedback every 5 samples

5. On Ctrl+C:
   ├── recording=False, stop_event.set()
   ├── csv_file_obj.flush() + .close()
   └── Prints summary (samples, duration, average rate)
```

**Timing:** Firmware emits `Q:`, then `F:`, then `P:` each loop iteration. The recorder writes on `P:`, so quaternion, flex, and pressure correspond to the same 100ms frame.

---

#### `calibrate_flex.py` — Flex Calibration Analyzer

Analyzes a recorded session CSV and prints ready-to-paste `FLEX_MIN`/`FLEX_MAX` constants for the firmware.

**Method:**

The firmware normalizes raw ADC values with:
```
normalized = (raw - FLEX_MIN) / (FLEX_MAX - FLEX_MIN)
```

The script inverts this to recover the raw ADC series:
```
raw_adc = normalized × (FLEX_MAX - FLEX_MIN) + FLEX_MIN
```

Then recommends:
- `FLEX_MIN_new` = 20th-percentile ADC value (lowest 20% of readings = resting/open state)
- `FLEX_MAX_new` = observed ADC maximum (full curl ceiling)

**Output table columns:**

| Column | Meaning | Target |
|---|---|---|
| `OBS_MIN` | Lowest normalized value seen | ~0.00 when finger fully open |
| `OBS_MAX` | Highest normalized value seen | ~1.00 when finger fully closed |
| `REST_MEAN` | Mean of lowest-20% readings | < 0.10 (resting should read near zero) |
| `ADC_MIN` | Recommended new `FLEX_MIN` | — |
| `ADC_MAX` | Recommended new `FLEX_MAX` | — |
| `STATUS` | `OK` or `RECALIBRATE` | — |

**Important:** `CURRENT_FLEX_MIN` and `CURRENT_FLEX_MAX` at the top of `calibrate_flex.py` must match the constants in `src/main.cpp` exactly. If they differ, the back-calculation will produce wrong values.

---

#### `visualize_imu.py` — Legacy IMU Visualizer

The original tool from before flex sensors were added. Renders a solid 3D hand mesh (box segments per finger, not a wire skeleton) with:
- All 5 fingers including ring and pinky
- 3 joint segments per finger with rest angles and per-segment shading
- FSR force sensor support from `P:` lines, kept as a legacy reference path

Not actively used. Kept as reference for the full-5-finger geometry model.

---

## Data Flow

```
┌─────────────────────────────────────────────────────────┐
│  ESP32-S3 (every 100ms)                                 │
│                                                         │
│  BNO085 FIFO drain → qw, qx, qy, qz                    │
│  mux Y0/Y1/Y2/Y4/Y3 → raw flex[5]                     │
│  mux Y5/Y6/Y7 → raw pressure[3]                       │
│  normalizeFlex() → flex[5]                              │
│  normalizePressure() → pressure[3]                      │
│                                                         │
│  Serial.println("Q:w,x,y,z")                           │
│  Serial.println("F:t,ui,li,um,lm")                     │
│  Serial.println("P:p1,p2,p3")                          │
└────────────────────┬────────────────────────────────────┘
                     │ USB CDC @ 115200 baud
                     │ auto-detected USB CDC port
                     ▼
┌─────────────────────────────────────────────────────────┐
│  Python Serial Thread (background daemon)               │
│                                                         │
│  Q: → _state['q'] = [w, x, y, z]                       │
│  F: → _state['f'] = [t, ui, li, um, lm]                │
│       _state['times'].append(monotonic())               │
│  P: → _state['p'] = [p1, p2, p3]                       │
└────────────────────┬────────────────────────────────────┘
                     │ threading.Lock
                     ▼
┌─────────────────────────────────────────────────────────┐
│  VisPy Timer Callback (30fps, main thread)              │
│                                                         │
│  q, flex, pressure, acc = snapshot(_state)              │
│                                                         │
│  qr = _qrel(_ref_q, q)      ← removes neutral offset   │
│  R  = _qmat(*qr)            ← quaternion → 3×3 matrix  │
│                                                         │
│  hand vertices, faces, colors =                         │
│      build_hand_mesh(flex, pressure, R)                 │
│                                                         │
│  hand_mesh.set_data(...)                                │
│  bar_fills.set_data(build_fill_mesh(flex))              │
│  pressure_fills.set_data(build_pressure_fill_mesh(p))   │
│  HUD text updates                                       │
│  canvas.update()                                        │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
              VisPy OpenGL render
         (3D robot hand + 2D flex/pressure panel)
```

---

## Setup & Installation

### Firmware Setup

Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI). All library dependencies are declared in `platformio.ini` and downloaded automatically on first build:

- `adafruit/Adafruit BNO08x` — BNO085 driver (SH2 protocol)
- `adafruit/Adafruit Unified Sensor` — Adafruit sensor abstraction layer
- `adafruit/Adafruit BusIO` — I2C/SPI transport abstraction

### Python Environment

```bash
pip3 install vispy pyserial numpy pyopengl pandas
```

| Package | Used by |
|---|---|
| `vispy` | All visualizers (OpenGL 3D + 2D rendering) |
| `pyserial` | All tools (serial port communication) |
| `numpy` | All visualizers (matrix math, vertex arrays) |
| `pyopengl` | VisPy backend |
| `pandas` | `calibrate_flex.py` only |

---

## Build & Flash

```bash
# Build firmware
platformio run

# Upload via built-in USB JTAG (no button presses needed)
platformio run --target upload

# Open serial monitor (close before running Python tools)
platformio device monitor
```

**Upload notes:**
- Uses `esp-builtin` upload protocol (OpenOCD over the ESP32-S3's native USB JTAG interface)
- Warnings like `Unexpected OCD_ID` during upload are harmless — verify succeeds regardless
- After flashing, the USB CDC port may need a manual `RST` button press to re-enumerate before the serial monitor shows output

---

## Running the Tools

> **Important:** Close the PlatformIO serial monitor before running any Python tool. Only one process can hold the auto-detected USB CDC port at a time.

### Full Glove Visualizer

```bash
python3 glove_visualization.py
```

- Opens a 1280×800 window with the 3D robot hand (left) and flex/pressure panel (right)
- Left-drag to orbit the 3D view, scroll to zoom
- Press `C` to capture the current hand orientation as the neutral reference pose — all subsequent motion is displayed relative to this pose
- HUD shows: live quaternion, IMU accuracy level (color-coded 0–3), sample rate

**IMU accuracy colors:**

| Color | Level | Meaning |
|---|---|---|
| Red | 0 | Unreliable — sensor still initializing or needs motion |
| Orange | 1–2 | Low / Medium — usable but imprecise |
| Green | 3 | High — fully calibrated |

---

### Flex Sensor Test Tool

```bash
python3 flex_visualization.py
```

- Same layout as the full visualizer but no IMU — hand is fixed at a front-facing diagonal
- Use this to verify flex sensor wiring and calibration in isolation
- Shows finger curl and bar values in real time without needing the IMU to be calibrated

---

### Pressure Sensor Test Tool

```bash
python3 pressure_visualization.py
```

- Shows only the three FSR pressure channels from `P:p1,p2,p3`
- Use this to verify mux channels Y5, Y6, and Y7
- Tune `PRESSURE_IDLE` from no-touch raw readings and `PRESSURE_PRESS` from firm intended-contact readings in the firmware serial debug output

---

### Session Recorder

```bash
python3 record_session.py
```

1. The script connects to the serial port and starts receiving data in the background
2. Press `Enter` to begin recording — a timestamped CSV file is created immediately
3. Perform the desired hand motion (full open → full close, or a specific gesture)
4. Press `Ctrl+C` to stop — the CSV is flushed and closed, and a summary is printed

Live feedback is printed every 5 samples:
```
  [  12.4s]  samples=  124  Q=(+0.999,+0.006,-0.021,-0.009)  Flex=(1.00,0.31,0.44,0.00,0.02)  P=(0.00,0.44,0.91)
```

---

### Flex Calibration Analyzer

```bash
# Use most recent session CSV automatically
python3 calibrate_flex.py

# Or specify a file
python3 calibrate_flex.py session_20260403_135814.csv
```

Prints a calibration table and ready-to-paste firmware constants:

```
  const int FLEX_MIN[5] = {2800, 2812, 2795, 2830, 2801};  // flat/extended
  const int FLEX_MAX[5] = {3700, 3641, 3588, 3712, 3623};  // fully curled
```

Paste these into `src/main.cpp` (replacing the existing lines) and reflash.

---

### Legacy IMU Visualizer

```bash
python3 visualize_imu.py
```

Renders a solid 3D hand mesh (all 5 fingers) driven by IMU quaternion only. Press `C` to calibrate neutral pose. Not actively maintained — use `glove_visualization.py` instead.

---

## Flex Calibration Workflow

Perform this workflow when flex readings look wrong (fingers show as partially curled at rest, or don't reach 1.0 when fully closed).

```
Step 1 — Record a calibration session
   python3 record_session.py
   → Flex all fingers fully open, then fully closed, repeat 3–5 times
   → Ctrl+C to stop

Step 2 — Analyze the recording
   python3 calibrate_flex.py
   → Check the STATUS column — "RECALIBRATE" means that sensor needs adjustment
   → Note the recommended ADC_MIN and ADC_MAX values

Step 3 — Update firmware constants
   Edit src/main.cpp lines:
     const int FLEX_MIN[5] = {...};
     const int FLEX_MAX[5] = {...};
   → Paste the values printed by calibrate_flex.py

Step 4 — Also update calibrate_flex.py
   Edit the CURRENT_FLEX_MIN / CURRENT_FLEX_MAX arrays
   → Must match main.cpp exactly for future back-calculations to be correct

Step 5 — Reflash
   platformio run --target upload

Step 6 — Verify
   python3 flex_visualization.py
   → Open hand should read ~0.00 on all bars
   → Closed fist should read ~1.00 on all active bars
```

---

## Recorded Data Format

Session CSVs are written by `record_session.py` with one row per `P:` line received (~10 rows/sec). Firmware emits `Q:`, then `F:`, then `P:` each frame, so each row contains the freshest quaternion, flex, and pressure readings.

**Filename:** `session_YYYYMMDD_HHMMSS.csv`

**Columns:**

| Column | Type | Description |
|---|---|---|
| `timestamp_ms` | int | Milliseconds since recording started |
| `qw` | float | Quaternion real component (w) |
| `qx` | float | Quaternion imaginary x |
| `qy` | float | Quaternion imaginary y |
| `qz` | float | Quaternion imaginary z |
| `flex_thumb` | float | Thumb curl 0.0–1.0 |
| `flex_upper_index` | float | Upper index segment curl 0.0–1.0 |
| `flex_lower_index` | float | Lower index segment curl 0.0–1.0 |
| `flex_upper_middle` | float | Upper middle segment curl 0.0–1.0 |
| `flex_lower_middle` | float | Lower middle segment curl 0.0–1.0 |
| `pressure_1` | float | FSR pressure 1, 0.0–1.0 |
| `pressure_2` | float | FSR pressure 2, 0.0–1.0 |
| `pressure_3` | float | FSR pressure 3, 0.0–1.0 |

**Example rows:**

```
timestamp_ms,qw,qx,qy,qz,flex_thumb,flex_upper_index,flex_lower_index,flex_upper_middle,flex_lower_middle,pressure_1,pressure_2,pressure_3
1,0.9997,0.0067,-0.0211,-0.0097,1.0,0.0,0.0,0.0,0.015,0.0,0.438,0.912
124,0.9997,0.006,-0.0212,-0.0098,1.0,0.0,0.0,0.0,0.009
```

---

## Known Issues & TODOs

| Issue | Location | Details |
|---|---|---|
| Flex calibration constants need final tuning | `src/main.cpp` | Per-sensor constants are present, but should be refined after the wiring is mechanically stable. Run the calibration workflow after the raw ADC values move smoothly. |
| Pressure calibration constants need final tuning | `src/main.cpp` | `PRESSURE_IDLE/PRESSURE_PRESS` are safe starting values. Replace them after measuring raw no-contact and firm-contact values for each FSR. |
| Ring and pinky are passive | `glove_visualization.py`, `flex_visualization.py` | The main visualizer renders ring and pinky as relaxed passive fingers because there are no live sensors for them. |
| `calibrate_flex.py` must stay in sync with `main.cpp` | `calibrate_flex.py:37–38` | `CURRENT_FLEX_MIN/MAX` must match the firmware constants exactly or the ADC back-calculation will be wrong. |
| Dead code in legacy visualizer | `visualize_imu.py:232–238` | Palm mesh is built, immediately discarded with `parts.pop()`, then rebuilt manually. Harmless but confusing. |
| `visualize_imu.py` runs at 20fps | `visualize_imu.py` | Timer interval is `0.05s` vs. `1/30` in the newer tools. Inconsistency with no functional impact since it is unused. |
