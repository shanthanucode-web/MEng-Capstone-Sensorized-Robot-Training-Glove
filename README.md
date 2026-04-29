# Sensorized Data Collection Glove for Dexterous Robot Manipulation Training
**UC Berkeley MEng Capstone - Team 74**
**Industry Partner: OMGrab**

---

## Overview

This repository contains the software stack for a sensorized wearable glove that
captures human hand motion during manipulation tasks. The captured data is
recorded, cleaned, stored in SQLite, and exported as training-ready datasets for
robot imitation learning workflows.

The glove captures three signal categories:

- **Wrist orientation** - BNO085 quaternion
- **Finger curl** - calibrated flex sensor values
- **Fingertip contact force** - calibrated FSR values

The glove records human demonstrations only. It does not control a robot.

---

## System Architecture

### Hardware

| Component | Part | Role |
|---|---|---|
| Microcontroller | ESP32-S3-N8R8 | Sensor acquisition and USB CDC serial streaming |
| IMU | Adafruit BNO085 | Wrist orientation quaternion |
| Flex sensors | 5x resistive flex sensors | Finger curl measurement |
| FSRs | 3x force-sensitive resistors | Fingertip contact force |
| Multiplexer | CD4051BE 8-channel analog MUX | Routes analog sensors to one ADC pin |

The CD4051BE signal output feeds GPIO4. Select lines use GPIO5, GPIO6, and
GPIO7. The BNO085 uses I2C on SDA=GPIO8 and SCL=GPIO9.

### Current MUX Channel Map

| Serial field | Sensor | MUX channel |
|---|---|---|
| `F:t` | Thumb flex | Y0 |
| `F:ui` | Upper index flex | Y1 |
| `F:li` | Lower index flex | Y2 |
| `F:um` | Upper middle flex | Y4 |
| `F:lm` | Lower middle flex | Y3 |
| `P:p1` | FSR 1 | Y5 |
| `P:p2` | FSR 2 | Y6 |
| `P:p3` | FSR 3 | Y7 |

Y3/Y4 are intentionally swapped in firmware to match the physical glove wiring.

### Firmware

`src/main.cpp` runs on the ESP32 using PlatformIO/Arduino. It:

1. Initializes the BNO085 in `SH2_GAME_ROTATION_VECTOR` mode.
2. Reads all flex and FSR channels through the CD4051BE.
3. Applies current firmware calibration constants.
4. Emits machine-readable serial lines over USB CDC.

Game Rotation Vector mode avoids magnetometer dependency, which is important near
metal hardware and motors.

### Serial Protocol

Firmware emits `SESSION_START` once after setup. The data stream then repeats:

```text
Q:w,x,y,z
F:t,ui,li,um,lm
P:p1,p2,p3
```

Flex and FSR values are normalized to 0.0-1.0 in firmware using the current
calibration constants. The current firmware keeps the existing slower loop
cadence with `delay(100)`.

---

## Python Pipeline

```text
data_collector.py  ->  sessions/raw/*.csv
data_cleaner.py    ->  sessions/cleaned/*.csv
db_writer.py       ->  glove_dataset.db
export_dataset.py  ->  dataset_<task>.csv + dataset_<task>.npz
```

All Python tools use `glove_serial.py` to auto-detect the USB CDC serial port.
Set `GLOVE_SERIAL_PORT` or pass `--port` where supported to override detection.

### Simulation Mode

Use simulation mode to validate the collector, cleaner, database writer, and
exporter without hardware:

```bash
python data_collector.py --task simulated_task --subject s00 --simulate
```

This generates synthetic raw sensor data, initializes the SQLite schema, writes
the cleaned session into the database, and exports `dataset_all.csv` and
`dataset_all.npz`. Generated outputs are ignored by git.

---

## Demo Visualization

`demo_visualizer.py` is the primary demo tool. It renders a skeletal hand with
VisPy lines and joint markers, using the same quaternion/flex/FSR state shape as
the live glove stream. It supports three input modes:

```bash
# Live glove over USB serial
python demo_visualizer.py

# Replay a recorded session once
python demo_visualizer.py --replay session_20260415_190134.csv

# Replay continuously for unattended demos
python demo_visualizer.py --replay session_20260415_190134.csv --loop

# Synthetic 8-second pick-and-place loop
python demo_visualizer.py --simulate

# Other synthetic task demos
python demo_visualizer.py --simulate --sim-task fold_cloth
python demo_visualizer.py --simulate --sim-task swirl_beaker
python demo_visualizer.py --simulate --sim-task use_drill
```

### Visual Style and Controls

- Skeletal hand: colored line bones plus white disc joint markers
- All five fingers are shown; ring and pinky passively follow the middle finger
- FSR contact pulses fingertip markers red and scales marker size by pressure
- Wrist quaternion rotates the full skeleton as a rigid body
- Camera: left-drag to orbit, scroll to zoom
- Press `C` in live or replay mode to capture the current quaternion as the
  neutral reference pose

HUD fields show the current mode, simulation phase, quaternion, sample rate, and
replay timestamp/total duration where applicable.
The HUD also keeps force visible at all times:

```text
USE_DRILL - TRIGGER_PULL | FSR: Index 0.82  Middle 0.61  Thumb 0.74
```

In simulation mode, a persistent one-line contact-force argument stays on screen
so the demo is legible to an audience without extra narration.

### Replay CSV Support

Replay mode detects supported CSV schemas automatically.

Legacy root-level session CSVs use:

```text
timestamp_ms,qw,qx,qy,qz,
flex_thumb,flex_upper_index,flex_lower_index,flex_upper_middle,flex_lower_middle
```

These map directly into the demo flex controls:

```text
[thumb, index upper, index lower, middle upper, middle lower]
```

New pipeline raw CSVs use:

```text
timestamp_ms,qw,qx,qy,qz,
flex_thumb,flex_index,flex_middle,flex_ring,flex_pinky,
fsr_index,fsr_middle,fsr_thumb,task,subject
```

For the skeletal demo, the single `flex_index` value drives both index segments
and the single `flex_middle` value drives both middle segments:

```text
[flex_thumb, flex_index, flex_index, flex_middle, flex_middle]
```

If FSR columns are missing, replay defaults pressure to `[0.0, 0.0, 0.0]`.
Replay timing is preserved by sleeping between rows according to
`timestamp_ms` deltas. `--loop` restarts after a short pause.

### Synthetic Demo Mode

`python demo_visualizer.py --simulate` defaults to `pick_beaker`, a continuous
8-second pick-and-place loop:

| Phase | Time | Behavior |
|---|---:|---|
| REST | 0.0-0.5s | Hand open, wrist neutral |
| REACHING | 0.5-2.0s | Wrist pitches downward, fingers open |
| PRE-GRASP | 2.0-2.5s | Fingers begin curling, no contact |
| GRASPING | 2.5-3.5s | Fingers curl, FSR contact ramps after flex passes ~0.7 |
| HOLDING | 3.5-4.5s | Stable grasp, wrist rotates toward transport pose |
| RELEASING | 4.5-5.3s | FSR drops before fingers fully open |
| RETRACTING | 5.3-6.3s | Wrist returns to neutral |
| REST | 6.3-8.0s | Open hand holds before the next loop |

Flex and pressure transitions use smoothstep easing. Wrist orientation uses
quaternion SLERP between keyframes.

Additional simulation tasks emphasize why FSRs matter:

| Task | Command | Contact-force argument |
|---|---|---|
| `pick_beaker` | `python demo_visualizer.py --simulate --sim-task pick_beaker` | Contact: fingertip grasp forces vary with object weight |
| `fold_cloth` | `python demo_visualizer.py --simulate --sim-task fold_cloth` | Contact: distributed palm pressure guides fabric smoothing |
| `swirl_beaker` | `python demo_visualizer.py --simulate --sim-task swirl_beaker` | Contact: grip adjusts dynamically to centripetal force |
| `use_drill` | `python demo_visualizer.py --simulate --sim-task use_drill` | Contact: trigger + torque reaction invisible to cameras |

The red contact markers use a lower demo threshold (`0.03`) than the data
pipeline contact threshold so light touch is visible during presentations.
`use_drill` is the clearest force-data demo: the hand shape stays nearly
constant while index trigger force, middle/thumb torque reaction, and small
wrist roll corrections change over time.

### Other Visualization Tools

- `glove_visualization.py` renders the solid robot-hand mesh and remains useful
  for robot-hand-style demos.
- `flex_visualization.py` is a flex-only skeleton debug view.
- `pressure_visualization.py` is a pressure-only FSR debug view.
- `visualize_imu.py` is a legacy all-finger mesh visualizer.

---

## Repository Structure

```text
src/main.cpp              ESP32 firmware
platformio.ini            PlatformIO build configuration
glove_serial.py           Shared serial-port auto-detection
demo_visualizer.py        Primary skeletal demo: live, replay, and simulation
visualize_imu.py          Legacy 3D hand visualization
glove_visualization.py    Main robot-hand visualization
flex_visualization.py     Flex-only visualization/debug tool
pressure_visualization.py Pressure-only visualization/debug tool
data_collector.py         Production recorder plus --simulate full-pipeline mode
data_cleaner.py           Post-processing and calibrated feature generation
db_schema.sql             SQLite schema
db_writer.py              Cleaned CSV -> SQLite
export_dataset.py         SQLite -> training CSV/NPZ
test_imu.py               BNO085 quaternion stream validation
test_flex.py              Flex range validation
test_fsr.py               FSR contact validation
test_mux.py               Optional MUX debug validation
sessions/raw/             Raw collector outputs, ignored except .gitkeep
sessions/cleaned/         Cleaned outputs, ignored except .gitkeep
```

---

## Setup

Install Python dependencies:

```bash
pip install pyserial pandas scipy numpy vispy pyopengl
```

Install PlatformIO via the VS Code extension or:

```bash
pip install platformio
```

Initialize the database when using the pipeline manually:

```bash
sqlite3 glove_dataset.db < db_schema.sql
```

Build firmware:

```bash
platformio run
```

Upload firmware:

```bash
platformio run --target upload
```

---

## Running a Recording Session

Validate hardware first:

```bash
python test_imu.py
python test_flex.py
python test_fsr.py
```

Record a demonstration:

```bash
python data_collector.py --task pick_beaker --subject s01
```

Stop with `Ctrl+C`. The raw CSV is saved under `sessions/raw/`.

Clean the recording:

```bash
python data_cleaner.py sessions/raw/pick_beaker_s01_<timestamp>.csv
```

Write to SQLite:

```bash
sqlite3 glove_dataset.db < db_schema.sql
python db_writer.py sessions/cleaned/pick_beaker_s01_<timestamp>.csv
```

Export training data:

```bash
python export_dataset.py --task pick_beaker
python export_dataset.py --all
```

---

## Calibration

Run flex calibration whenever the glove is adjusted or the subject changes:

1. Run `python test_flex.py`.
2. Capture open-hand and curled-hand readings.
3. Copy the printed calibration values into `FLEX_CALIB` in `data_cleaner.py`.
4. Run `python test_fsr.py` and confirm clean baseline values.

Firmware also contains `FLEX_MIN/FLEX_MAX` and `PRESSURE_IDLE/PRESSURE_PRESS`
constants used for live normalized serial output.

---

## Dataset Schema

### `sessions`

| Column | Type | Description |
|---|---|---|
| `session_id` | INTEGER | Primary key |
| `task` | TEXT | Task label, for example `pick_beaker` |
| `subject` | TEXT | Subject ID, for example `s01` |
| `recorded_at` | TEXT | Ingest timestamp |
| `source_file` | TEXT | Cleaned CSV filename |
| `row_count` | INTEGER | Number of frames |

### `frames`

| Column | Type | Description |
|---|---|---|
| `timestamp_ms` | INTEGER | Time since recording start |
| `qw`, `qx`, `qy`, `qz` | REAL | Wrist quaternion |
| `flex_*` | REAL | Raw normalized flex values |
| `flex_*_deg` | REAL | Calibrated flex angle estimates |
| `fsr_*` | REAL | Raw normalized FSR values |
| `fsr_*_contact` | INTEGER | Binary contact flag |

---

## Future Work

- Synchronize glove data with OMGrab glasses for multimodal episodes.
- Segment episodes automatically from FSR contact events.
- Add BLE wireless streaming after USB collection is validated.
- Store per-subject calibration profiles in the database.
- Export episode-grouped HDF5 for imitation learning frameworks such as LeRobot
  or RLDS.

---

## Team

- Shanthanu - Hardware, firmware, and pipeline
- Dylan Chen - Academic deliverables
- Sadat Uddin - Force sensor implementation

Faculty Liaison: Prof. Gabriel Gomes
Industry Advisors: Antoine Jamme, Hendrik Chiche (OMGrab)
