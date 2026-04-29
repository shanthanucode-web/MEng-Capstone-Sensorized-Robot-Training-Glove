# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3-N8R8 firmware for a sensorized robot training glove. Reads orientation from an Adafruit BNO085 IMU, curl from 5 resistive flex sensors, and pressure from 3 FSR sensors, then streams all values over USB CDC serial at 100ms intervals. A suite of Python tools handles real-time 3D visualization, session recording, and flex calibration.

## Build & Upload

```bash
# Build
platformio run

# Upload (uses esp-builtin JTAG — no button presses needed)
platformio run --target upload

# Serial monitor
platformio device monitor
```

The upload uses `esp-builtin` (OpenOCD over the ESP32-S3's built-in USB JTAG). The warnings `Unexpected OCD_ID` during upload are harmless — verify succeeds regardless.

## Python Tools

```bash
pip3 install vispy pyserial numpy pyopengl pandas
```

**Close the PlatformIO serial monitor before running any Python tool** — only one process can hold the auto-detected USB CDC port at a time.

| Script | Purpose | Command |
|---|---|---|
| `glove_visualization.py` | Full real-time visualizer: 3D robot hand + flex/pressure bars + HUD | `python3 glove_visualization.py` |
| `flex_visualization.py` | Flex-only test tool: fixed camera, finger curl only, no IMU | `python3 flex_visualization.py` |
| `pressure_visualization.py` | Pressure-only test tool for the 3 FSR channels | `python3 pressure_visualization.py` |
| `record_session.py` | Record quaternion + flex + pressure to timestamped CSV | `python3 record_session.py` |
| `calibrate_flex.py` | Analyze recorded CSV and recommend new FLEX_MIN/FLEX_MAX | `python3 calibrate_flex.py` |
| `visualize_imu.py` | Legacy cube visualizer (IMU only, no flex) | `python3 visualize_imu.py` |

## Hardware

- **Board:** ESP32-S3-N8R8 (`esp32-s3-devkitc-1`), 8MB flash, native USB CDC
- **IMU:** Adafruit BNO085 at I2C address `0x4A`, SDA=GPIO8, SCL=GPIO9
- **Analog sensors:** 5x flex sensors + 3x FSR pressure sensors through a CD4051BE mux — GPIO4 reads mux Z/common output, GPIO5/6/7 drive mux A/B/C select lines
- **Serial port:** auto-detected `/dev/cu.usbmodem*` or `/dev/cu.usbserial*` at 115200 baud (`GLOVE_SERIAL_PORT` overrides)
- **Mux channel order:** `F:t,ui,li,um,lm` is read from mux channels Y0, Y1, Y2, Y4, Y3. Y3/Y4 are intentionally swapped in code because upper/lower middle are physically swapped on the glove.
- **Pressure channel order:** `P:p1,p2,p3` is read from mux channels Y5, Y6, Y7.

## Architecture

### Firmware (`src/main.cpp`)

Every 100ms the loop:
1. Drains the BNO085 FIFO via `getSensorEvent()` in a while loop, keeping the freshest `SH2_GAME_ROTATION_VECTOR` quaternion
2. Reads all 5 flex sensors and 3 pressure sensors by selecting CD4051BE mux channels and sampling GPIO4 at 12-bit resolution
3. Normalizes flex values to [0.0=open, 1.0=closed] using per-sensor `FLEX_MIN`/`FLEX_MAX` constants
4. Normalizes pressure values to [0.0=no pressure, 1.0=firm press] using inverted per-sensor `PRESSURE_IDLE`/`PRESSURE_PRESS` constants
5. Prints three machine-readable lines: `Q:w,x,y,z`, `F:t,ui,li,um,lm`, and `P:p1,p2,p3`
6. Prints human-readable debug lines for serial monitor

**Why Game Rotation Vector (not Rotation Vector)?** Game Rotation Vector fuses only gyro + accel, avoiding magnetometer sensitivity to nearby motors and metal. Heading drifts slowly over minutes but is negligible for hand-pose capture sessions lasting seconds to tens of seconds.

**Flex calibration constants** — tune `FLEX_MIN` (finger open) and `FLEX_MAX` (finger closed) per sensor by recording a session and running `calibrate_flex.py`.

**Pressure calibration constants** — tune `PRESSURE_IDLE` (no contact) and `PRESSURE_PRESS` (firm intended contact) per FSR by watching the raw `Press | P1:raw(norm)` debug output in the serial monitor.

### Serial Protocol

Three machine-readable lines are emitted each 100ms frame:

```
Q:w,x,y,z          — BNO085 Game Rotation Vector quaternion (float, 4 decimal places)
F:t,ui,li,um,lm    — normalized flex 0.0=open..1.0=closed (float, 3 decimal places)
P:p1,p2,p3         — normalized pressure 0.0=none..1.0=firm press (float, 3 decimal places)
```

The visualizer and recorder parse only these prefixed lines and ignore all human-readable output.

### Python Visualizer (`glove_visualization.py`)

- Background serial thread parses `Q:`, `F:`, and `P:` lines, updates shared state protected by a lock
- VisPy `app.Timer` at 30fps calls `on_timer()` on the main thread
- `compute_hand_pose()` calculates live thumb/index/middle joints plus passive ring/pinky joints
- `build_hand_mesh()` builds the robot hand mesh, adds pressure pads, then applies the IMU rotation matrix to the whole hand
- 2D flex/pressure panel rendered in a separate VisPy view with `PanZoomCamera`
- Press `C` to capture the current orientation as the neutral/calibrated reference pose

### Flex Calibration Workflow

1. `python3 record_session.py` — record a session with full range of motion
2. `python3 calibrate_flex.py session_YYYYMMDD_HHMMSS.csv` — get recommended constants
3. Paste the printed `FLEX_MIN`/`FLEX_MAX` lines into `src/main.cpp` and reflash

## Known Quirks

- **Native USB CDC on macOS:** Opening the serial port can trigger `[Errno 6] Device not configured` if DTR/RTS are not suppressed. All Python tools use `dsrdtr=False, rtscts=False` and auto-reconnect.
- **Upload timing:** After flashing via `esp-builtin`, the USB CDC port may need a manual `RST` press to re-enumerate before the serial monitor shows output.
- **`Serial` availability:** The firmware uses `delay(500)` + `while (!Serial)` after `Serial.begin()`. The blocking wait ensures USB CDC is ready before data is sent.
## Known Bugs & TODOs

- **Flex calibration constants need final tuning:** Per-sensor constants are present, but should be refined once the wiring is mechanically stable and each raw ADC channel moves smoothly through its expected range.
- **`calibrate_flex.py` requires `CURRENT_FLEX_MIN/MAX` to match `main.cpp`:** If the firmware constants are updated without updating the script, the back-calculated ADC values will be wrong. Keep them in sync manually.
- **Pressure calibration constants need final tuning:** `PRESSURE_IDLE/PRESSURE_PRESS` are placeholder-safe starting values until real no-contact and firm-contact raw readings are captured.
- **Ring and pinky are passive in the main visualizer:** `glove_visualization.py` renders ring and pinky as relaxed passive fingers because there are no live sensors for them.
- **`visualize_imu.py` palm build has dead code:** `parts.pop()` on line 238 discards the first palm attempt before rebuilding it manually — harmless but worth cleaning up if the file is ever revived.
