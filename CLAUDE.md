# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3-N8R8 firmware for a sensorized robot training glove. Reads orientation from an Adafruit BNO085 IMU and curl from 5 resistive flex sensors, streams both over USB CDC serial at 100ms intervals. A suite of Python tools handles real-time 3D visualization, session recording, and flex calibration.

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

**Close the PlatformIO serial monitor before running any Python tool** — only one process can hold `/dev/cu.usbmodem101` at a time.

| Script | Purpose | Command |
|---|---|---|
| `glove_visualization.py` | Full real-time visualizer: 3D skeletal hand + flex bars + HUD | `python3 glove_visualization.py` |
| `flex_visualization.py` | Flex-only test tool: fixed camera, finger curl only, no IMU | `python3 flex_visualization.py` |
| `record_session.py` | Record quaternion + flex to timestamped CSV | `python3 record_session.py` |
| `calibrate_flex.py` | Analyze recorded CSV and recommend new FLEX_MIN/FLEX_MAX | `python3 calibrate_flex.py` |
| `visualize_imu.py` | Legacy cube visualizer (IMU only, no flex) | `python3 visualize_imu.py` |

## Hardware

- **Board:** ESP32-S3-N8R8 (`esp32-s3-devkitc-1`), 8MB flash, native USB CDC
- **IMU:** Adafruit BNO085 at I2C address `0x4A`, SDA=GPIO8, SCL=GPIO9
- **Flex sensors:** 5x resistive sensors on ADC1 — GPIO1 (thumb), GPIO2 (upper index), GPIO4 (lower index), GPIO5 (upper middle), GPIO6 (lower middle)
- **Serial port:** `/dev/cu.usbmodem101` at 115200 baud
- **Note:** GPIO3 is skipped (strapping pin — causes boot issues if pulled low). Thumb sensor is currently disconnected.

## Architecture

### Firmware (`src/main.cpp`)

Every 100ms the loop:
1. Drains the BNO085 FIFO via `getSensorEvent()` in a while loop, keeping the freshest `SH2_GAME_ROTATION_VECTOR` quaternion
2. Reads all 5 flex sensors via `analogRead()` at 12-bit resolution
3. Normalizes flex values to [0.0=open, 1.0=closed] using per-sensor `FLEX_MIN`/`FLEX_MAX` constants
4. Prints two machine-readable lines: `Q:w,x,y,z` and `F:t,ui,li,um,lm`
5. Prints human-readable debug lines for serial monitor

**Why Game Rotation Vector (not Rotation Vector)?** Game Rotation Vector fuses only gyro + accel, avoiding magnetometer sensitivity to nearby motors and metal. Heading drifts slowly over minutes but is negligible for hand-pose capture sessions lasting seconds to tens of seconds.

**Flex calibration constants** — tune `FLEX_MIN` (finger open) and `FLEX_MAX` (finger closed) per sensor by recording a session and running `calibrate_flex.py`.

### Serial Protocol

Two machine-readable lines are emitted each 100ms frame:

```
Q:w,x,y,z          — BNO085 Game Rotation Vector quaternion (float, 4 decimal places)
F:t,ui,li,um,lm    — normalized flex 0.0=open..1.0=closed (float, 3 decimal places)
```

Both the visualizer and recorder parse only these prefixed lines and ignore all human-readable output.

### Python Visualizer (`glove_visualization.py`)

- Background serial thread parses `Q:` and `F:` lines, updates shared state protected by a lock
- VisPy `app.Timer` at 30fps calls `on_timer()` on the main thread
- `compute_skeleton()` builds all bone endpoints and joint positions using chained rotation matrices (splay → upper curl → lower curl), then applies the IMU rotation matrix to the whole hand
- 2D flex panel rendered in a separate VisPy view with `PanZoomCamera`
- Press `C` to capture the current orientation as the neutral/calibrated reference pose

### Flex Calibration Workflow

1. `python3 record_session.py` — record a session with full range of motion
2. `python3 calibrate_flex.py session_YYYYMMDD_HHMMSS.csv` — get recommended constants
3. Paste the printed `FLEX_MIN`/`FLEX_MAX` lines into `src/main.cpp` and reflash

## Known Quirks

- **Native USB CDC on macOS:** Opening the serial port can trigger `[Errno 6] Device not configured` if DTR/RTS are not suppressed. All Python tools use `dsrdtr=False, rtscts=False` and auto-reconnect.
- **Upload timing:** After flashing via `esp-builtin`, the USB CDC port may need a manual `RST` press to re-enumerate before the serial monitor shows output.
- **`Serial` availability:** The firmware uses `delay(500)` + `while (!Serial)` after `Serial.begin()`. The blocking wait ensures USB CDC is ready before data is sent.
- **Thumb sensor:** Currently disconnected. The firmware reads GPIO1 but the value is unreliable. All tools render the thumb statically and label it "DISCONNECTED".

## Known Bugs & TODOs

- **Curl direction mismatch:** `glove_visualization.py` uses `_rx(-fu * MAX_CURL)` (negative sign, curls toward viewer) while `flex_visualization.py` uses `_rx(+fu * MAX_CURL)` (positive sign, curls away). The two tools curl in opposite directions. `glove_visualization.py` is the canonical behavior — fix `flex_visualization.py` to use the negative sign.
- **Flex calibration constants not per-sensor:** All 5 sensors currently share identical `FLEX_MIN=2800, FLEX_MAX=3700`. Per-sensor tuning via the calibration workflow has not been done yet. The thumb is excluded from calibration (disconnected).
- **`calibrate_flex.py` requires `CURRENT_FLEX_MIN/MAX` to match `main.cpp`:** If the firmware constants are updated without updating the script, the back-calculated ADC values will be wrong. Keep them in sync manually.
- **Ring and pinky fingers not rendered:** `glove_visualization.py` and `flex_visualization.py` only model thumb, index, and middle fingers. `visualize_imu.py` (legacy) renders all 5 but uses a different geometry model (solid box mesh vs. wire skeleton).
- **`visualize_imu.py` palm build has dead code:** `parts.pop()` on line 238 discards the first palm attempt before rebuilding it manually — harmless but worth cleaning up if the file is ever revived.
