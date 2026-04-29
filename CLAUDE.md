# CLAUDE.md

## Project Overview

ESP32-S3-N8R8 firmware that reads wrist orientation from a BNO085 IMU and
finger/contact data from 5 flex sensors and 3 FSRs via a CD4051BE 8-channel
analog MUX. Data streams over USB CDC serial to Python tools that record, clean,
store, and export demonstrations for robot manipulation training.

## Hardware

- Board: ESP32-S3-N8R8, 8MB flash, OPI PSRAM
- IMU: Adafruit BNO085, SDA=GPIO8, SCL=GPIO9, detected at 0x4A or 0x4B
- MUX: CD4051BE, select pins GPIO5/6/7, signal pin GPIO4
- Flex sensors: 5 channels in serial order `thumb, upper index, lower index, upper middle, lower middle`
- Current flex MUX map: Y0, Y1, Y2, Y4, Y3; Y3/Y4 are swapped to match glove wiring
- FSRs: MUX channels Y5-Y7, emitted as `P:p1,p2,p3`
- Serial: USB CDC at 115200 baud; Python tools auto-detect the port via `glove_serial.py`

## Build & Upload

    platformio run
    platformio run --target upload

Upload uses `esp-builtin` over the ESP32-S3 built-in JTAG. `OCD_ID` warnings
during upload are harmless.

## IMU Notes

Firmware uses `SH2_GAME_ROTATION_VECTOR`, not `SH2_ROTATION_VECTOR`. This avoids
magnetometer dependency and is more stable around metal, motors, and robot lab
hardware. The BNO085 performs onboard fusion; the ESP32 does not apply extra IMU
calibration.

## Serial Protocol

Boot/debug lines may be printed during setup. After `SESSION_START`, production
tools parse only these machine-readable lines:

    Q:w,x,y,z            - quaternion
    F:t,ui,li,um,lm      - calibrated flex values, 0.0=open to 1.0=closed
    P:p1,p2,p3           - calibrated FSR values, 0.0=no contact to 1.0=firm press

The firmware intentionally keeps the current slower loop cadence with
`delay(100)`.

## Python Pipeline

    pip install pyserial pandas scipy numpy vispy pyopengl

    python data_collector.py --task <task_name> --subject <subject_id>
    python data_cleaner.py sessions/raw/<file>.csv
    sqlite3 glove_dataset.db < db_schema.sql
    python db_writer.py sessions/cleaned/<file>.csv
    python export_dataset.py --all

Use the simulator to validate the full pipeline without hardware:

    python data_collector.py --task simulated_task --subject s00 --simulate

Generated databases, session CSVs, and dataset exports are local artifacts and
are ignored by git.

## Demo Visualizer

`demo_visualizer.py` is the primary visual demo. It uses a skeletal VisPy hand
style based on `flex_visualization.py`, not the solid robot mesh from
`glove_visualization.py`.

Launch modes:

    python demo_visualizer.py
    python demo_visualizer.py --replay session_20260415_190134.csv
    python demo_visualizer.py --replay session_20260415_190134.csv --loop
    python demo_visualizer.py --simulate

Modes:

- Live mode opens the glove serial port with `glove_serial.open_glove_serial()`
  and parses `Q:`, `F:`, and `P:` lines.
- Replay mode reads CSV rows in timestamp order, sleeps by `timestamp_ms`
  deltas, and can loop continuously with `--loop`.
- Simulate mode generates an 8-second pick-and-place loop with smooth flex/FSR
  transitions and quaternion SLERP.

Replay schemas:

- Legacy CSVs map
  `flex_thumb, flex_upper_index, flex_lower_index, flex_upper_middle, flex_lower_middle`
  directly to the 5 demo flex controls.
- New pipeline CSVs map `flex_index` to both index controls and `flex_middle`
  to both middle controls:
  `[flex_thumb, flex_index, flex_index, flex_middle, flex_middle]`.
- Missing FSR columns default to `[0.0, 0.0, 0.0]`.

Skeleton behavior:

- Palm and wrist are line segments.
- Thumb has two segments; all other fingers have MCP/PIP/DIP chains.
- Ring follows middle at 90%; pinky follows middle at 78%.
- FSR contact above 0.08 pulses index, middle, and thumb tips red.
- Press `C` in live/replay mode to recapture the neutral quaternion.

## Validation Tools

    python test_imu.py
    python test_flex.py
    python test_fsr.py
    python test_mux.py

Demo validation:

    python3 -m py_compile demo_visualizer.py
    python demo_visualizer.py --replay session_20260415_190134.csv --loop
    python demo_visualizer.py --simulate

`test_mux.py` requires a temporary firmware debug command and should not be left
enabled in production firmware.
