# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 (WEMOS LOLIN S3 Mini) firmware that reads orientation data from an Adafruit BNO055 IMU over I2C and streams it over USB CDC serial. A companion Python script (`visualize_imu.py`) renders a real-time 3D cube visualization using VisPy.

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

## Python Visualizer

```bash
pip3 install vispy pyserial pyopengl
python3 visualize_imu.py
```

**Close the PlatformIO serial monitor before running** — only one process can hold `/dev/cu.usbmodem101` at a time.

## Hardware

- **Board:** WEMOS LOLIN S3 Mini (ESP32-S3, native USB CDC)
- **IMU:** Adafruit BNO055 at I2C address `0x28`, SDA=GPIO3, SCL=GPIO4
- **Serial port:** `/dev/cu.usbmodem101` at 115200 baud

## Architecture

### Firmware (`src/main.cpp`)

Every 100ms the loop:
1. Reads Euler angles via `bno.getEvent()`
2. Reads quaternion via `bno.getQuat()`
3. Reads calibration status via `bno.getCalibration()`
4. Prints human-readable lines (`Euler |`, `Quat |`, `Calib |`) plus a machine-readable line `Q:w,x,y,z` for the Python parser
5. Auto-saves calibration offsets to EEPROM once all four calibration values reach 3; restores them on next boot

Calibration is stored at EEPROM address 1 with a magic byte `0xAB` at address 0 as a validity flag.

### Custom Board Definition (`boards/lolin_s3_mini.json`)

This overrides the stock PlatformIO board definition. The stock definition includes `-DBOARD_HAS_PSRAM` which causes a boot-loop on the Mini 1 variant (no PSRAM). The custom file omits that flag and adds `-DARDUINO_USB_CDC_ON_BOOT=1` so `Serial` routes to the native USB CDC port.

### Python Visualizer (`visualize_imu.py`)

- Background thread opens serial with `dsrdtr=False, rtscts=False` (prevents DTR/RTS toggling from resetting the ESP32-S3) and auto-reconnects on error
- Parses only `Q:` lines; ignores all other output
- VisPy `app.Timer` at 50ms fires `on_timer()` on the main thread, which recomputes rotated mesh vertices directly via quaternion→rotation matrix math and calls `cube.set_data()` + `canvas.update()`
- Uses `vispy.scene.SceneCanvas` with a `turntable` camera

## Known Quirks

- **Native USB CDC on macOS:** Opening the serial port can trigger `[Errno 6] Device not configured` if DTR/RTS are not suppressed. The visualizer handles this with auto-reconnect.
- **Upload timing:** After flashing via `esp-builtin`, the USB CDC port may need a manual `RST` press to re-enumerate before the serial monitor shows output.
- **`Serial` availability:** The firmware uses `delay(500)` after `Serial.begin()` instead of `while (!Serial)` — the blocking wait prevents non-PlatformIO clients (like the Python script) from receiving data if they connect after boot.
