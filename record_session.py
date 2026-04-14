#!/usr/bin/env python3
"""
record_session.py — Data Glove Session Recorder
================================================
Records IMU quaternion + flex sensor data from the ESP32 glove to a timestamped CSV.

Dependencies:
    pip install pyserial pandas

Usage:
    python3 record_session.py
    - Press Enter to start recording
    - Press Ctrl+C to stop and save

Output CSV columns:
    timestamp_ms          — milliseconds since recording started
    qw, qx, qy, qz        — quaternion from BNO085 Game Rotation Vector
    flex_thumb            — normalized thumb curl (0=open, 1=closed)
    flex_upper_index      — upper index finger segment
    flex_lower_index      — lower index finger segment
    flex_upper_middle     — upper middle finger segment
    flex_lower_middle     — lower middle finger segment

Notes:
    - Close PlatformIO serial monitor before running (only one process can
      hold /dev/cu.usbmodem101 at a time)
    - One CSV row is written per F: line received (~10 samples/sec at 100ms)
    - Q: always arrives just before F: in the firmware loop, so each row
      pairs the freshest quaternion with the flex reading
"""

import csv
import sys
import time
import threading
import serial
from datetime import datetime

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SERIAL_PORT = "/dev/cu.usbmodem101"
BAUD_RATE   = 115200

CSV_COLUMNS = [
    "timestamp_ms",
    "qw", "qx", "qy", "qz",
    "flex_thumb", "flex_upper_index", "flex_lower_index",
    "flex_upper_middle", "flex_lower_middle",
]

# How often to print live feedback (every N samples)
FEEDBACK_INTERVAL = 5

# ---------------------------------------------------------------------------
# Shared state — written by serial thread, read by main thread
# ---------------------------------------------------------------------------
latest_q     = [1.0, 0.0, 0.0, 0.0]  # [qw, qx, qy, qz]
state_lock   = threading.Lock()
recording    = False       # set True after user presses Enter
start_time_s = None        # monotonic clock time when recording began
sample_count = 0           # total rows written to CSV
csv_writer   = None        # csv.writer instance (set before recording starts)
csv_file_obj = None        # file handle (kept open until stop)
stop_event   = threading.Event()


# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------
def read_serial():
    """
    Runs in background. Connects to the glove serial port and parses lines.

    Q: lines update latest_q (quaternion) — these arrive just before F: lines
       in the firmware loop, so latest_q is always fresh when F: is processed.

    F: lines trigger a CSV row write (when recording is active). Each row
       combines the current timestamp, latest quaternion, and the flex values
       from this F: line.
    """
    global latest_q, recording, start_time_s, sample_count, csv_writer

    while not stop_event.is_set():
        try:
            # dsrdtr/rtscts=False prevents DTR/RTS toggling from resetting ESP32
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1,
                                dsrdtr=False, rtscts=False)
            ser.dtr = False
            ser.rts = False
            print(f"[serial] connected to {SERIAL_PORT}")

            while not stop_event.is_set():
                line = ser.readline().decode("utf-8", errors="ignore").strip()

                # --- Parse quaternion line: Q:w,x,y,z ---
                if line.startswith("Q:"):
                    parts = line[2:].split(",")
                    if len(parts) == 4:
                        try:
                            vals = [float(p) for p in parts]
                            with state_lock:
                                latest_q = vals
                        except ValueError:
                            pass  # malformed line, skip

                # --- Parse flex line: F:t,ui,li,um,lm ---
                elif line.startswith("F:"):
                    parts = line[2:].split(",")
                    if len(parts) != 5:
                        continue
                    try:
                        flex = [float(p) for p in parts]
                    except ValueError:
                        continue  # malformed line, skip

                    # Only record if the user has pressed Enter
                    if not recording:
                        continue

                    # Snapshot timestamp and quaternion atomically
                    with state_lock:
                        q = latest_q[:]
                        ts_ms = int((time.monotonic() - start_time_s) * 1000)

                    # Write one CSV row: timestamp + quaternion + flex values
                    csv_writer.writerow([ts_ms] + q + flex)
                    sample_count += 1

                    # Print live feedback so the user knows recording is active
                    if sample_count % FEEDBACK_INTERVAL == 0:
                        print(
                            f"\r  [{ts_ms / 1000:6.1f}s]  samples={sample_count:5d}  "
                            f"Q=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})  "
                            f"Flex=({flex[0]:.2f},{flex[1]:.2f},"
                            f"{flex[2]:.2f},{flex[3]:.2f},{flex[4]:.2f})",
                            end="",
                            flush=True,
                        )

        except serial.SerialException as e:
            if not stop_event.is_set():
                print(f"\n[serial] connection error: {e} — retrying in 2s...")
                time.sleep(2)
        except Exception as e:
            if not stop_event.is_set():
                print(f"\n[serial] unexpected error: {e} — retrying in 2s...")
                time.sleep(2)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    global recording, start_time_s, csv_writer, csv_file_obj

    print()
    print("=" * 62)
    print("  Data Glove Session Recorder")
    print("=" * 62)
    print(f"  Port : {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"  Cols : {', '.join(CSV_COLUMNS)}")
    print()
    print("  Make sure the PlatformIO serial monitor is closed.")
    print("  Press Enter to start recording, Ctrl+C to stop and save.")
    print("=" * 62)
    print()

    # Start serial reader in background — it connects and waits for recording flag
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()

    # Block until user presses Enter
    input("  >> Press Enter to start recording... ")
    print()

    # Open the CSV file and write the header row
    filename = datetime.now().strftime("session_%Y%m%d_%H%M%S.csv")
    csv_file_obj = open(filename, "w", newline="")
    csv_writer = csv.writer(csv_file_obj)
    csv_writer.writerow(CSV_COLUMNS)

    # Start the clock and flip the recording flag
    start_time_s = time.monotonic()
    recording = True
    print(f"  [record] Recording to {filename}")
    print("  [record] Press Ctrl+C to stop.\n")

    # Main thread just keeps the process alive until Ctrl+C
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    # ---------------------------------------------------------------------------
    # Shutdown: stop serial thread, flush and close CSV, print summary
    # ---------------------------------------------------------------------------
    recording = False
    stop_event.set()

    duration_s = time.monotonic() - start_time_s

    # Flush and close the CSV so all data is written to disk
    csv_file_obj.flush()
    csv_file_obj.close()

    avg_rate = sample_count / duration_s if duration_s > 0 else 0

    print(f"\n\n{'=' * 62}")
    print("  Recording complete.")
    print(f"  Samples recorded : {sample_count}")
    print(f"  Duration         : {duration_s:.1f}s")
    print(f"  Average rate     : {avg_rate:.1f} samples/sec")
    print(f"  File saved       : {filename}")
    print(f"{'=' * 62}\n")


if __name__ == "__main__":
    main()
