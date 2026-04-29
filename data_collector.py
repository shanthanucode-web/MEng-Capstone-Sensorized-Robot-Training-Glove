#!/usr/bin/env python3
"""
data_collector.py - Production glove recording script.

Output: sessions/raw/<task>_<subject>_<YYYYMMDD_HHMMSS>.csv

Usage:
    python data_collector.py --task pick_beaker --subject s01
    python data_collector.py --task pick_beaker --subject s01 --simulate
"""

import argparse
import csv
import math
import os
import random
import sqlite3
import threading
import time
from datetime import datetime

import serial

from glove_serial import open_glove_serial


COLUMNS = [
    'timestamp_ms',
    'qw', 'qx', 'qy', 'qz',
    'flex_thumb', 'flex_index', 'flex_middle', 'flex_ring', 'flex_pinky',
    'fsr_index', 'fsr_middle', 'fsr_thumb',
    'task', 'subject',
]


def parse_args():
    parser = argparse.ArgumentParser(description='Record a glove demonstration session.')
    parser.add_argument('--port', default=None, help='Serial port override')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--task', required=True, help='Task label, e.g. pick_beaker')
    parser.add_argument('--subject', required=True, help='Subject ID, e.g. s01')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Max duration in seconds (0 = manual stop)')
    parser.add_argument('--simulate', action='store_true',
                        help='Generate synthetic data and run cleaner -> DB -> export without hardware')
    parser.add_argument('--simulate-frames', type=int, default=250,
                        help='Number of synthetic frames to generate with --simulate')
    return parser.parse_args()


def _output_path(task, subject):
    os.makedirs('sessions/raw', exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return f"sessions/raw/{task}_{subject}_{timestamp}.csv"


def _write_row(writer, timestamp_ms, quat, flex, fsr, task, subject):
    writer.writerow({
        'timestamp_ms': timestamp_ms,
        'qw': quat[0],
        'qx': quat[1],
        'qy': quat[2],
        'qz': quat[3],
        'flex_thumb': flex[0],
        'flex_index': flex[1],
        'flex_middle': flex[2],
        'flex_ring': flex[3],
        'flex_pinky': flex[4],
        'fsr_index': fsr[0],
        'fsr_middle': fsr[1],
        'fsr_thumb': fsr[2],
        'task': task,
        'subject': subject,
    })


def _parse_floats(line, prefix, expected):
    if not line.startswith(prefix):
        return None
    parts = line[len(prefix):].split(',')
    if len(parts) != expected:
        return None
    try:
        return [float(part) for part in parts]
    except ValueError:
        return None


def record_serial(args):
    outfile = _output_path(args.task, args.subject)
    state = {
        'quat': [1.0, 0.0, 0.0, 0.0],
        'flex': [0.0] * 5,
        'fsr': [0.0] * 3,
        'dirty': False,
    }
    lock = threading.Lock()
    running = threading.Event()
    running.set()

    def serial_thread():
        while running.is_set():
            try:
                if args.port:
                    ser = serial.Serial(args.port, args.baud, timeout=1,
                                        dsrdtr=False, rtscts=False)
                    ser.dtr = False
                    ser.rts = False
                    port = args.port
                else:
                    ser, port = open_glove_serial(args.baud, timeout=1)
                print(f"[serial] connected: {port}")

                with ser:
                    while running.is_set():
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        quat = _parse_floats(line, 'Q:', 4)
                        flex = _parse_floats(line, 'F:', 5)
                        fsr = _parse_floats(line, 'P:', 3)
                        with lock:
                            if quat is not None:
                                state['quat'] = quat
                            elif flex is not None:
                                state['flex'] = flex
                            elif fsr is not None:
                                state['fsr'] = fsr
                                state['dirty'] = True
            except serial.SerialException as exc:
                if running.is_set():
                    print(f"[serial] {exc} - retrying in 2s...")
                    time.sleep(2)
            except Exception as exc:
                if running.is_set():
                    print(f"[serial] unexpected error: {exc} - retrying in 2s...")
                    time.sleep(2)

    thread = threading.Thread(target=serial_thread, daemon=True)
    thread.start()

    print(f"\nRecording: task={args.task}  subject={args.subject}")
    print(f"Output:    {outfile}")
    print(f"Duration:  {args.duration}s" if args.duration > 0 else "Duration:  manual (Ctrl+C to stop)")

    start = time.time()
    row_count = 0
    try:
        with open(outfile, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS)
            writer.writeheader()
            while True:
                elapsed = time.time() - start
                if args.duration > 0 and elapsed >= args.duration:
                    break

                with lock:
                    if not state['dirty']:
                        should_write = False
                    else:
                        quat = list(state['quat'])
                        flex = list(state['flex'])
                        fsr = list(state['fsr'])
                        state['dirty'] = False
                        should_write = True

                if should_write:
                    _write_row(writer, int(elapsed * 1000), quat, flex, fsr,
                               args.task, args.subject)
                    row_count += 1
                    if row_count % 100 == 0:
                        print(f"  {row_count} frames ({elapsed:.1f}s)")
                else:
                    time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        running.clear()

    elapsed = max(time.time() - start, 1e-6)
    print(f"\nStopped. {row_count} frames in {elapsed:.1f}s ({row_count / elapsed:.1f} fps)")
    print(f"Saved to: {outfile}")
    return outfile


def _synthetic_frame(i, total):
    t = i / max(total - 1, 1)
    angle = 0.35 * math.sin(2.0 * math.pi * t)
    qw = math.cos(angle / 2.0)
    qz = math.sin(angle / 2.0)
    phase = 0.5 - 0.5 * math.cos(2.0 * math.pi * t)
    rng = random.Random(i)

    flex = [
        max(0.0, min(1.0, 0.10 + 0.45 * phase + rng.uniform(-0.015, 0.015))),
        max(0.0, min(1.0, 0.12 + 0.62 * phase + rng.uniform(-0.015, 0.015))),
        max(0.0, min(1.0, 0.10 + 0.66 * phase + rng.uniform(-0.015, 0.015))),
        max(0.0, min(1.0, 0.08 + 0.58 * phase + rng.uniform(-0.015, 0.015))),
        max(0.0, min(1.0, 0.08 + 0.50 * phase + rng.uniform(-0.015, 0.015))),
    ]
    contact = 1.0 if 0.35 <= t <= 0.78 else 0.0
    fsr = [
        max(0.0, min(1.0, 0.01 + 0.55 * contact * phase + rng.uniform(0, 0.01))),
        max(0.0, min(1.0, 0.01 + 0.48 * contact * phase + rng.uniform(0, 0.01))),
        max(0.0, min(1.0, 0.01 + 0.35 * contact * phase + rng.uniform(0, 0.01))),
    ]
    return [qw, 0.0, 0.0, qz], flex, fsr


def _init_db(db_path='glove_dataset.db'):
    with open('db_schema.sql', 'r', encoding='utf-8') as handle:
        schema = handle.read()
    conn = sqlite3.connect(db_path)
    try:
        conn.executescript(schema)
        conn.commit()
    finally:
        conn.close()


def run_simulation(args):
    outfile = _output_path(args.task, args.subject)
    frames = max(args.simulate_frames, 1)
    print(f"\n[simulate] Writing synthetic raw session: {outfile}")

    with open(outfile, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        for i in range(frames):
            quat, flex, fsr = _synthetic_frame(i, frames)
            _write_row(writer, i * 100, quat, flex, fsr, args.task, args.subject)

    print("[simulate] Initializing database schema")
    _init_db()

    print("[simulate] Running cleaner")
    from data_cleaner import clean
    cleaned = clean(outfile)

    print("[simulate] Writing database")
    from db_writer import write
    write(cleaned)

    print("[simulate] Exporting dataset")
    from export_dataset import export
    export(None)

    print("[simulate] Full pipeline completed")
    return outfile


def main():
    args = parse_args()
    if args.simulate:
        run_simulation(args)
    else:
        record_serial(args)


if __name__ == '__main__':
    main()
