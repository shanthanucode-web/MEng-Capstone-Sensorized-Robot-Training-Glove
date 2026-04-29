#!/usr/bin/env python3
"""
test_imu.py - Validate BNO085 quaternion stream.

Pass criteria:
  - Receives Q: lines at sustained rate >= 8 Hz
  - Every quaternion norm is in [0.95, 1.05]
  - Zero parse errors over the test duration

Usage:
    python test_imu.py
"""

import argparse
import math
import time

import serial

from glove_serial import open_glove_serial


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None, help='Serial port override')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--min-rate', type=float, default=8.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.port:
        ser = serial.Serial(args.port, args.baud, timeout=1, dsrdtr=False, rtscts=False)
        ser.dtr = False
        ser.rts = False
        port = args.port
    else:
        ser, port = open_glove_serial(args.baud, timeout=1)

    received, errors, norm_warnings = 0, 0, 0
    start = time.time()
    print(f"Running IMU test on {port} for {args.duration:.1f}s...")

    with ser:
        while time.time() - start < args.duration:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('Q:'):
                continue
            parts = line[2:].split(',')
            if len(parts) != 4:
                errors += 1
                continue
            try:
                w, x, y, z = [float(part) for part in parts]
            except ValueError:
                errors += 1
                continue
            norm = math.sqrt(w*w + x*x + y*y + z*z)
            received += 1
            if not (0.95 <= norm <= 1.05):
                print(f"  [WARN] abnormal norm: {norm:.4f} raw={line}")
                norm_warnings += 1

    elapsed = time.time() - start
    rate = received / elapsed if elapsed else 0.0
    passed = rate >= args.min_rate and errors == 0 and norm_warnings == 0

    print("\n=== IMU TEST RESULTS ===")
    print(f"Duration:       {elapsed:.1f}s")
    print(f"Packets rx:     {received}")
    print(f"Rate:           {rate:.1f} Hz")
    print(f"Parse errors:   {errors}")
    print(f"Norm warnings:  {norm_warnings}")
    print(f"\nResult: {'PASS' if passed else 'FAIL'}")


if __name__ == '__main__':
    main()
