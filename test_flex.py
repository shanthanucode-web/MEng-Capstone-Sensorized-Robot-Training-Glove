#!/usr/bin/env python3
"""
test_flex.py - Validate flex sensor range per finger.

Usage:
    python test_flex.py
"""

import argparse

import serial

from glove_serial import open_glove_serial


NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
COLS = ["flex_thumb", "flex_index", "flex_middle", "flex_ring", "flex_pinky"]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None, help='Serial port override')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--samples', type=int, default=30)
    return parser.parse_args()


def read_flex_average(ser, n):
    samples = []
    while len(samples) < n:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line.startswith('F:'):
            continue
        parts = line[2:].split(',')
        if len(parts) != 5:
            continue
        try:
            samples.append([float(part) for part in parts])
        except ValueError:
            pass
    return [sum(sample[i] for sample in samples) / n for i in range(5)]


def main():
    args = parse_args()
    if args.port:
        ser = serial.Serial(args.port, args.baud, timeout=1, dsrdtr=False, rtscts=False)
        ser.dtr = False
        ser.rts = False
        port = args.port
    else:
        ser, port = open_glove_serial(args.baud, timeout=1)

    with ser:
        print(f"=== FLEX SENSOR TEST ({port}) ===\n")
        print("Hold hand fully OPEN and flat. Do not curl any fingers.")
        input("Press Enter when ready...")
        open_vals = read_flex_average(ser, args.samples)
        print(f"Open readings:   {[f'{v:.3f}' for v in open_vals]}\n")

        print("Now curl ALL fingers as tightly as possible.")
        input("Press Enter when ready...")
        curl_vals = read_flex_average(ser, args.samples)
        print(f"Curled readings: {[f'{v:.3f}' for v in curl_vals]}\n")

    print("=== RESULTS ===")
    all_pass = True
    for i, name in enumerate(NAMES):
        delta = curl_vals[i] - open_vals[i]
        passed = open_vals[i] < 0.30 and curl_vals[i] > 0.65 and delta > 0.35
        all_pass = all_pass and passed
        print(
            f"  {name:8s}: open={open_vals[i]:.3f} curl={curl_vals[i]:.3f} "
            f"delta={delta:.3f} {'PASS' if passed else 'FAIL'}"
        )

    print(f"\nOverall: {'PASS' if all_pass else 'FAIL - check wiring/calibration'}")
    print("\nCopy these values into FLEX_CALIB in data_cleaner.py:")
    for col, open_val, curl_val in zip(COLS, open_vals, curl_vals):
        print(f"  '{col}': ({open_val:.3f}, {curl_val:.3f}),")


if __name__ == '__main__':
    main()
