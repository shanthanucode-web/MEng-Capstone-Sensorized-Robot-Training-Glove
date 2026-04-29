#!/usr/bin/env python3
"""
test_fsr.py - Validate FSR contact response per fingertip.

Usage:
    python test_fsr.py
"""

import argparse

import serial

from glove_serial import open_glove_serial


NAMES = ["Index", "Middle", "Thumb"]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None, help='Serial port override')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--baseline-threshold', type=float, default=0.05)
    parser.add_argument('--press-threshold', type=float, default=0.60)
    return parser.parse_args()


def read_fsr_average(ser, n):
    samples = []
    while len(samples) < n:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line.startswith('P:'):
            continue
        parts = line[2:].split(',')
        if len(parts) != 3:
            continue
        try:
            samples.append([float(part) for part in parts])
        except ValueError:
            pass
    return [sum(sample[i] for sample in samples) / n for i in range(3)]


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
        print(f"=== FSR TEST ({port}) ===\n")
        print("Do NOT touch any fingertips.")
        input("Press Enter for baseline reading...")
        baseline = read_fsr_average(ser, args.samples)
        print(f"Baseline: {[f'{v:.3f}' for v in baseline]}")
        baseline_ok = all(v < args.baseline_threshold for v in baseline)
        print(f"Baseline check: {'PASS' if baseline_ok else 'FAIL - check bias/shorts'}\n")

        all_pass = baseline_ok
        for i, name in enumerate(NAMES):
            print(f"Apply firm pressure with {name} fingertip only.")
            input("Press Enter while pressing...")
            pressed = read_fsr_average(ser, args.samples)
            print(f"Pressed readings: {[f'{v:.3f}' for v in pressed]}")
            passed = pressed[i] > args.press_threshold
            all_pass = all_pass and passed
            print(f"  {name}: {'PASS' if passed else 'FAIL'}\n")

    print(f"Overall: {'PASS' if all_pass else 'FAIL'}")


if __name__ == '__main__':
    main()
