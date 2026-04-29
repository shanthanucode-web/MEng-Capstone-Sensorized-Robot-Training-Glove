#!/usr/bin/env python3
"""
test_mux.py - Validate CD4051BE MUX channel switching.

This requires a temporary firmware debug block inside loop(), before normal
sensor reads:

    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'd') {
        for (int ch = 0; ch < 8; ch++) {
          Serial.print("CH"); Serial.print(ch);
          Serial.print("="); Serial.println(readMuxAdc(ch));
        }
      }
    }

Remove that debug block before flashing production firmware.
"""

import argparse
import time

import serial

from glove_serial import open_glove_serial


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None, help='Serial port override')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--trials', type=int, default=3)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.port:
        ser = serial.Serial(args.port, args.baud, timeout=2, dsrdtr=False, rtscts=False)
        ser.dtr = False
        ser.rts = False
        port = args.port
    else:
        ser, port = open_glove_serial(args.baud, timeout=2)

    with ser:
        time.sleep(1)
        print(f"=== MUX CHANNEL TEST ({port}) ===")
        print("Sending debug dump command 'd'...\n")

        for trial in range(args.trials):
            ser.write(b'd')
            time.sleep(0.2)
            lines = []
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('CH'):
                    lines.append(line)
            if lines:
                print(f"Trial {trial + 1}:")
                for line in lines:
                    print(f"  {line}")
                break
            time.sleep(0.5)
        else:
            print("No CH lines received. Confirm temporary DEBUG_MUX firmware is flashed.")


if __name__ == '__main__':
    main()
