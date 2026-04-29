#!/usr/bin/env python3
"""
Shared serial-port discovery for the glove tools.

All Python tools in this repo talk to the same ESP32 USB CDC device, but the
macOS device node can change as the board resets or re-enumerates. Resolve the
port fresh on each reconnect attempt instead of hardcoding one path.
"""

import glob
import os

import serial
from serial.tools import list_ports


_PORT_PATTERNS = (
    '/dev/cu.usbmodem*',
    '/dev/tty.usbmodem*',
    '/dev/cu.usbserial*',
    '/dev/tty.usbserial*',
)


def _score_device(device, description=''):
    device = device or ''
    description = (description or '').lower()
    score = 100

    if 'cu.usbmodem' in device:
        score = 0
    elif 'tty.usbmodem' in device:
        score = 1
    elif 'cu.usbserial' in device:
        score = 2
    elif 'tty.usbserial' in device:
        score = 3

    if 'esp32' in description or 'espressif' in description:
        score -= 0.25

    return score


def detect_serial_port():
    """
    Return the best available glove serial port.

    Set GLOVE_SERIAL_PORT to override auto-detection for one-off cases.
    """
    override = os.environ.get('GLOVE_SERIAL_PORT')
    if override:
        return override

    candidates = []
    try:
        for port in list_ports.comports():
            if any(token in port.device for token in ('usbmodem', 'usbserial')):
                candidates.append((_score_device(port.device, port.description), port.device))
    except Exception:
        pass

    if not candidates:
        for pattern in _PORT_PATTERNS:
            for device in glob.glob(pattern):
                candidates.append((_score_device(device), device))

    if not candidates:
        raise RuntimeError(
            'No glove serial port found. Check the USB cable/board and run '
            '`ls /dev/cu.usbmodem*`.'
        )

    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[0][1]


def open_glove_serial(baud_rate, timeout=1):
    """
    Open the currently detected glove serial device with reset-safe settings.
    Returns (serial_object, detected_port).
    """
    port = detect_serial_port()
    ser = serial.Serial(port, baud_rate, timeout=timeout, dsrdtr=False, rtscts=False)
    ser.dtr = False
    ser.rts = False
    return ser, port
