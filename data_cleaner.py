#!/usr/bin/env python3
"""
data_cleaner.py - Clean and normalize raw glove session CSVs.

Output: sessions/cleaned/<same filename>

Usage:
    python data_cleaner.py sessions/raw/<file>.csv
"""

import os
import sys

import numpy as np
import pandas as pd

try:
    from scipy.signal import medfilt
except ImportError:
    medfilt = None


FLEX_CALIB = {
    'flex_thumb': (0.20, 0.75),
    'flex_index': (0.18, 0.72),
    'flex_middle': (0.19, 0.74),
    'flex_ring': (0.20, 0.73),
    'flex_pinky': (0.22, 0.70),
}

FSR_CONTACT_THRESHOLD = 0.08

ANALOG_COLS = [
    'flex_thumb', 'flex_index', 'flex_middle', 'flex_ring', 'flex_pinky',
    'fsr_index', 'fsr_middle', 'fsr_thumb',
]

REQUIRED_COLS = [
    'timestamp_ms', 'qw', 'qx', 'qy', 'qz',
    'flex_thumb', 'flex_index', 'flex_middle', 'flex_ring', 'flex_pinky',
    'fsr_index', 'fsr_middle', 'fsr_thumb', 'task', 'subject',
]


def _median_filter(values):
    if len(values) < 5:
        return values
    if medfilt is not None:
        return medfilt(values, kernel_size=5)
    return (
        pd.Series(values)
        .rolling(window=5, center=True, min_periods=1)
        .median()
        .to_numpy()
    )


def clean(filepath):
    print(f"\nCleaning: {filepath}")
    df = pd.read_csv(filepath)
    missing = [col for col in REQUIRED_COLS if col not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns in raw CSV: {missing}")

    original_len = len(df)

    norms = np.sqrt(df['qw']**2 + df['qx']**2 + df['qy']**2 + df['qz']**2)
    bad = (norms < 0.98) | (norms > 1.02)
    df = df[~bad].reset_index(drop=True)
    print(f"  Quaternion filter: removed {int(bad.sum())} / {original_len} rows")

    df = df.ffill().bfill()

    for col in ANALOG_COLS:
        df[col] = _median_filter(df[col].values)

    for col, (lo, hi) in FLEX_CALIB.items():
        if hi == lo:
            raise ValueError(f"Invalid calibration for {col}: open and curled match")
        normalized = (df[col] - lo) / (hi - lo)
        df[col + '_deg'] = normalized.clip(0.0, 1.0) * 90.0

    for fsr in ['fsr_index', 'fsr_middle', 'fsr_thumb']:
        df[fsr + '_contact'] = (df[fsr] > FSR_CONTACT_THRESHOLD).astype(int)

    os.makedirs('sessions/cleaned', exist_ok=True)
    if filepath.startswith('sessions/raw/'):
        out = filepath.replace('sessions/raw/', 'sessions/cleaned/', 1)
    else:
        out = os.path.join('sessions/cleaned', os.path.basename(filepath))
    df.to_csv(out, index=False)

    print(f"  Output rows:    {len(df)}")
    print(f"  Output columns: {len(df.columns)}")
    print(f"  Saved to:       {out}")
    return out


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python data_cleaner.py sessions/raw/<file>.csv")
        sys.exit(1)
    clean(sys.argv[1])
