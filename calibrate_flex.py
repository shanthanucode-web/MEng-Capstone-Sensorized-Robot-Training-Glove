#!/usr/bin/env python3
"""
calibrate_flex.py — Flex Sensor Calibration Analyzer
=====================================================
Reads the most recent session_*.csv recorded by record_session.py,
back-calculates raw ADC values from the normalized flex readings,
and recommends corrected FLEX_MIN / FLEX_MAX constants for main.cpp.

Dependencies:
    pip install pandas

Usage:
    python3 calibrate_flex.py
    (or) python3 calibrate_flex.py session_20260402_172415.csv

How it works:
    The firmware normalizes raw ADC readings with:
        normalized = (raw - FLEX_MIN) / (FLEX_MAX - FLEX_MIN)

    Inverting that formula:
        raw_adc = normalized × (FLEX_MAX - FLEX_MIN) + FLEX_MIN

    We reconstruct the raw ADC series, then:
        FLEX_MIN_new = 20th-percentile ADC value
                       (lowest 20% of readings represent the open/resting state)
        FLEX_MAX_new = maximum ADC value observed in the session
"""

import sys
import glob
import pandas as pd

# ---------------------------------------------------------------------------
# Current firmware calibration constants — must match main.cpp exactly
# so the back-calculation is correct
# ---------------------------------------------------------------------------
CURRENT_FLEX_MIN = [2800, 3273, 2972, 3642, 3271]
CURRENT_FLEX_MAX = [3700, 3700, 3584, 3700, 3682]

# Sensor names in CSV column order (index 0 = thumb)
SENSOR_NAMES = [
    "flex_thumb",
    "flex_upper_index",
    "flex_lower_index",
    "flex_upper_middle",
    "flex_lower_middle",
]

# Thresholds for flagging a sensor as needing recalibration
RESTING_THRESHOLD = 0.10   # resting mean above this → FLEX_MIN too low
RANGE_THRESHOLD   = 0.80   # observed max below this → FLEX_MAX too high


def find_csv():
    """Return the path to the most recent session CSV in the current directory."""
    files = sorted(glob.glob("session_*.csv"))
    if not files:
        print("ERROR: No session_*.csv files found in current directory.")
        sys.exit(1)
    return files[-1]


def back_calculate_adc(normalized_series, sensor_idx):
    """
    Reverse the firmware normalization to recover raw ADC values.
        raw = normalized × (FLEX_MAX - FLEX_MIN) + FLEX_MIN
    """
    lo = CURRENT_FLEX_MIN[sensor_idx]
    hi = CURRENT_FLEX_MAX[sensor_idx]
    return normalized_series * (hi - lo) + lo


def analyze(csv_path):
    print(f"\n  CSV file : {csv_path}")

    df = pd.read_csv(csv_path)

    # Sanity check
    for col in SENSOR_NAMES:
        if col not in df.columns:
            print(f"ERROR: Column '{col}' not found in CSV.")
            sys.exit(1)

    new_mins = list(CURRENT_FLEX_MIN)   # start with existing values
    new_maxs = list(CURRENT_FLEX_MAX)

    print()
    print("=" * 72)
    print(f"  {'SENSOR':<20} {'OBS_MIN':>8} {'OBS_MAX':>8} "
          f"{'REST_MEAN':>10} {'ADC_MIN':>8} {'ADC_MAX':>8}  STATUS")
    print("=" * 72)

    for idx, col in enumerate(SENSOR_NAMES):
        norm = df[col]

        # Back-calculate raw ADC series from normalized values
        adc = back_calculate_adc(norm, idx)

        obs_min  = norm.min()
        obs_max  = norm.max()

        # "Resting state" = lowest 20% of normalized readings
        rest_threshold = norm.quantile(0.20)
        rest_mean = norm[norm <= rest_threshold].mean()

        # New calibration recommendations
        adc_min_new = int(adc.quantile(0.20))   # 20th pct ADC = resting floor
        adc_max_new = int(adc.max())             # observed ADC peak = full curl

        new_mins[idx] = adc_min_new
        new_maxs[idx] = adc_max_new

        # Determine if recalibration is needed
        needs_recal = rest_mean > RESTING_THRESHOLD or obs_max < RANGE_THRESHOLD
        status = "RECALIBRATE" if needs_recal else "OK"

        print(f"  {col:<20}  {obs_min:>7.3f}  {obs_max:>7.3f}  "
              f"{rest_mean:>9.3f}  {adc_min_new:>7}  {adc_max_new:>7}  {status}")

    print("=" * 72)

    # -----------------------------------------------------------------------
    # Explanation of columns
    # -----------------------------------------------------------------------
    print("""
  OBS_MIN   — lowest normalized value seen (should be ~0.00 when fully open)
  OBS_MAX   — highest normalized value seen (should be ~1.00 when fully closed)
  REST_MEAN — mean of lowest-20% readings (resting/open state target: <0.10)
  ADC_MIN   — recommended new FLEX_MIN in ADC counts (resting floor)
  ADC_MAX   — recommended new FLEX_MAX in ADC counts (full-curl ceiling)
""")

    # -----------------------------------------------------------------------
    # Ready-to-paste firmware lines
    # -----------------------------------------------------------------------
    mins_str = ", ".join(str(v) for v in new_mins)
    maxs_str = ", ".join(str(v) for v in new_maxs)

    print("  Copy-paste into main.cpp:")
    print()
    print(f"  const int FLEX_MIN[5] = {{{mins_str}}};  // flat/extended")
    print(f"  const int FLEX_MAX[5] = {{{maxs_str}}};  // fully curled")
    print()


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else find_csv()
    analyze(csv_path)


if __name__ == "__main__":
    main()
