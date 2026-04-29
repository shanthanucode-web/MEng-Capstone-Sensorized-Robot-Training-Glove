#!/usr/bin/env python3
"""
db_writer.py - Write a cleaned glove session CSV into SQLite.

Usage:
    python db_writer.py sessions/cleaned/<file>.csv
"""

import os
import sqlite3
import sys
from datetime import datetime

import pandas as pd


DB_PATH = 'glove_dataset.db'

FRAME_COLS = [
    'timestamp_ms',
    'qw', 'qx', 'qy', 'qz',
    'flex_thumb', 'flex_thumb_deg',
    'flex_index', 'flex_index_deg',
    'flex_middle', 'flex_middle_deg',
    'flex_ring', 'flex_ring_deg',
    'flex_pinky', 'flex_pinky_deg',
    'fsr_index', 'fsr_index_contact',
    'fsr_middle', 'fsr_middle_contact',
    'fsr_thumb', 'fsr_thumb_contact',
]


def write(filepath, db_path=DB_PATH):
    if not os.path.exists(db_path):
        print(f"ERROR: Database not found at {db_path}")
        print(f"Run: sqlite3 {db_path} < db_schema.sql")
        sys.exit(1)

    df = pd.read_csv(filepath)
    missing = [col for col in FRAME_COLS + ['task', 'subject'] if col not in df.columns]
    if missing:
        print(f"ERROR: Missing columns in CSV: {missing}")
        print("Did you run data_cleaner.py on this file first?")
        sys.exit(1)

    tasks = sorted(str(v) for v in df['task'].dropna().unique())
    subjects = sorted(str(v) for v in df['subject'].dropna().unique())
    if len(tasks) != 1 or len(subjects) != 1:
        print(f"ERROR: Expected one task and one subject, got tasks={tasks}, subjects={subjects}")
        sys.exit(1)

    fname = os.path.basename(filepath)
    task = tasks[0]
    subject = subjects[0]

    conn = sqlite3.connect(db_path)
    try:
        cur = conn.cursor()
        cur.execute(
            """
            INSERT INTO sessions (task, subject, recorded_at, source_file, row_count)
            VALUES (?, ?, ?, ?, ?)
            """,
            (task, subject, datetime.now().isoformat(timespec='seconds'), fname, len(df)),
        )
        session_id = cur.lastrowid

        frame_df = df[FRAME_COLS].copy()
        frame_df.insert(0, 'session_id', session_id)
        frame_df.to_sql(
            'frames',
            conn,
            if_exists='append',
            index=False,
            method='multi',
            chunksize=500,
        )
        conn.commit()
    finally:
        conn.close()

    print(f"Wrote session_id={session_id}: {len(df)} frames  task={task}  subject={subject}")
    return session_id


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python db_writer.py sessions/cleaned/<file>.csv")
        sys.exit(1)
    write(sys.argv[1])
