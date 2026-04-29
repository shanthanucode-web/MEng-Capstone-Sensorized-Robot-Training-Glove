#!/usr/bin/env python3
"""
export_dataset.py - Export training-ready datasets from glove_dataset.db.

Usage:
    python export_dataset.py --task pick_beaker
    python export_dataset.py --all
"""

import argparse
import os
import sqlite3

import numpy as np
import pandas as pd


DB_PATH = 'glove_dataset.db'

FEATURE_COLS = [
    'qw', 'qx', 'qy', 'qz',
    'flex_thumb_deg', 'flex_index_deg', 'flex_middle_deg',
    'flex_ring_deg', 'flex_pinky_deg',
    'fsr_index', 'fsr_middle', 'fsr_thumb',
    'fsr_index_contact', 'fsr_middle_contact', 'fsr_thumb_contact',
]


def export(task_filter=None, db_path=DB_PATH):
    if not os.path.exists(db_path):
        print(f"ERROR: {db_path} not found. Run db_schema.sql first.")
        return None

    conn = sqlite3.connect(db_path)
    try:
        if task_filter:
            df = pd.read_sql_query(
                """
                SELECT f.*, s.task, s.subject
                FROM frames f
                JOIN sessions s ON f.session_id = s.session_id
                WHERE s.task = ?
                ORDER BY f.session_id, f.timestamp_ms
                """,
                conn,
                params=(task_filter,),
            )
        else:
            df = pd.read_sql_query(
                """
                SELECT f.*, s.task, s.subject
                FROM frames f
                JOIN sessions s ON f.session_id = s.session_id
                ORDER BY f.session_id, f.timestamp_ms
                """,
                conn,
            )
    finally:
        conn.close()

    if df.empty:
        print(f"No data found for filter: {task_filter or 'all'}")
        return None

    label = task_filter or 'all'
    csv_out = f"dataset_{label}.csv"
    df.to_csv(csv_out, index=False)
    print(f"CSV:      {csv_out}  ({len(df)} rows)")

    missing = [col for col in FEATURE_COLS if col not in df.columns]
    if missing:
        print(f"WARNING: Missing feature columns: {missing}")
        for col in missing:
            df[col] = 0.0

    X = df[FEATURE_COLS].values.astype(np.float32)
    tasks = df['task'].values
    subjects = df['subject'].values
    sessions = df['session_id'].values

    npz_out = f"dataset_{label}.npz"
    np.savez(
        npz_out,
        X=X,
        feature_names=np.array(FEATURE_COLS),
        tasks=tasks,
        subjects=subjects,
        session_ids=sessions,
    )

    print(f"NPZ:      {npz_out}  shape={X.shape}")
    print(f"Tasks:    {sorted(set(tasks))}")
    print(f"Subjects: {sorted(set(subjects))}")
    print(f"Sessions: {len(set(sessions))}")
    return csv_out, npz_out


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--task', help='Filter by task name')
    group.add_argument('--all', action='store_true', help='Export all sessions')
    args = parser.parse_args()
    export(None if args.all else args.task)
