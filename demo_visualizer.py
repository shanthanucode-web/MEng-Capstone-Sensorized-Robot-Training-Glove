#!/usr/bin/env python3
"""
demo_visualizer.py - Skeletal glove demo visualizer.

Usage:
    python demo_visualizer.py
    python demo_visualizer.py --replay session_20260415_190134.csv
    python demo_visualizer.py --replay session_20260415_190134.csv --loop
    python demo_visualizer.py --simulate
    python demo_visualizer.py --simulate --sim-task use_drill
"""

import argparse
import csv
import math
import os
import threading
import time
from collections import deque

import numpy as np
import serial
from vispy import app, scene

from glove_serial import open_glove_serial


WIN_W, WIN_H = 1280, 800
BAUD_RATE = 115200
TIMER_INTERVAL = 1.0 / 30.0
CONTACT_THRESHOLD = 0.03

BG_COLOR = '#0A0A1A'
PALM_COLOR = np.array([0.267, 0.267, 0.267, 1.00], dtype=np.float32)
JOINT_COLOR = np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32)
CONTACT_COLOR = np.array([1.0, 0.08, 0.05, 1.0], dtype=np.float32)
THUMB_COLOR = np.array([0.50, 0.50, 0.50, 1.0], dtype=np.float32)
INDEX_COLOR = np.array([0.0, 1.0, 1.0, 1.0], dtype=np.float32)
INDEX_PASSIVE = np.array([0.0, 0.55, 0.55, 1.0], dtype=np.float32)
MIDDLE_COLOR = np.array([1.0, 0.549, 0.0, 1.0], dtype=np.float32)
MIDDLE_PASSIVE = np.array([0.70, 0.38, 0.0, 1.0], dtype=np.float32)

MAX_CURL = 85.0
DIP_COUPLING = 0.65
RING_FOLLOW = 0.90
PINKY_FOLLOW = 0.78

_UP = np.array([0.0, 1.0, 0.0], dtype=np.float32)

PALM_NODES = np.array([
    [-0.36, -0.06, 0.0],
    [-0.18,  0.00, 0.0],
    [ 0.00,  0.02, 0.0],
    [ 0.16,  0.00, 0.0],
    [ 0.28, -0.04, 0.0],
], dtype=np.float32)
WRIST_L = np.array([-0.22, -0.32, 0.0], dtype=np.float32)
WRIST_R = np.array([ 0.22, -0.32, 0.0], dtype=np.float32)

FINGER_DEFS = [
    ('thumb', 0, 32.0, 0, 0, THUMB_COLOR, None, 0.14, 0.11, 0.00),
    ('index', 1,  4.0, 1, 2, INDEX_COLOR, None, 0.20, 0.13, 0.09),
    ('middle', 2, 0.0, 3, 4, MIDDLE_COLOR, None, 0.22, 0.14, 0.10),
    ('ring', 3, -5.0, 3, 4, INDEX_PASSIVE, 'ring', 0.19, 0.12, 0.09),
    ('pinky', 4, -12.0, 3, 4, MIDDLE_PASSIVE, 'pinky', 0.15, 0.10, 0.08),
]

SIM_TASKS = {
    'pick_beaker': {
        'description': 'Contact: fingertip grasp forces vary with object weight',
        'phases': [
            ('REST', 0.0, 0.5, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
            ('REACHING', 0.5, 2.0, 0.0, [0.0, 0.0, 0.0], (-35.0, 0.0, 0.0)),
            ('PRE-GRASP', 2.0, 2.5, 0.4, [0.0, 0.0, 0.0], (-35.0, 0.0, 0.0)),
            ('GRASPING', 2.5, 3.5, 0.85, [0.7, 0.64, 0.56], (-35.0, 0.0, 0.0)),
            ('HOLDING', 3.5, 4.5, 0.85, [0.7, 0.64, 0.56], (-35.0, 20.0, 0.0)),
            ('RELEASING', 4.5, 5.3, 0.0, [0.0, 0.0, 0.0], (-35.0, 20.0, 0.0)),
            ('RETRACTING', 5.3, 6.3, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
            ('REST', 6.3, 8.0, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
        ],
    },
    'fold_cloth': {
        'description': 'Contact: distributed palm pressure guides fabric smoothing',
        'phases': [
            ('OPEN', 0.0, 0.8, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
            ('REACH_OUT', 0.8, 1.6, 0.0, [0.0, 0.0, 0.0], (0.0, -25.0, 0.0)),
            ('PINCH', 1.6, 2.2, [0.6, 0.5, 0.5, 0.3, 0.3], [0.4, 0.05, 0.4], (0.0, -25.0, 0.0)),
            ('SWEEP_IN', 2.2, 3.4, [0.6, 0.5, 0.5, 0.3, 0.3], [0.4, 0.05, 0.4], (0.0, 20.0, 0.0)),
            ('PRESS_DOWN', 3.4, 4.2, 0.1, [0.6, 0.6, 0.6], (-20.0, 20.0, 0.0)),
            ('LIFT', 4.2, 4.8, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
            ('REST', 4.8, 5.4, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
        ],
    },
    'swirl_beaker': {
        'description': 'Contact: grip adjusts dynamically to centripetal force',
        'phases': [
            ('OPEN', 0.0, 0.5, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
            ('GRASP', 0.5, 1.3, 0.75, [0.55, 0.50, 0.58], (-15.0, 0.0, 0.0)),
            ('SWIRL_1', 1.3, 2.1, 0.75, [0.60, 0.52, 0.62], (-15.0, 30.0, 0.0)),
            ('SWIRL_2', 2.1, 2.9, 0.75, [0.48, 0.61, 0.54], (-15.0, 60.0, 0.0)),
            ('SWIRL_3', 2.9, 3.7, 0.75, [0.62, 0.50, 0.60], (-15.0, 30.0, 0.0)),
            ('SWIRL_4', 3.7, 4.5, 0.75, [0.50, 0.58, 0.55], (-15.0, 0.0, 0.0)),
            ('RELEASE', 4.5, 5.2, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
            ('REST', 5.2, 5.6, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
        ],
    },
    'use_drill': {
        'description': 'Contact: trigger + torque reaction invisible to cameras',
        'phases': [
            ('OPEN', 0.0, 0.5, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
            ('GRIP', 0.5, 1.5, 0.80, [0.60, 0.60, 0.60], (-20.0, 0.0, 0.0)),
            ('AIM', 1.5, 2.3, 0.80, [0.60, 0.60, 0.60], (-20.0, 8.0, 0.0)),
            ('TRIGGER_PULL', 2.3, 3.1, [0.80, 0.95, 0.95, 0.80, 0.80], [0.90, 0.60, 0.65], (-20.0, 8.0, 0.0)),
            ('DRILLING', 3.1, 4.3, [0.80, 0.95, 0.95, 0.80, 0.80], [0.75, 0.85, 0.88], (-20.0, 8.0, 0.0)),
            ('RELEASE_TRIGGER', 4.3, 4.9, 0.80, [0.60, 0.60, 0.65], (-20.0, 8.0, 0.0)),
            ('WITHDRAW', 4.9, 5.7, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
            ('REST', 5.7, 6.2, 0.0, [0.0, 0.0, 0.0], (0.0, 0.0, 0.0)),
        ],
    },
}

_state = {
    'q': [1.0, 0.0, 0.0, 0.0],
    'f': [0.0] * 5,
    'p': [0.0] * 3,
    'mode': 'LIVE',
    'phase': '',
    'sim_task': '',
    'description': '',
    'replay_time_ms': 0,
    'replay_duration_ms': 0,
    'times': deque(maxlen=40),
}
_lock = threading.Lock()
_stop = threading.Event()
_ref_q = [1.0, 0.0, 0.0, 0.0]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Skeletal glove demo visualizer.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--replay', help='Replay a recorded CSV session')
    group.add_argument('--simulate', action='store_true', help='Run synthetic pick-and-place demo')
    parser.add_argument('--sim-task', choices=sorted(SIM_TASKS), default='pick_beaker',
                        help='Simulation task to run with --simulate')
    parser.add_argument('--loop', action='store_true', help='Loop replay mode continuously')
    parser.add_argument('--baud', type=int, default=BAUD_RATE)
    return parser.parse_args(argv)


def _rx(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float32)


def _rz(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)


def quat_from_euler(pitch=0.0, yaw=0.0, roll=0.0):
    hp, hy, hr = np.radians([pitch, yaw, roll]) * 0.5
    cp, sp = np.cos(hp), np.sin(hp)
    cy, sy = np.cos(hy), np.sin(hy)
    cr, sr = np.cos(hr), np.sin(hr)
    # yaw(Y) * pitch(X) * roll(Z)
    qy = [cy, 0.0, sy, 0.0]
    qx = [cp, sp, 0.0, 0.0]
    qz = [cr, 0.0, 0.0, sr]
    return _qnorm(_qmul(_qmul(qy, qx), qz))


def _qnorm(q):
    q = np.asarray(q, dtype=np.float64)
    n = float(np.linalg.norm(q))
    if n < 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    return (q / n).tolist()


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ]


def _qrel(q_ref, q_raw):
    q_inv = [q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]]
    return _qmul(q_inv, q_raw)


def _qmat(w, x, y, z):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float32)


def slerp(q1, q2, t):
    q1 = np.asarray(_qnorm(q1), dtype=np.float64)
    q2 = np.asarray(_qnorm(q2), dtype=np.float64)
    dot = float(np.dot(q1, q2))
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return _qnorm(q1 + (q2 - q1) * t)
    theta_0 = math.acos(dot)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return _qnorm((s0 * q1) + (s1 * q2))


def smoothstep(t):
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


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


def update_state(q=None, f=None, p=None, phase=None, sim_task=None, description=None,
                 replay_time_ms=None, replay_duration_ms=None):
    now = time.monotonic()
    with _lock:
        if q is not None:
            _state['q'] = list(q)
        if f is not None:
            _state['f'] = list(f)
            _state['times'].append(now)
        if p is not None:
            _state['p'] = list(p)
        if phase is not None:
            _state['phase'] = phase
        if sim_task is not None:
            _state['sim_task'] = sim_task
        if description is not None:
            _state['description'] = description
        if replay_time_ms is not None:
            _state['replay_time_ms'] = int(replay_time_ms)
        if replay_duration_ms is not None:
            _state['replay_duration_ms'] = int(replay_duration_ms)


def _serial_thread(baud_rate):
    while not _stop.is_set():
        try:
            ser, port = open_glove_serial(baud_rate, timeout=1)
            print(f'[serial] connected: {port}')
            with ser:
                while not _stop.is_set():
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    q = _parse_floats(line, 'Q:', 4)
                    f = _parse_floats(line, 'F:', 5)
                    p = _parse_floats(line, 'P:', 3)
                    if q is not None:
                        update_state(q=q)
                    elif f is not None:
                        update_state(f=f)
                    elif p is not None:
                        update_state(p=p)
        except serial.SerialException as exc:
            if not _stop.is_set():
                print(f'[serial] {exc} - retrying in 2s')
                time.sleep(2)
        except Exception as exc:
            if not _stop.is_set():
                print(f'[serial] unexpected error: {exc} - retrying in 2s')
                time.sleep(2)


def _read_replay_rows(csv_path):
    if not os.path.exists(csv_path):
        raise FileNotFoundError(csv_path)

    with open(csv_path, newline='') as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        fields = set(reader.fieldnames or [])

    required_base = {'timestamp_ms', 'qw', 'qx', 'qy', 'qz', 'flex_thumb'}
    if not required_base.issubset(fields):
        raise ValueError(f'{csv_path} is missing required columns: {sorted(required_base - fields)}')

    legacy = {
        'flex_upper_index', 'flex_lower_index',
        'flex_upper_middle', 'flex_lower_middle',
    }
    new_schema = {'flex_index', 'flex_middle'}
    if legacy.issubset(fields):
        schema = 'legacy'
    elif new_schema.issubset(fields):
        schema = 'new'
    else:
        raise ValueError('CSV must contain legacy upper/lower flex columns or new flex_index/flex_middle columns')

    parsed = []
    for row in rows:
        try:
            ts = int(float(row['timestamp_ms']))
            q = [float(row[col]) for col in ('qw', 'qx', 'qy', 'qz')]
            if schema == 'legacy':
                f = [
                    float(row['flex_thumb']),
                    float(row['flex_upper_index']),
                    float(row['flex_lower_index']),
                    float(row['flex_upper_middle']),
                    float(row['flex_lower_middle']),
                ]
            else:
                index = float(row['flex_index'])
                middle = float(row['flex_middle'])
                f = [float(row['flex_thumb']), index, index, middle, middle]

            if {'fsr_index', 'fsr_middle', 'fsr_thumb'}.issubset(fields):
                p = [float(row['fsr_index']), float(row['fsr_middle']), float(row['fsr_thumb'])]
            else:
                p = [0.0, 0.0, 0.0]
        except (TypeError, ValueError):
            continue
        parsed.append({'timestamp_ms': ts, 'q': q, 'f': f, 'p': p})

    if not parsed:
        raise ValueError(f'{csv_path} did not contain any parseable rows')

    parsed.sort(key=lambda item: item['timestamp_ms'])
    duration = max(parsed[-1]['timestamp_ms'] - parsed[0]['timestamp_ms'], 0)
    return parsed, duration, schema


def _replay_thread(csv_path, should_loop):
    rows, duration, schema = _read_replay_rows(csv_path)
    print(f'[replay] loaded {len(rows)} rows from {csv_path} ({schema}, {duration / 1000.0:.1f}s)')
    update_state(replay_duration_ms=duration)

    while not _stop.is_set():
        prev_ts = rows[0]['timestamp_ms']
        base_ts = rows[0]['timestamp_ms']
        for row in rows:
            if _stop.is_set():
                return
            delta = max(row['timestamp_ms'] - prev_ts, 0) / 1000.0
            if delta > 0:
                time.sleep(delta)
            replay_ts = row['timestamp_ms'] - base_ts
            update_state(
                q=row['q'],
                f=row['f'],
                p=row['p'],
                replay_time_ms=replay_ts,
                replay_duration_ms=duration,
            )
            prev_ts = row['timestamp_ms']
        if not should_loop:
            _stop.set()
            app.quit()
            return
        time.sleep(0.5)


def _target_array(value, length):
    if isinstance(value, (list, tuple, np.ndarray)):
        arr = np.asarray(value, dtype=np.float64)
    else:
        arr = np.full(length, float(value), dtype=np.float64)
    if arr.size != length:
        raise ValueError(f'Expected {length} values, got {arr.size}')
    return arr


def _phase_at(task_config, t):
    phases = task_config['phases']
    active_idx = len(phases) - 1
    phase = phases[active_idx]
    for idx, item in enumerate(phases):
        if item[1] <= t < item[2]:
            active_idx = idx
            phase = item
            break
    return active_idx, phase


def _simulate_sample(t, task_name='pick_beaker'):
    task_config = SIM_TASKS[task_name]
    active_idx, phase = _phase_at(task_config, t)
    phases = task_config['phases']
    name, start, end, flex_target, pressure_target, wrist_target = phase
    local_raw = (t - start) / max(end - start, 1e-6)
    local = smoothstep(local_raw)

    if active_idx > 0:
        prev = phases[active_idx - 1]
        flex_start = _target_array(prev[3], 5)
        pressure_start = _target_array(prev[4], 3)
        wrist_start = prev[5]
    else:
        flex_start = _target_array(flex_target, 5)
        pressure_start = _target_array(pressure_target, 3)
        wrist_start = wrist_target

    flex_end = _target_array(flex_target, 5)
    pressure_end = _target_array(pressure_target, 3)
    flex = flex_start + (flex_end - flex_start) * local
    pressure = pressure_start + (pressure_end - pressure_start) * local

    if task_name == 'pick_beaker' and name == 'GRASPING':
        contact_gate = smoothstep((float(np.max(flex)) - 0.70) / 0.15)
        pressure = pressure_end * contact_gate
    elif task_name == 'pick_beaker' and name == 'RELEASING':
        pressure = pressure_start * (1.0 - smoothstep(local_raw * 1.5))

    q_start = quat_from_euler(*wrist_start)
    q_end = quat_from_euler(*wrist_target)
    q = slerp(q_start, q_end, local)
    if task_name == 'use_drill' and name == 'DRILLING':
        roll_q = quat_from_euler(roll=8.0 * math.sin(local_raw * 4.0 * math.pi))
        q = _qnorm(_qmul(q, roll_q))
    return name, q, flex.tolist(), pressure.tolist()


def _simulate_thread(task_name):
    task_config = SIM_TASKS[task_name]
    duration = task_config['phases'][-1][2]
    start = time.monotonic()
    while not _stop.is_set():
        t = (time.monotonic() - start) % duration
        phase, q, flex, pressure = _simulate_sample(t, task_name)
        update_state(
            q=q,
            f=flex,
            p=pressure,
            phase=phase,
            sim_task=task_name,
            description=task_config['description'],
        )
        time.sleep(1.0 / 30.0)


def compute_skeleton(flex, pressure, R_imu):
    v_s, v_e, v_c = [], [], []
    joint_pos, joint_col = [], []
    contact_tips = {}

    for i in range(4):
        v_s.append(PALM_NODES[i])
        v_e.append(PALM_NODES[i + 1])
        v_c += [PALM_COLOR, PALM_COLOR]
    v_s.append(PALM_NODES[0])
    v_e.append(WRIST_L)
    v_c += [PALM_COLOR, PALM_COLOR]
    v_s.append(PALM_NODES[4])
    v_e.append(WRIST_R)
    v_c += [PALM_COLOR, PALM_COLOR]
    v_s.append(WRIST_L)
    v_e.append(WRIST_R)
    v_c += [PALM_COLOR, PALM_COLOR]

    for node in PALM_NODES:
        joint_pos.append(node)
        joint_col.append(JOINT_COLOR)
    joint_pos += [WRIST_L, WRIST_R]
    joint_col += [JOINT_COLOR, JOINT_COLOR]

    mid_upper = float(np.clip(flex[3], 0.0, 1.0))
    mid_lower = float(np.clip(flex[4], 0.0, 1.0))

    for name, pidx, splay, fui, fli, color, passive, l1, l2, l3 in FINGER_DEFS:
        base = PALM_NODES[pidx]
        if passive == 'ring':
            fu, fl = mid_upper * RING_FOLLOW, mid_lower * RING_FOLLOW
        elif passive == 'pinky':
            fu, fl = mid_upper * PINKY_FOLLOW, mid_lower * PINKY_FOLLOW
        else:
            fu = float(np.clip(flex[fui], 0.0, 1.0))
            fl = float(np.clip(flex[fli], 0.0, 1.0))

        R_sp = _rz(splay)
        R_mcp = _rx(-fu * MAX_CURL)
        R_pip = _rx(-fl * MAX_CURL)

        p1 = base + (R_sp @ R_mcp @ _UP) * l1
        p2 = p1 + (R_sp @ R_mcp @ R_pip @ _UP) * l2

        if name == 'thumb':
            points = [base, p1, p2]
        else:
            R_dip = _rx(-(fl * DIP_COUPLING) * MAX_CURL)
            p3 = p2 + (R_sp @ R_mcp @ R_pip @ R_dip @ _UP) * l3
            points = [base, p1, p2, p3]

        for a, b in zip(points[:-1], points[1:]):
            v_s.append(a)
            v_e.append(b)
            v_c += [color, color]
        for point in points[1:]:
            joint_pos.append(point)
            joint_col.append(JOINT_COLOR)

        if name == 'index':
            contact_tips['index'] = points[-1]
        elif name == 'middle':
            contact_tips['middle'] = points[-1]
        elif name == 'thumb':
            contact_tips['thumb'] = points[-1]

    bone_verts = np.empty((len(v_s) * 2, 3), dtype=np.float32)
    bone_verts[0::2] = np.asarray(v_s, dtype=np.float32)
    bone_verts[1::2] = np.asarray(v_e, dtype=np.float32)
    joint_pos = np.asarray(joint_pos, dtype=np.float32)
    contact_pos = np.asarray([
        contact_tips['index'],
        contact_tips['middle'],
        contact_tips['thumb'],
    ], dtype=np.float32)

    bone_verts = (R_imu @ bone_verts.T).T
    joint_pos = (R_imu @ joint_pos.T).T
    contact_pos = (R_imu @ contact_pos.T).T

    contact_values = np.asarray(pressure[:3], dtype=np.float32)
    contact_sizes = np.where(
        contact_values > CONTACT_THRESHOLD,
        10.0 + 12.0 * np.clip(contact_values, 0.0, 1.0),
        0.0,
    ).astype(np.float32)
    contact_colors = np.tile(CONTACT_COLOR, (3, 1)).astype(np.float32)
    contact_colors[:, 3] = np.where(contact_values > CONTACT_THRESHOLD, 1.0, 0.0)

    return (
        bone_verts,
        np.asarray(v_c, dtype=np.float32),
        joint_pos,
        np.asarray(joint_col, dtype=np.float32),
        contact_pos,
        contact_colors,
        contact_sizes,
    )


def _force_line(task_name, phase, pressure):
    prefix = ''
    if task_name:
        prefix = f'{task_name.upper()} - {phase or "--"} | '
    return (
        f'{prefix}FSR: Index {pressure[0]:.2f}  '
        f'Middle {pressure[1]:.2f}  Thumb {pressure[2]:.2f}'
    )


def run_app(args):
    global _ref_q

    if args.simulate:
        mode = 'SIMULATE'
        worker = threading.Thread(target=_simulate_thread, args=(args.sim_task,), daemon=True)
    elif args.replay:
        mode = 'REPLAY'
        worker = threading.Thread(target=_replay_thread, args=(args.replay, args.loop), daemon=True)
    else:
        mode = 'LIVE'
        worker = threading.Thread(target=_serial_thread, args=(args.baud,), daemon=True)

    with _lock:
        _state['mode'] = mode
        if mode == 'SIMULATE':
            _state['sim_task'] = args.sim_task
            _state['description'] = SIM_TASKS[args.sim_task]['description']
    worker.start()

    canvas = scene.SceneCanvas(
        title='Glove Demo Visualizer',
        keys='interactive',
        size=(WIN_W, WIN_H),
        bgcolor=BG_COLOR,
        show=True,
    )
    view = canvas.central_widget.add_view()
    view.camera = scene.TurntableCamera(
        fov=38, distance=2.2, elevation=25, azimuth=0, interactive=True
    )
    view.camera.center = (0.0, 0.0, 0.0)
    scene.visuals.XYZAxis(parent=view.scene)

    R0 = np.eye(3, dtype=np.float32)
    bv, bc, jp, jc, cp, cc, cs = compute_skeleton([0.0] * 5, [0.0] * 3, R0)
    hand_lines = scene.visuals.Line(
        pos=bv, color=bc, connect='segments', width=3, antialias=True, parent=view.scene
    )
    hand_joints = scene.visuals.Markers(parent=view.scene)
    hand_joints.set_data(pos=jp, face_color=jc, symbol='disc', size=10, edge_width=0)
    contact_markers = scene.visuals.Markers(parent=view.scene)
    contact_markers.set_data(pos=cp, face_color=cc, symbol='disc', size=cs, edge_width=0)

    hud_mode = scene.visuals.Text(
        f'Mode: {mode}', pos=(14, 16), color='white', font_size=12, bold=True,
        anchor_x='left', anchor_y='top', parent=canvas.scene
    )
    hud_phase = scene.visuals.Text(
        '', pos=(14, 38), color='#FFAA33', font_size=10, bold=True,
        anchor_x='left', anchor_y='top', parent=canvas.scene
    )
    hud_description = scene.visuals.Text(
        '', pos=(14, 60), color='#FFDD88', font_size=9,
        anchor_x='left', anchor_y='top', parent=canvas.scene
    )
    hud_quat = scene.visuals.Text(
        'Q: --', pos=(14, 84), color='#CCCCFF', font_size=9,
        anchor_x='left', anchor_y='top', parent=canvas.scene
    )
    hud_rate = scene.visuals.Text(
        '-- Hz', pos=(14, 104), color='#BBBBBB', font_size=9,
        anchor_x='left', anchor_y='top', parent=canvas.scene
    )
    hud_replay = scene.visuals.Text(
        '', pos=(14, 124), color='#BBBBBB', font_size=9,
        anchor_x='left', anchor_y='top', parent=canvas.scene
    )
    hud_cal = scene.visuals.Text(
        'Press C to calibrate neutral pose', pos=(14, WIN_H - 16),
        color='#777777', font_size=8, anchor_x='left', anchor_y='bottom',
        parent=canvas.scene
    )

    def on_timer(_event):
        with _lock:
            q = list(_state['q'])
            flex = list(_state['f'])
            pressure = list(_state['p'])
            phase = _state['phase']
            sim_task = _state['sim_task']
            description = _state['description']
            times = list(_state['times'])
            replay_time_ms = _state['replay_time_ms']
            replay_duration_ms = _state['replay_duration_ms']

        qr = _qrel(_ref_q, q)
        R = _qmat(*qr)
        bv, bc, jp, jc, cp, cc, cs = compute_skeleton(flex, pressure, R)
        hand_lines.set_data(pos=bv, color=bc, connect='segments', width=3)
        hand_joints.set_data(pos=jp, face_color=jc, symbol='disc', size=10, edge_width=0)
        contact_markers.set_data(pos=cp, face_color=cc, symbol='disc', size=cs, edge_width=0)

        hud_mode.text = f'Mode: {mode}'
        hud_phase.text = _force_line(sim_task if mode == 'SIMULATE' else '', phase, pressure)
        hud_description.text = description if mode == 'SIMULATE' else ''
        hud_quat.text = f'Q: W:{q[0]:+.3f} X:{q[1]:+.3f} Y:{q[2]:+.3f} Z:{q[3]:+.3f}'
        if len(times) >= 2:
            dt = times[-1] - times[0]
            rate = (len(times) - 1) / dt if dt > 0 else 0.0
            hud_rate.text = f'{rate:.1f} Hz'
        if mode == 'REPLAY':
            hud_replay.text = (
                f'Replay: {replay_time_ms / 1000.0:.1f}s / '
                f'{replay_duration_ms / 1000.0:.1f}s'
            )
        else:
            hud_replay.text = ''
        canvas.update()

    def on_key_press(event):
        global _ref_q
        if event.key.name.lower() == 'c' and mode in ('LIVE', 'REPLAY'):
            with _lock:
                _ref_q = list(_state['q'])
            hud_cal.text = 'Calibrated - C to recalibrate'
            hud_cal.color = '#44AA44'
            print(f'[calibrate] neutral pose captured: {[f"{v:.4f}" for v in _ref_q]}')

    canvas.events.key_press.connect(on_key_press)
    timer = app.Timer(interval=TIMER_INTERVAL, connect=on_timer, start=True)
    print(f'[demo] mode={mode}')
    if mode == 'SIMULATE':
        print(f'[demo] sim_task={args.sim_task}')
    if mode == 'REPLAY':
        print(f'[demo] replay={args.replay} loop={args.loop}')
    print('[demo] left-drag to orbit | scroll to zoom | C to calibrate in live/replay')
    try:
        app.run()
    finally:
        timer.stop()
        _stop.set()


def main(argv=None):
    args = parse_args(argv)
    if args.loop and not args.replay:
        raise SystemExit('--loop is only valid with --replay')
    if args.sim_task != 'pick_beaker' and not args.simulate:
        raise SystemExit('--sim-task is only valid with --simulate')
    run_app(args)


if __name__ == '__main__':
    main()
