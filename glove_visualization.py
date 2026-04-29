#!/usr/bin/env python3
"""
visualize_glove.py — Real-time 3D Robot Hand Visualizer
========================================================
Renders a solid robot hand that rotates with the IMU and curls fingers with
flex sensors. Includes a flex indicator panel and full HUD overlay.

Dependencies:
    pip install vispy pyserial numpy pyopengl

Usage:
    python3 visualize_glove.py
    Left-drag to orbit  |  Scroll to zoom  |  C to calibrate neutral pose

Serial input on an auto-detected USB CDC port at 115200 baud:
    Q:w,x,y,z        — BNO085 Game Rotation Vector quaternion
    F:t,ui,li,um,lm  — normalized flex values [0.0=open .. 1.0=closed]
    P:p1,p2,p3       — normalized pressure values [0.0=none .. 1.0=firm]

Flex sensor mapping:
    F[0] Thumb        — grey
    F[1] Upper Index  — cyan
    F[2] Lower Index  — lighter cyan
    F[3] Upper Middle — orange
    F[4] Lower Middle — lighter orange
"""

import threading
import time
import numpy as np
import serial
from vispy import scene, app
from collections import deque
from glove_serial import open_glove_serial

# =============================================================================
# SECTION 1: CONSTANTS
# All magic numbers live here for easy tuning.
# =============================================================================

# --- Window ---
WIN_W, WIN_H = 1280, 800

# --- Serial ---
BAUD_RATE   = 115200

# --- Colors (numpy RGBA float32 arrays) ---
BG_COLOR      = '#F2F3F1'
PALM_COLOR    = np.array([0.050, 0.055, 0.052, 1.00], dtype=np.float32)
PALM_TOP      = np.array([0.095, 0.105, 0.098, 1.00], dtype=np.float32)
FINGER_BODY   = np.array([0.100, 0.112, 0.108, 1.00], dtype=np.float32)
PASSIVE_COLOR = np.array([0.080, 0.090, 0.087, 1.00], dtype=np.float32)
JOINT_COLOR   = np.array([0.018, 0.020, 0.020, 1.00], dtype=np.float32)
RIVET_COLOR   = np.array([0.010, 0.012, 0.012, 1.00], dtype=np.float32)
BASE_COLOR    = np.array([0.680, 0.690, 0.670, 1.00], dtype=np.float32)
BASE_TOP      = np.array([0.820, 0.825, 0.800, 1.00], dtype=np.float32)
THUMB_COLOR   = np.array([0.330, 0.345, 0.340, 1.00], dtype=np.float32)
INDEX_COLOR   = np.array([0.110, 0.230, 0.235, 1.00], dtype=np.float32)
MIDDLE_COLOR  = np.array([0.260, 0.180, 0.075, 1.00], dtype=np.float32)
PRESSURE_COLOR = np.array([0.950, 0.060, 0.050, 0.95], dtype=np.float32)

# Flex indicator bar colors [Thumb, UpperIndex, LowerIndex, UpperMiddle, LowerMiddle]
BAR_COLORS = [
    np.array([0.50,  0.50,  0.50,  0.85], dtype=np.float32),  # Thumb: grey
    np.array([0.00,  1.00,  1.00,  0.90], dtype=np.float32),  # UI:    cyan
    np.array([0.00,  0.60,  0.60,  0.90], dtype=np.float32),  # LI:    muted cyan
    np.array([1.00,  0.549, 0.00,  0.90], dtype=np.float32),  # UM:    orange
    np.array([0.72,  0.39,  0.00,  0.90], dtype=np.float32),  # LM:    muted orange
]
BAR_TRACK_COLOR  = np.array([0.10, 0.10, 0.20, 0.85], dtype=np.float32)
BAR_LABEL_COLORS = ['#888888', '#00FFFF', '#00AAAA', '#FF8C00', '#BB6600']
BAR_LABELS       = ['T', 'UI', 'LI', 'UM', 'LM']
PRESSURE_LABELS  = ['P1', 'P2', 'P3']

# --- Flex panel 2D layout ---
# Camera rect = (0, 0, PANEL_W, WIN_H) → y-axis points UP in panel space.
PANEL_W   = 320   # pixels allocated to flex panel (right 25% of 1280)
BAR_W     = 42    # bar width in panel data units
BAR_MAX_H = 520   # bar height when flex = 1.0
BAR_BOT   = 80    # y coordinate of bar bottom
BAR_TOP   = BAR_BOT + BAR_MAX_H
BAR_LBL_Y = BAR_TOP + 16   # label text y (above bar)
BAR_VAL_Y = BAR_BOT - 22   # value text y (below bar)
_s = PANEL_W / 6.0
BAR_CX = [_s * (i + 1) for i in range(5)]  # x-centers: ~53,107,160,213,267

# --- Pressure panel 2D layout ---
PRESSURE_X0 = 62
PRESSURE_X1 = PANEL_W - 44
PRESSURE_W = PRESSURE_X1 - PRESSURE_X0
PRESSURE_H = 14
PRESSURE_Y = [718, 688, 658]
PRESSURE_TRACK_COLOR = np.array([0.16, 0.12, 0.12, 0.85], dtype=np.float32)

# --- Hand geometry ---
MAX_CURL = 85.0   # max joint bend angle in degrees (flex=1.0 → this angle)
FLEX_SMOOTH_ALPHA = 0.30

# Segment lengths (dimensionless world units, tuned to fit turntable camera)
# 3 phalanges per finger: proximal (MCP→PIP), intermediate (PIP→DIP), distal (DIP→tip)
# Total per-finger length unchanged vs. the 2-segment version — just split into 3 parts.
PROX_L  = {'thumb': 0.14, 'index': 0.20, 'middle': 0.22, 'ring': 0.18, 'pinky': 0.14}
INTER_L = {'thumb': 0.10, 'index': 0.13, 'middle': 0.14, 'ring': 0.12, 'pinky': 0.10}
DIST_L  = {'thumb': 0.08, 'index': 0.09, 'middle': 0.10, 'ring': 0.09, 'pinky': 0.08}
FINGER_WIDTH = {'thumb': 0.082, 'index': 0.074, 'middle': 0.080,
                'ring': 0.072, 'pinky': 0.062}
FINGER_DEPTH = {'thumb': 0.052, 'index': 0.054, 'middle': 0.056,
                'ring': 0.052, 'pinky': 0.048}

# DIP angle is naturally coupled to PIP via tendons (~65% of PIP angle)
DIP_COUPLING = 0.65

# Thumb opposition: as the thumb curls, it rotates inward to oppose the fingertips.
# THUMB_SPLAY_REDUCTION: how many degrees the Z-splay decreases at full flex.
# THUMB_OPPOSITION_DEG: Y-axis rotation added at full flex (brings tip toward palm center).
THUMB_SPLAY_REDUCTION = 20.0
THUMB_OPPOSITION_DEG  = 30.0

# Ring and pinky coupling to middle finger (tendon-style passive follow)
RING_FOLLOW  = 0.90
PINKY_FOLLOW = 0.78

# Palm metacarpal bases: 5 nodes across the palm (X axis), fingers extend +Y
PALM_NODES = np.array([
    [-0.36, -0.06, 0.0],  # 0: thumb base
    [-0.18,  0.00, 0.0],  # 1: index base
    [ 0.00,  0.02, 0.0],  # 2: middle base
    [ 0.16,  0.00, 0.0],  # 3: ring base  (palm shape only, no finger rendered)
    [ 0.28, -0.04, 0.0],  # 4: pinky base (palm shape only, no finger rendered)
], dtype=np.float32)

# Wrist nodes: two anchors forming the heel of the hand
WRIST_L = np.array([-0.22, -0.32, 0.0], dtype=np.float32)
WRIST_R = np.array([ 0.22, -0.32, 0.0], dtype=np.float32)

# Finger definitions: (name, palm_node_idx, splay_deg, fu_idx, fl_idx, accent, passive_curl)
#   splay_deg    — outward Z-rotation from vertical in the rest pose
#   fu_idx       — F[i] index controlling proximal (MCP) segment curl
#   fl_idx       — F[i] index controlling intermediate (PIP) segment curl
#                  DIP is automatically coupled to PIP via DIP_COUPLING
#   passive_curl — None for live sensors, 'follow_middle' to derive from middle flex,
#                  or a fixed (upper, lower) tuple
FINGER_DEFS = [
    ('thumb',  0,  34, 0, 0, THUMB_COLOR,   None),
    ('index',  1,   4, 1, 2, INDEX_COLOR,   None),
    ('middle', 2,   0, 3, 4, MIDDLE_COLOR,  None),
    ('ring',   3,  -5, 0, 0, PASSIVE_COLOR, 'follow_middle'),
    ('pinky',  4, -12, 0, 0, PASSIVE_COLOR, 'follow_middle'),
]

# --- Rendering ---
TIMER_INTERVAL = 1.0 / 30.0   # 30 fps timer

# --- IMU accuracy HUD ---
ACC_COLOR = {0: '#FF3333', 1: '#FFAA00', 2: '#FFAA00', 3: '#44FF44'}
ACC_LABEL = {0: 'UNRELIABLE (0/3)', 1: 'LOW (1/3)',
             2: 'MED (2/3)',        3: 'HIGH (3/3)'}

# =============================================================================
# SECTION 2: SHARED STATE
# Written by the serial thread; read by the timer callback.
# All accesses protected by _lock.
# =============================================================================
_state = {
    'q':     [1.0, 0.0, 0.0, 0.0],  # latest quaternion [w, x, y, z]
    'f':     [0.0] * 5,              # latest flex values [0..1]
    'p':     [0.0] * 3,              # latest pressure values [0..1]
    'acc':   0,                      # BNO085 accuracy 0–3
    'times': deque(maxlen=40),       # F: line timestamps for Hz calculation
}
_lock  = threading.Lock()
_ref_q = [1.0, 0.0, 0.0, 0.0]   # calibration reference (press C to update)
_stop  = threading.Event()

# =============================================================================
# SECTION 3: SERIAL READER THREAD
# Opens the serial port, parses Q:, F:, and P: lines, updates _state.
# Auto-reconnects if port drops (USB CDC devices can disconnect briefly).
# dsrdtr=False, rtscts=False: prevents DTR/RTS from resetting the ESP32.
# =============================================================================
def _serial_thread():
    while not _stop.is_set():
        try:
            ser, port = open_glove_serial(BAUD_RATE, timeout=1)
            print(f'[serial] connected: {port}')

            while not _stop.is_set():
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith('Q:'):
                    # Q:w,x,y,z  — BNO085 Game Rotation Vector quaternion
                    parts = line[2:].split(',')
                    if len(parts) == 4:
                        try:
                            with _lock:
                                _state['q'] = list(map(float, parts))
                        except ValueError:
                            pass

                elif line.startswith('F:'):
                    # F:thumb,ui,li,um,lm  — normalized flex [0.0=open, 1.0=closed]
                    parts = line[2:].split(',')
                    if len(parts) == 5:
                        try:
                            flex = list(map(float, parts))
                            with _lock:
                                _state['f'] = flex
                                _state['times'].append(time.monotonic())
                        except ValueError:
                            pass

                elif line.startswith('P:'):
                    # P:p1,p2,p3 — normalized FSR pressure [0.0=none, 1.0=firm]
                    parts = line[2:].split(',')
                    if len(parts) == 3:
                        try:
                            pressure = list(map(float, parts))
                            with _lock:
                                _state['p'] = pressure
                        except ValueError:
                            pass

        except serial.SerialException as e:
            if not _stop.is_set():
                print(f'[serial] {e} — retrying in 2s')
                time.sleep(2)
        except Exception as e:
            if not _stop.is_set():
                print(f'[serial] unexpected error: {e} — retrying in 2s')
                time.sleep(2)

# =============================================================================
# SECTION 4: QUATERNION MATH
# Minimal quaternion library (no external dependency).
# =============================================================================

def _qmul(a, b):
    """Hamilton product of two unit quaternions [w, x, y, z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ]

def _qrel(q_ref, q_raw):
    """
    Compute relative quaternion: q_rel = q_ref^{-1} * q_raw.
    For a unit quaternion, the inverse is the conjugate: [w, -x, -y, -z].
    Subtracts the reference orientation so the hand shows its pose relative
    to whatever orientation the user pressed C to calibrate.
    """
    q_inv = [q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]]
    return _qmul(q_inv, q_raw)

def _qmat(w, x, y, z):
    """
    Convert unit quaternion to 3×3 rotation matrix.
    Standard formula: derived from rotating unit vectors with the quaternion.
    The resulting matrix R satisfies: R @ v rotates vector v.
    """
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-z*w),   2*(x*z+y*w)  ],
        [2*(x*y+z*w),    1-2*(x*x+z*z), 2*(y*z-x*w)  ],
        [2*(x*z-y*w),    2*(y*z+x*w),   1-2*(x*x+y*y)],
    ], dtype=np.float32)

# =============================================================================
# SECTION 5: ROBOT HAND GEOMETRY
# Computes a finger pose from flex values, then builds a solid low-poly mesh.
# =============================================================================

_UP = np.array([0., 1., 0.], dtype=np.float32)  # resting finger direction

def _rx(deg):
    """
    Rotation matrix around X axis (finger curl axis).
    Positive degrees rotate +Y toward +Z (forward curl).
    We use negative degrees so that flex > 0 curls fingers toward the viewer.
    """
    a = np.radians(deg);  c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float32)

def _rz(deg):
    """
    Rotation matrix around Z axis (finger splay axis).
    Used to angle the thumb ~30° outward from the palm.
    """
    a = np.radians(deg);  c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)

def _ry(deg):
    """
    Rotation matrix around Y axis (thumb opposition axis).
    Used to rotate the thumb inward (toward palm center) as it curls.
    Positive degrees rotate +X toward +Z; negative brings thumb tip toward -Z
    (same direction as finger curl), producing natural opposition.
    """
    a = np.radians(deg);  c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float32)

def _unit(v):
    """Return a normalized vector, falling back to +Y for degenerate segments."""
    n = float(np.linalg.norm(v))
    if n < 1e-7:
        return np.array([0., 1., 0.], dtype=np.float32)
    return (v / n).astype(np.float32)

def _segment_basis(p0, p1):
    """Build a stable local basis around a finger segment."""
    t = _unit(p1 - p0)
    ref = np.array([0., 0., 1.], dtype=np.float32)
    if abs(float(np.dot(t, ref))) > 0.92:
        ref = np.array([1., 0., 0.], dtype=np.float32)
    n1 = _unit(np.cross(t, ref))
    n2 = _unit(np.cross(n1, t))
    return t, n1, n2

def _append_mesh(vs, fs, cs, verts, faces, colors):
    """Append one mesh part into shared vertex/face/color buffers."""
    base = len(vs)
    vs.extend(verts)
    fs.extend((np.asarray(faces, dtype=np.uint32) + base).tolist())
    if np.asarray(colors).ndim == 1:
        cs.extend([np.asarray(colors, dtype=np.float32)] * len(verts))
    else:
        cs.extend(np.asarray(colors, dtype=np.float32))

def _sphere_mesh(center, radius, color, stacks=5, slices=12):
    """Small faceted sphere for dark robot joint caps."""
    center = np.asarray(center, dtype=np.float32)
    verts, faces = [], []
    for s in range(stacks + 1):
        theta = np.pi * s / stacks
        z = np.cos(theta) * radius
        rr = np.sin(theta) * radius
        for i in range(slices):
            phi = 2.0 * np.pi * i / slices
            verts.append(center + np.array([
                np.cos(phi) * rr, np.sin(phi) * rr, z
            ], dtype=np.float32))

    for s in range(stacks):
        for i in range(slices):
            j = (i + 1) % slices
            a = s * slices + i
            b = s * slices + j
            c = (s + 1) * slices + j
            d = (s + 1) * slices + i
            faces.append([a, b, c])
            faces.append([a, c, d])

    return np.asarray(verts, dtype=np.float32), np.asarray(faces, dtype=np.uint32), color

def _box_mesh(center, axes, half_sizes, color):
    """Oriented rectangular prism used for the blocky robot-hand shell."""
    center = np.asarray(center, dtype=np.float32)
    ax0, ax1, ax2 = [_unit(np.asarray(a, dtype=np.float32)) for a in axes]
    hx, hy, hz = [float(v) for v in half_sizes]
    verts = []
    for sx, sy, sz in (
        (-1, -1, -1), ( 1, -1, -1), ( 1,  1, -1), (-1,  1, -1),
        (-1, -1,  1), ( 1, -1,  1), ( 1,  1,  1), (-1,  1,  1),
    ):
        verts.append(center + sx * hx * ax0 + sy * hy * ax1 + sz * hz * ax2)

    faces = np.array([
        [0, 1, 2], [0, 2, 3],
        [4, 6, 5], [4, 7, 6],
        [0, 4, 5], [0, 5, 1],
        [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3],
        [3, 7, 4], [3, 4, 0],
    ], dtype=np.uint32)
    return np.asarray(verts, dtype=np.float32), faces, color

def _box_between_mesh(p0, p1, width, depth, color, shrink=0.78):
    """Build a robot phalanx plate between two joint centers."""
    p0 = np.asarray(p0, dtype=np.float32)
    p1 = np.asarray(p1, dtype=np.float32)
    length = float(np.linalg.norm(p1 - p0))
    t, n1, n2 = _segment_basis(p0, p1)
    half_sizes = (
        float(width) * 0.5,
        max(length * float(shrink) * 0.5, 0.001),
        float(depth) * 0.5,
    )
    return _box_mesh((p0 + p1) * 0.5, (n1, t, n2), half_sizes, color)

def _hinge_block_mesh(joint, neighbor, width, depth):
    """Dark blocky hinge centered at a finger joint."""
    joint = np.asarray(joint, dtype=np.float32)
    neighbor = np.asarray(neighbor, dtype=np.float32)
    t, n1, n2 = _segment_basis(joint, neighbor)
    half_sizes = (float(width) * 0.58, float(width) * 0.18, float(depth) * 0.62)
    return _box_mesh(joint, (n1, t, n2), half_sizes, JOINT_COLOR)

def _sensor_plate_mesh(p0, p1, width, depth, color):
    """Slim colored inset on live sensor-driven finger plates."""
    p0 = np.asarray(p0, dtype=np.float32)
    p1 = np.asarray(p1, dtype=np.float32)
    _t, _n1, n2 = _segment_basis(p0, p1)
    front_offset = n2 * float(depth) * 0.58
    return _box_between_mesh(p0 + front_offset, p1 + front_offset,
                             float(width) * 0.22, 0.008, color, shrink=0.58)

def _append_segment_rivets(vs, fs, cs, p0, p1, width, depth):
    """Add small front-face screw heads to a robot finger plate."""
    p0 = np.asarray(p0, dtype=np.float32)
    p1 = np.asarray(p1, dtype=np.float32)
    t, n1, n2 = _segment_basis(p0, p1)
    for along in (0.27, 0.73):
        base = p0 + (p1 - p0) * along + n2 * float(depth) * 0.61
        for side in (-0.26, 0.26):
            verts, faces, color = _sphere_mesh(
                base + n1 * float(width) * side,
                max(float(width) * 0.045, 0.0035),
                RIVET_COLOR,
                stacks=3,
                slices=8,
            )
            _append_mesh(vs, fs, cs, verts, faces, color)

def _palm_plate_mesh():
    """Extruded palm shell shaped like a broad robot-hand back plate."""
    contour = np.array([
        [-0.34, -0.36], [ 0.25, -0.36], [ 0.34, -0.30],
        [ 0.38, -0.18], [ 0.38,  0.05], [ 0.31,  0.14],
        [ 0.12,  0.17], [-0.12,  0.17], [-0.33,  0.12],
        [-0.42,  0.00], [-0.42, -0.22], [-0.39, -0.31],
    ], dtype=np.float32)
    z0, z1 = -0.065, 0.065
    n = len(contour)
    verts = [[x, y, z0] for x, y in contour] + [[x, y, z1] for x, y in contour]
    verts.append([float(contour[:, 0].mean()), float(contour[:, 1].mean()), z0])
    verts.append([float(contour[:, 0].mean()), float(contour[:, 1].mean()), z1])
    bottom_center = n * 2
    top_center = n * 2 + 1
    faces = []

    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j])
        faces.append([i, n + j, n + i])
        faces.append([bottom_center, j, i])
        faces.append([top_center, n + i, n + j])

    colors = [PALM_COLOR] * n + [PALM_TOP] * n + [PALM_COLOR, PALM_TOP]
    return (np.asarray(verts, dtype=np.float32),
            np.asarray(faces, dtype=np.uint32),
            np.asarray(colors, dtype=np.float32))

def compute_hand_pose(flex):
    """
    Return articulated joint centers for all five fingers.
    Each finger now has 4 positions: (base, knuckle1, knuckle2, tip)
    corresponding to MCP, PIP, DIP joints + fingertip.

    Thumb/index/middle are live (sensor-driven).
    Ring/pinky dynamically follow middle finger scaled by RING/PINKY_FOLLOW,
    producing the natural tendon-coupled grasping closure.
    Thumb adds opposition: as it curls, splay reduces and it rotates inward.
    DIP angle is coupled to PIP via DIP_COUPLING (≈65%) as in real hands.
    """
    fingers = []

    # Pre-sample middle finger flex for ring/pinky coupling
    mid_fu = float(np.clip(flex[3], 0.0, 1.0))
    mid_fl = float(np.clip(flex[4], 0.0, 1.0))

    for (name, pidx, splay, fui, fli, accent, passive) in FINGER_DEFS:
        base = PALM_NODES[pidx]

        if passive == 'follow_middle':
            # Ring and pinky couple to middle via a tendon-like scaling.
            # This makes all four fingers curl together when grasping.
            scale = RING_FOLLOW if name == 'ring' else PINKY_FOLLOW
            fu   = mid_fu * scale
            fl   = mid_fl * scale
            live = False
        elif passive is None:
            fu   = float(np.clip(flex[fui], 0.0, 1.0))
            fl   = float(np.clip(flex[fli], 0.0, 1.0))
            live = True
        else:
            fu, fl = passive
            live   = False

        # DIP naturally follows PIP via the flexor digitorum profundus tendon
        fd = fl * DIP_COUPLING

        if name == 'thumb':
            # Opposition motion: as the thumb curls the splay narrows and the
            # whole thumb rotates around Y toward the fingertip side.
            splay_actual = splay - fu * THUMB_SPLAY_REDUCTION
            R_sp = _rz(splay_actual) @ _ry(-fu * THUMB_OPPOSITION_DEG)
        else:
            R_sp = _rz(splay)

        R_mcp = _rx(-fu * MAX_CURL)   # proximal phalanx rotation
        R_pip = _rx(-fl * MAX_CURL)   # intermediate phalanx rotation
        R_dip = _rx(-fd * MAX_CURL)   # distal phalanx rotation (coupled)

        dir_mcp  = R_sp @ R_mcp @ _UP
        knuckle1 = base + dir_mcp * PROX_L[name]       # PIP joint center

        dir_pip  = R_sp @ R_mcp @ R_pip @ _UP
        knuckle2 = knuckle1 + dir_pip * INTER_L[name]  # DIP joint center

        dir_dip  = R_sp @ R_mcp @ R_pip @ R_dip @ _UP
        tip      = knuckle2 + dir_dip * DIST_L[name]   # fingertip

        fingers.append({
            'name':   name,
            'joints': (base, knuckle1, knuckle2, tip),
            'accent': accent,
            'live':   live,
        })
    return fingers

def build_hand_mesh(flex, pressure, R_imu):
    """
    Build a complete robot-hand mesh in local coordinates, then rotate it by IMU.
    Returns vertices, faces, and per-vertex colors for one VisPy Mesh visual.
    """
    vs, fs, cs = [], [], []
    world_axes = (
        np.array([1., 0., 0.], dtype=np.float32),
        np.array([0., 1., 0.], dtype=np.float32),
        np.array([0., 0., 1.], dtype=np.float32),
    )

    # Light wrist/base block below the black hand, matching the reference mount.
    for center, half_sizes, color in (
        (np.array([-0.02, -0.58, 0.0], dtype=np.float32), (0.20, 0.13, 0.075), BASE_COLOR),
        (np.array([-0.02, -0.44, 0.0], dtype=np.float32), (0.17, 0.035, 0.082), BASE_TOP),
        (np.array([-0.02, -0.39, 0.0], dtype=np.float32), (0.13, 0.035, 0.073), JOINT_COLOR),
    ):
        verts, faces, color = _box_mesh(center, world_axes, half_sizes, color)
        _append_mesh(vs, fs, cs, verts, faces, color)

    for x in (-0.15, 0.11):
        for y in (-0.51, -0.63):
            verts, faces, color = _sphere_mesh(
                np.array([x, y, 0.079], dtype=np.float32),
                0.010,
                RIVET_COLOR,
                stacks=3,
                slices=8,
            )
            _append_mesh(vs, fs, cs, verts, faces, color)

    verts, faces, colors = _palm_plate_mesh()
    _append_mesh(vs, fs, cs, verts, faces, colors)

    # Small rectangular detail and screw heads on the palm shell front.
    verts, faces, color = _box_mesh(
        np.array([-0.04, -0.30, 0.071], dtype=np.float32),
        world_axes,
        (0.055, 0.008, 0.004),
        BASE_TOP,
    )
    _append_mesh(vs, fs, cs, verts, faces, color)
    for x, y in ((-0.30, -0.24), (0.28, -0.24), (-0.29, 0.03), (0.27, 0.04)):
        verts, faces, color = _sphere_mesh(
            np.array([x, y, 0.073], dtype=np.float32),
            0.009,
            RIVET_COLOR,
            stacks=3,
            slices=8,
        )
        _append_mesh(vs, fs, cs, verts, faces, color)

    live_tips = []
    for finger in compute_hand_pose(flex):
        name = finger['name']
        base, knuckle1, knuckle2, tip = finger['joints']   # 4 joints, 3 segments
        width = FINGER_WIDTH[name]
        depth = FINGER_DEPTH[name]
        body_color = FINGER_BODY if finger['live'] else PASSIVE_COLOR
        if finger['live']:
            live_tips.append(tip)

        # Three phalanx plates: MCP→PIP, PIP→DIP, DIP→tip
        # Width tapers toward the fingertip (as in real robotic fingers)
        for p0, p1, w_scale in (
            (base,     knuckle1, 1.00),   # proximal phalanx
            (knuckle1, knuckle2, 0.88),   # intermediate phalanx
            (knuckle2, tip,      0.74),   # distal phalanx
        ):
            seg_width = width * w_scale
            verts, faces, color = _box_between_mesh(
                p0, p1, seg_width, depth, body_color, shrink=0.72
            )
            _append_mesh(vs, fs, cs, verts, faces, color)
            if finger['live']:
                verts, faces, color = _sensor_plate_mesh(
                    p0, p1, seg_width, depth, finger['accent']
                )
                _append_mesh(vs, fs, cs, verts, faces, color)
            _append_segment_rivets(vs, fs, cs, p0, p1, seg_width, depth)

        # Hinge blocks at every joint (MCP, PIP, DIP) plus a fingertip end-cap
        for joint, neighbor, scale in (
            (base,     knuckle1, 1.00),   # MCP
            (knuckle1, knuckle2, 0.90),   # PIP
            (knuckle2, tip,      0.78),   # DIP
            (tip,      knuckle2, 0.64),   # fingertip cap
        ):
            verts, faces, color = _hinge_block_mesh(
                joint, neighbor, width * scale, depth
            )
            _append_mesh(vs, fs, cs, verts, faces, color)

    for i, tip in enumerate(live_tips[:3]):
        p = float(np.clip(pressure[i], 0.0, 1.0)) if i < len(pressure) else 0.0
        color = JOINT_COLOR * (1.0 - p) + PRESSURE_COLOR * p
        center = np.asarray(tip, dtype=np.float32) + np.array(
            [0.0, 0.0, 0.076], dtype=np.float32
        )
        verts, faces, color = _sphere_mesh(
            center,
            0.012 + 0.030 * p,
            color,
            stacks=4,
            slices=10,
        )
        _append_mesh(vs, fs, cs, verts, faces, color)

    vertices = np.asarray(vs, dtype=np.float32)
    vertices = (R_imu @ vertices.T).T
    return (vertices,
            np.asarray(fs, dtype=np.uint32),
            np.asarray(cs, dtype=np.float32))

# =============================================================================
# SECTION 6: FLEX BAR GEOMETRY (2D panel coordinate space)
# Bars fill upward (increasing y) as flex increases from 0.0 to 1.0.
# All 5 bars share a single Mesh visual, rebuilt each timer tick.
# =============================================================================

def _bar_quad(cx, flex_val):
    """
    Build a filled quad for one flex bar.
    Bottom is fixed at BAR_BOT; top rises to BAR_BOT + flex_val * BAR_MAX_H.
    Minimum height of 1 unit prevents a degenerate (zero-area) mesh.
    """
    h  = max(flex_val * BAR_MAX_H, 1.0)
    x0, x1 = cx - BAR_W / 2.0, cx + BAR_W / 2.0
    y0, y1 = float(BAR_BOT), float(BAR_BOT + h)
    return np.array([
        [x0, y0, 0.], [x1, y0, 0.],
        [x1, y1, 0.], [x0, y1, 0.],
    ], dtype=np.float32)

def build_fill_mesh(flex_vals):
    """Combine all 5 fill quads into one Mesh (vertices, faces, vertex_colors)."""
    vs, fs, cs = [], [], []
    for i, fv in enumerate(flex_vals):
        v  = _bar_quad(BAR_CX[i], fv)
        b  = len(vs)
        vs.extend(v)
        fs.extend([[b, b+1, b+2], [b, b+2, b+3]])
        cs.extend([BAR_COLORS[i]] * 4)
    return (np.array(vs, dtype=np.float32),
            np.array(fs, dtype=np.uint32),
            np.array(cs, dtype=np.float32))

def build_track_mesh():
    """Build static background track quads (full-height dark rectangles)."""
    vs, fs, cs = [], [], []
    for i in range(5):
        x0, x1 = BAR_CX[i] - BAR_W / 2., BAR_CX[i] + BAR_W / 2.
        y0, y1 = float(BAR_BOT), float(BAR_TOP)
        b = len(vs)
        vs.extend([[x0,y0,0.],[x1,y0,0.],[x1,y1,0.],[x0,y1,0.]])
        fs.extend([[b,b+1,b+2],[b,b+2,b+3]])
        cs.extend([BAR_TRACK_COLOR] * 4)
    return (np.array(vs, dtype=np.float32),
            np.array(fs, dtype=np.uint32),
            np.array(cs, dtype=np.float32))

def _pressure_quad(idx, pressure_val, full_width=False):
    """Build one horizontal pressure bar in panel coordinates."""
    val = 1.0 if full_width else float(np.clip(pressure_val, 0.0, 1.0))
    x0 = float(PRESSURE_X0)
    width = PRESSURE_W if full_width else max(PRESSURE_W * val, 1.0)
    x1 = float(PRESSURE_X0 + width)
    y0 = float(PRESSURE_Y[idx])
    y1 = y0 + float(PRESSURE_H)
    return np.array([
        [x0, y0, 0.], [x1, y0, 0.],
        [x1, y1, 0.], [x0, y1, 0.],
    ], dtype=np.float32)

def build_pressure_track_mesh():
    """Build static horizontal pressure bar tracks."""
    vs, fs, cs = [], [], []
    for i in range(3):
        v = _pressure_quad(i, 1.0, full_width=True)
        b = len(vs)
        vs.extend(v)
        fs.extend([[b, b+1, b+2], [b, b+2, b+3]])
        cs.extend([PRESSURE_TRACK_COLOR] * 4)
    return (np.array(vs, dtype=np.float32),
            np.array(fs, dtype=np.uint32),
            np.array(cs, dtype=np.float32))

def build_pressure_fill_mesh(pressure_vals):
    """Build dynamic pressure bar fills."""
    vs, fs, cs = [], [], []
    for i, pv in enumerate(pressure_vals):
        v = _pressure_quad(i, pv)
        b = len(vs)
        vs.extend(v)
        fs.extend([[b, b+1, b+2], [b, b+2, b+3]])
        intensity = float(np.clip(pv, 0.0, 1.0))
        color = PRESSURE_TRACK_COLOR * (1.0 - intensity) + PRESSURE_COLOR * intensity
        cs.extend([color.astype(np.float32)] * 4)
    return (np.array(vs, dtype=np.float32),
            np.array(fs, dtype=np.uint32),
            np.array(cs, dtype=np.float32))

# =============================================================================
# SECTION 7: SERIAL THREAD START
# =============================================================================
threading.Thread(target=_serial_thread, daemon=True).start()

# =============================================================================
# SECTION 8: VISPY SCENE SETUP
# Grid layout: 3D viewport (left 75%) + 2D flex panel (right 25%).
# col_span=3 of 4 equal columns → 960px for 3D, 320px for panel at 1280 wide.
# =============================================================================
canvas = scene.SceneCanvas(
    title='Glove Visualizer v1.0',
    keys='interactive',
    size=(WIN_W, WIN_H),
    bgcolor=BG_COLOR,
    show=True,
)

grid    = canvas.central_widget.add_grid(margin=0)
view3d  = grid.add_view(row=0, col=0, col_span=3)
view2d  = grid.add_view(row=0, col=3, col_span=1)

# --- 3D camera: turntable (orbit with left-drag, zoom with scroll) ---
view3d.camera = scene.TurntableCamera(
    fov=38, distance=2.05, elevation=70, azimuth=0
)
view3d.camera.center = (0.0, -0.05, 0.0)  # keep the wrist mount in frame

# --- 2D panel camera: fixed y-up coordinate space [0,PANEL_W] × [0,WIN_H] ---
view2d.camera = scene.PanZoomCamera(
    rect=(0, 0, PANEL_W, WIN_H), aspect=None
)

# ---- 3D hand visual ----
_hv0, _hf0, _hc0 = build_hand_mesh(
    [0.0] * 5, [0.0] * 3, np.eye(3, dtype=np.float32)
)
hand_mesh = scene.visuals.Mesh(
    vertices=_hv0, faces=_hf0, vertex_colors=_hc0,
    parent=view3d.scene
)

# XYZ reference axes in the 3D scene (X=red, Y=green, Z=blue)
scene.visuals.XYZAxis(parent=view3d.scene)

# ---- 2D flex panel visuals ----

# Static dark background tracks (one per bar, built once)
_tv, _tf, _tc = build_track_mesh()
bar_tracks = scene.visuals.Mesh(
    vertices=_tv, faces=_tf, vertex_colors=_tc,
    parent=view2d.scene
)

# Dynamic fill bars (rebuilt each frame to update heights)
_fv, _ff, _fc = build_fill_mesh([0.0] * 5)
bar_fills = scene.visuals.Mesh(
    vertices=_fv, faces=_ff, vertex_colors=_fc,
    parent=view2d.scene
)

# Static and dynamic pressure bars near the top of the sensor panel.
_ptv, _ptf, _ptc = build_pressure_track_mesh()
pressure_tracks = scene.visuals.Mesh(
    vertices=_ptv, faces=_ptf, vertex_colors=_ptc,
    parent=view2d.scene
)

_ppv, _ppf, _ppc = build_pressure_fill_mesh([0.0] * 3)
pressure_fills = scene.visuals.Mesh(
    vertices=_ppv, faces=_ppf, vertex_colors=_ppc,
    parent=view2d.scene
)

# Bar label text above each track (static)
for i, (lbl, lc) in enumerate(zip(BAR_LABELS, BAR_LABEL_COLORS)):
    scene.visuals.Text(
        lbl, pos=(BAR_CX[i], BAR_LBL_Y),
        color=lc, font_size=11, bold=True,
        anchor_x='center', anchor_y='bottom',
        parent=view2d.scene
    )

# Panel section title (static)
scene.visuals.Text(
    'FLEX SENSORS', pos=(PANEL_W / 2.0, WIN_H - 18),
    color='#888888', font_size=11, bold=True,
    anchor_x='center', anchor_y='top',
    parent=view2d.scene
)

scene.visuals.Text(
    'PRESSURE', pos=(PANEL_W / 2.0, 756),
    color='#AA3333', font_size=10, bold=True,
    anchor_x='center', anchor_y='top',
    parent=view2d.scene
)

pressure_value_texts = []
for i, lbl in enumerate(PRESSURE_LABELS):
    scene.visuals.Text(
        lbl, pos=(PRESSURE_X0 - 14, PRESSURE_Y[i] + PRESSURE_H / 2.0),
        color='#AA3333', font_size=8, bold=True,
        anchor_x='right', anchor_y='center',
        parent=view2d.scene
    )
    t = scene.visuals.Text(
        '0.00', pos=(PRESSURE_X1 + 8, PRESSURE_Y[i] + PRESSURE_H / 2.0),
        color='#AA3333', font_size=8,
        anchor_x='left', anchor_y='center',
        parent=view2d.scene
    )
    pressure_value_texts.append(t)

# Dynamic flex value text below each bar (updated each tick)
value_texts = []
for i in range(5):
    t = scene.visuals.Text(
        '0.00', pos=(BAR_CX[i], BAR_VAL_Y),
        color='#CCCCCC', font_size=9,
        anchor_x='center', anchor_y='top',
        parent=view2d.scene
    )
    value_texts.append(t)

# Left border divider line of the panel (vertical line at x=0)
scene.visuals.Line(
    pos=np.array([[1., 0., 0.], [1., WIN_H, 0.]], dtype=np.float32),
    color=(0.25, 0.25, 0.35, 1.0), width=1,
    parent=view2d.scene
)

# ---- HUD text overlays ----
# Attached to canvas.scene (screen pixel coordinates: origin top-left, y downward)
hud_title = scene.visuals.Text(
    'GLOVE VIS v1.0', pos=(14, 16),
    color='#111111', font_size=14, bold=True,
    anchor_x='left', anchor_y='top',
    parent=canvas.scene
)
hud_quat = scene.visuals.Text(
    'Q: waiting for serial...', pos=(14, 42),
    color='#333366', font_size=9,
    anchor_x='left', anchor_y='top',
    parent=canvas.scene
)
hud_acc = scene.visuals.Text(
    'IMU Accuracy: --', pos=(14, 62),
    color='#FF4444', font_size=9,
    anchor_x='left', anchor_y='top',
    parent=canvas.scene
)
hud_cal = scene.visuals.Text(
    'Press C to calibrate neutral pose', pos=(14, 80),
    color='#555555', font_size=8,
    anchor_x='left', anchor_y='top',
    parent=canvas.scene
)
hud_rate = scene.visuals.Text(
    '-- samples/sec', pos=(14, WIN_H - 14),
    color='#555555', font_size=8,
    anchor_x='left', anchor_y='bottom',
    parent=canvas.scene
)
# =============================================================================
# SECTION 9: TIMER CALLBACK (~30 fps)
# Reads latest sensor state and updates all dynamic visuals.
# =============================================================================
_frame = 0
_smooth_flex = np.zeros(5, dtype=np.float32)

def on_timer(_ev):
    global _frame, _ref_q, _smooth_flex
    _frame += 1

    # Snapshot shared state (grab lock briefly, copy values out, release)
    with _lock:
        q     = list(_state['q'])
        flex  = list(_state['f'])
        pressure = list(_state['p'])
        acc   = int(_state['acc'])
        times = list(_state['times'])

    # ---- IMU rotation matrix ----
    # Remove calibration reference then convert to 3×3 rotation matrix.
    # The rotation matrix is applied to all hand vertices before rendering.
    qr = _qrel(_ref_q, q)
    R  = _qmat(*qr)

    # ---- Update 3D robot hand ----
    # Smooth only the 3D mesh. Flex bars stay raw for wiring/debug visibility.
    raw_flex = np.asarray(flex, dtype=np.float32)
    _smooth_flex += (raw_flex - _smooth_flex) * FLEX_SMOOTH_ALPHA
    hv, hf, hc = build_hand_mesh(_smooth_flex, pressure, R)
    hand_mesh.set_data(vertices=hv, faces=hf, vertex_colors=hc)

    # ---- Update flex bars ----
    fv, ff, fc = build_fill_mesh(flex)
    bar_fills.set_data(vertices=fv, faces=ff, vertex_colors=fc)

    for i, txt in enumerate(value_texts):
        txt.text = f'{flex[i]:.2f}'

    pv, pf, pc = build_pressure_fill_mesh(pressure)
    pressure_fills.set_data(vertices=pv, faces=pf, vertex_colors=pc)
    for i, txt in enumerate(pressure_value_texts):
        txt.text = f'{pressure[i]:.2f}'

    # ---- Update HUD ----
    hud_quat.text = (
        f'W:{q[0]:+.3f}  X:{q[1]:+.3f}  Y:{q[2]:+.3f}  Z:{q[3]:+.3f}'
    )
    hud_acc.text  = f'IMU Accuracy: {ACC_LABEL.get(acc, "?")}'
    hud_acc.color = ACC_COLOR.get(acc, '#FF3333')

    # Rolling sample rate over last 40 F: arrivals
    if len(times) >= 2:
        dt   = times[-1] - times[0]
        rate = (len(times) - 1) / dt if dt > 0 else 0.0
        hud_rate.text = f'{rate:.1f} samples/sec'

    canvas.update()

timer = app.Timer(interval=TIMER_INTERVAL, connect=on_timer, start=True)

# =============================================================================
# SECTION 10: KEYBOARD HANDLER
# C — capture current orientation as the neutral/calibrated reference pose.
#     All subsequent orientations are displayed relative to this pose.
# =============================================================================
def on_key_press(ev):
    global _ref_q
    if ev.key.name.lower() == 'c':
        with _lock:
            _ref_q = list(_state['q'])
        print(f'[calibrate] neutral pose captured: '
              f'{[f"{v:.4f}" for v in _ref_q]}')
        hud_cal.text  = 'Calibrated — C to recalibrate'
        hud_cal.color = '#44AA44'

canvas.events.key_press.connect(on_key_press)

# =============================================================================
# SECTION 11: MAIN
# =============================================================================
print('[glove] window open — press C to calibrate the neutral hand pose')
print('[glove] left-drag to orbit  |  scroll to zoom  |  Ctrl+C to quit')
app.run()
_stop.set()   # signal serial thread to exit cleanly
