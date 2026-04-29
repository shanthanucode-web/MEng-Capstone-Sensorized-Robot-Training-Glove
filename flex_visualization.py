#!/usr/bin/env python3
"""
flex_visualization.py — Flex Sensor Hand Skeleton Test Environment
===================================================================
Shows a skeletal hand with bending fingers driven purely by flex sensor data.
No IMU — the hand is fixed at a diagonal angle chosen to clearly show all
three fingers curling. Use this to verify and calibrate flex sensors.

Dependencies:
    pip install vispy pyserial numpy pyopengl

Usage:
    python3 flex_visualization.py
    Close the PlatformIO serial monitor first.
    Left-drag to orbit  |  Scroll to zoom

Serial input on an auto-detected USB CDC port at 115200 baud — only F: lines are used:
    F:thumb,ui,li,um,lm   (normalized 0.0=open .. 1.0=closed)

Sensor mapping:
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
# =============================================================================

# --- Window ---
WIN_W, WIN_H = 1280, 800

# --- Serial ---
BAUD_RATE   = 115200

# --- Colors (numpy RGBA float32 arrays) ---
BG_COLOR     = '#0A0A1A'
PALM_COLOR   = np.array([0.267, 0.267, 0.267, 1.00], dtype=np.float32)
THUMB_COLOR  = np.array([0.45,  0.45,  0.45,  0.55], dtype=np.float32)
INDEX_COLOR  = np.array([0.0,   1.0,   1.0,   1.00], dtype=np.float32)
MIDDLE_COLOR = np.array([1.0,   0.549, 0.0,   1.00], dtype=np.float32)
JOINT_COLOR  = np.array([1.0,   1.0,   1.0,   1.00], dtype=np.float32)

# Flex indicator bar colors [Thumb, UpperIndex, LowerIndex, UpperMiddle, LowerMiddle]
BAR_COLORS = [
    np.array([0.50,  0.50,  0.50,  0.85], dtype=np.float32),
    np.array([0.00,  1.00,  1.00,  0.90], dtype=np.float32),
    np.array([0.00,  0.60,  0.60,  0.90], dtype=np.float32),
    np.array([1.00,  0.549, 0.00,  0.90], dtype=np.float32),
    np.array([0.72,  0.39,  0.00,  0.90], dtype=np.float32),
]
BAR_TRACK_COLOR  = np.array([0.10, 0.10, 0.20, 0.85], dtype=np.float32)
BAR_LABEL_COLORS = ['#888888', '#00FFFF', '#00AAAA', '#FF8C00', '#BB6600']
BAR_LABELS       = ['T', 'UI', 'LI', 'UM', 'LM']

# --- Flex panel 2D layout ---
PANEL_W   = 320
BAR_W     = 42
BAR_MAX_H = 520
BAR_BOT   = 80
BAR_TOP   = BAR_BOT + BAR_MAX_H
BAR_LBL_Y = BAR_TOP + 16
BAR_VAL_Y = BAR_BOT - 22
_s = PANEL_W / 6.0
BAR_CX = [_s * (i + 1) for i in range(5)]

# --- Hand geometry ---
MAX_CURL = 85.0

PROX_L = {'thumb': 0.18, 'index': 0.26, 'middle': 0.28}
DIST_L = {'thumb': 0.14, 'index': 0.16, 'middle': 0.18}

PALM_NODES = np.array([
    [-0.36, -0.06, 0.0],  # 0: thumb base
    [-0.18,  0.00, 0.0],  # 1: index base
    [ 0.00,  0.02, 0.0],  # 2: middle base
    [ 0.16,  0.00, 0.0],  # 3: ring base
    [ 0.28, -0.04, 0.0],  # 4: pinky base
], dtype=np.float32)

WRIST_L = np.array([-0.22, -0.32, 0.0], dtype=np.float32)
WRIST_R = np.array([ 0.22, -0.32, 0.0], dtype=np.float32)

FINGER_DEFS = [
    ('thumb',  0, 30, 0, 0, THUMB_COLOR ),
    ('index',  1,  0, 1, 2, INDEX_COLOR ),
    ('middle', 2,  0, 3, 4, MIDDLE_COLOR),
]

TIMER_INTERVAL = 1.0 / 30.0

# =============================================================================
# SECTION 2: SHARED STATE
# =============================================================================
_state = {
    'f':     [0.0] * 5,
    'times': deque(maxlen=40),
}
_lock = threading.Lock()
_stop = threading.Event()

# =============================================================================
# SECTION 3: SERIAL READER THREAD — F: lines only
# =============================================================================
def _serial_thread():
    while not _stop.is_set():
        try:
            ser, port = open_glove_serial(BAUD_RATE, timeout=1)
            print(f'[serial] connected: {port}')

            while not _stop.is_set():
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith('F:'):
                    parts = line[2:].split(',')
                    if len(parts) == 5:
                        try:
                            flex = list(map(float, parts))
                            with _lock:
                                _state['f'] = flex
                                _state['times'].append(time.monotonic())
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
# SECTION 4: HAND SKELETON GEOMETRY
# =============================================================================
_UP = np.array([0., 1., 0.], dtype=np.float32)

def _rx(deg):
    a = np.radians(deg);  c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float32)

def _rz(deg):
    a = np.radians(deg);  c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)

def compute_skeleton(flex, R_imu):
    v_s, v_e, v_c = [], [], []
    j_pos, j_col  = [], []

    for i in range(4):
        v_s.append(PALM_NODES[i]);   v_e.append(PALM_NODES[i + 1])
        v_c += [PALM_COLOR, PALM_COLOR]
    v_s.append(PALM_NODES[0]);  v_e.append(WRIST_L);  v_c += [PALM_COLOR, PALM_COLOR]
    v_s.append(PALM_NODES[4]);  v_e.append(WRIST_R);  v_c += [PALM_COLOR, PALM_COLOR]
    v_s.append(WRIST_L);        v_e.append(WRIST_R);  v_c += [PALM_COLOR, PALM_COLOR]

    for node in PALM_NODES:
        j_pos.append(node);  j_col.append(JOINT_COLOR)
    j_pos += [WRIST_L, WRIST_R]
    j_col += [JOINT_COLOR, JOINT_COLOR]

    for (name, pidx, splay, fui, fli, color) in FINGER_DEFS:
        base = PALM_NODES[pidx]
        fu = float(np.clip(flex[fui], 0.0, 1.0))
        fl = float(np.clip(flex[fli], 0.0, 1.0))

        R_sp = _rz(splay)
        R_u  = _rx(-fu * MAX_CURL)
        R_l  = _rx(-fl * MAX_CURL)

        dir_u   = R_sp @ R_u @ _UP
        knuckle = base + dir_u * PROX_L[name]
        dir_l   = R_sp @ R_u @ R_l @ _UP
        tip     = knuckle + dir_l * DIST_L[name]

        v_s += [base, knuckle];   v_e += [knuckle, tip]
        v_c += [color, color, color, color]
        j_pos += [knuckle, tip]
        j_col += [JOINT_COLOR, JOINT_COLOR]

    n = len(v_s)
    bone_verts  = np.empty((n * 2, 3), dtype=np.float32)
    bone_verts[0::2] = np.array(v_s, dtype=np.float32)
    bone_verts[1::2] = np.array(v_e, dtype=np.float32)
    bone_colors = np.array(v_c,    dtype=np.float32)
    joint_pos   = np.array(j_pos,  dtype=np.float32)
    joint_col   = np.array(j_col,  dtype=np.float32)

    bone_verts = (R_imu @ bone_verts.T).T
    joint_pos  = (R_imu @ joint_pos.T).T

    return bone_verts, bone_colors, joint_pos, joint_col

# =============================================================================
# SECTION 5: FLEX BAR GEOMETRY
# =============================================================================
def _bar_quad(cx, flex_val):
    h  = max(flex_val * BAR_MAX_H, 1.0)
    x0, x1 = cx - BAR_W / 2.0, cx + BAR_W / 2.0
    y0, y1 = float(BAR_BOT), float(BAR_BOT + h)
    return np.array([
        [x0, y0, 0.], [x1, y0, 0.],
        [x1, y1, 0.], [x0, y1, 0.],
    ], dtype=np.float32)

def build_fill_mesh(flex_vals):
    vs, fs, cs = [], [], []
    for i, fv in enumerate(flex_vals):
        v = _bar_quad(BAR_CX[i], fv)
        b = len(vs)
        vs.extend(v)
        fs.extend([[b, b+1, b+2], [b, b+2, b+3]])
        cs.extend([BAR_COLORS[i]] * 4)
    return (np.array(vs, dtype=np.float32),
            np.array(fs, dtype=np.uint32),
            np.array(cs, dtype=np.float32))

def build_track_mesh():
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

# =============================================================================
# SECTION 6: SERIAL THREAD START
# =============================================================================
threading.Thread(target=_serial_thread, daemon=True).start()

# =============================================================================
# SECTION 7: VISPY SCENE SETUP
# =============================================================================
canvas = scene.SceneCanvas(
    title='Flex Sensor — Hand Skeleton',
    keys='interactive',
    size=(WIN_W, WIN_H),
    bgcolor=BG_COLOR,
    show=True,
)

grid   = canvas.central_widget.add_grid(margin=0)
view3d = grid.add_view(row=0, col=0, col_span=3)
view2d = grid.add_view(row=0, col=3, col_span=1)

# Front-facing camera — straight on, slightly elevated
view3d.camera = scene.TurntableCamera(
    fov=38, distance=2.0, elevation=20, azimuth=0,
    interactive=True
)
view3d.camera.center = (0.0, 0.0, 0.0)

view2d.camera = scene.PanZoomCamera(rect=(0, 0, PANEL_W, WIN_H), aspect=None)

# ---- 3D hand visuals ----
_R0 = np.eye(3, dtype=np.float32)
_bv0, _bc0, _jp0, _jc0 = compute_skeleton([0.0] * 5, _R0)

hand_lines = scene.visuals.Line(
    pos=_bv0, color=_bc0, connect='segments',
    width=3, antialias=True,
    parent=view3d.scene
)
hand_joints = scene.visuals.Markers(parent=view3d.scene)
hand_joints.set_data(pos=_jp0, face_color=_jc0, symbol='disc', size=11, edge_width=0)

scene.visuals.XYZAxis(parent=view3d.scene)

# ---- 2D flex panel visuals ----
_tv, _tf, _tc = build_track_mesh()
scene.visuals.Mesh(vertices=_tv, faces=_tf, vertex_colors=_tc, parent=view2d.scene)

_fv, _ff, _fc = build_fill_mesh([0.0] * 5)
bar_fills = scene.visuals.Mesh(vertices=_fv, faces=_ff, vertex_colors=_fc, parent=view2d.scene)

for i, (lbl, lc) in enumerate(zip(BAR_LABELS, BAR_LABEL_COLORS)):
    scene.visuals.Text(
        lbl, pos=(BAR_CX[i], BAR_LBL_Y),
        color=lc, font_size=11, bold=True,
        anchor_x='center', anchor_y='bottom',
        parent=view2d.scene
    )

scene.visuals.Text(
    'FLEX SENSORS', pos=(PANEL_W / 2.0, WIN_H - 18),
    color='#888888', font_size=11, bold=True,
    anchor_x='center', anchor_y='top',
    parent=view2d.scene
)

value_texts = []
for i in range(5):
    t = scene.visuals.Text(
        '0.00', pos=(BAR_CX[i], BAR_VAL_Y),
        color='#CCCCCC', font_size=9,
        anchor_x='center', anchor_y='top',
        parent=view2d.scene
    )
    value_texts.append(t)

scene.visuals.Line(
    pos=np.array([[1., 0., 0.], [1., WIN_H, 0.]], dtype=np.float32),
    color=(0.25, 0.25, 0.35, 1.0), width=1,
    parent=view2d.scene
)

# ---- HUD ----
scene.visuals.Text(
    'FLEX HAND SKELETON', pos=(14, 16),
    color='white', font_size=14, bold=True,
    anchor_x='left', anchor_y='top',
    parent=canvas.scene
)
scene.visuals.Text(
    'Left-drag to orbit  |  Scroll to zoom', pos=(14, 42),
    color='#445566', font_size=8,
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
# SECTION 8: TIMER CALLBACK (~30 fps)
# =============================================================================
R_FIXED = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]], dtype=np.float32)

def on_timer(_ev):
    with _lock:
        flex  = list(_state['f'])
        times = list(_state['times'])

    bv, bc, jp, jc = compute_skeleton(flex, R_FIXED)
    hand_lines.set_data(pos=bv, color=bc, connect='segments', width=3)
    hand_joints.set_data(pos=jp, face_color=jc, symbol='disc', size=11, edge_width=0)

    fv, ff, fc = build_fill_mesh(flex)
    bar_fills.set_data(vertices=fv, faces=ff, vertex_colors=fc)

    for i, txt in enumerate(value_texts):
        txt.text = f'{flex[i]:.2f}'

    if len(times) >= 2:
        dt   = times[-1] - times[0]
        rate = (len(times) - 1) / dt if dt > 0 else 0.0
        hud_rate.text = f'{rate:.1f} samples/sec'

    canvas.update()

_timer = app.Timer(interval=TIMER_INTERVAL, connect=on_timer, start=True)

# =============================================================================
# SECTION 9: MAIN
# =============================================================================
print('[flex hand] window open — bend fingers to see skeleton curl')
print('[flex hand] left-drag to orbit  |  scroll to zoom  |  Ctrl+C to quit')
app.run()
_stop.set()
