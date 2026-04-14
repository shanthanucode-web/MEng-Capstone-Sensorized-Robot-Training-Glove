#!/usr/bin/env python3
"""
visualize_glove.py — Real-time 3D Skeletal Hand Visualizer
===========================================================
Renders a skeletal hand that rotates with the IMU and curls fingers with
flex sensors. Includes a flex indicator panel and full HUD overlay.

Dependencies:
    pip install vispy pyserial numpy pyopengl

Usage:
    python3 visualize_glove.py
    Left-drag to orbit  |  Scroll to zoom  |  C to calibrate neutral pose

Serial input on /dev/cu.usbmodem101 at 115200 baud:
    Q:w,x,y,z        — BNO085 Game Rotation Vector quaternion
    F:t,ui,li,um,lm  — normalized flex values [0.0=open .. 1.0=closed]

Flex sensor mapping:
    F[0] Thumb        — always 1.0, sensor disconnected (grey placeholder)
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

# =============================================================================
# SECTION 1: CONSTANTS
# All magic numbers live here for easy tuning.
# =============================================================================

# --- Window ---
WIN_W, WIN_H = 1280, 800

# --- Serial ---
SERIAL_PORT = '/dev/cu.usbmodem101'
BAUD_RATE   = 115200

# --- Colors (numpy RGBA float32 arrays) ---
BG_COLOR     = '#0A0A1A'
PALM_COLOR   = np.array([0.267, 0.267, 0.267, 1.00], dtype=np.float32)  # #444
THUMB_COLOR  = np.array([0.45,  0.45,  0.45,  0.55], dtype=np.float32)  # grey, semi-transparent
INDEX_COLOR  = np.array([0.0,   1.0,   1.0,   1.00], dtype=np.float32)  # #00FFFF cyan
MIDDLE_COLOR = np.array([1.0,   0.549, 0.0,   1.00], dtype=np.float32)  # #FF8C00 orange
JOINT_COLOR  = np.array([1.0,   1.0,   1.0,   1.00], dtype=np.float32)  # white

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

# --- Hand geometry ---
MAX_CURL = 85.0   # max joint bend angle in degrees (flex=1.0 → this angle)

# Segment lengths (dimensionless world units, tuned to fit turntable camera)
PROX_L = {'thumb': 0.18, 'index': 0.26, 'middle': 0.28}  # proximal (knuckle→MCP)
DIST_L = {'thumb': 0.14, 'index': 0.16, 'middle': 0.18}  # distal   (MCP→tip)

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

# Finger definitions: (name, palm_node_idx, splay_deg, fu_idx, fl_idx, color, is_thumb)
#   splay_deg — outward rotation from vertical around Z (thumb only)
#   fu_idx    — F[i] index controlling upper (proximal) segment curl
#   fl_idx    — F[i] index controlling lower (distal)  segment curl
FINGER_DEFS = [
    ('thumb',  0, 30, 0, 0, THUMB_COLOR,  True ),
    ('index',  1,  0, 1, 2, INDEX_COLOR,  False),
    ('middle', 2,  0, 3, 4, MIDDLE_COLOR, False),
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
    'acc':   0,                      # BNO085 accuracy 0–3
    'times': deque(maxlen=40),       # F: line timestamps for Hz calculation
}
_lock  = threading.Lock()
_ref_q = [1.0, 0.0, 0.0, 0.0]   # calibration reference (press C to update)
_stop  = threading.Event()

# =============================================================================
# SECTION 3: SERIAL READER THREAD
# Opens the serial port, parses Q: and F: lines, updates _state.
# Auto-reconnects if port drops (USB CDC devices can disconnect briefly).
# dsrdtr=False, rtscts=False: prevents DTR/RTS from resetting the ESP32.
# =============================================================================
def _serial_thread():
    while not _stop.is_set():
        try:
            ser = serial.Serial(
                SERIAL_PORT, BAUD_RATE, timeout=1,
                dsrdtr=False, rtscts=False
            )
            ser.dtr = False
            ser.rts = False
            print(f'[serial] connected: {SERIAL_PORT}')

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
# SECTION 5: HAND SKELETON GEOMETRY
# Builds all bone endpoints and joint positions each frame.
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

def compute_skeleton(flex, R_imu):
    """
    Compute all bone vertex pairs and joint positions for the skeletal hand.

    Coordinate system (local, before R_imu applied):
        +X  right across palm
        +Y  upward — direction fingers point when fully extended
        +Z  toward viewer
        origin at palm center

    Finger curl mechanics (chained rotation model):
        Each finger has two segments joined at the knuckle and middle joints.
        Upper segment curls at the knuckle (MCP joint), rotating around X:
            dir_upper = R_splay @ R_curl_upper @ UP
        Lower segment curls relative to upper (PIP joint):
            dir_lower = R_splay @ R_curl_upper @ R_curl_lower @ UP
        This chaining means a fully curled finger (both flex=1.0) produces
        2 × MAX_CURL total angular displacement — realistic closed-fist shape.

        flex 0.0 → 0°   (straight, pointing up)
        flex 1.0 → 85°  (strongly curled forward)

    IMU rotation:
        R_imu is applied last to rotate the whole hand into world space.
        It is derived from the BNO085 Game Rotation Vector, calibrated so
        the hand appears upright and aligned in the neutral pose.

    Returns:
        bone_verts  — (2N, 3) float32: interleaved (start, end) pairs for
                      VisPy Line with connect='segments'
        bone_colors — (2N, 4) float32: per-vertex RGBA
        joint_pos   — (M, 3) float32: sphere centers for Markers visual
        joint_col   — (M, 4) float32: per-joint RGBA
    """
    v_s, v_e, v_c = [], [], []   # bone segments: start, end, color (parallel)
    j_pos, j_col  = [], []        # joint spheres: position, color

    # ---- Palm structure ----
    # Horizontal metacarpal bar connecting the 5 base nodes
    for i in range(4):
        v_s.append(PALM_NODES[i]);   v_e.append(PALM_NODES[i + 1])
        v_c += [PALM_COLOR, PALM_COLOR]

    # Left lateral edge of palm, tapering down to wrist
    v_s.append(PALM_NODES[0]);  v_e.append(WRIST_L)
    v_c += [PALM_COLOR, PALM_COLOR]

    # Right lateral edge of palm to wrist
    v_s.append(PALM_NODES[4]);  v_e.append(WRIST_R)
    v_c += [PALM_COLOR, PALM_COLOR]

    # Wrist crossbar completing the palm outline
    v_s.append(WRIST_L);  v_e.append(WRIST_R)
    v_c += [PALM_COLOR, PALM_COLOR]

    # Joint spheres at each node (palm + wrist)
    for node in PALM_NODES:
        j_pos.append(node);  j_col.append(JOINT_COLOR)
    j_pos += [WRIST_L, WRIST_R]
    j_col += [JOINT_COLOR, JOINT_COLOR]

    # ---- Fingers ----
    for (name, pidx, splay, fui, fli, color, is_thumb) in FINGER_DEFS:
        base = PALM_NODES[pidx]

        # Thumb sensor is disconnected — render uncurled regardless of F[0]
        fu = 0.0 if is_thumb else float(np.clip(flex[fui], 0.0, 1.0))
        fl = 0.0 if is_thumb else float(np.clip(flex[fli], 0.0, 1.0))

        R_sp = _rz(splay)           # outward splay (thumb only; 0° for others)
        R_u  = _rx(-fu * MAX_CURL)  # upper joint curl (negative = forward/+Z)
        R_l  = _rx(-fl * MAX_CURL)  # lower joint curl (in upper segment's frame)

        # Proximal (upper) segment: palm base → knuckle
        dir_u   = R_sp @ R_u @ _UP
        knuckle = base + dir_u * PROX_L[name]

        # Distal (lower) segment: knuckle → fingertip
        dir_l   = R_sp @ R_u @ R_l @ _UP
        tip     = knuckle + dir_l * DIST_L[name]

        v_s += [base, knuckle];   v_e += [knuckle, tip]
        v_c += [color, color, color, color]

        j_pos += [knuckle, tip]
        j_col += [JOINT_COLOR, JOINT_COLOR]

    # ---- Assemble interleaved vertex array ----
    # VisPy 'segments' connect mode: (v[0],v[1]), (v[2],v[3]), ... are segments
    n = len(v_s)
    bone_verts  = np.empty((n * 2, 3), dtype=np.float32)
    bone_verts[0::2] = np.array(v_s, dtype=np.float32)
    bone_verts[1::2] = np.array(v_e, dtype=np.float32)
    bone_colors = np.array(v_c,    dtype=np.float32)
    joint_pos   = np.array(j_pos,  dtype=np.float32)
    joint_col   = np.array(j_col,  dtype=np.float32)

    # ---- Apply IMU rotation: R_imu @ v for each row vector ----
    bone_verts = (R_imu @ bone_verts.T).T
    joint_pos  = (R_imu @ joint_pos.T).T

    return bone_verts, bone_colors, joint_pos, joint_col

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
    fov=40, distance=2.2, elevation=20, azimuth=25
)
view3d.camera.center = (0.0, 0.05, 0.0)  # slightly above palm centre

# --- 2D panel camera: fixed y-up coordinate space [0,PANEL_W] × [0,WIN_H] ---
view2d.camera = scene.PanZoomCamera(
    rect=(0, 0, PANEL_W, WIN_H), aspect=None
)

# ---- 3D hand visuals ----
_bv0, _bc0, _jp0, _jc0 = compute_skeleton([0.0]*5, np.eye(3, dtype=np.float32))

hand_lines = scene.visuals.Line(
    pos=_bv0, color=_bc0, connect='segments',
    width=3, antialias=True,
    parent=view3d.scene
)
hand_joints = scene.visuals.Markers(parent=view3d.scene)
hand_joints.set_data(
    pos=_jp0, face_color=_jc0,
    symbol='disc', size=11, edge_width=0
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
    color='white', font_size=14, bold=True,
    anchor_x='left', anchor_y='top',
    parent=canvas.scene
)
hud_quat = scene.visuals.Text(
    'Q: waiting for serial...', pos=(14, 42),
    color='#AAAAFF', font_size=9,
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
# "THUMB: DISCONNECTED" notice — bottom-right of 3D viewport area
hud_thumb = scene.visuals.Text(
    'THUMB: DISCONNECTED',
    pos=(WIN_W - PANEL_W - 14, WIN_H - 14),
    color='#444444', font_size=8,
    anchor_x='right', anchor_y='bottom',
    parent=canvas.scene
)

# =============================================================================
# SECTION 9: TIMER CALLBACK (~30 fps)
# Reads latest sensor state and updates all dynamic visuals.
# =============================================================================
_frame = 0

def on_timer(_ev):
    global _frame, _ref_q
    _frame += 1

    # Snapshot shared state (grab lock briefly, copy values out, release)
    with _lock:
        q     = list(_state['q'])
        flex  = list(_state['f'])
        acc   = int(_state['acc'])
        times = list(_state['times'])

    # ---- IMU rotation matrix ----
    # Remove calibration reference then convert to 3×3 rotation matrix.
    # The rotation matrix is applied to all hand vertices before rendering.
    qr = _qrel(_ref_q, q)
    R  = _qmat(*qr)

    # ---- Update 3D hand skeleton ----
    bv, bc, jp, jc = compute_skeleton(flex, R)
    hand_lines.set_data(pos=bv, color=bc, connect='segments', width=3)
    hand_joints.set_data(pos=jp, face_color=jc, symbol='disc', size=11, edge_width=0)

    # ---- Update flex bars ----
    fv, ff, fc = build_fill_mesh(flex)
    bar_fills.set_data(vertices=fv, faces=ff, vertex_colors=fc)

    for i, txt in enumerate(value_texts):
        txt.text = f'{flex[i]:.2f}'

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
