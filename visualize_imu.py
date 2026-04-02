# Glove IMU Hand Visualizer (VisPy)
#
# Dependencies:
#   pip3 install vispy pyserial pyopengl
#
# Usage:
#   python3 visualize_imu.py
#
# Close the PlatformIO serial monitor before running.
# Left-click drag to orbit, scroll to zoom.
#
# Future serial lines (not yet sent by firmware):
#   F:f0,f1,f2,f3,f4  — flex sensor values per finger (0.0=open, 1.0=closed)
#   P:p0,p1,p2,p3,p4  — FSR force per fingertip (0.0=none, 1.0=max)

import threading
import time
import numpy as np
import serial
from vispy import scene, app

SERIAL_PORT = "/dev/cu.usbmodem101"
BAUD_RATE   = 115200

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
latest_quat  = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
latest_flex  = [0.2] * 5             # per-finger, 0=extended, 1=fully curled
latest_force = [0.2] * 5             # per-fingertip, 0=none, 1=max
quat_lock    = threading.Lock()
flex_lock    = threading.Lock()
force_lock   = threading.Lock()
serial_ok    = False
frame_count  = 0
ref_quat     = [1.0, 0.0, 0.0, 0.0]  # identity — updated by pressing C

# ---------------------------------------------------------------------------
# Serial thread
# ---------------------------------------------------------------------------
def read_serial():
    global serial_ok
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1,
                                dsrdtr=False, rtscts=False)
            ser.dtr = False
            ser.rts = False
            print(f"[serial] port opened: {SERIAL_PORT}")
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('Q:'):
                    parts = line[2:].split(',')
                    if len(parts) == 4:
                        vals = [float(p) for p in parts]
                        with quat_lock:
                            latest_quat[:] = vals
                        if not serial_ok:
                            serial_ok = True
                            print(f"[serial] first Q: received: {line}")
                elif line.startswith('F:'):
                    parts = line[2:].split(',')
                    if len(parts) == 5:
                        with flex_lock:
                            latest_flex[:] = [float(p) for p in parts]
                elif line.startswith('P:'):
                    parts = line[2:].split(',')
                    if len(parts) == 5:
                        with force_lock:
                            latest_force[:] = [float(p) for p in parts]
        except serial.SerialException as e:
            print(f"[serial] connection error: {e} — retrying in 2s...")
            time.sleep(2)
        except Exception as e:
            print(f"[serial] error: {e} — retrying in 2s...")
            time.sleep(2)


# ---------------------------------------------------------------------------
# Hand geometry
# ---------------------------------------------------------------------------

# Skin tones per finger (thumb, index, middle, ring, pinky)
FINGER_COLORS = [
    (0.90, 0.75, 0.55, 1.0),  # thumb  — base tan
    (0.85, 0.68, 0.48, 1.0),  # index
    (0.88, 0.72, 0.52, 1.0),  # middle
    (0.85, 0.68, 0.48, 1.0),  # ring
    (0.82, 0.65, 0.45, 1.0),  # pinky
]
PALM_COLOR    = (0.87, 0.72, 0.53, 1.0)
FORCE_COLOR   = (0.95, 0.20, 0.15, 1.0)   # red when FSR active
JOINT_COLOR   = (0.70, 0.55, 0.38, 1.0)   # slightly darker knuckle tone

# Finger layout: (x_base, y_base, length_scale, n_segments, splay_deg)
#   x/y_base: position on palm top edge in hand-local space
#   splay_deg: outward angle from vertical (for thumb/pinky)
FINGER_CONFIG = [
    (-0.30,  0.05, 0.85, 2, 62),  # thumb  (2 segments, large splay)
    (-0.22,  0.25, 1.00, 3,  2),  # index
    (-0.07,  0.25, 1.10, 3,  0),  # middle (longest)
    ( 0.07,  0.25, 1.00, 3, -2),  # ring
    ( 0.22,  0.23, 0.85, 3, -6),  # pinky  (slight outward splay)
]

BASE_LENGTHS = [0.26, 0.19, 0.13]   # proximal, middle, distal (world units)
BASE_WIDTHS  = [0.09, 0.075, 0.060] # cross-section widths per segment

# Joint angles at rest pose (degrees): [MCP/CMC, PIP/MCP, DIP/IP]
REST_ANGLES = [8.0, 4.0, 0.0]
MAX_ANGLES  = [85.0, 88.0, 65.0]   # fully curled


def _rot_x(angle_deg):
    """3×3 rotation matrix around X axis."""
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]], dtype=np.float32)


def _rot_z(angle_deg):
    """3×3 rotation matrix around Z axis."""
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[ c, -s, 0],
                     [ s,  c, 0],
                     [ 0,  0, 1]], dtype=np.float32)


def make_box(p0, p1, width, color):
    """
    Build a rectangular box segment from point p0 to p1.
    The box has a square cross-section of `width`.
    Returns (vertices, faces, colors) — same quad-face pattern as the original cube.
    """
    p0 = np.array(p0, dtype=np.float32)
    p1 = np.array(p1, dtype=np.float32)
    axis = p1 - p0
    length = np.linalg.norm(axis)
    if length < 1e-6:
        return (np.zeros((0, 3), np.float32),
                np.zeros((0, 3), np.uint32),
                np.zeros((0, 4), np.float32))

    # Build an orthonormal frame: forward=axis, and two perpendicular axes
    fwd = axis / length
    # Pick a helper vector not parallel to fwd
    helper = np.array([0, 0, 1], dtype=np.float32)
    if abs(np.dot(fwd, helper)) > 0.9:
        helper = np.array([0, 1, 0], dtype=np.float32)
    right = np.cross(fwd, helper)
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    up /= np.linalg.norm(up)

    h = width * 0.5
    # 8 corner offsets in local frame
    corners = []
    for s_fwd in [0.0, length]:
        for s_up in [-h, h]:
            for s_right in [-h, h]:
                corners.append(p0 + fwd * s_fwd + up * s_up + right * s_right)
    # corners layout:
    # 0: (0, -h, -h)  1: (0, -h, +h)  2: (0, +h, -h)  3: (0, +h, +h)
    # 4: (L, -h, -h)  5: (L, -h, +h)  6: (L, +h, -h)  7: (L, +h, +h)

    face_indices = [
        [0, 1, 3, 2],  # start cap
        [4, 6, 7, 5],  # end cap
        [0, 4, 5, 1],  # bottom
        [2, 3, 7, 6],  # top
        [0, 2, 6, 4],  # left
        [1, 5, 7, 3],  # right
    ]

    verts, faces, colors = [], [], []
    for fi, quad in enumerate(face_indices):
        base = len(verts)
        for ci in quad:
            verts.append(corners[ci])
        faces.append([base, base+1, base+2])
        faces.append([base, base+2, base+3])
        colors.extend([color] * 4)

    return (
        np.array(verts,   dtype=np.float32),
        np.array(faces,   dtype=np.uint32),
        np.array(colors,  dtype=np.float32),
    )


def combine_meshes(parts):
    """Concatenate a list of (verts, faces, colors) tuples into one mesh."""
    all_v, all_f, all_c = [], [], []
    offset = 0
    for v, f, c in parts:
        if len(v) == 0:
            continue
        all_v.append(v)
        all_f.append(f + offset)
        all_c.append(c)
        offset += len(v)
    if not all_v:
        # return empty single-triangle fallback
        return (np.zeros((3, 3), np.float32),
                np.array([[0, 1, 2]], np.uint32),
                np.ones((3, 4), np.float32))
    return (
        np.concatenate(all_v, axis=0),
        np.concatenate(all_f, axis=0),
        np.concatenate(all_c, axis=0),
    )


def build_hand_mesh(flex, force):
    """
    Build the full hand mesh.
    flex:  list of 5 floats [0,1] — finger curl (0=extended, 1=closed)
    force: list of 5 floats [0,1] — fingertip pressure (0=none, 1=max)
    Returns (vertices, faces, vertex_colors).
    """
    parts = []

    # --- Palm ---
    pw, ph, pd = 0.72, 0.50, 0.12
    palm_corners = [
        np.array([-pw/2, -ph/2, -pd/2]),
        np.array([ pw/2,  ph/2,  pd/2]),
    ]
    parts.append(make_box(
        [-pw/2, -ph/2, 0], [ pw/2, -ph/2+ph, 0],  # use box helper via two diagonal points
        pd, PALM_COLOR
    ))
    # Re-do palm as a proper flat box:
    # make_box expects p0→p1 along the long axis; palm is wide so use X axis
    parts.pop()
    # Build palm manually as a flat slab
    px, py, pz = pw/2, ph/2, pd/2
    palm_face_defs = [
        ([[-px,-py,-pz],[ px,-py,-pz],[ px, py,-pz],[-px, py,-pz]], PALM_COLOR),
        ([[-px,-py, pz],[ px,-py, pz],[ px, py, pz],[-px, py, pz]], PALM_COLOR),
        ([[-px,-py,-pz],[ px,-py,-pz],[ px,-py, pz],[-px,-py, pz]], PALM_COLOR),
        ([[-px, py,-pz],[ px, py,-pz],[ px, py, pz],[-px, py, pz]], PALM_COLOR),
        ([[-px,-py,-pz],[-px, py,-pz],[-px, py, pz],[-px,-py, pz]], PALM_COLOR),
        ([[ px,-py,-pz],[ px, py,-pz],[ px, py, pz],[ px,-py, pz]], PALM_COLOR),
    ]
    pv, pf, pc = [], [], []
    for i, (quad, col) in enumerate(palm_face_defs):
        base = i * 4
        pv.extend(quad)
        pf.extend([[base, base+1, base+2], [base, base+2, base+3]])
        pc.extend([col] * 4)
    parts.append((
        np.array(pv, dtype=np.float32),
        np.array(pf, dtype=np.uint32),
        np.array(pc, dtype=np.float32),
    ))

    # --- Fingers ---
    for fi, (xb, yb, scale, n_seg, splay_deg) in enumerate(FINGER_CONFIG):
        base_color = FINGER_COLORS[fi]
        tip_force  = force[fi]
        flex_val   = flex[fi]

        # Starting position and direction
        pos = np.array([xb, yb, 0.0], dtype=np.float32)
        # Base direction is "up" (Y), splayed outward (around Z)
        R_splay = _rot_z(-splay_deg)
        direction = R_splay @ np.array([0.0, 1.0, 0.0], dtype=np.float32)

        # Accumulated rotation for this finger's chain
        R_chain = R_splay.copy()

        for si in range(n_seg):
            seg_len   = BASE_LENGTHS[si] * scale
            seg_width = BASE_WIDTHS[si]

            # Bend angle for this joint
            rest = REST_ANGLES[si] if si < len(REST_ANGLES) else 0.0
            maxa = MAX_ANGLES[si]  if si < len(MAX_ANGLES)  else 0.0
            bend = rest + (maxa - rest) * flex_val
            R_chain = R_chain @ _rot_x(bend)

            direction = R_chain @ np.array([0.0, 1.0, 0.0], dtype=np.float32)
            next_pos  = pos + direction * seg_len

            # Fingertip segment gets force color blend
            is_tip = (si == n_seg - 1)
            if is_tip and tip_force > 0.01:
                fc = np.array(base_color)
                rc = np.array((*FORCE_COLOR[:3], 1.0))
                color = tuple(fc * (1 - tip_force) + rc * tip_force)
            else:
                # Slightly darken middle/distal segments for depth
                darken = 1.0 - si * 0.06
                color = (base_color[0]*darken, base_color[1]*darken,
                         base_color[2]*darken, 1.0)

            parts.append(make_box(pos, next_pos, seg_width, color))

            # Small knuckle sphere approximation: slightly wider disc at joint
            if si < n_seg - 1:
                knuckle_len = seg_width * 0.5
                knuckle_pos = next_pos - direction * (knuckle_len * 0.5)
                parts.append(make_box(knuckle_pos,
                                      knuckle_pos + direction * knuckle_len,
                                      seg_width * 1.18, JOINT_COLOR))

            pos = next_pos

    return combine_meshes(parts)


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ]


def quat_relative(q_ref, q_raw):
    q_ref_inv = [q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]]
    return quat_mul(q_ref_inv, q_raw)


def quat_to_matrix3(w, x, y, z):
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float32)


# ---------------------------------------------------------------------------
# Serial thread start
# ---------------------------------------------------------------------------
t = threading.Thread(target=read_serial, daemon=True)
t.start()

# ---------------------------------------------------------------------------
# VisPy scene
# ---------------------------------------------------------------------------
canvas = scene.SceneCanvas(
    title='Glove IMU Visualizer',
    keys='interactive',
    size=(900, 750),
    bgcolor='#1a1a2e',
    show=True,
)

view = canvas.central_widget.add_view()
view.camera = 'turntable'
view.camera.fov = 45
view.camera.distance = 4.0
view.camera.elevation = 30
view.camera.azimuth = 20

# Build initial hand mesh
_init_verts, _init_faces, _init_colors = build_hand_mesh([0.0]*5, [0.0]*5)
hand_mesh = scene.visuals.Mesh(
    vertices=_init_verts,
    faces=_init_faces,
    vertex_colors=_init_colors,
    parent=view.scene,
)

scene.visuals.XYZAxis(parent=view.scene)

quat_text = scene.visuals.Text(
    'Press C to calibrate neutral pose',
    color='white', font_size=10,
    pos=(10, 20), anchor_x='left', anchor_y='top',
    parent=canvas.scene,
)


# ---------------------------------------------------------------------------
# Timer callback
# ---------------------------------------------------------------------------
def on_timer(_event):
    global frame_count
    frame_count += 1

    with quat_lock:
        w, x, y, z = latest_quat
    with flex_lock:
        flex = latest_flex[:]
    with force_lock:
        force = latest_force[:]

    if frame_count % 20 == 0:
        print(f"[timer] frame={frame_count}  quat=({w:.3f},{x:.3f},{y:.3f},{z:.3f})"
              f"  flex={[f'{v:.2f}' for v in flex]}  serial_ok={serial_ok}")

    qr = quat_relative(ref_quat, [w, x, y, z])
    R = quat_to_matrix3(*qr)
    verts, faces, colors = build_hand_mesh(flex, force)
    rotated = (R @ verts.T).T

    hand_mesh.set_data(vertices=rotated, faces=faces, vertex_colors=colors)
    quat_text.text = (f'W:{w:.3f}  X:{x:.3f}  Y:{y:.3f}  Z:{z:.3f}  '
                      f'| Flex: {" ".join(f"{v:.2f}" for v in flex)}')
    canvas.update()


def on_key_press(event):
    global ref_quat
    if event.key.name.lower() == 'c':
        with quat_lock:
            ref_quat = latest_quat[:]
        print(f"[calibrate] reference set: {ref_quat}")
        quat_text.text = 'Calibrated! (C to recalibrate)'

canvas.events.key_press.connect(on_key_press)
timer = app.Timer(interval=0.05, connect=on_timer, start=True)

print("[main] starting — press C to calibrate neutral pose, then move the IMU")
app.run()
