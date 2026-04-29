#!/usr/bin/env python3
"""
pressure_visualization.py - Pressure Sensor Test Environment
============================================================
Shows three live FSR pressure channels from the glove mux.
No IMU and no flex parsing. Use this to verify pressure wiring and tune
PRESSURE_IDLE/PRESSURE_PRESS in src/main.cpp.

Dependencies:
    pip install vispy pyserial numpy pyopengl

Usage:
    python3 pressure_visualization.py
    Close the PlatformIO serial monitor first.

Serial input on an auto-detected USB CDC port at 115200 baud - only P: lines are used:
    P:p1,p2,p3   (normalized 0.0=no pressure .. 1.0=firm pressure)

Sensor mapping:
    P[0] Pressure 1 - CD4051BE Y5
    P[1] Pressure 2 - CD4051BE Y6
    P[2] Pressure 3 - CD4051BE Y7
"""

import threading
import time
from collections import deque

import numpy as np
import serial
from vispy import app, scene
from glove_serial import open_glove_serial

# =============================================================================
# SECTION 1: CONSTANTS
# =============================================================================

WIN_W, WIN_H = 1000, 620

BAUD_RATE = 115200

BG_COLOR = '#101214'
PANEL_BG = np.array([0.070, 0.075, 0.080, 1.00], dtype=np.float32)
TRACK_COLOR = np.array([0.140, 0.120, 0.120, 1.00], dtype=np.float32)
PAD_IDLE = np.array([0.120, 0.130, 0.135, 1.00], dtype=np.float32)
PRESSURE_RED = np.array([0.950, 0.060, 0.040, 1.00], dtype=np.float32)
TEXT_COLOR = '#E8E8E8'
MUTED_TEXT = '#888888'

LABELS = ['P1', 'P2', 'P3']
CHANNEL_NOTES = ['CD4051 Y5', 'CD4051 Y6', 'CD4051 Y7']

PAD_CENTERS = [
    np.array([200.0, 340.0, 0.0], dtype=np.float32),
    np.array([500.0, 340.0, 0.0], dtype=np.float32),
    np.array([800.0, 340.0, 0.0], dtype=np.float32),
]
PAD_RADIUS = 86.0
PAD_RING_RADIUS = 102.0

BAR_X0 = 125.0
BAR_X1 = 875.0
BAR_W = BAR_X1 - BAR_X0
BAR_H = 22.0
BAR_Y = [155.0, 110.0, 65.0]

TIMER_INTERVAL = 1.0 / 30.0

# =============================================================================
# SECTION 2: SHARED STATE
# =============================================================================

_state = {
    'p': [0.0] * 3,
    'times': deque(maxlen=40),
}
_lock = threading.Lock()
_stop = threading.Event()

# =============================================================================
# SECTION 3: SERIAL READER THREAD - P: lines only
# =============================================================================

def _serial_thread():
    while not _stop.is_set():
        try:
            ser, port = open_glove_serial(BAUD_RATE, timeout=1)
            print(f'[serial] connected: {port}')

            while not _stop.is_set():
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith('P:'):
                    continue

                parts = line[2:].split(',')
                if len(parts) != 3:
                    continue

                try:
                    pressure = [float(p) for p in parts]
                except ValueError:
                    continue

                pressure = [float(np.clip(p, 0.0, 1.0)) for p in pressure]
                with _lock:
                    _state['p'] = pressure
                    _state['times'].append(time.monotonic())

        except serial.SerialException as e:
            if not _stop.is_set():
                print(f'[serial] {e} - retrying in 2s')
                time.sleep(2)
        except Exception as e:
            if not _stop.is_set():
                print(f'[serial] unexpected error: {e} - retrying in 2s')
                time.sleep(2)

# =============================================================================
# SECTION 4: 2D GEOMETRY
# =============================================================================

def _blend_pressure(value):
    v = float(np.clip(value, 0.0, 1.0))
    return (PAD_IDLE * (1.0 - v) + PRESSURE_RED * v).astype(np.float32)

def _quad(x0, y0, x1, y1):
    return np.array([
        [x0, y0, 0.0],
        [x1, y0, 0.0],
        [x1, y1, 0.0],
        [x0, y1, 0.0],
    ], dtype=np.float32)

def _circle(center, radius, color, segments=72):
    verts = [np.asarray(center, dtype=np.float32)]
    faces = []
    for i in range(segments):
        a = 2.0 * np.pi * i / segments
        verts.append(center + np.array([
            np.cos(a) * radius,
            np.sin(a) * radius,
            0.0,
        ], dtype=np.float32))

    for i in range(segments):
        j = 1 + ((i + 1) % segments)
        faces.append([0, 1 + i, j])

    colors = np.repeat(np.asarray(color, dtype=np.float32)[None, :], len(verts), axis=0)
    return (np.asarray(verts, dtype=np.float32),
            np.asarray(faces, dtype=np.uint32),
            colors)

def _append_mesh(vs, fs, cs, verts, faces, colors):
    base = len(vs)
    vs.extend(verts)
    fs.extend((np.asarray(faces, dtype=np.uint32) + base).tolist())
    cs.extend(np.asarray(colors, dtype=np.float32))

def build_pad_mesh(pressure_vals):
    vs, fs, cs = [], [], []
    for i, value in enumerate(pressure_vals):
        ring_color = TRACK_COLOR
        if value > 0.02:
            ring_color = _blend_pressure(min(value + 0.18, 1.0))

        verts, faces, colors = _circle(PAD_CENTERS[i], PAD_RING_RADIUS, ring_color)
        _append_mesh(vs, fs, cs, verts, faces, colors)

        verts, faces, colors = _circle(
            PAD_CENTERS[i],
            PAD_RADIUS + 20.0 * float(np.clip(value, 0.0, 1.0)),
            _blend_pressure(value),
        )
        _append_mesh(vs, fs, cs, verts, faces, colors)

    return (np.asarray(vs, dtype=np.float32),
            np.asarray(fs, dtype=np.uint32),
            np.asarray(cs, dtype=np.float32))

def build_bar_track_mesh():
    vs, fs, cs = [], [], []
    for y in BAR_Y:
        v = _quad(BAR_X0, y, BAR_X1, y + BAR_H)
        b = len(vs)
        vs.extend(v)
        fs.extend([[b, b + 1, b + 2], [b, b + 2, b + 3]])
        cs.extend([TRACK_COLOR] * 4)
    return (np.asarray(vs, dtype=np.float32),
            np.asarray(fs, dtype=np.uint32),
            np.asarray(cs, dtype=np.float32))

def build_bar_fill_mesh(pressure_vals):
    vs, fs, cs = [], [], []
    for i, value in enumerate(pressure_vals):
        v = float(np.clip(value, 0.0, 1.0))
        width = max(BAR_W * v, 1.0)
        quad = _quad(BAR_X0, BAR_Y[i], BAR_X0 + width, BAR_Y[i] + BAR_H)
        b = len(vs)
        vs.extend(quad)
        fs.extend([[b, b + 1, b + 2], [b, b + 2, b + 3]])
        cs.extend([_blend_pressure(v)] * 4)
    return (np.asarray(vs, dtype=np.float32),
            np.asarray(fs, dtype=np.uint32),
            np.asarray(cs, dtype=np.float32))

# =============================================================================
# SECTION 5: SERIAL THREAD START
# =============================================================================

threading.Thread(target=_serial_thread, daemon=True).start()

# =============================================================================
# SECTION 6: VISPY SCENE SETUP
# =============================================================================

canvas = scene.SceneCanvas(
    title='Pressure Sensor Visualizer',
    keys='interactive',
    size=(WIN_W, WIN_H),
    bgcolor=BG_COLOR,
    show=True,
)

view = canvas.central_widget.add_view()
view.camera = scene.PanZoomCamera(rect=(0, 0, WIN_W, WIN_H), aspect=None)

panel = scene.visuals.Rectangle(
    center=(WIN_W / 2.0, WIN_H / 2.0),
    width=WIN_W,
    height=WIN_H,
    color=PANEL_BG,
    parent=view.scene,
)

_pv, _pf, _pc = build_pad_mesh([0.0] * 3)
pad_mesh = scene.visuals.Mesh(
    vertices=_pv, faces=_pf, vertex_colors=_pc,
    parent=view.scene
)

_tv, _tf, _tc = build_bar_track_mesh()
track_mesh = scene.visuals.Mesh(
    vertices=_tv, faces=_tf, vertex_colors=_tc,
    parent=view.scene
)

_bv, _bf, _bc = build_bar_fill_mesh([0.0] * 3)
bar_mesh = scene.visuals.Mesh(
    vertices=_bv, faces=_bf, vertex_colors=_bc,
    parent=view.scene
)

scene.visuals.Text(
    'PRESSURE SENSORS', pos=(WIN_W / 2.0, WIN_H - 42),
    color=TEXT_COLOR, font_size=18, bold=True,
    anchor_x='center', anchor_y='top',
    parent=canvas.scene,
)

scene.visuals.Text(
    'Listening for P:p1,p2,p3 from mux channels Y5-Y7',
    pos=(WIN_W / 2.0, WIN_H - 76),
    color=MUTED_TEXT, font_size=10,
    anchor_x='center', anchor_y='top',
    parent=canvas.scene,
)

value_texts = []
for i, label in enumerate(LABELS):
    scene.visuals.Text(
        label,
        pos=(PAD_CENTERS[i][0], PAD_CENTERS[i][1] + PAD_RING_RADIUS + 34),
        color=TEXT_COLOR,
        font_size=18,
        bold=True,
        anchor_x='center',
        anchor_y='center',
        parent=view.scene,
    )
    scene.visuals.Text(
        CHANNEL_NOTES[i],
        pos=(PAD_CENTERS[i][0], PAD_CENTERS[i][1] - PAD_RING_RADIUS - 24),
        color=MUTED_TEXT,
        font_size=9,
        anchor_x='center',
        anchor_y='center',
        parent=view.scene,
    )
    t = scene.visuals.Text(
        '0.00',
        pos=(PAD_CENTERS[i][0], PAD_CENTERS[i][1]),
        color=TEXT_COLOR,
        font_size=20,
        bold=True,
        anchor_x='center',
        anchor_y='center',
        parent=view.scene,
    )
    value_texts.append(t)

for i, label in enumerate(LABELS):
    scene.visuals.Text(
        label,
        pos=(BAR_X0 - 22, BAR_Y[i] + BAR_H / 2.0),
        color=TEXT_COLOR,
        font_size=10,
        bold=True,
        anchor_x='right',
        anchor_y='center',
        parent=view.scene,
    )

hud_rate = scene.visuals.Text(
    '-- samples/sec',
    pos=(16, 18),
    color=MUTED_TEXT,
    font_size=9,
    anchor_x='left',
    anchor_y='bottom',
    parent=canvas.scene,
)

# =============================================================================
# SECTION 7: TIMER CALLBACK
# =============================================================================

def on_timer(_ev):
    with _lock:
        pressure = list(_state['p'])
        times = list(_state['times'])

    pv, pf, pc = build_pad_mesh(pressure)
    pad_mesh.set_data(vertices=pv, faces=pf, vertex_colors=pc)

    bv, bf, bc = build_bar_fill_mesh(pressure)
    bar_mesh.set_data(vertices=bv, faces=bf, vertex_colors=bc)

    for i, txt in enumerate(value_texts):
        txt.text = f'{pressure[i]:.2f}'

    if len(times) >= 2:
        dt = times[-1] - times[0]
        rate = (len(times) - 1) / dt if dt > 0 else 0.0
        hud_rate.text = f'{rate:.1f} samples/sec'

    canvas.update()

_timer = app.Timer(interval=TIMER_INTERVAL, connect=on_timer, start=True)

# =============================================================================
# SECTION 8: MAIN
# =============================================================================

print('[pressure] window open - press FSRs to see pressure values')
print('[pressure] close PlatformIO serial monitor first  |  Ctrl+C to quit')
app.run()
_stop.set()
