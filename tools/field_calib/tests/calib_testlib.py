"""Synthetic calibration recordings for the field_calib tests (no real data, no GNSS).

A small plant with known truth drives through an orchard-like scene (two lines of posts, end walls,
a few boxes); the recording is written in the real Jetson archive layout (metadata.json, motor.jsonl
as orchard.motor_pwm.timeline.v1, mapping_source/sensors.jsonl.gz with /scan and /imu/data_raw rows as
mapping_sensor_writer.py writes them, manifest.json). The LiDAR is mounted reversed (travel axis at
-179.08 deg in the scan frame, like the real S2 mount). Status rows are 2 Hz and show the firmware ramp
(100 %/s); STOP sets the duty to 0 at once and disarms.

Plant truth (shared with the assertions): breakaway 25 %, v = 0.0098 * duty - 0.04 m/s above it, speed
lag 0.25 s, arc yaw gain 0.15 deg/s per % of (R - L), pivot yaw 0.25 deg/s per %, gyro bias 0.02 deg/s.
"""
import gzip
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
sys.dont_write_bytecode = True

BREAKAWAY = 25.
GAIN = .15
PIVOT_GAIN = .25
BIAS_DPS = .02
LIDAR_YAW = math.radians(179.08)    # LiDAR frame = body frame rotated by +179.08 deg
BEAMS = 720
T0 = 1_800_000_000.0                 # host epoch of the recording start
START_X = -20.                       # the session drives about 60 m forward inside the 92 m long scene


def v_of(duty):
    return 0. if duty < BREAKAWAY else .0098 * duty - .04


def scene():
    posts = []
    for x in np.arange(-30., 60., 1.5):
        for y in (-1.75, 1.75):
            posts.append((x + .1 * math.sin(3 * x), y + .05 * math.cos(2 * x), .09))
    for x, y in ((5., 4.5), (12., -4.), (-6., 3.8), (25., 4.2), (33., -4.6)):
        posts.append((x, y, .4))
    segs = [(-32., -8., -32., 8.), (62., -8., 62., 8.), (-32., 8., 62., 8.), (-32., -8., 62., -8.)]
    return np.array(posts), np.array(segs)


def raycast(px, py, heading, posts, segs, rng):
    ang = -math.pi + np.arange(BEAMS) * (2 * math.pi / BEAMS)
    world = heading + LIDAR_YAW + ang
    dx, dy = np.cos(world), np.sin(world)
    best = np.full(BEAMS, np.inf)
    cx = posts[:, 0] - px
    cy = posts[:, 1] - py
    b = dx[:, None] * cx[None] + dy[:, None] * cy[None]
    c = cx ** 2 + cy ** 2 - posts[:, 2] ** 2
    disc = b ** 2 - c[None]
    t = b - np.sqrt(np.where(disc > 0, disc, np.nan))
    t = np.where((disc > 0) & (t > 0), t, np.inf)
    best = np.minimum(best, t.min(axis=1))
    for x1, y1, x2, y2 in segs:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            tt = ((x1 - px) * ey - (y1 - py) * ex) / den
            uu = ((x1 - px) * dy - (y1 - py) * dx) / den
        ok = (np.abs(den) > 1e-9) & (tt > 0) & (uu >= 0) & (uu <= 1)
        best = np.where(ok & (tt < best), tt, best)
    best = best + rng.normal(0., .01, BEAMS)
    best[best > 25.] = np.inf
    return ang, best


class Plant:
    def __init__(self):
        self.x, self.y, self.h = START_X, 0., 0.
        self.v = 0.
        self.duty = [0., 0.]
        self.target = [0, 0]
        self.hard = False

    def step(self, dt):
        for i in range(2):
            if self.hard:
                self.duty[i] = 0.
            else:
                d = self.target[i] - self.duty[i]
                self.duty[i] += max(-100. * dt, min(100. * dt, d))
        l, r = self.duty
        if l * r < 0:
            yaw = math.radians(PIVOT_GAIN * (r - l) / 2.) if min(abs(l), abs(r)) >= BREAKAWAY else 0.
            v_t = 0.
        else:
            mean = .5 * (l + r)
            v_t = v_of(min(l, r)) + .0098 * (mean - min(l, r)) if min(l, r) >= BREAKAWAY else 0.
            yaw = math.radians(GAIN * (r - l)) if v_t > 0 else 0.
        tau = .12 if self.hard else .25
        self.v += (v_t - self.v) * (1 - math.exp(-dt / tau))
        if abs(self.v) < 1e-4 and v_t == 0:
            self.v = 0.
        self.h += yaw * dt
        self.x += self.v * math.cos(self.h) * dt
        self.y += self.v * math.sin(self.h) * dt
        return yaw


def write_recording(root, rid, program, seed=0, imu_hz=33.):
    """program: list of (duration_s, left, right, kind) with kind 'drive' | 'release' | 'stop'."""
    rng = np.random.default_rng(seed)
    posts, segs = scene()
    path = os.path.join(root, rid)
    os.makedirs(os.path.join(path, 'mapping_source'))
    plant = Plant()
    dt = .005
    t = 0.
    next_scan, next_imu, next_status = 0., 0., .3
    lines, motor = [], []
    yaw = 0.
    state = 'armed_manual'
    req = [0, 0]
    for duration, left, right, kind in program:
        if kind == 'stop':
            plant.hard = True
            plant.target = [0, 0]
            req = [0, 0]
            state = 'disarmed'
        else:
            plant.hard = False
            plant.target = [left, right]
            req = [left, right]
            state = 'armed_manual'
        end = t + duration
        while t < end:
            yaw = plant.step(dt)
            t += dt
            if t >= next_imu:
                next_imu += 1. / imu_hz
                g = np.radians(BIAS_DPS) + yaw + rng.normal(0., math.radians(.05))
                msg = dict(frame_id='imu_link', stamp=dict(sec=int(T0 + t), nanosec=int((T0 + t) % 1 * 1e9)),
                           orientation=dict(x=0., y=0., z=0., w=1.),
                           angular_velocity=dict(x=0., y=0., z=float(g)),
                           linear_acceleration=dict(x=float(rng.normal(0, .05)), y=float(rng.normal(0, .05)),
                                                    z=float(9.78 + rng.normal(0, .05))),
                           header=dict(stamp=dict(sec=int(T0 + t), nanosec=int((T0 + t) % 1 * 1e9)), frame_id='imu_link'))
                lines.append(json.dumps(dict(topic='/imu/data_raw', message=msg, bag_time_ns=int((T0 + t) * 1e9)),
                                        separators=(',', ':')))
            if t >= next_scan:
                next_scan += .1
                ang, rngs = raycast(plant.x, plant.y, plant.h, posts, segs, rng)
                ranges = [float(v) if math.isfinite(v) else None for v in rngs]
                ts = T0 + t - .1
                msg = dict(header=dict(stamp=dict(sec=int(ts), nanosec=int(ts % 1 * 1e9)), frame_id='laser'),
                           ranges=ranges, angle_min=float(ang[0]), angle_increment=float(2 * math.pi / BEAMS),
                           range_min=.05, range_max=30., scan_time=.1, time_increment=.1 / BEAMS)
                lines.append(json.dumps(dict(topic='/scan', message=msg, bag_time_ns=int((T0 + t) * 1e9)),
                                        separators=(',', ':')))
            if t >= next_status:
                next_status += .5
                pins = {}
                for side, (fw, rv) in (('left', ('D8', 'D5')), ('right', ('D3', 'D0'))):
                    d = plant.duty[0 if side == 'left' else 1]
                    pwm = int(round(abs(d) * 255 / 100))
                    pins[fw] = dict(reported_pwm=pwm if d > 0 else 0)
                    pins[rv] = dict(reported_pwm=pwm if d < 0 else 0)
                for p in ('D1', 'D2', 'D6', 'D7'):
                    pins[p] = dict(reported_pwm=255)
                motor.append(json.dumps(dict(
                    t_s=round(t, 6), received_at=T0 + t, source_kind='mcu_status', source_updated_at=T0 + t - .001,
                    source_stale=False, status=dict(ok=True, connected=True, armed=state == 'armed_manual',
                                                    moving=any(req), state=state, firmware='UBTS_WHEEL_PWM_V2'),
                    requested=dict(left=req[0], right=req[1]), pins=pins), separators=(',', ':')))
    with gzip.open(os.path.join(path, 'mapping_source', 'sensors.jsonl.gz'), 'wt', encoding='utf-8') as fh:
        for line in lines:
            fh.write(line + '\n')
    with open(os.path.join(path, 'motor.jsonl'), 'w', encoding='utf-8', newline='\n') as fh:
        for line in motor:
            fh.write(line + '\n')
    with open(os.path.join(path, 'metadata.json'), 'w', encoding='utf-8') as fh:
        json.dump(dict(id=rid, kind='both', state='archived', duration_s=round(t, 2)), fh)
    with open(os.path.join(path, 'mapping_source', 'manifest.json'), 'w', encoding='utf-8') as fh:
        json.dump(dict(schema='ubts.field_extraction.v1', complete=True, dropped=0, error=None), fh)
    return path


# A compact C1-C7 session (about 190 s, about 60 m). Breakaway 25 %: 20 % is a conclusive non-start, 25 %
# starts 3/3 (the B_fwd rule needs >= 3 attempts that all started); three >= 8 m straights at 50 % (G5);
# three arcs per direction (C5 needs >= 3 segments per side).
SESSION = [
    (16., 0, 0, 'release'),            # C1 standstill
    (4.5, 20, 20, 'drive'), (2.5, 0, 0, 'release'),       # C2: no start
    (4.5, 25, 25, 'drive'), (3., 0, 0, 'release'),        # C2: start 1/3
    (4.5, 25, 25, 'drive'), (3., 0, 0, 'release'),        # C2: start 2/3
    (4.5, 25, 25, 'drive'), (3., 0, 0, 'release'),        # C2: start 3/3
    (4.5, 40, 40, 'drive'), (3., 0, 0, 'release'),        # C2/C3 40 % + release stop
    (6., 50, 50, 'drive'), (3., 0, 0, 'release'),         # C3/C4 50 % + release stop
    (5., 50, 50, 'drive'), (3., 0, 0, 'stop'), (2., 0, 0, 'release'),   # hard STOP
    (5., 45, 45, 'drive'), (3., 0, 0, 'release'),
    (24., 50, 50, 'drive'), (3., 0, 0, 'release'),        # C4 long straights (~9.5 m each)
    (24., 50, 50, 'drive'), (3., 0, 0, 'release'),
    (24., 50, 50, 'drive'), (3., 0, 0, 'release'),
    (5., 40, 60, 'drive'), (2.5, 0, 0, 'release'),        # C5 left arc (R-L = 20)
    (5., 60, 40, 'drive'), (2.5, 0, 0, 'release'),        # C5 right arc
    (5., 40, 60, 'drive'), (2.5, 0, 0, 'release'),
    (5., 60, 40, 'drive'), (2.5, 0, 0, 'release'),
    (5., 40, 60, 'drive'), (2.5, 0, 0, 'release'),
    (5., 60, 40, 'drive'), (2.5, 0, 0, 'release'),
    (5., 50, 50, 'drive'), (3., 0, 0, 'release'),
    (3., -60, 60, 'drive'), (2.5, 0, 0, 'release'),       # C6 pivot
]
