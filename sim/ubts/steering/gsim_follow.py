"""G-SIM (Gazebo stack) confirmation of the row-following controller: the evaluation-side definitions (no ROS here).

Pre-registration: docs/steering/PREREG_STEER_FOLLOW_GSIM_KO.md and PREREG_STEER_FOLLOW_GSIM.json (the JSON is prereg()
of this module). Do not change CRITERIA, RUNS or the measuring rules after a run; an amendment is written in the
pre-registration document first. The limits are the NOMINAL limits of the fast SIM registration (steer_follow_v1,
safety_scripts/follow_matrix.py CRITERIA; a test compares the shared values).

Scene w1_force: the original orchard of orchard_gazebo.world_gen (rover.yaml: preset original, quality lite, seed 7).
The scene folder of the stack has world_metadata.json of the generator: the trees {'row', 'x', 'y', 'radius'}, the
spawn point and the generator configuration. That file is truth: only the evaluation side reads it (the check
container; the executor container does not mount the runtime volume). From it:
  centre line   the mean of the two least-squares lines through the trunk positions of the two tree rows next to the
                start lane; the row axis is its direction
  row end s_E   the largest along-row coordinate of a trunk of those two rows (the last trunk line, design v1 1.1)
  obstacles     the trunks of those two rows (generator radius at 0.5 m) and their concrete posts (0.10 m square,
                taken as the circumscribed circle); the low rocks of the lanes are terrain bumps and are not judged
Truth pose = /ground_truth/odom (the model origin base_footprint = the footprint centre of the profile). The drive
interval is from the first to the last truth sample with an MCU drive duty above 0 (/sim/truth/steering).

Every number is a placeholder of the SIM (자리값 기반 SIM 결과), not field performance.
"""
from __future__ import annotations

import collections
import math

SCHEMA = 'ubts.steer_sim.gsim_follow.v1'
PREREG_ID = 'steer_follow_gsim_v1'
RESULT_KIND = '자리값 기반 SIM 결과'
SCENE = 'w1_force'
CONTROLLER = {'kind': 'row_follow'}
FRONT_BUMPER_M = .61                 # footprint length 1.22 m / 2 (profile nominal, as the fast SIM registration)
HALF_WIDTH_M = .35                   # footprint width 0.70 m / 2
POST_SIDE_M = .10                    # concrete post of the generator (0.10 m square)
POST_RADIUS_M = round(POST_SIDE_M * math.sqrt(2.) / 2., 4)
# Registered criteria: the values of the fast SIM registration steer_follow_v1 (design v1 7.1 NOMINAL, v2 item 8).
CRITERIA = dict(lateral_rmse_m=.20, lateral_p95_m=.35, lateral_max_m=.50, heading_rmse_deg=5., heading_max_deg=15.,
                row_end_error_m=1.0, speed_max_m_s=.50, min_clearance_m=0., after_terminal_window_s=3.,
                rest_speed_m_s=.02)
SETTLE_M = 6.0                       # offset start: RMSE / p95 after this travelled distance (as steer_follow_v1)
LATERAL_OFFSET_M = .40
RUN_LIMIT_S = 300.                   # wall limit of one run (the row is 33 m from the start; about 100 s at 0.36 m/s)
PAUSE_GIVE_UP_S = 15.                # a run that stays PAUSED this long is ended by the check (no second press)
INVALID_RERUNS = 2                   # an invalid run (host pause) is replaced at most this many times per slot
RUNS = (
    dict(name='nominal_1', expect='row_end', start='spawn'),
    dict(name='nominal_2', expect='row_end', start='spawn'),
    dict(name='nominal_3', expect='row_end', start='spawn'),
    dict(name='lateral_p040', expect='row_end', start='set_pose', lateral_offset_m=LATERAL_OFFSET_M,
         settle_m=SETTLE_M),
)


def prereg():
    """The machine-readable registration (PREREG_STEER_FOLLOW_GSIM.json)."""
    return dict(schema=SCHEMA + '.prereg', prereg_id=PREREG_ID, result_kind=RESULT_KIND, scene=SCENE,
                world='W1: orchard_gazebo.world_gen, preset original, quality lite, seed 7 (rover.yaml)',
                controller=dict(CONTROLLER), criteria=dict(CRITERIA), settle_m=SETTLE_M,
                runs=[dict(r) for r in RUNS],
                geometry=dict(front_bumper_m=FRONT_BUMPER_M, half_width_m=HALF_WIDTH_M, post_radius_m=POST_RADIUS_M),
                limits=dict(run_limit_s=RUN_LIMIT_S, pause_give_up_s=PAUSE_GIVE_UP_S, invalid_reruns=INVALID_RERUNS),
                validity='A10 rule of the wall clock mode: a run is invalid when the /clock RTF of the run is below '
                         '0.9, the emulator saw a /clock wall gap above 100 ms in the run, or the host latched '
                         'because of a SIM stall; an invalid run is recorded, not judged, and replaced',
                based_on='steer_follow_v1 (the same limits)')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _fit_line(points):
    """Least-squares y = a + b x through (x, y) points."""
    n = len(points)
    if n < 2:
        raise ValueError('gsim_follow: a tree row needs at least 2 trunks')
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    if sxx <= 0.:
        raise ValueError('gsim_follow: the trunks of a row have one x')
    b = sum((p[0] - mx) * (p[1] - my) for p in points) / sxx
    return my - b * mx, b


def post_positions(config, row_index):
    """World (x, y) of the concrete posts of one tree row (vegetation.py: between trees, every trellis_every trees,
    and the end post)."""
    spacing = float(config['tree_spacing'])
    n = int(float(config['row_length']) / spacing) + 1
    every = int(config.get('trellis_every') or 0)
    xs = [i * spacing - spacing / 2. for i in range(n) if every and i % every == 0]
    xs.append(n * spacing - spacing / 2.)
    y = (row_index - .5) * float(config['row_spacing'])
    return [(x, y) for x in xs]


def row_geometry(metadata, lane=None):
    """Centre line, row axis, row end and obstacles of the start lane from the generator metadata (truth)."""
    config = metadata['config']
    spawn = metadata['spawn']
    centres = list(metadata['lane_centers_y'])
    if lane is None:
        lane = min(range(len(centres)), key=lambda k: abs(centres[k] - float(spawn[1])))
    rows = (lane, lane + 1)                                  # world_gen: lane k lies between tree rows k and k + 1
    trunks = {j: [(float(t['x']), float(t['y']), float(t['radius'])) for t in metadata['trees'] if t['row'] == j]
              for j in rows}
    (a_r, b_r), (a_l, b_l) = _fit_line(trunks[rows[0]]), _fit_line(trunks[rows[1]])
    a, b = (a_r + a_l) / 2., (b_r + b_l) / 2.
    axis = math.atan(b)
    c, s = math.cos(axis), math.sin(axis)
    along = lambda x, y: c * x + s * (y - a)                 # noqa: E731
    every = [t for j in rows for t in trunks[j]]
    s_values = [along(t[0], t[1]) for t in every]
    markers = [(t[0], t[1], t[2], 'trunk') for t in every]
    for j in rows:
        markers.extend((x, y, POST_RADIUS_M, 'post') for x, y in post_positions(config, j))
    rocks = [dict(x=float(r['x']), y=float(r['y']), r=float(r['r'])) for r in metadata.get('rocks', ())
             if r.get('lane') == lane]
    return dict(lane=lane, rows=list(rows), centre_a=a, centre_b=b, axis_rad=axis,
                half_width_m=abs(a_l - a_r) / 2. * c, row_start_s=min(s_values), row_end_s=max(s_values),
                trunks=len(every), markers=markers, rocks=rocks,
                spawn=[float(spawn[0]), float(spawn[1]), float(spawn[2])],
                spawn_yaw=float(config.get('spawn_yaw') or 0.))


def to_row(geometry, x, y, yaw):
    """(along s, lateral d (left positive), heading error) of a world pose in the row frame."""
    a, axis = geometry['centre_a'], geometry['axis_rad']
    c, s = math.cos(axis), math.sin(axis)
    return c * x + s * (y - a), -s * x + c * (y - a), wrap(yaw - axis)


def from_row(geometry, along, lateral, heading=0.):
    """World (x, y, yaw) of a row-frame pose (the inverse of to_row)."""
    a, axis = geometry['centre_a'], geometry['axis_rad']
    c, s = math.cos(axis), math.sin(axis)
    return c * along - s * lateral, a + s * along + c * lateral, wrap(axis + heading)


def clearance(poses, markers):
    """Smallest distance [m] from the footprint rectangle to a marker surface over the poses (x, y, yaw); the marker
    kind of the smallest."""
    best, kind = math.inf, None
    for x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        for mx, my, radius, name in markers:
            dx, dy = mx - x, my - y
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            gap = math.hypot(max(abs(lx) - FRONT_BUMPER_M, 0.), max(abs(ly) - HALF_WIDTH_M, 0.)) - radius
            if gap < best:
                best, kind = gap, name
    return best, kind


def _percentile(values, q):
    """numpy.percentile (linear) of a list."""
    ordered = sorted(values)
    if not ordered:
        return None
    pos = (len(ordered) - 1) * q / 100.
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def measure(odom, duty, geometry, settle_m=0.):
    """Metrics of one run. odom: truth rows (sim t, x, y, yaw, v, ...) from before the start to after the rest;
    duty: (sim t, MCU drive duty) truth rows. The definitions of follow_matrix.measure in the row frame."""
    out = dict(drove=False, samples=len(odom), settle_m=float(settle_m))
    if not odom:
        return out
    frame = [to_row(geometry, r[1], r[2], r[3]) for r in odom]
    last = odom[-1]
    s_rest, d_rest, psi_rest = frame[-1]
    gap, kind = clearance([(r[1], r[2], r[3]) for r in odom[::5]] + [(last[1], last[2], last[3])],
                          geometry['markers'])
    out.update(min_clearance_m=round(gap, 4), min_clearance_to=kind, rest_speed_m_s=round(abs(last[4]), 4),
               rest_along_m=round(s_rest, 3), rest_lateral_m=round(d_rest, 3),
               rest_heading_deg=round(math.degrees(psi_rest), 3),
               row_end_error_m=round(s_rest + FRONT_BUMPER_M * math.cos(psi_rest) - geometry['row_end_s'], 3),
               start_along_m=round(frame[0][0], 3), start_lateral_m=round(frame[0][1], 3),
               start_heading_deg=round(math.degrees(frame[0][2]), 3))
    driving = [t for t, value in duty if value > 0.]
    if not driving:
        return out
    t_a, t_b = driving[0], driving[-1]
    index = [i for i, r in enumerate(odom) if t_a <= r[0] <= t_b]
    if len(index) < 2:
        return out
    a, b = index[0], index[-1]
    path = [0.]
    for i in range(a + 1, b + 1):
        path.append(path[-1] + math.hypot(odom[i][1] - odom[i - 1][1], odom[i][2] - odom[i - 1][2]))
    k = next((i for i, p in enumerate(path) if p >= settle_m), len(path))
    lateral = [frame[i][1] for i in range(a, b + 1)]
    heading = [frame[i][2] for i in range(a, b + 1)]
    speed = [odom[i][4] for i in range(a, b + 1)]
    out.update(drove=True, drive_start_s=round(t_a, 3), drive_stop_s=round(t_b, 3),
               duration_s=round(t_b - t_a, 2), distance_m=round(path[-1], 2),
               lateral_max_m=round(max(abs(v) for v in lateral), 4),
               heading_max_deg=round(math.degrees(max(abs(v) for v in heading)), 3),
               speed_mean_m_s=round(sum(speed) / len(speed), 3), speed_max_m_s=round(max(speed), 3))
    if k < len(path) - 1:
        tail_d, tail_h = lateral[k:], heading[k:]
        out.update(lateral_rmse_m=round(math.sqrt(sum(v * v for v in tail_d) / len(tail_d)), 4),
                   lateral_p95_m=round(_percentile([abs(v) for v in tail_d], 95.), 4),
                   heading_rmse_deg=round(math.degrees(math.sqrt(sum(v * v for v in tail_h) / len(tail_h))), 3))
    return out


def after_terminal(c_lines, t_end, t_record_end, window_s=None):
    """The A22 rule on the stack: no commanded steering move after the terminal moment. c_lines: (sim t, S permille)
    of the C lines the MCU received. Passed when every C line in (t_end, t_end + window] carries the S of the last C
    line at or before t_end (no line is a pass); complete when the record covers the whole window."""
    window_s = CRITERIA['after_terminal_window_s'] if window_s is None else window_s
    if t_end is None:
        return dict(passed=False, complete=False, from_s=None)
    before = [s for t, s in c_lines if t <= t_end]
    inside = [s for t, s in c_lines if t_end < t <= t_end + window_s]
    reference = before[-1] if before else (inside[0] if inside else None)
    moved = sorted({s for s in inside if s != reference})
    complete = t_record_end is not None and t_record_end >= t_end + window_s
    return dict(passed=bool(complete and not moved), complete=bool(complete), from_s=round(t_end, 3),
                window_s=window_s, c_lines_in_window=len(inside), reference_s_permille=reference,
                other_s_permille=moved[:8])


def tick_stats(ticks):
    """Executor tick statistics of one run (the executor's own estimates, not truth): corridor validity over the
    ACTIVE ticks, pauses with reasons, controller phases."""
    active = [r for r in ticks if r.get('state') == 'ACTIVE']
    moving = [r for r in active if (r.get('T') or 0) > 0]
    invalid = collections.Counter(str(r.get('corridor_reason')) for r in active if not r.get('corridor'))
    pauses, previous, started = [], None, False
    for r in ticks:
        state = r.get('state')
        started = started or (state == 'ACTIVE' and (r.get('T') or 0) > 0)
        if state == 'PAUSED' and previous != 'PAUSED' and started:
            pauses.append([r.get('t'), r.get('reason')])
        previous = state
    phases = collections.Counter(str((r.get('ctl') or [None, None])[1]) for r in active)
    ages = [r['lidar_age'] for r in active if isinstance(r.get('lidar_age'), (int, float))]
    loops = sorted(r['loop_ms'] for r in active if isinstance(r.get('loop_ms'), (int, float)))
    return dict(ticks=len(ticks), active_ticks=len(active), driving_ticks=len(moving),
                corridor_valid_ticks=sum(1 for r in active if r.get('corridor')),
                corridor_valid_ratio=round(sum(1 for r in active if r.get('corridor')) / len(active), 4)
                if active else None,
                corridor_valid_ratio_driving=round(sum(1 for r in moving if r.get('corridor')) / len(moving), 4)
                if moving else None,
                corridor_invalid_reasons=dict(invalid.most_common(8)),
                pauses_after_start=pauses, controller_reasons=dict(phases.most_common(12)),
                lidar_age_max_s=round(max(ages), 3) if ages else None,
                loop_ms_p50=loops[len(loops) // 2] if loops else None, loop_ms_max=loops[-1] if loops else None)


def judge(expect, m):
    """Registered checks of one run: list of dict(check, ok, measured, limit) (follow_matrix.judge, row end case)."""
    c = CRITERIA
    checks = []

    def add(name, ok, measured, limit):
        checks.append(dict(check=name, ok=bool(ok), measured=measured, limit=limit))

    def below(name, key, unit):
        value = m.get(key)
        add(name, value is not None and value <= c[key], value, '<= %g %s' % (c[key], unit))
    gap, rest = m.get('min_clearance_m'), m.get('rest_speed_m_s')
    add('no contact', gap is not None and gap > c['min_clearance_m'], gap, '> %g m' % c['min_clearance_m'])
    add('at rest at the end', rest is not None and rest <= c['rest_speed_m_s'], rest, '<= %g m/s' % c['rest_speed_m_s'])
    add('no guard violation', not str(m.get('end_reason')).startswith('guard_violation'), m.get('end_reason'), 'none')
    after = m.get('after_terminal') or {}
    add('no commanded steering move after the terminal moment (A22 rule)', after.get('passed') is True,
        dict((k, after.get(k)) for k in ('passed', 'complete', 'c_lines_in_window', 'other_s_permille')), 'passed')
    if expect == 'row_end':
        add('ends at the row end', m.get('end_reason') == 'row_end', m.get('end_reason'), 'row_end')
        add('no pause after the drive started', not m.get('pauses_after_start'), m.get('pauses_after_start'), 'none')
        below('lateral RMSE', 'lateral_rmse_m', 'm')
        below('lateral p95', 'lateral_p95_m', 'm')
        below('lateral max', 'lateral_max_m', 'm')
        below('heading RMSE', 'heading_rmse_deg', 'deg')
        below('heading max', 'heading_max_deg', 'deg')
        below('true speed max', 'speed_max_m_s', 'm/s')
        e = m.get('row_end_error_m')
        add('row end stop position e', e is not None and abs(e) <= c['row_end_error_m'], e,
            '[-%g, +%g] m' % (c['row_end_error_m'], c['row_end_error_m']))
    else:
        add('ends with %s' % expect, m.get('end_reason') == expect, m.get('end_reason'), expect)
    return checks


def verdict(checks, valid=True):
    """'pass' / 'fail', or 'invalid' for a run the validity rule excludes (recorded, not judged)."""
    if not valid:
        return 'invalid'
    return 'pass' if all(item['ok'] for item in checks) else 'fail'
