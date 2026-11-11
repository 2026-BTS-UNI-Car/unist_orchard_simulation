"""A12 two-SIM agreement (design 7.2 A12, SIM_ACCEPTANCE amendment 3): F-SIM vs G-SIM, the same open-loop scripts.

(1) Fit: the F-SIM gets the G-SIM's A1(G) force-mode result (g_update A12_INPUT_G.json): per-delta form
    (amendment 3, judged): eta(|delta|) at 5 / 10 / 20 deg (linear in |delta|, eta(0) = 1) with K = 0, and the
    turning speed loss table loss(|delta|, v) at 5 / 10 / 20 deg x 0.3 / 0.5 m/s (the same A1(G) runs; a drag-loss
    term set to 'the same value in both SIMs', step 2): fast_sim plant_tables (TableSteeredPlant).
    Registered form (reported): constant eta = L / L_eff and understeer K_us = K eta from the registered fit, and the
    least-squares constant scrub loss per deg of the same loss table, as plain profile overrides (the F-SIM as it is).
(2) Random terms off in both SIMs (A12_OVERRIDES: zero offset, calibration residual, backlash wander, curvature
    wander, lateral slip, speed run scale and fluctuation, left / right asymmetry; the steering actuator is the shared
    L0 servo in both, with the same profile view; L0 has no backlash, noise or deadband).
(3) The same scripts (maneuvers 'schedule' knots) with the F-SIM direct runner's timing (20 Hz commands, 50 ms
    latency, MCU ramp 100 %/s, soft-limit clamp) on W0 and on W2 mode R (040927 c02 fwd, frozen development
    direction; the G-SIM scene field1010 with the field world of the same corridor, start poses in W).

Judgement (definitions fixed before the first A12 measurement; WORLDS_KO.md):
  position difference  max over the script time of |p_F(t) - p_G(t)| of the base (footprint centre), time-synchronised
                       (G-SIM samples interpolated to the F-SIM times), in the start frame (W0) or W (W2)
  heading difference   max over the script time of |wrap(psi_F(t) - psi_G(t))|
  predicted            the same differences between two F-SIM runs: the fit and the fit perturbed by the A1(G) fit
                       residual (eta x (1 + r_max) with r_max = the largest |curvature residual| of the form; and the
                       straight speed by the largest A4(G) steady-speed error, as duty x (1 + e)); larger of +/- runs
  pass                 position <= max(0.10 m, 1.5 x predicted) and heading <= max(2 deg, 1.5 x predicted), every script
Also reported: the distance-synchronised position difference (same path length) and the end-pose differences.

CLI (sim/ubts on sys.path):
  python -B -m field_world.a12_twin fsim --a1-input <A12_INPUT_G.json> --out <run folder>      (Windows / host)
  python -B -m field_world.a12_twin compare --a1-input ... --fsim <run folder> --gsim-w0 <gz_probe out>/a12_w0
      --gsim-w2 <gz_probe out>/a12_w2 --out <json>
  (the G-SIM side: python3 -B -m steering.gz_probe run a12_w0 a12_w2 --out <gz_probe out>, SIM image)
Placeholder-based SIM results (자리값 기반 SIM 결과), not field validation.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

from . import raster_ops as ro

SCHEMA = 'ubts.steer_field_world.a12.v1'
A12_SEED = 12
COMMAND_PERIOD_S = .05            # fast_sim CONTROL_EVERY x DT_S
COMMAND_LATENCY_S = .05           # scene_spec.DEFAULT_COMMAND_LATENCY_S
POSITION_FLOOR_M = .10
HEADING_FLOOR_DEG = 2.
PREDICTION_FACTOR = 1.5
A4G_SPEED_ERROR = .0022           # largest A4(G) steady-speed error of the v0.1 + sphere model (g_update: -0.22 %)
W2_RECORD, W2_CORRIDOR, W2_DIRECTION = '040927', 'c02', 'fwd'
A12_LABEL = 'a12_twin_random_terms_off'
A12_OVERRIDES = {
    'steer_actuator_level': 'L0',
    'steer_zero_offset_deg': {'dist': 'normal', 'mean': 0.0, 'sd': 0.0},
    'pot_cal_residual_deg': {'dist': 'normal', 'mean': 0.0, 'sd': 0.0},
    'steer_backlash_wander': {'sd_deg': 0.0, 'corr_length_m': 5.0},
    'steer_lr_asymmetry': 0.0,
    'curvature_wander': {'mean_deg_per_m': 0.0, 'sd_deg_per_m': 0.0, 'corr_length_m': 8.0},
    'lateral_slip_sd_m_s': 0.0,
    'speed_run_scale': {'mode': 1.0, 'sd_low': 0.0, 'sd_high': 0.0},
    'speed_fluctuation_sd': 0.0,
    'steering_efficiency': {'dist': 'uniform', 'low': 1.0, 'high': 1.0},
    'scrub_speed_loss_per_deg': {'dist': 'uniform', 'low': 0.0, 'high': 0.0},
    'understeer_gradient': {'dist': 'uniform', 'low': 0.0, 'high': 0.0},
}
DUTY_03 = 36.875                  # nominal 0.30 m/s of the main curve (maneuvers.duty_for_speed, v0.1)
DUTY_04 = 47.5                    # nominal 0.40 m/s


def _sine_knots(duty, amplitude_deg, period_s, t0, t1, step=.25):
    knots = [[0., duty, 0.]]
    t = t0
    while t <= t1 + 1e-9:
        knots.append([round(t, 6), duty, round(amplitude_deg * math.sin(2. * math.pi * (t - t0) / period_s), 9)])
        t += step
    return knots


def scripts(world):
    """{name: dict(schedule=<maneuvers schedule params>, world, start)} of one world ('W0' or 'W2')."""
    w0 = {
        's1_sine5_v04': dict(schedule=dict(knots=_sine_knots(DUTY_04, 5., 10., 3., 33.) + [[36., DUTY_04, 0.]],
                                           interpolation='linear', end_s=36.)),
        's2_steps10_v03': dict(schedule=dict(knots=[[0., DUTY_03, 0.], [4., DUTY_03, 10.], [14., DUTY_03, -10.],
                                                    [24., DUTY_03, 0.]], interpolation='hold', end_s=30.)),
        's3_arc20_v03': dict(schedule=dict(knots=[[0., DUTY_03, 0.], [2., DUTY_03, 20.]], interpolation='hold',
                                           end_s=20.)),
        's4_rowlike_v04': dict(schedule=dict(knots=[[0., DUTY_04, 0.], [3., DUTY_04, 0.], [8., DUTY_04, 2.],
                                                    [14., DUTY_04, 2.], [18., DUTY_04, -3.], [24., DUTY_04, -3.],
                                                    [27., DUTY_04, 1.], [32., DUTY_04, 0.]],
                                             interpolation='linear', end_s=34.)),
    }
    for item in w0.values():
        item.update(world='W0', start=dict(s_m=0., d_m=0., heading_deg=0.))
    if world == 'W0':
        return w0
    if world != 'W2':
        raise ValueError('a12 world %r' % (world,))
    w2 = {'s4_rowlike_v04': dict(w0['s4_rowlike_v04'], world='W2', start=dict(s_m=1., d_m=0., heading_deg=0.)),
          's1_sine5_v04': dict(w0['s1_sine5_v04'], world='W2', start=dict(s_m=40., d_m=0., heading_deg=0.))}
    return w2


def w2_field_spec():
    from .scene_spec import catalogue_spec
    return catalogue_spec(W2_RECORD, W2_CORRIDOR, W2_DIRECTION, mode='R', name='a12_w2_f040927_c02_fwd_r')


_FIELD_CACHE = {}


def w2_field_world():
    if 'fw' not in _FIELD_CACHE:
        from . import build_field_world as fwb
        _FIELD_CACHE['fw'] = fwb.build(w2_field_spec())
    return _FIELD_CACHE['fw']


def w2_start_pose_w(script):
    """(x, y, yaw) in W of a W2 script start (corridor frame s, d, heading)."""
    fw = w2_field_world()
    start = script['start']
    x, y, heading = fw.frame.pose(start['s_m'], start['d_m'], math.radians(start['heading_deg']))
    wx, wy = fw.transform.to_w(x, y)
    return float(wx), float(wy), float(fw.transform.yaw_to_w(heading))


# --- fit inputs ------------------------------------------------------------------------------------------------------
def load_a1_input(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def plant_tables(a1, *, eta_scale=1.):
    """fast_sim plant tables of the per-delta form (eta x eta_scale) and the G-SIM turning speed loss table."""
    eta = a1['per_delta_form']['eta_by_delta_deg']
    rows = [[float(d), float(e) * eta_scale] for d, e in sorted(eta.items(), key=lambda kv: float(kv[0]))]
    loss = a1['turning_speed_loss']['table']
    speeds = sorted({float(v) for row in loss.values() for v in row['by_speed']})
    table = []
    for d, row in sorted(loss.items(), key=lambda kv: float(kv[0])):
        table.append([float(d)] + [float(row['by_speed']['%g' % v]['loss']) for v in speeds])
    return dict(label='a12_per_delta_g_update', eta_by_delta_deg=rows, loss_by_delta_deg=table, loss_speeds_m_s=speeds)


def registered_overrides(a1, *, eta_scale=1.):
    """Profile overrides of the registered form: constant eta, understeer K_us, least-squares scrub loss per deg."""
    reg = a1['registered_form']
    eta = float(reg['eta']) * eta_scale
    k_us = float(reg['understeer_gradient_s2_m'])
    num = den = 0.
    for d, row in a1['turning_speed_loss']['table'].items():
        for item in row['by_speed'].values():
            num += float(d) * float(item['loss'])
            den += float(d) ** 2
    scrub = num / den if den else 0.
    out = dict(A12_OVERRIDES)
    out.update(steering_efficiency={'dist': 'uniform', 'low': eta, 'high': eta},
               understeer_gradient={'dist': 'uniform', 'low': k_us, 'high': k_us},
               scrub_speed_loss_per_deg={'dist': 'uniform', 'low': scrub, 'high': scrub})
    return out, dict(eta=eta, understeer_gradient_s2_m=k_us, scrub_speed_loss_per_deg=scrub)


def residuals(a1):
    """Largest |curvature residual| of the per-delta and of the registered form."""
    per = a1['per_delta_form']['max_abs_curvature_rel_residual']
    reg = a1['registered_form']['max_abs_curvature_rel_residual']
    return dict(per_delta=float(per), registered=float(reg))


# --- F-SIM scenes ----------------------------------------------------------------------------------------------------
def fsim_scene(name, script, *, form, a1, eta_scale=1., duty_scale=1.):
    from steering import scene_spec as ss
    schedule = json.loads(json.dumps(script['schedule']))
    schedule['knots'] = [[t, a * duty_scale, s] for t, a, s in schedule['knots']]
    start = script['start']
    if script['world'] == 'W0':
        world = dict(kind='W0')
        start_spec = dict(x_m=0., y_m=0., yaw_deg=0., steer_deg=0.)
    else:
        world = dict(kind='W2', record=W2_RECORD, corridor_id=W2_CORRIDOR, direction=W2_DIRECTION)
        start_spec = dict(x_m=start['s_m'], y_m=start['d_m'], yaw_deg=start['heading_deg'], steer_deg=0.)
    if form == 'per_delta':
        profile = dict(overrides=dict(A12_OVERRIDES), override_label=A12_LABEL, allow_out_of_range=True,
                       plant_tables=plant_tables(a1, eta_scale=eta_scale))
    else:
        overrides, _ = registered_overrides(a1, eta_scale=eta_scale)
        profile = dict(overrides=overrides, override_label=A12_LABEL + '_registered', allow_out_of_range=True)
    return ss.scene(name, world=world, start=start_spec, profile=profile, seeds=[A12_SEED], backend='L0',
                    runner='direct', maneuver=dict(kind='schedule', params=schedule),
                    duration_s=float(schedule['end_s']), command_latency_s=COMMAND_LATENCY_S, stream_key='a12_twin')


def fsim_scenes(a1):
    """Every F-SIM scene of A12: per script and world, the forms (per_delta, registered) and the residual runs."""
    res = residuals(a1)
    out = []
    for world in ('W0', 'W2'):
        for name, script in sorted(scripts(world).items()):
            base = '%s_%s' % (world.lower(), name.replace('_', '-'))
            for form in ('per_delta', 'registered'):
                r = res[form]
                tag = 'pd' if form == 'per_delta' else 'reg'
                out.append(fsim_scene('a12.%s.%s.fit' % (base, tag), script, form=form, a1=a1))
                for sign, label in ((1., 'p'), (-1., 'm')):
                    out.append(fsim_scene('a12.%s.%s.eta%s' % (base, tag, label), script, form=form, a1=a1,
                                          eta_scale=1. + sign * r))
                    out.append(fsim_scene('a12.%s.%s.speed%s' % (base, tag, label), script, form=form, a1=a1,
                                          duty_scale=1. + sign * A4G_SPEED_ERROR))
    return out


# --- comparison ---------------------------------------------------------------------------------------------------------
def _trace(run_dir, name):
    from steering import fast_sim as fs
    arrays, _ = fs.load_trace(Path(run_dir) / 'trace' / ('%s__seed%d.npz' % (name, A12_SEED)))
    return arrays


def _fsim_track(arrays, world, transform=None):
    t = arrays['t_s']
    x, y, yaw = arrays['truth_x_m'], arrays['truth_y_m'], arrays['truth_yaw_unwrapped_rad']
    if world == 'W2':
        x, y = transform.to_w(x, y)
        yaw = yaw - transform.theta
    return np.asarray(t), np.asarray(x), np.asarray(y), np.asarray(yaw)


def _gsim_track(logs, vehicle, start_xy=None):
    t = logs['%s__t' % vehicle]
    x, y = logs['%s__x' % vehicle], logs['%s__y' % vehicle]
    yaw = np.unwrap(logs['%s__yaw' % vehicle])
    if start_xy is not None:
        x, y = x - start_xy[0], y - start_xy[1]
    return t, x, y, yaw


def track_difference(a, b):
    """Max position / heading difference of track b against track a (time-synchronised on a's times inside b's
    span), the end differences, and the distance-synchronised position difference."""
    ta, xa, ya, ha = a
    tb, xb, yb, hb = b
    sel = (ta >= tb[0]) & (ta <= tb[-1])
    ta, xa, ya, ha = ta[sel], xa[sel], ya[sel], ha[sel]
    xi, yi, hi = np.interp(ta, tb, xb), np.interp(ta, tb, yb), np.interp(ta, tb, hb)
    pos = np.hypot(xa - xi, ya - yi)
    head = np.degrees(np.abs(np.arctan2(np.sin(ha - hi), np.cos(ha - hi))))
    sa = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(xa), np.diff(ya)))))
    sb = np.concatenate(([0.], np.cumsum(np.hypot(np.diff(xi), np.diff(yi)))))
    grid = np.linspace(0., min(sa[-1], sb[-1]), 400)
    da = np.hypot(np.interp(grid, sa, xa) - np.interp(grid, sb, xi), np.interp(grid, sa, ya) - np.interp(grid, sb, yi))
    return dict(max_position_m=float(pos.max()), max_heading_deg=float(head.max()), end_position_m=float(pos[-1]),
                end_heading_deg=float(head[-1]), max_position_same_distance_m=float(da.max()),
                path_a_m=float(sa[-1]), path_b_m=float(sb[-1]), samples=int(len(ta)))


def compare(a1, fsim_dir, gsim_dirs):
    """A12 result dict (see the module docstring). gsim_dirs: {world: gz_probe group folder (logs.npz, result.json)}."""
    transform = w2_field_world().transform
    res = residuals(a1)
    rows, passed = [], True
    for world in ('W0', 'W2'):
        folder = Path(gsim_dirs[world])
        result = json.loads((folder / 'result.json').read_text(encoding='utf-8'))
        with np.load(str(folder / 'logs.npz'), allow_pickle=False) as data:
            logs = {k: data[k] for k in data.files}
        by_script = {spec['tags']['script']: (name, spec) for name, spec in result['vehicles'].items()}
        for name, script in sorted(scripts(world).items()):
            vehicle, spec = by_script[name]
            start_xy = spec['start_pose'][:2] if world == 'W0' else None
            g = _gsim_track(logs, vehicle, start_xy)
            base = '%s_%s' % (world.lower(), name.replace('_', '-'))
            row = dict(world=world, script=name, vehicle=vehicle, forms={})
            for form, tag in (('per_delta', 'pd'), ('registered', 'reg')):
                f = _fsim_track(_trace(fsim_dir, 'a12.%s.%s.fit' % (base, tag)), world, transform)
                diff = track_difference(f, g)
                pred_pos = pred_head = 0.
                for label in ('etap', 'etam', 'speedp', 'speedm'):
                    p = _fsim_track(_trace(fsim_dir, 'a12.%s.%s.%s' % (base, tag, label)), world, transform)
                    d = track_difference(f, p)
                    pred_pos = max(pred_pos, d['max_position_m'])
                    pred_head = max(pred_head, d['max_heading_deg'])
                limit_pos = max(POSITION_FLOOR_M, PREDICTION_FACTOR * pred_pos)
                limit_head = max(HEADING_FLOOR_DEG, PREDICTION_FACTOR * pred_head)
                ok = diff['max_position_m'] <= limit_pos and diff['max_heading_deg'] <= limit_head
                row['forms'][form] = dict(difference=diff, predicted_position_m=pred_pos,
                                          predicted_heading_deg=pred_head, limit_position_m=limit_pos,
                                          limit_heading_deg=limit_head, passed=bool(ok),
                                          fit_residual_used=res[form], judged=form == 'per_delta')
                if form == 'per_delta':
                    passed = passed and ok
            rows.append(row)
    reg_fit = registered_overrides(a1)[1]
    return ro.rounded(dict(
        schema=SCHEMA, check='A12', passed=bool(passed), judged_form='per_delta (amendment 3)',
        criterion='max position difference <= max(0.10 m, 1.5 x predicted) and max heading difference <= max(2 deg, '
                  '1.5 x predicted), every script, time-synchronised (definitions in WORLDS_KO.md)',
        rows=rows, fit=dict(per_delta=plant_tables(a1), registered=reg_fit, residuals=res,
                            a4g_speed_error=A4G_SPEED_ERROR, a1_input_source=a1.get('measurement'),
                            a1_input_profile_sha256=a1.get('profile_sha256')),
        random_terms_off=A12_OVERRIDES, command_timing=dict(period_s=COMMAND_PERIOD_S, latency_s=COMMAND_LATENCY_S),
        w2=dict(record=W2_RECORD, corridor_id=W2_CORRIDOR, direction=W2_DIRECTION, field_spec_sha256=w2_field_spec()
                .sha256(), frame='W (corridor start frame)'),
        result_kind='자리값 기반 SIM 결과', field_validated=False), 6)


def main(argv=None):
    parser = argparse.ArgumentParser(description='A12 F-SIM vs G-SIM agreement (design 7.2 A12).')
    sub = parser.add_subparsers(dest='command', required=True)
    p_f = sub.add_parser('fsim')
    p_f.add_argument('--a1-input', type=Path, required=True)
    p_f.add_argument('--out', type=Path, required=True)
    p_f.add_argument('--workers', type=int, default=1)
    p_c = sub.add_parser('compare')
    p_c.add_argument('--a1-input', type=Path, required=True)
    p_c.add_argument('--fsim', type=Path, required=True)
    p_c.add_argument('--gsim-w0', type=Path, required=True)
    p_c.add_argument('--gsim-w2', type=Path, required=True)
    p_c.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    a1 = load_a1_input(args.a1_input)
    if args.command == 'fsim':
        from steering import fast_sim as fs
        scenes = fsim_scenes(a1)
        summary = fs.run_scenes(scenes, args.out, workers=args.workers, progress=print)
        print(json.dumps(dict(scenes=len(scenes), status=summary.get('status') if isinstance(summary, dict) else None)))
        return 0
    result = compare(a1, args.fsim, {'W0': args.gsim_w0, 'W2': args.gsim_w2})
    args.out.write_bytes((json.dumps(result, indent=1, sort_keys=True) + '\n').encode('utf-8'))
    for row in result['rows']:
        pd = row['forms']['per_delta']
        print('%s %s pos %.4f m (limit %.3f) head %.3f deg (limit %.2f) %s' % (
            row['world'], row['script'], pd['difference']['max_position_m'], pd['limit_position_m'],
            pd['difference']['max_heading_deg'], pd['limit_heading_deg'], 'PASS' if pd['passed'] else 'FAIL'))
    print('A12 passed:', result['passed'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
