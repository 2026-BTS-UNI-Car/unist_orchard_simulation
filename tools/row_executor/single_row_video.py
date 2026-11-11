"""Top-down video of one single-row episode on a 10/10 raster corridor (SIM only, visualisation only).

The video never feeds anything back into a run. Per frame, in the unrolled corridor frame (s along the
labelled centreline, d left of travel positive):
  overview strip of the whole corridor (vertical exaggeration) and a true-scale zoom around the vehicle:
  raster classes (black occupied, light grey free, mid grey uncertain, white unobserved; added hard cells red,
  added branch cells green), the labelled centreline (d = 0, blue dashed), the start (s = 0, green) and the
  row end (last trunk on both rows, red), walls, the TRUE footprint (orange rectangle) and the true path,
  the executor's ESTIMATED corridor: its centreline from est_lateral / est_heading drawn relative to the true
  pose (green), its band edges and its row-end estimate when a snapshot log is given;
  text: time, true s / d / heading error, estimated lateral / heading, speed, state, reason, corridor reason,
  final PREREG end / stop category (from closed_loop.raster_corridor.evaluate_episode); L/R duty bars.
Frames: matplotlib (Agg) RGBA piped to ffmpeg (libx264, yuv420p).

Smoke usage (from the worktree root; existing RowController continuous mode; smoke only, not evaluation):
  python -B tools/row_executor/single_row_video.py --smoke --record 042301 --corridor c03 --direction fwd \
      --seconds 45 --out <runs folder outside the repository>/smoke.mp4
ffmpeg: an explicit path, else sim_paths 'ffmpeg' (UBTS_FFMPEG, the path config file, or ffmpeg on PATH; steering branch,
docs/steering/EXECUTOR_IMPORT_KO.md).
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SIM_DIR = os.path.join(REPO, 'sim', 'ubts')
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)
sys.dont_write_bytecode = True

import numpy as np  # noqa: E402

from closed_loop import raster_corridor as RC  # noqa: E402

try:
    from row_executor import sim_paths as XP  # noqa: E402  (imported as row_executor.single_row_video)
except ImportError:
    import sim_paths as XP  # noqa: E402  (run as a script: this folder is sys.path[0])

COLOURS = {RC.FREE: (.88, .88, .88), RC.UNCERTAIN: (.76, .76, .78), RC.UNCERTAIN_OCC: (.6, .6, .64),
           RC.UNOBSERVED: (1., 1., 1.), RC.OCCUPIED: (0., 0., 0.), RC.ADDED_HARD: (.8, 0., 0.),
           RC.ADDED_BRANCH: (0., .55, 0.)}


def ffmpeg_path(explicit=None):
    """explicit when it is a file, else sim_paths 'ffmpeg' (UBTS_FFMPEG, the path config file, or ffmpeg on PATH).
    FileNotFoundError when none is found."""
    if explicit and os.path.isfile(explicit):
        return explicit
    return str(XP.configured_path('ffmpeg', 'file'))


class SnapshotLog:
    """Estimator wrapper factory for harness.run_episode: records the estimated corridor per controller tick."""

    def __init__(self, base_factory=None):
        from closed_loop.estimator import SensorEstimator
        self.base = base_factory or SensorEstimator
        self.rows = []

    def __call__(self, config):
        log, base = self.rows, self.base

        class Recording(base):
            def snapshot(self, now):
                snap = super().snapshot(now)
                log.append(dict(t=round(now, 2), corridor_width=snap.get('corridor_width'),
                                row_end_ahead_m=snap.get('row_end_ahead_m'),
                                side_ends_ahead_m=snap.get('side_ends_ahead_m')))
                return snap
        return Recording(config)


def _unrolled(world, s_lo, s_hi, half, step=.05):
    ss = np.arange(s_lo, s_hi + 1e-9, step)
    dd = np.arange(-half, half + 1e-9, step)
    S, D = np.meshgrid(ss, dd)
    x, y = world.frame.point_many(S.ravel(), D.ravel())
    classes = world.map.class_at(x, y).reshape(S.shape)
    lut = np.array([COLOURS[k] for k in range(len(RC.CLASS_NAMES))])
    return lut[classes[::-1]], (ss[0], ss[-1], dd[0], dd[-1])


def _field_points(world, xs, ys):
    s, d = world.frame.project_many(xs, ys)
    return s, d


def render_video(result, world, out_path, *, snapshots=None, evaluation=None, fps=10, speed=2., title=None,
                 ffmpeg=None, footprint=(1.22, .70), hold_end_s=2.):
    """Render one traced episode (result['trace'], harness 20 Hz) to an MP4. Returns a small report dict."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    trace = result['trace']
    t = np.array([r['t'] for r in trace])
    s = np.array([r['fx'] for r in trace])
    d = np.array([r['fy'] for r in trace])
    e = np.radians(np.array([r['fyaw_deg'] for r in trace]))
    poses = np.array([world.pose_in_field(si, di, ei) for si, di, ei in zip(s, d, e)])
    s_end = world.s_end
    half = max(4., world.config.row_width_m / 2 + 2.)
    s_lo, s_hi = min(-6., float(s.min()) - 3.), max(s_end + 10., float(s.max()) + 5.)
    image, extent = _unrolled(world, s_lo, s_hi, half)
    snap_by_t = {round(r['t'], 2): r for r in (snapshots or [])}
    W, H, dpi = 1280, 720, 100
    fig = plt.figure(figsize=(W / dpi, H / dpi), dpi=dpi)
    gs = fig.add_gridspec(2, 2, width_ratios=(1.75, 1.), height_ratios=(.32, 1.), hspace=.28, wspace=.12,
                          left=.05, right=.98, top=.93, bottom=.07)
    ax_o = fig.add_subplot(gs[0, :])
    ax_z = fig.add_subplot(gs[1, 0])
    ax_i = fig.add_subplot(gs[1, 1])
    ax_i.axis('off')
    cfg = world.config
    fig.suptitle(title or '%s %s %s  (raster SIM, SLAM-reconstructed map, not physical truth)' % (
        cfg.record, cfg.corridor_id, cfg.direction), fontsize=11, x=.01, ha='left')
    profile_s = np.asarray(cfg.profile_s_m)
    for ax in (ax_o, ax_z):
        ax.imshow(image, extent=extent, aspect='auto' if ax is ax_o else 'equal', interpolation='nearest')
        ax.axhline(0., color=(.1, .4, 1.), ls='--', lw=1)
        ax.plot(profile_s, cfg.left_row_d_m, '-', color=(.3, .5, 1.), lw=.6, alpha=.6)
        ax.plot(profile_s, cfg.right_row_d_m, '-', color=(.3, .5, 1.), lw=.6, alpha=.6)
        ax.axvline(0., color=(0., .7, 0.), lw=1.3)
        ax.axvline(s_end, color=(.9, 0., 0.), lw=1.6)
        for w in world.walls_s:
            ax.axvline(w, color=(.6, 0., 0.), lw=2.5, alpha=.6)
        ax.set_ylim(-half, half)
    ax_o.set_xlim(s_lo, s_hi)
    ax_o.set_ylabel('d [m]', fontsize=8)
    ax_o.tick_params(labelsize=7)
    ax_o.set_title('whole corridor (vertical exaggeration): red line = row end (last trunk on both rows), '
                   'green = start; orange = true path', fontsize=8)
    ax_z.set_xlabel('s [m] along the labelled centreline', fontsize=8)
    ax_z.set_ylabel('d [m] (left +)', fontsize=8)
    ax_z.tick_params(labelsize=7)
    path_o, = ax_o.plot([], [], '-', color=(1., .5, 0.), lw=1.2)
    dot_o, = ax_o.plot([], [], 'o', color=(1., .4, 0.), ms=4)
    window = Rectangle((0, -half), 20., 2 * half, fill=False, ec=(.2, .2, .2), lw=.8, ls=':')
    ax_o.add_patch(window)
    path_z, = ax_z.plot([], [], '-', color=(1., .5, 0.), lw=1.5)
    body = Polygon(np.zeros((4, 2)), closed=True, fc=(1., .55, 0., .45), ec=(.9, .35, 0.), lw=1.5)
    ax_z.add_patch(body)
    nose, = ax_z.plot([], [], '-', color=(.9, .35, 0.), lw=2.5)
    est_c, = ax_z.plot([], [], '-', color=(0., .65, .2), lw=2.)
    est_l, = ax_z.plot([], [], ':', color=(0., .65, .2), lw=1.2)
    est_r, = ax_z.plot([], [], ':', color=(0., .65, .2), lw=1.2)
    est_end, = ax_z.plot([], [], '-', color=(.8, 0., .8), lw=2.5)
    text = ax_i.text(0., 1., '', va='top', ha='left', family='monospace', fontsize=8.6, transform=ax_i.transAxes)
    bar_ax = fig.add_axes([.69, .1, .27, .2])
    bars = bar_ax.barh([1, 0], [0, 0], color=[(.2, .4, .9), (.9, .3, .3)], height=.6)
    bar_ax.set_yticks([1, 0], ['L', 'R'])
    bar_ax.set_xlim(-100, 100)
    bar_ax.axvline(0, color='k', lw=.8)
    bar_ax.set_title('commanded duty [%] (PWM is not speed)', fontsize=8)
    bar_ax.tick_params(labelsize=7)
    footer = ('legend: black occupied / grey uncertain / white unobserved (no return) / red added hard / green '
              'added branch;  green line = executor ESTIMATED centreline (dotted: est. band), magenta = est. row end')
    fig.text(.01, .012, footer, fontsize=7)
    end_index = len(trace) - 1
    final_text = ''
    if evaluation is not None:
        final_text = ('\nfinal: %s  category %s\n e (front - end) %s m  overshoot %s m\n lateral rmse %s p95 %s max %s m'
                      % (evaluation.get('prereg_end'), evaluation.get('stop_category'),
                         evaluation.get('stop_position_e_m'), evaluation.get('overshoot_m'),
                         evaluation['lateral_m']['rmse'], evaluation['lateral_m']['p95'], evaluation['lateral_m']['max']))
    step = speed / fps
    frame_times = list(np.arange(t[0], t[-1] + 1e-9, step)) + [t[-1]] * int(hold_end_s * fps)
    cmd = [ffmpeg_path(ffmpeg), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', '%dx%d' % (W, H),
           '-r', str(fps), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23', out_path]
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    started = time.perf_counter()
    L, Wd = footprint
    corners_body = np.array([(L / 2, Wd / 2), (L / 2, -Wd / 2), (-L / 2, -Wd / 2), (-L / 2, Wd / 2)])
    try:
        for k, ft in enumerate(frame_times):
            i = min(int(np.searchsorted(t, ft + 1e-9, side='right')) - 1, end_index)
            i = max(i, 0)
            row = trace[i]
            x, y, yaw = poses[i]
            c, sn = math.cos(yaw), math.sin(yaw)
            cx = x + c * corners_body[:, 0] - sn * corners_body[:, 1]
            cy = y + sn * corners_body[:, 0] + c * corners_body[:, 1]
            bs, bd = _field_points(world, cx, cy)
            body.set_xy(np.column_stack((bs, bd)))
            nose.set_data(bs[:2], bd[:2])
            path_o.set_data(s[:i + 1], d[:i + 1])
            dot_o.set_data([s[i]], [d[i]])
            path_z.set_data(s[:i + 1], d[:i + 1])
            window.set_x(s[i] - 6.)
            ax_z.set_xlim(s[i] - 6., s[i] + 14.)
            est_lat, est_hdg = row.get('est_lateral'), row.get('est_heading_deg')
            snap = snap_by_t.get(round(row['t'], 2), {})
            if est_lat is not None and est_hdg is not None:
                beta = yaw + math.radians(est_hdg)
                ux, uy, nx, ny = math.cos(beta), math.sin(beta), -math.sin(beta), math.cos(beta)
                px, py = x - est_lat * nx, y - est_lat * ny
                tau = np.linspace(-5., 14., 40)
                es, ed = _field_points(world, px + tau * ux, py + tau * uy)
                est_c.set_data(es, ed)
                width = snap.get('corridor_width')
                if width:
                    ls_, ld_ = _field_points(world, px + tau * ux + width / 2 * nx, py + tau * uy + width / 2 * ny)
                    rs_, rd_ = _field_points(world, px + tau * ux - width / 2 * nx, py + tau * uy - width / 2 * ny)
                    est_l.set_data(ls_, ld_)
                    est_r.set_data(rs_, rd_)
                else:
                    est_l.set_data([], [])
                    est_r.set_data([], [])
                ahead = snap.get('row_end_ahead_m')
                if ahead is not None:
                    ex, ey = px + ahead * ux, py + ahead * uy
                    qs, qd = _field_points(world, np.array([ex - 1.5 * nx, ex + 1.5 * nx]),
                                           np.array([ey - 1.5 * ny, ey + 1.5 * ny]))
                    est_end.set_data(qs, qd)
                else:
                    est_end.set_data([], [])
            else:
                for artist in (est_c, est_l, est_r, est_end):
                    artist.set_data([], [])
            for bar, value in zip(bars, (row['left'], row['right'])):
                bar.set_width(value)
            text.set_text(
                't %6.2f s   speed %.2f m/s\n'
                'true  s %6.2f m  d %+.3f m  hdg %+.2f deg\n'
                'est.  lateral %s m  heading %s deg\n'
                'row end s_E %.2f m  front - s_E %+.2f m\n'
                'state  %s\nreason %s\ncorridor %s\nL/R    %d / %d%s' % (
                    row['t'], row['v'], row['fx'], row['fy'], row['fyaw_deg'],
                    '--' if est_lat is None else '%+.3f' % est_lat, '--' if est_hdg is None else '%+.2f' % est_hdg,
                    s_end, float(bs[:2].mean() - s_end), row['state'], str(row['reason'])[:34],
                    str(row['corridor_reason'])[:34], row['left'], row['right'],
                    final_text if i == end_index else ''))
            fig.canvas.draw()
            proc.stdin.write(bytes(fig.canvas.buffer_rgba()))
    finally:
        proc.stdin.close()
        code = proc.wait()
        plt.close(fig)
    if code != 0:
        raise RuntimeError('ffmpeg exit %d' % code)
    return dict(out=out_path, frames=len(frame_times), fps=fps, speed=speed, render_s=round(time.perf_counter() - started, 2),
                bytes=os.path.getsize(out_path))


def smoke(args):
    """Short smoke episode with the EXISTING RowController (continuous mode) on one catalogued corridor."""
    from closed_loop.harness import EpisodeConfig
    world_config = RC.corridor_world_config(args.record, args.corridor, args.direction)
    log = SnapshotLog()
    config = EpisodeConfig(name='smoke_%s_%s_%s' % (args.record, args.corridor, args.direction), mode='continuous',
                           world=world_config, start_x_m=args.start_s, start_y_m=args.start_d,
                           start_yaw_deg=args.start_yaw, max_time_s=args.seconds, seed=args.seed)
    result, world = RC.run_raster_episode(config, estimator_factory=log)
    report = render_video(result, world, args.out, snapshots=log.rows, evaluation=result['raster_evaluation'],
                          fps=args.fps, speed=args.speed,
                          title='SMOKE (not evaluation): RowController continuous, %s %s %s, %.0f s SIM' % (
                              args.record, args.corridor, args.direction, args.seconds))
    summary = dict(schema='ubts.single_row.raster_smoke.v1', purpose='smoke only (video/world plumbing), NOT evaluation',
                   controller='closed_loop.controllers.RowController mode continuous (existing)',
                   episode=dict(record=args.record, corridor=args.corridor, direction=args.direction,
                                seconds=args.seconds, start_s=args.start_s, start_d=args.start_d,
                                start_yaw_deg=args.start_yaw, seed=args.seed),
                   wall_s=result['wall_s'], end=result['end'], final_reason=result['final_reason'],
                   progress_m=result['progress_m'], raster_world=result['raster_world'],
                   evaluation=result['raster_evaluation'], video=report)
    with open(os.path.splitext(args.out)[0] + '.json', 'w', encoding='utf-8', newline='\n') as fh:
        json.dump(summary, fh, indent=1, default=str)
        fh.write('\n')
    print(json.dumps(dict(video=report, end=result['end'], reason=result['final_reason'],
                          lidar=result['raster_world']['lidar'], wall_s=result['wall_s']), indent=1))
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    p.add_argument('--smoke', action='store_true', help='run the smoke episode and render it')
    p.add_argument('--record', default='042301')
    p.add_argument('--corridor', default='c03')
    p.add_argument('--direction', default='fwd', choices=('fwd', 'rev'))
    p.add_argument('--seconds', type=float, default=45.)
    p.add_argument('--start-s', type=float, default=1.)
    p.add_argument('--start-d', type=float, default=.2)
    p.add_argument('--start-yaw', type=float, default=3.)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--fps', type=int, default=10)
    p.add_argument('--speed', type=float, default=2.)
    p.add_argument('--out', required=True)
    args = p.parse_args(argv)
    if not args.smoke:
        p.error('only --smoke is a CLI mode; use render_video() from Python for other episodes')
    return smoke(args)


if __name__ == '__main__':
    sys.exit(main())
