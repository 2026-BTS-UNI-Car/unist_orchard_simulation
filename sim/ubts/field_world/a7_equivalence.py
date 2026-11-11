"""A7 world equivalence (design 7.2 A7, SIM_ACCEPTANCE amendment 6): mode R + Embree + clip node vs the raster DDA.

prepare (host, needs the maps and the catalogue): builds a mode R field world (crop fsim) for every development
  direction (field_world.scene_spec.development_directions: 32 directions; the 5 sealed / sealed-pair-reverse
  directions are never built) and draws POSES_PER_DIRECTION sensor poses per direction with the original check's
  rule (single-row raycast_check: s ~ U(1, s_E - 1), d ~ U(-1, 1), heading error ~ U(-0.6, 0.6) rad; new seed
  POSE_SEED). Poses are written in the field world frame W (local SIM frame).
run (SIM image: needs trimesh + Embree, the pinned mesh_raycast.StaticVisualWorld that cpu_lidar.py uses): per pose
  3200 rays at -pi + i 2 pi / 3200 + heading from (x, y, LIDAR_Z_M) through the field world SDF (Embree, first hit,
  no minimum-range clamp), then steering.scan_unobserved_clip.clip_ranges with the world's raster copy; against the
  reference supercover DDA of closed_loop/testdata/raster_world_ref_e71e27d4.py (RasterGrid.raycast: reflectors
  return, unobserved / outside end the ray) on the same raster copy.

Judged (registered A7, amendment 6; definitions fixed before the first measurement, written in WORLDS_KO.md):
  hit_mismatch_fraction      rays whose return / no-return differs, over all rays          <= 0.001
  range_mismatch_fraction    rays with both returns and |r_G - r_raster| > 0.071 m (cell diagonal), over the rays
                             with both returns                                              <= 0.005
                             (also reported over all rays)
Every mismatch is classified: 'grazing' when the reference result changes to the Embree + clip result under a
perturbation of the ray by +-1e-6 rad or of the origin by +-1e-5 m (a cell corner or edge within numerical reach of
the ray), else 'not_grazing'. Mode T worlds can be measured the same way (report only).

Reported with it (2026-11-09 reviews RW-03, RW-04):
  time limits   per world the Embree load, the first scan and their sum against build_field_world.TIME_LIMITS
  range noise   the judged comparison uses exact ranges. With range noise (sd 0.01 = rover.yaml / cpu_lidar, 0.015 =
                the F-SIM raster LiDAR; one draw per pose, the same for both orders) the hit mismatch against the
                raster DDA is counted for two orders: noise then clip (cpu_lidar with its noise in front of the clip
                node, the stack before the fix) and clip then noise (the F-SIM order, the steering stack now: cpu_lidar
                without noise, the clip node adds it). The noise moves no return across the UNOBSERVED end in the
                second order; the first can lose a return just in front of it or keep one just behind it.

CLI (sim/ubts on sys.path):
  python -B -m field_world.a7_equivalence prepare --out <folder> [--mode R|T]
  python3 -B -m field_world.a7_equivalence run --worlds <folder> --out <result.json>      (SIM image)
Placeholder-based SIM results, not field validation.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import sys
import time

import numpy as np

from . import raster_ops as ro

SCHEMA = 'ubts.steer_field_world.a7.v1'
POSE_SEED = 20261109
POSES_PER_DIRECTION = 6
RAYS = 3200
MAX_RANGE_M = 30.
LIDAR_Z_M = .6
CELL_DIAGONAL_M = .05 * math.sqrt(2.)
HIT_LIMIT = .001
RANGE_LIMIT = .005
PERTURB_ANGLE_RAD = 1e-6
PERTURB_ORIGIN_M = 1e-5
NOISE_SDS = (.01, .015)
MIN_RANGE_M = .05
UBTS_DIR = Path(__file__).resolve().parents[1]
REF_PATH = UBTS_DIR / 'closed_loop' / 'testdata' / 'raster_world_ref_e71e27d4.py'
REF_SHA256 = 'e71e27d40522073908d2b5212a4a05f92cb1187c783f69256fc5080bd9d5f507'


def reference_module():
    raw = REF_PATH.read_bytes()
    if hashlib.sha256(raw).hexdigest() != REF_SHA256:
        raise RuntimeError('reference raster world changed (sha256)')
    spec = importlib.util.spec_from_file_location('raster_world_ref', REF_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# --- prepare --------------------------------------------------------------------------------------------------------
def prepare(out_dir, *, mode='R', catalogue=None, directions=None):
    from closed_loop import raster_corridor as RC
    from . import build_field_world as fwb
    from .scene_spec import catalogue_spec, development_directions
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    catalogue = catalogue if isinstance(catalogue, dict) else RC.load_catalogue(catalogue)
    dev = development_directions(catalogue) if directions is None else directions
    rng = np.random.default_rng(POSE_SEED)
    rows = []
    for record, corridor, direction, role in dev:
        spec = catalogue_spec(record, corridor, direction, mode=mode)
        started = time.perf_counter()
        fw = fwb.build(spec, catalogue=catalogue)
        folder = out / spec.name
        manifest = fwb.write(fw, folder)
        build_s = time.perf_counter() - started
        poses = []
        for _ in range(POSES_PER_DIRECTION):
            s = float(rng.uniform(1., fw.config.end_s_m - 1.))
            d = float(rng.uniform(-1., 1.))
            e = float(rng.uniform(-.6, .6))
            x, y, yaw = fw.frame.pose(s, d, e)
            wx, wy = fw.transform.to_w(x, y)
            # positions at 5 decimals (10 um; coord_lint), angles at 9
            poses.append(dict(s_m=round(s, 5), d_m=round(d, 5), heading_error_rad=round(e, 9),
                              x_w=round(float(wx), 5), y_w=round(float(wy), 5),
                              yaw_w=round(float(fw.transform.yaw_to_w(yaw)), 9)))
        (folder / 'a7_poses.json').write_bytes((json.dumps(dict(
            schema=SCHEMA + '.poses', name=spec.name, spec_sha256=spec.sha256(), pose_seed=POSE_SEED,
            rule='s ~ U(1, s_E - 1), d ~ U(-1, 1), heading error ~ U(-0.6, 0.6) rad (single-row raycast_check)',
            poses=poses), indent=1) + '\n').encode('utf-8'))
        rows.append(dict(name=spec.name, record=record, corridor_id=corridor, direction=direction, role=role,
                         boxes=manifest['counts']['boxes'], sdf_bytes=manifest['counts']['sdf_bytes'],
                         crop_cells=manifest['counts']['crop_cells'], build_s=round(build_s, 3),
                         spec_sha256=spec.sha256(), coord_lint_ok=manifest['coord_lint']['ok']))
    index = dict(schema=SCHEMA + '.index', mode=mode, pose_seed=POSE_SEED, poses_per_direction=POSES_PER_DIRECTION,
                 directions=rows, deferred='sealed and sealed-pair-reverse directions are not built (amendment 6)')
    (out / 'A7_WORLDS.json').write_bytes((json.dumps(index, indent=1) + '\n').encode('utf-8'))
    return index


# --- run ----------------------------------------------------------------------------------------------------------------
def embree_ranges(intersector, origin, angles):
    """First-hit distance per ray (inf: none or beyond MAX_RANGE_M), horizontal rays from origin (3,)."""
    directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
    origins = np.tile(np.asarray(origin, dtype=float), (len(angles), 1))
    locations, index_ray, _ = intersector.intersects_location(origins, directions, multiple_hits=False)
    ranges = np.full(len(angles), np.inf)
    if len(index_ray):
        distance = np.linalg.norm(locations - np.asarray(origin, dtype=float), axis=1)
        ranges[index_ray] = distance
    ranges[ranges > MAX_RANGE_M] = np.inf
    return ranges, directions


def reference_ranges(grid, REF, raster, x_w, y_w, angles_w):
    x_l, y_l = raster.transform.to_l(x_w, y_w)
    gx = float(x_l) - raster.col0 * raster.resolution
    gy = float(y_l) - raster.row0 * raster.resolution
    ranges, kinds = grid.raycast(gx, gy, np.asarray(angles_w) + raster.transform.theta, MAX_RANGE_M)
    out = np.where(kinds == REF.OCCUPIED, ranges, np.inf)
    out[out > MAX_RANGE_M] = np.inf
    return out


def _agrees(a, b):
    if np.isfinite(a) != np.isfinite(b):
        return False
    return (not np.isfinite(a)) or abs(a - b) <= CELL_DIAGONAL_M


def classify(grid, REF, raster, x_w, y_w, angle_w, g_value):
    """'grazing' when a tiny perturbation of the ray or the origin makes the reference agree with g_value."""
    for da in (-PERTURB_ANGLE_RAD, PERTURB_ANGLE_RAD):
        if _agrees(reference_ranges(grid, REF, raster, x_w, y_w, [angle_w + da])[0], g_value):
            return 'grazing'
    for dx, dy in ((PERTURB_ORIGIN_M, 0.), (-PERTURB_ORIGIN_M, 0.), (0., PERTURB_ORIGIN_M), (0., -PERTURB_ORIGIN_M)):
        if _agrees(reference_ranges(grid, REF, raster, x_w + dx, y_w + dy, [angle_w])[0], g_value):
            return 'grazing'
    return 'not_grazing'


def run(worlds_dir, *, limit=None):
    from closed_loop import raster_corridor as RC
    from mesh_raycast import StaticVisualWorld
    from steering import scan_unobserved_clip as clip
    from .build_field_world import RasterCopy
    REF = reference_module()
    worlds_dir = Path(worlds_dir)
    index = json.loads((worlds_dir / 'A7_WORLDS.json').read_text(encoding='utf-8'))
    rows = index['directions'][:limit] if limit else index['directions']
    from .build_field_world import TIME_LIMITS, time_limits_report
    totals = dict(rays=0, hit_mismatch=0, both_hit=0, range_mismatch=0, grazing=0, not_grazing=0, clipped=0,
                  returns=0, max_abs_range_diff_m=0.)
    noise_totals = {'%g' % sd: dict(noise_then_clip=0, clip_then_noise=0) for sd in NOISE_SDS}
    per = []
    started_all = time.perf_counter()
    for i_dir, row in enumerate(rows):
        folder = worlds_dir / row['name']
        raster = RasterCopy(folder / 'raster.npz')
        cr = clip.ClipRaster(raster)
        cls = raster.classes
        cells = np.where(np.isin(cls, RC.REFLECTORS), REF.OCCUPIED,
                         np.where(cls == RC.UNOBSERVED, REF.UNKNOWN, REF.FREE)).astype(np.uint8)
        grid = REF.RasterGrid(cells, raster.resolution)
        t0 = time.perf_counter()
        world = StaticVisualWorld(folder / 'field_world.sdf', excluded_dynamic_models=())
        load_s = time.perf_counter() - t0
        poses = json.loads((folder / 'a7_poses.json').read_text(encoding='utf-8'))['poses']
        stats = dict(rays=0, hit_mismatch=0, both_hit=0, range_mismatch=0, grazing=0, not_grazing=0, clipped=0,
                     returns=0, max_abs_range_diff_m=0.)
        embree_ms, clip_ms, examples = [], [], []
        noise_dir = {'%g' % sd: dict(noise_then_clip=0, clip_then_noise=0) for sd in NOISE_SDS}
        for i_pose, pose in enumerate(poses):
            angles = -math.pi + np.arange(RAYS) * (2. * math.pi / RAYS) + pose['yaw_w']
            t1 = time.perf_counter()
            g_raw, directions = embree_ranges(world.rays, (pose['x_w'], pose['y_w'], LIDAR_Z_M), angles)
            t2 = time.perf_counter()
            g, info = clip.clip_ranges(cr, (pose['x_w'], pose['y_w'], LIDAR_Z_M), directions, g_raw)
            t3 = time.perf_counter()
            embree_ms.append((t2 - t1) * 1000.)
            clip_ms.append((t3 - t2) * 1000.)
            ref = reference_ranges(grid, REF, raster, pose['x_w'], pose['y_w'], angles)
            hit_mis = np.isfinite(g) != np.isfinite(ref)
            both = np.isfinite(g) & np.isfinite(ref)
            rng = np.random.default_rng([POSE_SEED, i_dir, i_pose])
            for sd in NOISE_SDS:
                noise = rng.normal(0., sd, RAYS)
                ref_hit = np.isfinite(ref)               # the raster with noise after the DDA: same returns (< 30 m)
                ref_hit &= ~(ref + noise > MAX_RANGE_M)
                raw = g_raw.copy()
                ok = np.isfinite(raw)
                raw[ok] = np.maximum(raw[ok] + noise[ok], MIN_RANGE_M)
                raw[raw > MAX_RANGE_M] = np.inf
                before, _ = clip.clip_ranges(cr, (pose['x_w'], pose['y_w'], LIDAR_Z_M), directions, raw)
                after = g.copy()
                ok = np.isfinite(after)
                after[ok] = np.maximum(after[ok] + noise[ok], MIN_RANGE_M)
                after[after > MAX_RANGE_M] = np.inf
                key = '%g' % sd
                noise_dir[key]['noise_then_clip'] += int((np.isfinite(before) != ref_hit).sum())
                noise_dir[key]['clip_then_noise'] += int((np.isfinite(after) != ref_hit).sum())
            diff = np.abs(g[both] - ref[both])
            range_mis = np.zeros(RAYS, dtype=bool)
            range_mis[np.flatnonzero(both)[diff > CELL_DIAGONAL_M]] = True
            for i in np.flatnonzero(hit_mis | range_mis):
                kind = classify(grid, REF, raster, pose['x_w'], pose['y_w'], float(angles[i]), float(g[i]))
                stats[kind] += 1
                if len(examples) < 12:
                    examples.append(dict(ray=int(i), kind=kind, hit_mismatch=bool(hit_mis[i]),
                                         g_m=None if not np.isfinite(g[i]) else round(float(g[i]), 6),
                                         raster_m=None if not np.isfinite(ref[i]) else round(float(ref[i]), 6),
                                         embree_raw_m=None if not np.isfinite(g_raw[i]) else round(float(g_raw[i]), 6)))
            stats['rays'] += RAYS
            stats['hit_mismatch'] += int(hit_mis.sum())
            stats['both_hit'] += int(both.sum())
            stats['range_mismatch'] += int(range_mis.sum())
            stats['clipped'] += info['clipped']
            stats['returns'] += int(np.isfinite(ref).sum())
            if diff.size:
                stats['max_abs_range_diff_m'] = max(stats['max_abs_range_diff_m'], float(diff.max()))
        for key in totals:
            if key == 'max_abs_range_diff_m':
                totals[key] = max(totals[key], stats[key])
            else:
                totals[key] += stats[key]
        for key, value in noise_dir.items():
            for order, count in value.items():
                noise_totals[key][order] += count
        times = dict(embree_load_s=load_s, embree_first_scan_s=embree_ms[0] / 1000.,
                     time_to_first_scan_s=load_s + embree_ms[0] / 1000.)
        per.append(dict(row, **stats, noise_hit_mismatch=noise_dir, time_limits=time_limits_report(times),
                        embree_load_s=round(load_s, 3), triangles=int(len(world.mesh.faces)),
                        embree_first_scan_ms=round(float(embree_ms[0]), 2),
                        time_to_first_scan_s=round(load_s + embree_ms[0] / 1000., 3),
                        embree_ms_median=round(float(np.median(embree_ms)), 2),
                        clip_ms_median=round(float(np.median(clip_ms)), 2), examples=examples,
                        raster_sha256=raster.sha256))
        print('[a7] %s rays %d hit-mis %d range-mis %d (load %.2f s)' % (row['name'], stats['rays'],
                                                                      stats['hit_mismatch'], stats['range_mismatch'],
                                                                      load_s), flush=True)
    hit_fraction = totals['hit_mismatch'] / max(totals['rays'], 1)
    range_fraction = totals['range_mismatch'] / max(totals['both_hit'], 1)
    passed = hit_fraction <= HIT_LIMIT and range_fraction <= RANGE_LIMIT
    import trimesh
    result = dict(
        schema=SCHEMA, check='A7', mode=index['mode'], directions=len(per), poses=len(per) * POSES_PER_DIRECTION,
        rays_per_pose=RAYS, totals=totals, hit_mismatch_fraction=hit_fraction,
        range_mismatch_fraction_of_both_hit=range_fraction,
        range_mismatch_fraction_of_all=totals['range_mismatch'] / max(totals['rays'], 1),
        noise_hit_mismatch=dict(counts=noise_totals, rays=totals['rays'],
                                fractions={k: {o: c / max(totals['rays'], 1) for o, c in v.items()}
                                           for k, v in noise_totals.items()},
                                note='report (RW-04): hit mismatch against the raster DDA with range noise, '
                                     'noise then clip (old stack order) and clip then noise (F-SIM and the stack now)'),
        time_limits=dict(within=all(p['time_limits']['within'] for p in per),
                         over=[p['name'] for p in per if not p['time_limits']['within']],
                         limits=TIME_LIMITS),
        criteria=dict(hit_mismatch_fraction_max=HIT_LIMIT, range_mismatch_fraction_max=RANGE_LIMIT,
                      range_tolerance_m=CELL_DIAGONAL_M,
                      denominators='hit: all rays; range: rays with both returns (also reported over all rays)'),
        passed=bool(passed) if index['mode'] == 'R' else None,
        judged=index['mode'] == 'R', reference=dict(path='closed_loop/testdata/raster_world_ref_e71e27d4.py',
                                                    sha256=REF_SHA256),
        classification_rule='grazing: the reference agrees under +-%g rad or +-%g m perturbation'
                            % (PERTURB_ANGLE_RAD, PERTURB_ORIGIN_M),
        lidar_z_m=LIDAR_Z_M, max_range_m=MAX_RANGE_M, pose_seed=index['pose_seed'],
        environment=dict(python=platform.python_version(), numpy=np.__version__, trimesh=trimesh.__version__,
                         platform=sys.platform),
        wall_s=round(time.perf_counter() - started_all, 2), per_direction=per,
        sealed_deferred='5 directions (sealed 3 + sealed pair reverse 2) not measured (amendment 6)',
        result_kind='자리값 기반 SIM 결과', field_validated=False)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description='A7 world equivalence (mode R + Embree + clip vs raster DDA).')
    sub = parser.add_subparsers(dest='command', required=True)
    p_prep = sub.add_parser('prepare')
    p_prep.add_argument('--out', type=Path, required=True)
    p_prep.add_argument('--mode', choices=('R', 'T'), default='R')
    p_run = sub.add_parser('run')
    p_run.add_argument('--worlds', type=Path, required=True)
    p_run.add_argument('--out', type=Path, required=True)
    p_run.add_argument('--limit', type=int)
    args = parser.parse_args(argv)
    if args.command == 'prepare':
        index = prepare(args.out, mode=args.mode)
        print(json.dumps(dict(directions=len(index['directions']),
                              boxes=[r['boxes'] for r in index['directions']]), indent=0)[:2000])
        return 0
    result = run(args.worlds, limit=args.limit)
    try:
        from steering import gz_assemble
        result['provenance'] = dict(source=gz_assemble.source_identity())
    except Exception as exc:                       # information only
        result['provenance'] = dict(error='%s: %s' % (type(exc).__name__, exc))
    args.out.write_bytes((json.dumps(ro.rounded(result, 6), indent=1, sort_keys=True) + '\n').encode('utf-8'))
    print(json.dumps({k: result[k] for k in ('totals', 'hit_mismatch_fraction', 'range_mismatch_fraction_of_both_hit',
                                             'passed', 'wall_s')}, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
