"""body_mask_static (C0 standstill body mask): synthetic scans with known parts, the recorder sidecar reader / CLI, and
the real 10/10 standstill windows where near returns were seen (review_correctness_day1 F3: 042200, 035952, 051127).

The real-data cases run where the 10/10 compact extracts are available (UBTS_RD_1010_LIDAR, or the lidar/ folder of
row_executor.sim_paths 'field_manual_1010': UBTS_RD_1010 or the path config file; no built-in machine path);
elsewhere they are skipped with that reason.
"""
import gzip
import json
import math
import os
import random
import tempfile
import unittest

import numpy as np

import rowexec_testlib as L
from row_executor import body_mask_static as B
from row_executor import config as C
from row_executor.core import ScanAdapter

CFG = C.ExecutorConfig()
# standstill windows of the 10/10 records with near returns (offline ICP reference |v| < 0.03 m/s, |w| < 1.5 deg/s;
# found with the review's near_vs_motion analysis): (record, first scan, end scan)
REAL_WINDOWS = (('20261010T042200Z_both', 3, 80), ('20261010T051127Z_both', 132, 181),
                ('20261010T035952Z_both', 2612, 2705))


def rd_candidates():
    from row_executor import sim_paths
    base = sim_paths.optional_path('field_manual_1010')
    return (os.environ.get('UBTS_RD_1010_LIDAR'), None if base is None else os.path.join(str(base), 'lidar'))


def rd_dir():
    for path in rd_candidates():
        if path and os.path.isfile(os.path.join(path, REAL_WINDOWS[0][0] + '.scans.npz')):
            return path
    return None


def idx(bearing_deg):
    return L.body_bearing_to_raw_index(math.radians(bearing_deg))


BRACKET = (100., .30, 6)          # inside the footprint (left side, near the LiDAR)
FRONT_PART = (0., .40, 3)         # inside the footprint ahead of the LiDAR (front edge 0.43 m)
PERSON = (-90., .45, 30)          # just outside the right side (limit 0.35 + 0.05 m)
STRAP = (45., .30, 4)             # inside, every other scan


def scan_set(n=60, seed=1, parts=(BRACKET, FRONT_PART, PERSON), strap=True, crossing=False, shift=None, noise=.006):
    rng = random.Random(seed)
    base = L.raw_scan(L.orchard_trunks(), self_returns=[(math.radians(b), r, h) for b, r, h in parts])
    scans = []
    for i in range(n):
        r = base.copy()
        if strap and i % 2:
            c = idx(STRAP[0])
            r[c - STRAP[2]:c + STRAP[2] + 1] = STRAP[1]
        if crossing:                                      # one object across the limit near bearing -120 deg
            c = idx(-120.)
            r[c - 10:c] = .38
            r[c:c + 10] = .50
        ranges = []
        for v in r.tolist():
            if v <= 0:
                ranges.append(None)
            else:
                v += rng.uniform(-noise, noise)
                if shift is not None and i % shift == 0:
                    v += .06
                ranges.append(v)
        scans.append(dict(ranges=ranges, angle_min=L.RAW_AMIN, increment=L.RAW_INC, range_min=.05, range_max=30.,
                          t=i * .1))
    return scans


def beams_of(rows):
    out = set()
    for row in rows:
        out.update(range(row['beams'][0], row['beams'][1] + 1))
    return out


class SyntheticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = B.fit_static_mask(CFG, scan_set())

    def test_parts_masked_person_and_strap_not(self):
        r = self.result
        self.assertIsNone(r['refused'])
        masked = set()
        for seg in r['segments']:
            masked.update(range(seg['first'], seg['last'] + 1))
        for part in (BRACKET, FRONT_PART):
            c = idx(part[0])
            self.assertTrue(set(range(c - part[2], c + part[2] + 1)) <= masked, part)
        c = idx(PERSON[0])
        self.assertFalse(masked & set(range(c - PERSON[2], c + PERSON[2] + 1)))
        self.assertTrue(set(range(c - PERSON[2] + 2, c + PERSON[2] - 1)) <= beams_of(r['outside']))
        c = idx(STRAP[0])
        self.assertFalse(masked & set(range(c - STRAP[2], c + STRAP[2] + 1)))
        strap = [row for row in r['inconsistent'] if row['beams'][0] <= c <= row['beams'][1]]
        self.assertTrue(strap and 'intermittent' in strap[0]['why'])
        self.assertEqual(len(r['warnings']), 2)
        self.assertEqual(masked, set(range(idx(BRACKET[0]) - 6, idx(BRACKET[0]) + 7)) |
                         set(range(idx(FRONT_PART[0]) - 3, idx(FRONT_PART[0]) + 4)))

    def test_fragment_respects_s5_and_masks_only_the_parts(self):
        r = self.result
        boundary, forward, _ = C.body_mask_beam_limits(CFG)
        for first, last, rng in r['fragment']['body_mask']:
            for g in range(first, last + 1):
                self.assertLessEqual(rng, min(boundary[g], forward[g]) + 1e-9)
            part = BRACKET if abs(first - idx(BRACKET[0])) < 20 else FRONT_PART
            self.assertLess(rng, part[1] + .035)                  # mask = max return + 2 cm, not the full limit
        cfg = C.from_dict(dict(CFG.to_dict(), **r['fragment']))
        adapter = ScanAdapter.from_config(cfg)
        raw = np.array([v if v is not None else 0. for v in scan_set(n=1)[0]['ranges']])
        out, *_ = adapter.convert(raw, L.RAW_AMIN, L.RAW_INC, 30., .05)
        self.assertTrue(np.isinf(out[idx(BRACKET[0])]))
        self.assertTrue(np.isinf(out[idx(FRONT_PART[0])]))
        self.assertTrue(np.isfinite(out[idx(PERSON[0])]))
        e = r['effect']
        self.assertGreater(e['inside_footprint_returns_p50_before'], e['inside_footprint_returns_p50_after'])

    def test_forward_sector_object_beyond_the_front_edge_is_never_masked(self):
        r = B.fit_static_mask(CFG, scan_set(parts=((0., .55, 3),), strap=False))
        self.assertEqual(r['segments'], [])
        self.assertIn(idx(0.), beams_of(r['outside']))

    def test_object_across_the_limit_is_not_masked(self):
        r = B.fit_static_mask(CFG, scan_set(parts=(), strap=False, crossing=True))
        self.assertEqual(r['segments'], [])
        c = idx(-120.)
        whys = [row['why'] for row in r['inconsistent'] if row['beams'][0] <= c - 5 <= row['beams'][1]]
        self.assertTrue(whys and 'object_crosses_limit' in whys[0], r['inconsistent'])

    def test_refusals(self):
        self.assertIn('not_stationary', B.fit_static_mask(CFG, scan_set(shift=5))['refused'])
        self.assertIn('too_few_scans', B.fit_static_mask(CFG, scan_set(n=10))['refused'])
        scans = scan_set(n=25)
        scans[3] = dict(scans[3], ranges=scans[3]['ranges'][:-1])
        with self.assertRaises(B.InputError):
            B.fit_static_mask(CFG, scans)
        empty = [dict(s, ranges=[None] * len(s['ranges'])) for s in scan_set(n=25)]
        self.assertIn('too_few_reference_returns', B.fit_static_mask(CFG, empty)['refused'])

    def test_footprint_from_the_config(self):
        # a longer footprint (protruding rear rack) makes returns behind the old rear edge maskable
        cfg = C.ExecutorConfig(footprint_length_m=1.6)
        part = (180., .95, 5)                  # 0.95 m behind the LiDAR: outside 1.22 m, inside 1.6 m
        self.assertIn(idx(180.), beams_of(B.fit_static_mask(CFG, scan_set(parts=(part,), strap=False))['outside']))
        r = B.fit_static_mask(cfg, scan_set(parts=(part,), strap=False))
        self.assertTrue(r['segments'])
        C.from_dict(dict(cfg.to_dict(), **r['fragment']))


class SidecarCliTest(unittest.TestCase):
    def write_recording(self, tmp, scans):
        os.makedirs(os.path.join(tmp, 'rec', 'mapping_source'))
        path = os.path.join(tmp, 'rec', 'mapping_source', 'sensors.jsonl.gz')
        with gzip.open(path, 'wt', encoding='utf-8') as fh:
            for k, s in enumerate(scans):
                t = 1760000000. + k * .1
                fh.write(json.dumps(dict(topic='/imu/data_raw', message=dict(angular_velocity=dict(x=0, y=0, z=0)),
                                         bag_time_ns=int(t * 1e9))) + '\n')
                msg = dict(header=dict(stamp=dict(sec=int(t), nanosec=0), frame_id='laser'), angle_min=s['angle_min'],
                           angle_max=-s['angle_min'], angle_increment=s['increment'], range_min=.05, range_max=30.,
                           scan_time=.1, time_increment=0., ranges=s['ranges'])
                fh.write(json.dumps(dict(topic='/scan', message=msg, bag_time_ns=int((t + .05) * 1e9))) + '\n')
        return os.path.join(tmp, 'rec')

    def test_cli_on_a_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = self.write_recording(tmp, scan_set(n=40))
            cfg_path = os.path.join(tmp, 'config.json')
            with open(cfg_path, 'w', encoding='utf-8', newline='\n') as fh:
                fh.write(C.dumps(C.ExecutorConfig()))
            frag, merged, report = (os.path.join(tmp, n) for n in ('frag.json', 'merged.json', 'report.json'))
            args = ['--recording', rec, '--from', '0.5', '--to', '3.5', '--config', cfg_path, '--out-fragment',
                    frag, '--out-config', merged, '--report', report]
            # day-2 N5: a mask in the forward sector is written only after the parts were matched (exit 4, nothing)
            self.assertEqual(B.main(args), 4)
            self.assertFalse(os.path.exists(frag) or os.path.exists(merged))
            rc = B.main(args + ['--confirm-forward-sector'])
            self.assertEqual(rc, 0)
            with open(report, encoding='utf-8') as fh:
                self.assertEqual(json.load(fh)['scans_in'], 31)       # 0.5 .. 3.5 s inclusive
            with open(frag, encoding='utf-8') as fh:
                body_mask = json.load(fh)['body_mask']
            self.assertTrue(body_mask)
            cfg = C.load_config(merged)
            self.assertEqual([list(s) for s in cfg.body_mask], body_mask)
            scans = B.read_sidecar(B.sidecar_path(rec))
            self.assertEqual(len(scans), 40)
            with self.assertRaises(B.InputError):
                B.read_sidecar(B.sidecar_path(rec), max_scans=20)
            # refused input: nothing written, exit 3
            moving = self.write_recording(os.path.join(tmp, 'm'), scan_set(n=40, shift=4))
            rc = B.main(['--recording', moving, '--out-fragment', os.path.join(tmp, 'f2.json')])
            self.assertEqual(rc, 3)
            self.assertFalse(os.path.exists(os.path.join(tmp, 'f2.json')))
            self.assertEqual(B.main(['--recording', os.path.join(tmp, 'nothing')]), 2)


@unittest.skipUnless(rd_dir(), '10/10 compact extracts not found (set UBTS_RD_1010_LIDAR or UBTS_RD_1010 / '
                               'sim_paths field_manual_1010)')
class RealStandstillTest(unittest.TestCase):
    """Behaviour on the real standstill windows (no ground truth: the near returns there were people/objects next
    to or leaning on the vehicle, never seen while moving; nothing of it may become a mask)."""

    @classmethod
    def setUpClass(cls):
        cls.results = {}
        for rid, a, b in REAL_WINDOWS:
            cls.results[rid] = B.fit_static_mask(CFG, B.read_npz(os.path.join(rd_dir(), rid + '.scans.npz'), a, b))

    def test_stationary_and_nothing_masked(self):
        for rid, r in self.results.items():
            self.assertIsNone(r['refused'], rid)
            self.assertEqual(r['scans_moving'], 0, rid)
            self.assertEqual(r['segments'], [], rid)
            self.assertEqual(r['fragment'], dict(body_mask=[]), rid)

    def test_042200_object_at_the_rear_right_corner_crosses_the_limit(self):
        r = self.results['20261010T042200Z_both']
        crossing = [row for row in r['inconsistent'] if 'object_crosses_limit' in row['why']]
        self.assertTrue(crossing)
        self.assertTrue(all(-160 < row['bearing_deg'][0] < -150 for row in crossing))
        self.assertTrue(any(-160 < row['bearing_deg'][0] < -145 for row in r['outside']))
        self.assertGreaterEqual(r['effect']['inside_footprint_returns_p50_after'], 20)   # still visible: start refused

    def test_051127_inside_the_footprint_but_intermittent(self):
        r = self.results['20261010T051127Z_both']
        rows = [row for row in r['inconsistent'] if 'intermittent' in row['why']]
        self.assertTrue(rows)
        self.assertLess(min(row['range_m_p5_p95'][0] for row in rows), .3)
        self.assertTrue(all(row['fraction_in_max'] < B.RULE['mask_fraction'] for row in rows))

    def test_035952_rare(self):
        r = self.results['20261010T035952Z_both']
        self.assertTrue(r['inconsistent'])
        self.assertTrue(all(row['fraction_in_max'] < .2 for row in r['inconsistent']))


if __name__ == '__main__':
    unittest.main()
