"""Runner with the perception layers (perception_layers.py): the REAL vendored estimator on synthetic orchard scans.

The scene moves past the LiDAR (the vehicle drives forward along the row at 0.4 m/s, scans only); the layers must leave
the Runner unchanged while OFF, change nothing in the estimator with fusion gain 0, fuse with a gain, never let a
'log_only' GNSS change a decision, and end a row run on a confirmed geofence exit only in mode 'geofence'.
"""
from dataclasses import replace
import math
import types
import unittest

import numpy as np

import rowexec_testlib as L
from row_executor import config as C
from row_executor.config import ExecutorConfig
from row_executor.core import ENDED, Runner

TRUNKS = L.orchard_trunks(row_y=1.8, spacing=1.0, x0=-10., x1=60., radius=.1)
V = .4


def moving_scene(t0, lateral=.25):
    """Scans of the vehicle driving along the row (x = V * (t - t0), offset `lateral` left of the centre line)."""
    def scan(t):
        dx = V * max(0., t - t0 - 1.)
        return L.raw_scan([(x - dx, y - lateral, r) for x, y, r in TRUNKS])
    return scan


def make(cfg, t0=1000.):
    runner = Runner(cfg)
    driver = L.FakeTimeDriver(runner, moving_scene(t0), t0=t0)
    runner.start_run(driver.t, 60.)
    return runner, driver


def gnss_msg(east, north, hacc=7., ok=True):
    return types.SimpleNamespace(east_m=east, north_m=north, hacc_m=hacc, fix_ok=ok)


def strip(summary):
    return {k: v for k, v in summary.items() if k not in ('local_grid', 'gnss')}


class OffTest(unittest.TestCase):
    def test_defaults_build_no_layer_and_keep_the_digest_fields(self):
        cfg = ExecutorConfig()
        self.assertIsNone(Runner(cfg).perception)
        keys = cfg.to_dict()
        self.assertFalse([k for k in keys if k.startswith(('local_grid_', 'gnss_'))])
        self.assertNotIn('external_corridor_gain', cfg.estimator_overrides())
        on = replace(cfg, local_grid_enabled=True).validate()
        self.assertIn('local_grid_enabled', on.to_dict())
        self.assertNotEqual(on.digest(), cfg.digest())

    def test_config_rules(self):
        cfg = ExecutorConfig()
        for bad in (dict(local_grid_fusion_gain=.2), dict(local_grid_publish=True), dict(gnss_mode='weak'),
                    dict(local_grid_enabled=True, local_grid_cell_m=.6), dict(gnss_mode='log_only', gnss_max_hacc_m=0.)):
            with self.subTest(bad=bad):
                with self.assertRaises(C.ConfigError):
                    replace(cfg, **bad).validate()
        ok = replace(cfg, local_grid_enabled=True, local_grid_fusion_gain=.2, gnss_mode='geofence').validate()
        overrides = ok.estimator_overrides()
        self.assertEqual(overrides['external_corridor_gain'], .2)
        self.assertEqual(overrides['external_corridor_sources'], ('local_grid',))


class GridRunnerTest(unittest.TestCase):
    def run_pair(self, cfg_on, seconds=8.):
        off, d_off = make(ExecutorConfig())
        on, d_on = make(cfg_on)
        a = d_off.run(seconds, hold=False)
        b = d_on.run(seconds, hold=False)
        return off, on, a, b

    def test_gain_zero_leaves_the_estimator_unchanged(self):
        cfg = replace(ExecutorConfig(), local_grid_enabled=True).validate()
        off, on, a, b = self.run_pair(cfg)
        self.assertEqual([strip(d.summary) for _, d in a], [strip(d.summary) for _, d in b])
        self.assertEqual(off.estimator.snapshot(1007.9), on.estimator.snapshot(1007.9))
        block = b[-1][1].summary['local_grid']
        self.assertGreater(block['updates'], 10)
        self.assertIn('fit', block)
        self.assertAlmostEqual(block['fit']['lateral_m'], b[-1][1].summary['lateral_offset_m'], delta=.1)
        self.assertNotIn('fusion', block)

    def test_fusion_gain_pulls_the_band(self):
        cfg = replace(ExecutorConfig(), local_grid_enabled=True, local_grid_fusion_gain=.2).validate()
        off, on, a, b = self.run_pair(cfg)
        outcomes = b[-1][1].summary['local_grid']['fusion']['outcomes']
        self.assertGreater(outcomes.get('fused', 0), 5)
        snap = on.estimator.snapshot(1007.9)
        self.assertEqual(snap['external_corridor']['source'], 'local_grid')
        self.assertEqual(snap['external_corridor']['fused'], outcomes['fused'])
        self.assertTrue(snap['corridor_valid'])
        self.assertAlmostEqual(snap['lateral_offset'], off.estimator.snapshot(1007.9)['lateral_offset'], delta=.1)

    def test_export_is_rate_limited_and_bounded(self):
        cfg = replace(ExecutorConfig(), local_grid_enabled=True, local_grid_publish=True,
                      local_grid_publish_period_s=1.).validate()
        on, d = make(cfg)
        d.run(4., hold=False)
        first = on.local_grid_export(d.t)
        self.assertIs(on.local_grid_export(d.t + .5), first)
        d.run(1.1, hold=False)
        second = on.local_grid_export(d.t)
        self.assertIsNot(second, first)
        self.assertLessEqual(len(second['bits']), cfg.local_grid_publish_max_bytes)
        self.assertIsNone(Runner(ExecutorConfig()).local_grid_export(0.))


class GnssRunnerTest(unittest.TestCase):
    def feed(self, runner, driver, seconds, where):
        out = []
        for _ in range(int(seconds)):
            out += driver.run(1., hold=False)
            east, north = where(driver.t)
            runner.feed_gnss(driver.t, gnss_msg(east, north))
        return out

    def test_log_only_changes_no_decision(self):
        plain, d0 = make(ExecutorConfig())
        cfg = replace(ExecutorConfig(), gnss_mode='log_only').validate()
        logged, d1 = make(cfg)
        a = d0.run(10., hold=False)
        b = self.feed(logged, d1, 10, lambda t: (V * (t - 1001.) + 3., 2.))
        self.assertEqual([(d.left, d.right, d.state, d.reason) for _, d in a],
                         [(d.left, d.right, d.state, d.reason) for _, d in b])
        self.assertEqual(plain.estimator.snapshot(1009.9), logged.estimator.snapshot(1009.9))
        block = b[-1][1].summary['gnss']
        self.assertEqual(block['mode'], 'log_only')
        self.assertGreaterEqual(block['counts']['accepted'], 9)

    def test_geofence_ends_a_row_run(self):
        cfg = replace(ExecutorConfig(), gnss_mode='geofence').validate()
        runner, d = make(cfg)
        runner.perception.gnss.new_run(5.)                 # a 5 m row (radial fence before the alignment)
        out = self.feed(runner, d, 12, lambda t: (0., 0.) if t < 1004. else (40. + 2. * (t - 1004.), 0.))
        ends = [dd for _, dd in out if dd.state == ENDED]
        self.assertTrue(ends)
        self.assertEqual(ends[0].reason, 'gnss_geofence')
        self.assertEqual(runner.end_reason, 'gnss_geofence')

    def test_log_only_never_ends(self):
        cfg = replace(ExecutorConfig(), gnss_mode='log_only').validate()
        runner, d = make(cfg)
        runner.perception.gnss.new_run(5.)
        out = self.feed(runner, d, 12, lambda t: (0., 0.) if t < 1004. else (40. + 2. * (t - 1004.), 0.))
        self.assertFalse([dd for _, dd in out if dd.state == ENDED])
        self.assertTrue(runner.perception.gnss.fence_tripped)

    def test_no_run_no_input(self):
        cfg = replace(ExecutorConfig(), gnss_mode='log_only').validate()
        runner = Runner(cfg)
        self.assertIsNone(runner.feed_gnss(1., gnss_msg(0., 0.)))
        self.assertIsNone(Runner(ExecutorConfig()).feed_gnss(1., gnss_msg(0., 0.)))


if __name__ == '__main__':
    unittest.main()
