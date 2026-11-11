"""steering.s2_format: the real RPLIDAR S2 LaserScan format and RealFormat (moved unchanged from the row executor's
sim_in_loop.py at 546fa8c; design 3.7, A5, 8.3 P4).

Format facts: 3240 beams, angle_min float32(-pi), increment about 0.1111 deg, frame 'laser', 10 Hz, range 0.05-30 m,
lidar_yaw_offset_deg convention (raw beam j looks at the body angle angle_min + j * increment + offset), raw order,
0 = no return, and the NaN / inf / >= 30 m / rounding behaviour exactly as RealFormat had it (also its edges: a value
just below 30 m can round up to 30.0, negative values pass, there is no range_min clip).
unittest style: runs under pytest (Windows 3.13, image 3.12) and plain unittest (WSL 3.10, no pytest).
"""
import ast
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True

from steering import s2_format as F  # noqa: E402

REPO = HERE.parents[1]
SIM_IN_LOOP = REPO / 'tools' / 'row_executor' / 'sim_in_loop.py'
EXECUTOR_CONFIG = REPO / 'tools' / 'row_executor' / 'config.py'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def body_angle(j, offset_deg):
    return wrap(F.RAW_AMIN + j * F.RAW_INC + math.radians(offset_deg))


def sim_angle(k):
    return -math.pi + k * 2 * math.pi / F.SIM_BEAMS


def ramp():
    """SIM ranges that name their own beam: beam k -> 1 + k / 1000 m (1.000 .. 4.199 m, exact after the 1 mm round)."""
    return 1. + np.arange(F.SIM_BEAMS) / 1000.


def beam_of(value):
    return int(round(float(value) * 1000.)) - 1000


class FormatFacts(unittest.TestCase):
    def test_real_s2_constants(self):
        self.assertEqual(F.BEAMS, 3240)
        self.assertEqual(F.SIM_BEAMS, 3200)
        self.assertEqual(F.RAW_AMIN, float(np.float32(-math.pi)))
        self.assertEqual(F.RAW_INC, float(np.float32(F.RAW_INC)))              # a float32 value, as the driver sends
        self.assertAlmostEqual(math.degrees(F.RAW_INC), .1111454, places=6)
        self.assertGreater(F.BEAMS * F.RAW_INC, 2 * math.pi)                     # 3240 beams span about 360.11 deg
        self.assertLess(F.BEAMS * F.RAW_INC, 2 * math.pi + 1.01 * F.RAW_INC)    # ... and are not trimmed by ScanAdapter
        self.assertEqual(F.FRAME_ID, 'laser')
        self.assertEqual(F.SCAN_HZ, 10.)
        self.assertEqual((F.RANGE_MIN_M, F.RANGE_MAX_M), (.05, 30.))
        self.assertEqual(F.DEFAULT_YAW_OFFSET_DEG, 179.08)

    @unittest.skipUnless(EXECUTOR_CONFIG.is_file(), 'tools/row_executor not in this tree')
    def test_default_offset_is_the_executor_default(self):
        tree = ast.parse(EXECUTOR_CONFIG.read_bytes())
        values = [n.value.value for n in ast.walk(tree) if isinstance(n, ast.AnnAssign)
                  and getattr(n.target, 'id', None) == 'lidar_yaw_offset_deg']
        self.assertEqual(values, [F.DEFAULT_YAW_OFFSET_DEG])

    def test_default_constructor_offset(self):
        np.testing.assert_array_equal(F.RealFormat().index, F.RealFormat(F.DEFAULT_YAW_OFFSET_DEG).index)


class Conversion(unittest.TestCase):
    def test_shape_dtype_and_constant_scan(self):
        out = F.RealFormat().convert(np.full(F.SIM_BEAMS, 5.))
        self.assertEqual(out.shape, (F.BEAMS,))
        self.assertEqual(out.dtype, np.float32)
        self.assertTrue(np.all(out == np.float32(5.)))

    def test_every_real_beam_takes_the_nearest_sim_beam_of_its_body_angle(self):
        for offset in (179.08, 0., 90., -90., -179.9, 33.3):
            fmt = F.RealFormat(offset)
            out = fmt.convert(ramp())
            bin_half = math.pi / F.SIM_BEAMS
            for j in range(0, F.BEAMS, 7):
                k = beam_of(out[j])
                self.assertLessEqual(abs(wrap(body_angle(j, offset) - sim_angle(k))), bin_half + 1e-9, (offset, j))

    def test_raw_order_and_yaw_offset_sign(self):
        # body straight ahead (SIM beam 1600, angle 0) lands at raw angle -offset: the executor adds the offset back
        # (core.ScanAdapter: angle_min + lidar_yaw_offset_deg), the array is never reordered
        for offset, expected_j in ((179.08, 8), (0., 1619), (90., 810), (-90., 2429)):
            ranges = np.full(F.SIM_BEAMS, np.inf)
            ranges[1600] = 2.
            out = F.RealFormat(offset).convert(ranges)
            hits = np.nonzero(out)[0]
            self.assertIn(expected_j, hits, offset)
            self.assertTrue(all(abs(body_angle(j, offset)) <= math.pi / F.SIM_BEAMS + 1e-9 for j in hits))
            self.assertTrue(np.all(out[hits] == np.float32(2.)))
            raw = F.RAW_AMIN + expected_j * F.RAW_INC
            self.assertAlmostEqual(wrap(raw + math.radians(offset)), 0., delta=F.RAW_INC)

    def test_every_sim_beam_is_used_once_or_twice(self):
        counts = np.bincount(F.RealFormat().index, minlength=F.SIM_BEAMS)
        self.assertEqual(counts.shape, (F.SIM_BEAMS,))
        self.assertEqual((int(counts.min()), int(counts.max()), int(counts.sum())), (1, 2, F.BEAMS))

    def test_no_return_is_zero(self):
        fmt = F.RealFormat()
        for value in (np.inf, -np.inf, np.nan, 30., 30.0001, 45., 1e9):
            out = fmt.convert(np.full(F.SIM_BEAMS, value))
            self.assertTrue(np.all(out == 0.), value)

    def test_rounding_exactly_as_realformat(self):
        fmt = F.RealFormat()
        cases = ((1.0625, 1.062), (1.1875, 1.188),                  # numpy round half to even at exact halves
                 (29.9994, 29.999), (29.9996, 30.),                # 'r < 30' is tested BEFORE the 1 mm round
                 (.0004, 0.), (.0006, .001), (0., 0.),             # no range_min clip
                 (-1.234, -1.234), (2.34567, 2.346))               # negative finite values pass unchanged
        for value, expected in cases:
            out = fmt.convert(np.full(F.SIM_BEAMS, value))
            self.assertTrue(np.all(out == np.float32(expected)), (value, out[0], expected))

    def test_outputs_are_whole_millimetres(self):
        rng = np.random.default_rng(7)
        r = rng.uniform(0., 31., F.SIM_BEAMS)
        out = F.RealFormat().convert(r).astype(np.float64)
        mm = out * 1000.
        self.assertLess(float(np.max(np.abs(mm - np.round(mm)))), 2e-3)
        self.assertTrue(np.all((out == 0.) | ((out > 0.) & (out <= 30.))))

    def test_input_types_and_no_mutation(self):
        fmt = F.RealFormat()
        base = ramp()
        expected = fmt.convert(base)
        for value in (list(base), tuple(base), base.astype(np.float32).astype(np.float64)):
            np.testing.assert_array_equal(fmt.convert(value), expected)
        ints = np.full(F.SIM_BEAMS, 3, dtype=np.int64)
        self.assertTrue(np.all(fmt.convert(ints) == np.float32(3.)))
        source = ramp()
        fmt.convert(source)
        np.testing.assert_array_equal(source, ramp())

    def test_beam_count_behaviour(self):
        fmt = F.RealFormat()
        with self.assertRaises(IndexError):
            fmt.convert(np.ones(F.SIM_BEAMS - 1))                  # fewer than 3200 SIM beams
        longer = np.concatenate([ramp(), np.full(100, 7.)])
        np.testing.assert_array_equal(fmt.convert(longer), fmt.convert(ramp()))   # extra beams are not used


class ModuleBoundary(unittest.TestCase):
    def test_imports_only_numpy_and_the_standard_library(self):
        tree = ast.parse((HERE / 'steering' / 's2_format.py').read_bytes(), feature_version=(3, 10))
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names.update(a.name.split('.')[0] for a in node.names)
            elif isinstance(node, ast.ImportFrom):
                names.add((node.module or '').split('.')[0])
        self.assertEqual(names, {'math', 'numpy'})

    def test_fresh_import_loads_no_harness_estimator_or_scipy(self):
        code = ('import json, sys; sys.dont_write_bytecode = True; sys.path.insert(0, sys.argv[1]); '
                'import steering.s2_format; '
                'print(json.dumps(sorted(n for n in sys.modules if n.split(".")[0] in '
                '("scipy", "closed_loop", "field_response", "cpu_lidar", "row_executor"))))')
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
        result = subprocess.run([sys.executable, '-B', '-c', code, str(HERE)], capture_output=True, text=True,
                                env=env, timeout=120)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout.strip().splitlines()[-1]), [])

    @unittest.skipUnless(SIM_IN_LOOP.is_file(), 'tools/row_executor not in this tree')
    def test_sim_in_loop_uses_this_module(self):
        tree = ast.parse(SIM_IN_LOOP.read_bytes())
        imported = [(n.module, sorted(a.name for a in n.names)) for n in ast.walk(tree)
                    if isinstance(n, ast.ImportFrom) and n.module == 'steering.s2_format']
        self.assertEqual(imported, [('steering.s2_format', ['BEAMS', 'RAW_AMIN', 'RAW_INC', 'RealFormat', 'SIM_BEAMS'])])
        defined = [n.name for n in tree.body if isinstance(n, ast.ClassDef)]
        self.assertNotIn('RealFormat', defined)


if __name__ == '__main__':
    unittest.main()
