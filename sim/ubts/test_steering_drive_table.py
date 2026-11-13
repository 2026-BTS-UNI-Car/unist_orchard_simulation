"""The plant drive table of the steered plant and the T1 rig (PC-11 / PW-08; steered_plant.drive_table_spec,
RigConfig.drive_table): spec checks, the table interpolation (the executor's SpeedTable rule), default off, the plant's
speed target reading the table, the rig config's canonical form. The plant runs with the L0 servo backend (no native
build). Every number is a test value (자리값 기반 SIM 결과).

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_drive_table.py
"""
import json
import unittest

from steering import steered_plant as sp
from steering import t1_rig as tr


class DriveTableTest(unittest.TestCase):
    TABLE = ((25., .239), (80., .709), (90., .715), (100., .899))

    def test_spec_checks(self):
        for bad in ((), ((10., .1),), ((50., .5), (40., .6)), ((10., .5), (20., .4)), ((0., .1), (10., .2)),
                    ((10., .0), (20., .0))):
            with self.assertRaises(ValueError):
                sp.drive_table_spec(bad)
        for kw in (dict(factor=.1), dict(factor=3.), dict(duty_scale=.2), dict(duty_offset_pct=-1.),
                   dict(duty_offset_pct=60.), dict(factor=True)):
            with self.assertRaises(ValueError):
                sp.drive_table_spec(self.TABLE, **kw)

    def test_interpolation(self):
        spec = sp.drive_table_spec(self.TABLE)
        self.assertAlmostEqual(sp.drive_table_speed(spec, 25.), .239)
        self.assertAlmostEqual(sp.drive_table_speed(spec, 12.5), .1195)            # through (0, 0)
        self.assertAlmostEqual(sp.drive_table_speed(spec, 52.5), .474)
        self.assertAlmostEqual(sp.drive_table_speed(spec, 100.), .899)
        self.assertEqual(sp.drive_table_speed(spec, 0.), 0.)
        scaled = sp.drive_table_spec(self.TABLE, factor=.7, duty_scale=.85, duty_offset_pct=15.)
        self.assertAlmostEqual(sp.drive_table_speed(scaled, 50.), .7 * sp.drive_table_speed(spec, 50. * .85 - 15.))
        self.assertEqual(sp.drive_table_speed(scaled, 15.), 0.)

    def test_plant_default_off_and_table_on(self):
        from steering import vehicle_profile as vp
        l0 = vp.load_profile().with_overrides({'steer_actuator_level': 'L0'}, label='l0_backend')
        params = sp.SteeredPlantParams.from_profile(l0)
        self.assertIsNone(params.drive_table)
        plant = sp.SteeredPlant(params, seed=3)
        curve = plant._speed_target(50., 0.)
        from dataclasses import replace
        table = replace(params, drive_table=sp.drive_table_spec(self.TABLE, factor=1.3))
        plant2 = sp.SteeredPlant(table, seed=3)
        want = 1.3 * sp.drive_table_speed(sp.drive_table_spec(self.TABLE), 50.) * (1. + plant2.speed_fluctuation)
        self.assertAlmostEqual(plant2._speed_target(50., 0.), want)
        self.assertNotAlmostEqual(curve, plant2._speed_target(50., 0.))

    def test_rig_config_canonical(self):
        cfg = tr.RigConfig(drive_table=dict(points=[list(p) for p in self.TABLE], factor=.85, source='x'))
        self.assertEqual(json.loads(cfg.drive_table)['factor'], .85)
        self.assertIsNone(tr.RigConfig().drive_table)
        with self.assertRaises(tr.RigError):
            tr.RigConfig(drive_table=dict(points=[[10., .5]]))
        with self.assertRaises(tr.RigError):
            tr.RigConfig(drive_table=dict(points=[list(p) for p in self.TABLE], speed=1.))


if __name__ == '__main__':
    unittest.main()
