"""Actuator priors in the executor (design v2 V2-1 item 2, docs/steering/ACTUATOR_PRIOR_KO.md): the config block
(untouched = same digest), the Runner feeding the steering status to the vendored estimator, weight 0 = the same
decisions, and the prediction bound (PAUSED 'prediction_limit', no resume while it lasts).

Synthetic inputs only (a static orchard scan or featureless scans, fake steering status replies). No hardware, no ROS.
Every number is a test value, not a vehicle value.
"""
import math
import unittest
from dataclasses import replace

import numpy as np

import rowexec_testlib as L
from row_executor import config as C
from row_executor import core
from row_executor.config import ExecutorConfig
from row_executor.core import ACTIVE, ENDED, PAUSED, Runner
from test_steer_executor import DIGEST_DEFAULT_546FA8C, MOTOR, SCIPY_PROBLEM, StubSteer, block

SCENE = L.raw_scan(L.orchard_trunks())
BLIND = np.zeros(L.BEAMS)                        # no return on any beam: every scan match fails
TABLE = ((20., .2), (40., .4), (60., .6))        # test table: 1 % duty = 0.01 m/s
PRIOR = dict(prior_enabled=True, prior_speed_table=TABLE, prior_wheelbase_m=.8)


def motor_config(**over):
    base = dict(MOTOR)
    base.update(over)
    return ExecutorConfig(**base).validate()


def problems(**over):
    base = dict(MOTOR)
    base.update(over)
    try:
        ExecutorConfig(**base).validate()
    except C.ConfigError as exc:
        return exc.problems
    return []


def host(drive=102, **over):
    out = dict(arm_state='confirmed', stop_state='none', stale=False, latches={}, drive_l=drive, drive_r=drive)
    out.update(over)
    return out


class Drive:
    """A front_steer Runner on scripted scans with a steering status before every tick (10 ms old)."""

    def __init__(self, cfg, scan_fn=None, drive=102, measured=20, controller=None):
        self.controller = controller or StubSteer(throttle=40, step=0)
        self.runner = Runner(cfg, controller_factory=lambda: self.controller)
        self.runner.steer_required = True
        self.scan_fn = scan_fn or (lambda t: SCENE)
        self.driver = L.FakeTimeDriver(self.runner, lambda t: self.scan_fn(t - self.t0), t0=2000.)
        self.t0 = 2000.
        self.drive, self.measured = drive, measured
        self.status = True
        original = self.runner.tick

        def tick(now, hold_active):
            if self.status:
                self.runner.feed_steering(now, block(measured=self.measured, age_ms=10), host(self.drive))
            return original(now, hold_active)
        self.runner.tick = tick
        self.runner.start_run(self.driver.t, None)

    def run(self, seconds, hold=True):
        return self.driver.run(seconds, hold=hold)


class ConfigBlockTest(unittest.TestCase):
    def test_untouched_block_keeps_the_digests(self):
        cfg = ExecutorConfig()
        self.assertEqual(cfg.digest(), DIGEST_DEFAULT_546FA8C)
        self.assertFalse(set(C.PRIOR_KEYS) & set(cfg.to_dict()))
        self.assertIsNone(cfg.actuator_prior_settings())
        single = ExecutorConfig(mode='single_row', require_native_navigation=False, max_yaw_rate_deg_s=3.6)
        self.assertFalse(set(C.PRIOR_KEYS) & set(single.validate().to_dict()))
        steer = motor_config()
        self.assertFalse(set(C.PRIOR_KEYS) & set(steer.to_dict()))
        self.assertEqual(C.PRIOR_KEYS, ('prior_enabled', 'prior_weight', 'prior_speed_table', 'prior_speed_scale',
                                        'prior_wheelbase_m', 'prior_predict_max_s', 'prior_predict_max_m',
                                        'prior_mismatch_fraction', 'corridor_quality'))

    def test_touched_block_is_dumped_and_round_trips(self):
        cfg = motor_config(prior_weight=.5, **PRIOR)
        data = cfg.to_dict()
        self.assertTrue(set(C.PRIOR_KEYS) <= set(data))
        self.assertEqual(data['prior_speed_table'], [[20., .2], [40., .4], [60., .6]])
        self.assertNotEqual(cfg.digest(), motor_config().digest())
        again = C.from_dict(data)
        self.assertEqual(again, cfg)
        self.assertEqual(again.prior_speed_table, TABLE)
        self.assertTrue(set(C.PRIOR_KEYS) <= set(motor_config(corridor_quality=True).to_dict()))

    def test_settings_of_the_vendored_prior(self):
        cfg = motor_config(prior_weight=.5, prior_speed_scale=1.1, prior_mismatch_fraction=.25, **PRIOR)
        settings = cfg.actuator_prior_settings()
        self.assertEqual(settings, dict(speed_scale=1.1, wheelbase_m=.8, status_max_age_s=cfg.steer_status_max_age_s,
                                        weight=.5, predict_max_s=1., predict_max_m=.5, mismatch_fraction=.25,
                                        speed_table=TABLE))
        self.assertNotIn('speed_table', motor_config(prior_enabled=True).actuator_prior_settings())   # the placeholder

    def test_rules(self):
        self.assertEqual(problems(**PRIOR), [])
        self.assertTrue(any('prior_weight' in p and 'prior_enabled' in p
                            for p in problems(prior_weight=.5, prior_enabled=False)))
        self.assertEqual(problems(prior_weight=.5), [])          # PW-10: front_steer default = the prior is active
        differential = dict(vehicle_profile='differential', steer_allow_sim=False, steer_cal_id=None,
                            socket_path='/run/ubts-rover/control.sock')
        self.assertTrue(any('front_steer' in p for p in problems(prior_enabled=True, **differential)))
        self.assertEqual([p for p in problems(**differential) if 'prior' in p], [])   # None, differential: off
        self.assertTrue(any('prior_weight' in p for p in problems(prior_weight=.5, **differential)))

    def test_profile_default_of_the_prior(self):
        """PW-10: prior_enabled None (the default) = on for front_steer (weight 0: record and online-scale monitor),
        off for the differential profile; true / false are explicit. The untouched block keeps the digests."""
        self.assertIsNone(ExecutorConfig().prior_enabled)
        self.assertFalse(C.prior_active(ExecutorConfig()))                       # the differential default
        self.assertTrue(C.prior_active(motor_config()))
        self.assertFalse(C.prior_active(motor_config(prior_enabled=False)))
        self.assertEqual(motor_config().actuator_prior_settings()['weight'], 0.)
        self.assertIsNone(motor_config(prior_enabled=False).actuator_prior_settings())
        self.assertIsNone(ExecutorConfig().actuator_prior_settings())
        self.assertEqual(motor_config(prior_enabled=False).to_dict()['prior_enabled'], False)
        for bad in (dict(prior_weight=1.5), dict(prior_speed_scale=1.4), dict(prior_speed_scale=.4),
                    dict(prior_wheelbase_m=0.), dict(prior_predict_max_s=5.), dict(prior_predict_max_m=0.),
                    dict(prior_mismatch_fraction=1.), dict(prior_enabled=1), dict(corridor_quality='yes'),
                    dict(prior_speed_table=((40., .4),)), dict(prior_speed_table=((40., .4), (30., .5))),
                    dict(prior_speed_table=((30., .4), (40., .3))), dict(prior_speed_table=((30., .3), (120., .9))),
                    dict(prior_speed_table=[(30., .3), (40., .4)]), dict(prior_speed_table=((30., .3, 1.), (40., .4))),
                    dict(prior_speed_table=((30., 0.), (40., 0.)))):
            self.assertTrue(problems(**dict(PRIOR, **bad)), bad)


@unittest.skipIf(SCIPY_PROBLEM, SCIPY_PROBLEM)
class RunnerPriorTest(unittest.TestCase):
    def test_vendored_prior_module_is_verified(self):
        estimator, _ = core.load_vendor()
        module = core._prior_module(estimator)
        self.assertTrue(module.__name__.endswith('vendor.closed_loop_copy.actuator_prior'))
        self.assertIn(core.VENDOR_SOURCES['actuator_prior'], {e['source_path'] for e in core.vendor_manifest()['files']})

    def test_default_front_steer_runner_monitors_with_weight_zero(self):
        """PW-10: the default front_steer Runner feeds the applied duty to a weight-0 prior (record and online-scale
        monitor only; test_weight_zero_gives_the_same_decisions shows the decisions do not change)."""
        d = Drive(motor_config())
        out = d.run(3.)
        prior = d.runner.estimator_config.actuator_prior
        self.assertIsNotNone(prior)
        self.assertEqual(prior.weight, 0.)
        self.assertGreater(d.runner.estimator.prior.accepted, 0)
        block = out[-1][1].summary['actuator_prior']
        self.assertEqual((block['weight'], block['limit'], block['predicting']), (0., False, False))
        self.assertIn('online_scale_hold', block)

    def test_prior_off_runner_feeds_nothing(self):
        d = Drive(motor_config(prior_enabled=False))
        out = d.run(3.)
        self.assertIsNone(d.runner.estimator_config.actuator_prior)
        self.assertFalse(d.runner.estimator_config.corridor_quality)
        self.assertIsNone(d.runner.estimator.prior)
        self.assertNotIn('actuator_prior', d.runner.last_snapshot)
        self.assertNotIn('actuator_prior', out[-1][1].summary)
        self.assertFalse([k for k in d.runner.stats if k.startswith('prior')])

    def test_status_reaches_the_estimator_as_applied_duty_and_angle(self):
        d = Drive(motor_config(**PRIOR), drive=102, measured=50, controller=StubSteer(throttle=40, start=50, step=0))
        d.run(1.5, hold=False)
        out = d.run(3.)
        prior = d.runner.estimator.prior
        self.assertGreater(prior.accepted, 50)
        stamp, speed, yaw, duty, steer = prior.samples[-1]
        self.assertAlmostEqual(duty, 40., places=9)                    # 102 / 255 of full scale
        self.assertAlmostEqual(speed, .4, places=9)
        self.assertAlmostEqual(steer, .05, places=12)                   # 50 mrad
        self.assertAlmostEqual(yaw, .4 * math.tan(.05) / .8, places=9)
        self.assertAlmostEqual(stamp, d.driver.t - .010, places=9)      # receipt - age_ms
        summary = out[-1][1].summary['actuator_prior']
        self.assertTrue(summary['fresh'])
        self.assertEqual((summary['weight'], summary['limit'], summary['predicting']), (0., False, False))
        self.assertEqual(d.runner.stats['prior_fed'], prior.accepted)
        self.assertEqual(out[-1][1].state, ACTIVE)

    def test_status_without_usable_values_is_not_fed(self):
        d = Drive(motor_config(**PRIOR))
        d.status = False
        runner = d.runner
        d.run(.5, hold=False)
        for bad_block, bad_host in ((block(age_ms=None), host()), (block(), host(drive_l=None)),
                                    (block(), host(stale=True)), (block(), host(drive_l=300)),
                                    (block(), host(drive_r=40.))):
            runner.feed_steering(d.driver.t, bad_block, bad_host)
        self.assertEqual(runner.estimator.prior.accepted, 0)
        self.assertEqual(runner.stats['prior_not_fed'], 5)
        runner.feed_steering(d.driver.t, block(calibrated=False), host(drive=-51))    # reverse duty, no angle
        sample = runner.estimator.prior.samples[-1]
        self.assertAlmostEqual(sample[3], -20., places=9)
        self.assertAlmostEqual(sample[1], -.2, places=9)
        self.assertIsNone(sample[4])
        self.assertIsNone(sample[2])
        self.assertEqual(runner.stats['prior_fed'], 1)

    def test_weight_zero_gives_the_same_decisions(self):
        def run(cfg):
            d = Drive(cfg, controller=StubSteer(throttle=40, step=2))
            out = d.run(1.5, hold=False) + d.run(4.)
            return [(round(t, 3), dec.state, dec.reason, dec.throttle, dec.steer_mrad) for t, dec in out], d
        base, _ = run(motor_config())
        monitor, d = run(motor_config(corridor_quality=True, **PRIOR))
        self.assertEqual(base, monitor)
        self.assertTrue(any(state == ACTIVE and throttle == 40 for _, state, _, throttle, _ in base))
        self.assertIn('corridor_quality', d.runner.last_snapshot)
        self.assertGreater(d.runner.estimator.prior.accepted, 0)

    def test_failed_matches_pause_at_the_prediction_bound(self):
        cfg = motor_config(prior_weight=1., **PRIOR)

        def scans(t):
            return BLIND if 6. <= t < 8.5 else SCENE         # 2.5 s of featureless scans (fresh, but no match)
        d = Drive(cfg, scan_fn=scans)
        d.run(1.5, hold=False)
        out = d.run(4.3)                                      # up to 5.8 s: driving
        self.assertEqual(out[-1][1].state, ACTIVE)
        out = d.run(2.)                                       # 5.8 .. 7.8 s
        t_pause, dec = L.first(out, lambda x: x.state == PAUSED)
        self.assertEqual(dec.reason, 'prediction_limit')
        # the bound counts from the stamp of the last scan-matched pose (receipt 5.913 s - scan_latency_s 0.08 s)
        self.assertAlmostEqual(t_pause - d.t0, 5.913 - .08 + 1., delta=.06)
        self.assertTrue(all(x.summary['actuator_prior']['predict_s'] <= 1. + 1e-9
                            for t, x in out if x.state == ACTIVE))    # never driving on the prior beyond 1 s
        self.assertEqual((dec.throttle, dec.steer_mrad), (0, d.runner._held_steer))
        self.assertEqual(d.runner.stale_events, 1)            # counted with the stale pauses of the run
        block_ = dec.summary['actuator_prior']
        self.assertTrue(block_['limit'])
        self.assertLessEqual(block_['predict_m'], .5 + 1e-9)
        # a press while the bound lasts is ignored: release, press again
        d.run(.2, hold=False)
        out = d.run(.4)
        self.assertEqual(out[-1][1].state, PAUSED)
        self.assertEqual(out[-1][1].reason, 'press_ignored_inputs_not_fresh')
        # scans match again from 8.5 s: after fresh_recover_s a NEW press resumes
        d.run(1.6, hold=False)                                # .. 10.0 s
        out = d.run(.5)
        self.assertEqual(out[-1][1].state, ACTIVE)
        self.assertFalse(out[-1][1].summary['actuator_prior']['limit'])
        self.assertNotEqual(d.runner.state, ENDED)

    def test_without_a_weight_failed_matches_never_reach_the_bound(self):
        d = Drive(motor_config(**PRIOR), scan_fn=lambda t: BLIND if 6. <= t < 8.5 else SCENE)
        d.run(1.5, hold=False)
        out = d.run(7.)
        reasons = {dec.reason for _, dec in out if dec.state == PAUSED}
        self.assertNotIn('prediction_limit', reasons)
        self.assertFalse(any(dec.summary['actuator_prior']['limit'] for _, dec in out))

    def test_scan_blackout_is_the_stale_rule_first(self):
        d = Drive(motor_config(prior_weight=1., **PRIOR), scan_fn=lambda t: None if 6. <= t < 7.3 else SCENE)
        d.run(1.5, hold=False)
        out = d.run(5.5)                                      # .. 7.0 s: inside the blackout
        t_pause, dec = L.first(out, lambda x: x.state == PAUSED)
        self.assertEqual(dec.reason, 'stale_lidar')           # lidar_max_age_s 0.45 s comes before the 1 s bound
        self.assertLess(t_pause - d.t0, 6.7)
        last = out[-1][1].summary['actuator_prior']
        self.assertTrue(last['predicting'])
        self.assertLessEqual(last['predict_m'], .5 + 1e-9)

    def test_stale_status_switches_the_prior_off(self):
        d = Drive(motor_config(prior_weight=1., **PRIOR))
        d.run(1.5, hold=False)
        d.run(1.)
        d.status = False
        d.runner.steer_required = False                       # only the prior is watched here
        out = d.run(1.)
        block_ = out[-1][1].summary['actuator_prior']
        self.assertFalse(block_['fresh'])
        self.assertIsNone(block_['speed_m_s'])
        self.assertFalse(block_['limit'])


if __name__ == '__main__':
    unittest.main()
