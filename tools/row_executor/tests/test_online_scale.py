"""Online scale of the front-steer executor (PW-10; online_scale.py, docs/steering/ACTUATOR_PRIOR_KO.md 4,
OPERATOR_API_KO.md 6.1): the proposal rule (new k_field = prior_speed_scale x k / (surface x battery)), the tracker
(one offer per ended row run, superseded by the next run, the status block), and the executor loop's answer form of
POST /api/params (accept only at rest and only for the offered run, dismiss, both logged; accept is a tuning change
like a screen change). The F-SIM convergence and freeze checks are in sim/ubts/test_steering_online_scale.py.

Synthetic inputs only (no hardware, no ROS). Every number is a test value, not a vehicle value.
"""
import os
import shutil
import tempfile
import unittest

import rowexec_testlib as L  # noqa: F401  (path setup)
from row_executor import executor_main as M
from row_executor import field_tuning as T
from row_executor import online_scale as OS
from row_executor.core import PAUSED
from test_executor_loop import Clock
from test_field_tuning import steer_shadow
from test_steer_executor import SCIPY_PROBLEM

KEY_RANGE = (T.BY_NAME['speed_scale'].low, T.BY_NAME['speed_scale'].high)


def prior_block(k=.85, window=20., frozen=False, hold=None, mismatch=False):
    """The scale part of an estimator 'actuator_prior' block (closed_loop/actuator_prior.py snapshot)."""
    return dict(online_scale=k, monitor_s=window, monitor_window_s=20., monitor_min_s=10., online_scale_frozen=frozen,
                online_scale_hold=hold, online_scale_age_s=.1, table_mismatch=mismatch, unmatched_scans=0)


def values(**over):
    out = T.defaults()
    out.update(over)
    return out


class ProposalTest(unittest.TestCase):
    def propose(self, block, tuning=None, prior_scale=1.):
        return OS.proposal(OS.compact(block), values() if tuning is None else tuning, prior_scale, KEY_RANGE,
                           T.SCALE_RANGE)

    def test_new_k_field_is_the_old_one_times_the_online_scale(self):
        self.assertEqual(self.propose(prior_block(.8512)), (.851, None))
        self.assertEqual(self.propose(prior_block(1.1), values(speed_scale=.9), .9), (.99, None))
        # surface and battery stay as they are: prior scale 1.0 x 0.9 x 1.0 = 0.9, k = 0.9 -> k_field 1.0 x 0.9 = 0.9
        self.assertEqual(self.propose(prior_block(.9), values(surface_scale=.9), .9), (.9, None))

    def test_reasons_not_to_offer(self):
        self.assertEqual(self.propose(prior_block(None)), (None, 'no_scale'))
        self.assertEqual(self.propose(prior_block(.85, window=8.)), (None, 'no_scale'))
        self.assertEqual(self.propose(prior_block(0.)), (None, 'no_forward_motion'))
        self.assertEqual(self.propose(prior_block(.85), prior_scale=1.2), (None, 'scale_set_in_config'))
        self.assertEqual(self.propose(prior_block(.4)), (.4, 'out_of_range'))           # k_field range 0.5 .. 1.3
        self.assertEqual(self.propose(prior_block(1.25), values(battery_scale=1.1), 1.1),
                         (1.25, 'out_of_range'))                                         # product 1.375 > 1.3
        self.assertEqual(self.propose(prior_block(1.0003)), (1., 'no_change'))
        self.assertEqual(OS.proposal(OS.compact(prior_block()), None, 1., KEY_RANGE, T.SCALE_RANGE),
                         (None, 'no_field_tuning'))


class TrackerTest(unittest.TestCase):
    def track(self, tracker, run, ended, block, reason=None):
        return tracker.track(run, ended, block, reason, values(), 1., KEY_RANGE, T.SCALE_RANGE)

    def test_one_offer_per_ended_run_and_a_new_run_supersedes_it(self):
        t = OS.OnlineScale()
        self.assertEqual(self.track(t, None, False, prior_block()), [])
        self.assertEqual(self.track(t, 1, False, prior_block(None, window=3.)), [])
        self.assertIsNone(t.status(True, values(), 1.)['value'])
        self.assertEqual(self.track(t, 1, False, prior_block(.86)), [])
        status = t.status(True, values(), 1.)
        self.assertEqual((status['run_id'], status['value'], status['window_s'], status['offer']), (1, .86, 20., None))
        events = self.track(t, 1, True, prior_block(.85, frozen=True, hold='not_driving'), 'row_end')
        self.assertEqual([name for name, _ in events], ['online_scale_offer'])
        offer = events[0][1]
        self.assertEqual((offer['run_id'], offer['value'], offer['speed_scale_now'], offer['speed_scale_new'],
                          offer['ok'], offer['why_not'], offer['end_reason']), (1, .85, 1., .85, True, None, 'row_end'))
        status = t.status(True, values(), 1.)
        self.assertTrue(status['frozen'])
        self.assertEqual((status['hold'], status['offer']['run_id']), ('not_driving', 1))
        self.assertEqual(self.track(t, 1, True, prior_block(.85, frozen=True, hold='not_driving')), [])   # no repeat
        events = self.track(t, 2, False, prior_block(None, window=0.))          # the next run starts unanswered
        self.assertEqual(events, [('online_scale_answer', dict(run_id=1, value=.85, window_s=20., speed_scale_now=1.,
                                                               speed_scale_new=.85, answer='superseded', by_run=2))])
        self.assertIsNone(t.offer)
        self.assertEqual(t.status(True, values(), 1.)['run_id'], 2)

    def test_status_of_a_loop_without_the_monitor(self):
        t = OS.OnlineScale()
        status = t.status(False, values(), 1.)
        self.assertEqual((status['enabled'], status['value'], status['hold'], status['prior_scale']),
                         (False, None, 'monitor_off', None))


@unittest.skipIf(SCIPY_PROBLEM, SCIPY_PROBLEM)
class LoopAnswerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='online_scale_test_')
        self.path = os.path.join(self.tmp, 'field_tuning.json')
        shutil.copyfile(T.PACKAGE_FILE, self.path)
        from closed_loop import steer_control
        from row_executor.core import Runner
        self.clock = Clock()
        records = self.records = []

        class Log(M._NullLog):
            def put(self, record, tick=False):
                records.append(record)
        self.loop = M.ExecutorLoop(steer_shadow(tuning_file=self.path), clock=self.clock, wall=lambda: 1.7e9, log=Log(),
                                   runner_factory=lambda c: Runner(c, controller_factory=steer_control.make_factory(
                                       {'kind': 'row_follow'}, c)))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def request(self, kind, **kw):
        future = self.loop.submit(kind, **kw)
        self.clock.t += .05
        self.loop.step()
        return future.result()

    def events(self, name):
        return [r for r in self.records if r.get('event') == name]

    def offer(self, k=.85):
        """A run ended with the scale k (the tracker fed as the loop feeds it every tick)."""
        tracker, run = self.loop.online_scale, self.loop.runner.run_id + 7
        tracker.track(run, False, prior_block(k), None, self.loop.tuning.values, self.loop.config.prior_speed_scale,
                      KEY_RANGE, T.SCALE_RANGE)
        tracker.track(run, True, prior_block(k, frozen=True, hold='not_driving'), 'row_end', self.loop.tuning.values,
                      self.loop.config.prior_speed_scale, KEY_RANGE, T.SCALE_RANGE)
        return run

    def test_monitor_runs_by_default_and_a_run_end_makes_an_offer(self):
        self.assertTrue(self.request('start', row_len_m=None)['ok'])
        self.clock.t += .05
        self.loop.step()
        status = self.loop.status()['online_scale']
        self.assertTrue(status['enabled'])
        self.assertEqual((status['run_id'], status['value'], status['target_s'], status['min_s']), (1, None, 20., 10.))
        self.assertTrue(self.request('end')['ok'])
        for _ in range(3):
            self.clock.t += .05
            self.loop.step()
        offers = self.events('online_scale_offer')
        self.assertEqual(len(offers), 1)
        self.assertEqual((offers[0]['run_id'], offers[0]['why_not'], offers[0]['ok']), (1, 'no_scale', False))
        self.assertEqual(self.loop.status()['online_scale']['offer']['why_not'], 'no_scale')
        reply = self.request('params', params={'online_scale': 'accept', 'run_id': 1})
        self.assertEqual((reply['ok'], reply['error'], reply['http']), (False, 'no_scale', 409))
        reply = self.request('params', params={'online_scale': 'dismiss', 'run_id': 1})
        self.assertEqual((reply['ok'], reply['action']), (True, 'online_scale_dismiss'))
        self.assertEqual([e['answer'] for e in self.events('online_scale_answer')], ['dismiss'])
        self.assertIsNone(self.loop.status()['online_scale']['offer'])
        self.assertEqual(self.loop.tuning.revision, 0)

    def test_accept_sets_k_field_like_a_screen_change(self):
        run = self.offer(.8512)
        self.clock.t += .05
        self.loop.step()
        offer = self.loop.status()['online_scale']['offer']
        self.assertEqual((offer['run_id'], offer['speed_scale_new'], offer['ok']), (run, .851, True))
        reply = self.request('params', params={'online_scale': 'accept', 'run_id': run})
        self.assertTrue(reply['ok'], reply)
        self.assertEqual((reply['action'], reply['run_id']), ('online_scale', run))
        self.assertEqual(reply['changes'], [dict(key='speed_scale', old=1., new=.851)])
        self.assertEqual(self.loop.tuning.values['speed_scale'], .851)
        self.assertEqual((self.loop.tuning.revision, self.loop.tuning.preset), (1, 'default*'))
        self.assertAlmostEqual(self.loop.config.prior_speed_scale, .851)
        self.assertAlmostEqual(self.loop.config.tuning_values['speed_scale'], .851)
        change = self.events('tuning_change')
        self.assertEqual(len(change), 1)
        self.assertEqual((change[0]['action'], change[0]['source_run_id'], change[0]['online_scale'], change[0]['key']),
                         ('online_scale', run, .8512, 'speed_scale'))
        answer = self.events('online_scale_answer')
        self.assertEqual((answer[-1]['answer'], answer[-1]['speed_scale_new'], answer[-1]['revision']), ('accept', .851, 1))
        self.assertIsNone(self.loop.status()['online_scale']['offer'])
        reply = self.request('params', params={'online_scale': 'accept', 'run_id': run})       # answered once
        self.assertEqual((reply['error'], reply['http']), ('no_online_scale_offer', 409))

    def test_refusals(self):
        run = self.offer()
        for params, error, http in (({'online_scale': 'accept', 'run_id': run + 1}, 'online_scale_offer_changed', 409),
                                    ({'online_scale': 'yes', 'run_id': run}, 'invalid_params', 400),
                                    ({'online_scale': 'accept', 'run_id': str(run)}, 'invalid_params', 400),
                                    ({'online_scale': 'accept', 'run_id': run, 'k_d': 1.}, 'invalid_params', 400)):
            reply = self.request('params', params=params)
            self.assertEqual((reply['ok'], reply['error'], reply['http']), (False, error, http), params)
        # the tuning's scale changed after the run: the offer's basis is gone
        self.assertTrue(self.request('params', params={'speed_scale': .9})['ok'])
        reply = self.request('params', params={'online_scale': 'accept', 'run_id': run})
        self.assertEqual((reply['error'], reply['http']), ('online_scale_basis_changed', 409))
        self.assertEqual(self.loop.tuning.values['speed_scale'], .9)

    def test_accept_only_at_rest(self):
        run = self.offer()
        tracker_offer = dict(self.loop.online_scale.offer)
        self.assertTrue(self.request('start', row_len_m=None)['ok'])
        self.assertEqual(self.loop.runner.state, PAUSED)
        self.loop.online_scale.offer = tracker_offer             # an offer still open while a run is open
        reply = self.request('params', params={'online_scale': 'accept', 'run_id': run})
        self.assertEqual((reply['error'], reply['http']), ('tuning_locked_while_running', 409))
        self.assertEqual(self.loop.tuning.values['speed_scale'], 1.)


if __name__ == '__main__':
    unittest.main()
