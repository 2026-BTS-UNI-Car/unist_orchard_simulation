"""PW-10 online scale in the fast SIM (steering/safety_scripts/online_scale_checks.py; docs/steering/ACTUATOR_PRIOR_KO.md 4).

The executor runs in the loop on the T1 rig with its default front_steer configuration (the weight-0 actuator prior is
the online-scale monitor) and the plant's drive table at 0.85 x the field tuning table (follow_wide's drive_table
mechanism, with the speed fluctuation and scrub loss of the profile set to 0 so the true ratio is 0.85):
  - the screen value (executor status 'online_scale', what the dashboard relays) converges to 0.85 +- 0.03 once the 20 s
    window is full, and the run log has it as a time series (tick 'prior' blocks: k, mon, fz, hold);
  - a LiDAR blackout freezes it (no change from the cut on, frozen flag, hold lidar_stale);
  - after the run ends the executor offers the new k_field (= 1.0 x 0.85) and accept applies it like a screen change;
  - the screen value follows the SIM truth ratio when the default plant wanders (table_x085 as registered).
Shortened runs of the development seed 7 (the registered measurement is seeds 100-104, full length:
docs/steering/PREREG_PW10_ONLINE_SCALE.json). The pre-registration JSON is the one this module prints.

Needs the native steering program (built into UBTS_STEER_BUILD_DIR or a pytest temporary folder, never into the work
tree); skipped with the reason otherwise. Every number is a placeholder-based SIM result (자리값 기반 SIM 결과).

Run from sim/ubts:  python -B -m pytest -q -p no:cacheprovider test_steering_online_scale.py
"""
import json
import os

import pytest

from steering.safety_scripts import online_scale_checks as osc

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SEED = 7                              # development seed: not one of the registered seeds


@pytest.fixture(scope='module', autouse=True)
def native_build_dir_outside_the_work_tree(tmp_path_factory):
    if os.environ.get('UBTS_STEER_BUILD_DIR'):
        yield
        return
    os.environ['UBTS_STEER_BUILD_DIR'] = str(tmp_path_factory.mktemp('steer_native_build'))
    try:
        yield
    finally:
        os.environ.pop('UBTS_STEER_BUILD_DIR', None)


def run_or_skip(run, duration_s, keep=False):
    try:
        from steering.native import build as native_build
        native_build.find_compiler()
        if not native_build.firmware_present():
            pytest.skip(native_build.missing_firmware_message())
    except Exception as exc:
        pytest.skip('native steering program not buildable here: %s' % exc)
    return osc.run_one(run, SEED, duration_s=duration_s, keep=keep)


def test_prereg_json_is_this_module():
    with open(os.path.join(REPO, 'docs', 'steering', 'PREREG_PW10_ONLINE_SCALE.json'), encoding='utf-8') as fh:
        assert json.load(fh) == json.loads(json.dumps(osc.prereg()))


def test_specs():
    exact, cut, wide = (osc.episode(run, SEED) for run in osc.RUNS)
    assert exact['rig']['drive_table']['factor'] == cut['rig']['drive_table']['factor'] == .85
    assert wide['rig']['drive_table']['factor'] == .85
    assert exact['config'] == {} == wide['config']                   # the executor's defaults (monitor on, weight 0)
    assert exact['rig']['profile'].value('speed_fluctuation_sd') == 0.
    assert wide['rig']['profile'].value('speed_fluctuation_sd') == .05   # the registered table_x085 plant
    assert cut['faults'] == [dict(kind='sensor_blackout', at_s=osc.CUT_AT_S,
                                  params=dict(duration_s=osc.CUT_S, which='lidar'))]
    with pytest.raises(ValueError):
        osc.episode('nope', SEED)


def test_screen_value_converges_to_the_plant_factor_and_is_logged_and_offered():
    row, facade = run_or_skip('exact', 34., keep=True)
    try:
        assert row['full_window_reads'] >= osc.MIN_JUDGED_SAMPLES
        assert .82 <= row['values']['min'] <= row['values']['max'] <= .88, row['values']
        assert row['judged']['series'], row['series']
        ticks = osc.ticks(facade)
        assert all(fz is False and hold is None for t, state, k, mon, fz, hold, v in ticks
                   if t >= row['series']['first_value_t'] and state == 'ACTIVE' and v is not None and v > .2)
        # the operator ends the run: the executor offers k_field 1.0 x the scale and accept applies it
        facade.executor.submit('end')
        facade.run(facade.rig.now + 3.)              # the 0/0 fade, then the bridge stop (terminal_stop_wait_s 1.5 s)
        assert facade.executor.status()['tuning']['editable']
        offer = facade.executor.status()['online_scale']['offer']
        assert offer['ok'] and offer['why_not'] is None and offer['speed_scale_now'] == 1.
        assert offer['speed_scale_new'] == round(offer['value'], 3) and .82 <= offer['value'] <= .88
        reply = osc.answer(facade, {'online_scale': 'accept', 'run_id': offer['run_id']})
        assert reply['ok'] and reply['action'] == 'online_scale', reply
        assert facade.executor.tuning.values['speed_scale'] == offer['speed_scale_new']
        assert facade.executor.config.prior_speed_scale == pytest.approx(offer['speed_scale_new'])
        events = [r['event'] for r in facade.log.records if r.get('k') == 'event'
                  and r.get('event') in ('online_scale_offer', 'tuning_change', 'online_scale_answer')]
        assert events == ['online_scale_offer', 'tuning_change', 'online_scale_answer']
    finally:
        facade.rig.close()


def test_lidar_blackout_freezes_the_scale():
    row, _ = run_or_skip('cut', osc.CUT_AT_S + 6.)
    assert row['judged']['frozen'], row['cut']
    assert row['cut']['k_settled'] is not None and .82 <= row['cut']['k_settled'] <= .88
    assert 'lidar_stale' in row['cut']['holds']


def test_screen_value_follows_the_truth_ratio_of_the_default_plant():
    row, _ = run_or_skip('wide', 34.)
    assert row['judged']['tracking'], row['tracking']
