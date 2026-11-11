"""Deterministic phase accounting and immutable, bounded fault diagnostics."""
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE));sys.path.insert(0,str(HERE.parents[1]/'dashboard'))
import loop_timing as module
from loop_timing import LoopTiming,capture_context


def iteration(timing,clock,times,*,active=True,after=True,context=None,snapshot=False):
    context=context or (lambda receipts:{'autonomy_reason':'tracking','sim_s':1.})
    clock[0]=times[0];timing.begin(active=active,wait_since=4.,wait_reason='waiting_for_wheel_imu_pair')
    clock[0]=times[1];timing.after_spin()
    clock[0]=times[2];timing.after_step(active=after,context=context)
    clock[0]=times[3];timing.finish(active=after,had_snapshot=snapshot,context=context)


def test_fast_loop_has_no_context_work_and_slow_phases_and_between_loop_gap_are_distinct():
    clock=[0.];timing=LoopTiming(clock=lambda:clock[0]);context=Mock(return_value={'sim_s':7.})
    iteration(timing,clock,[0.,.005,.009,.010],context=context)
    context.assert_not_called() # A short dispatch phase is not a stall.
    iteration(timing,clock,[.04,.045,.07,.10],context=context,snapshot=True)
    event=timing.snapshot()['recent_slow_iterations'][0]
    assert event['between_iterations_s']==pytest.approx(.030)
    assert event['spin_s']==pytest.approx(.005)
    assert event['step_s']==pytest.approx(.025)
    assert event['snapshot_s']==pytest.approx(.030)
    assert event['loop_s']==pytest.approx(.060)
    assert timing.snapshot()['spin_includes_wait_s']==0.
    assert timing.snapshot()['dispatch_budget_raw_s']==.008
    assert timing.snapshot()['dispatch_max_callbacks']==16
    assert timing.snapshot()['empty_idle_sleep_s']==.001
    context.assert_called_once_with(False)


def test_first_failure_keeps_pre_cleanup_reason_receipts_and_recent_slow_history():
    clock=[0.];timing=LoopTiming(clock=lambda:clock[0],capacity=2)
    for start in (0.,.1,.2):iteration(timing,clock,[start,start+.025,start+.030,start+.031])
    reason=['imu_synchronization_timeout']
    context=lambda receipts:{'autonomy_reason':reason[0],'receipts_included':receipts}
    clock[0]=.3;timing.begin(active=True,wait_since=9.,wait_reason='waiting_for_wheel_imu_pair')
    clock[0]=.31;timing.after_spin()
    clock[0]=.32;timing.after_step(active=False,context=context)
    # HTTP cleanup changes live status during optional snapshot I/O.
    reason[0]='autonomy_integration_end'
    clock[0]=.35;timing.finish(active=False,had_snapshot=True,context=context)
    frozen=timing.snapshot()['first_autonomy_failure']
    assert frozen['reason']=='imu_synchronization_timeout'
    assert frozen['event']['context']['receipts_included'] is True
    assert frozen['event']['wait_at_iteration_start']['since_monotonic']==9.
    assert len(frozen['recent_slow_iterations'])==2
    iteration(timing,clock,[1.,1.1,1.2,1.3],active=False,after=False)
    assert timing.snapshot()['first_autonomy_failure']==frozen
    frozen['event']['context']['autonomy_reason']='caller_mutation'
    assert timing.snapshot()['first_autonomy_failure']['reason']=='imu_synchronization_timeout'


def test_expected_stop_does_not_consume_first_failure_and_external_between_loop_stop_is_detected():
    clock=[0.];timing=LoopTiming(clock=lambda:clock[0])
    iteration(timing,clock,[0.,.001,.002,.003],active=True,after=False,
              context=lambda receipts:{'autonomy_reason':'planned_lanes_completed'})
    assert timing.snapshot()['first_autonomy_failure'] is None
    iteration(timing,clock,[.1,.101,.102,.103],active=True,after=True)
    iteration(timing,clock,[.2,.201,.202,.203],active=False,after=False,
              context=lambda receipts:{'autonomy_reason':'odometry_fault'})
    assert timing.snapshot()['first_autonomy_failure']['reason']=='odometry_fault'


def test_invalid_clock_or_context_error_cannot_throw_into_control_and_json_is_strict():
    clock=[0.];timing=LoopTiming(clock=lambda:clock[0])
    iteration(timing,clock,[0.,None,1.,2.])
    assert timing.snapshot()['invalid_clock_iterations']==1
    def broken(_):raise ValueError('private-token')
    iteration(timing,clock,[3.,3.1,3.2,3.3],active=True,after=False,context=broken)
    output=json.dumps(timing.snapshot(),allow_nan=False)
    assert 'private-token' not in output and 'ValueError' in output
    assert len(timing.snapshot()['recent_slow_iterations'])<=16


def test_capture_is_allowlisted_and_bounds_receipts_without_mutating_live_data(monkeypatch):
    monkeypatch.setattr(module.time,'monotonic',lambda:20.)
    receipts=[{'stamp':i,'receipt_monotonic':i,'token':'private-token'} for i in range(30)]
    auto=NS(yaw_measurement={'sim_stamp':3.,'wall_stamp':19.9,'token':'private-token'},
            odom={'sim_stamp':2.99,'wall_stamp':19.8,'owner':'private-token'},
            mission=NS(state='STOPPED',reason='imu_synchronization_timeout'),
            sensor_wait_reason=None,sensor_wait_since=None,yaw_status={'measurement_wall_age_s':.3})
    est=NS(pending_wheel=(3.1,),pending_since=19.7,fault=None,wheel_receipts=receipts,imu_receipts=receipts)
    node=NS(autonomy=auto,planar_odometry=est,sim_time=3.05,clock_seen=19.95)
    lean=capture_context(node)
    full=capture_context(node,include_receipts=True)
    assert 'wheel_receipts' not in lean and len(full['wheel_receipts'])==8
    assert full['clock_arrival_age_s']==pytest.approx(.05)
    assert full['imu_arrival_age_s']==pytest.approx(.1)
    assert full['arrival_age_clock']=='CLOCK_MONOTONIC'
    assert 'private-token' not in json.dumps(full,allow_nan=False)
    assert receipts[0]['token']=='private-token'
    assert full['input_counts']=={'occupied_cells':0,'path_points':0,'scan_points':0}


def test_thread_cpu_lock_wait_and_stage_accounting_remain_distinct_and_freeze():
    raw=[0.];cpu=[0]
    timing=LoopTiming(clock=lambda:raw[0],cpu_clock=lambda:cpu[0])
    context=lambda receipts:{'autonomy_reason':'imu_synchronization_timeout'}
    timing.begin(active=True)
    raw[0]=.05;cpu[0]=5_000_000;timing.after_spin()
    started=timing.start_stage('step_lock_wait')
    raw[0]=.09;cpu[0]=5_100_000;timing.end_stage('step_lock_wait',started)
    def map_work(value):
        raw[0]=.13;cpu[0]=25_100_000
        return value
    assert timing.measure_call('map_ready',map_work,42)==42
    raw[0]=.15;cpu[0]=30_000_000;timing.after_step(active=False,context=context)
    started=timing.start_stage('snapshot_lock_wait')
    raw[0]=.16;cpu[0]=30_100_000;timing.end_stage('snapshot_lock_wait',started)
    raw[0]=.19;cpu[0]=40_000_000;timing.finish(active=False,had_snapshot=True,context=context)
    frozen=timing.snapshot()['first_autonomy_failure']['event']
    assert frozen['thread_cpu_s']==pytest.approx({'loop_s':.04,'spin_s':.005,'step_s':.025,'snapshot_s':.01})
    assert frozen['step_s']==pytest.approx(.1)
    assert frozen['subphases']['step_lock_wait']==pytest.approx({'calls':1,'elapsed_s':.04,'thread_cpu_s':.0001})
    assert frozen['subphases']['map_ready']==pytest.approx({'calls':1,'elapsed_s':.04,'thread_cpu_s':.02})
    assert frozen['subphases']['snapshot_lock_wait']==pytest.approx({'calls':1,'elapsed_s':.01,'thread_cpu_s':.0001})
    assert timing.current is None
    assert timing.snapshot()['cpu_scope']=='adapter_main_thread_only'
    assert timing.snapshot()['subphases_overlap_parent_phase'] is True
    frozen['subphases']['map_ready']['calls']=900
    assert timing.snapshot()['first_autonomy_failure']['event']['subphases']['map_ready']['calls']==1


def test_call_observer_preserves_exceptions_and_ignores_http_thread_and_unknown_stages():
    raw=[0.];cpu=[0];timing=LoopTiming(clock=lambda:raw[0],cpu_clock=lambda:cpu[0])
    timing.begin(active=True)
    calls=[]
    def ordinary(value):calls.append(value);return value
    worker=threading.Thread(target=lambda:timing.measure_call('map_ready',ordinary,'http'))
    worker.start();worker.join(timeout=1)
    assert not worker.is_alive() and calls==['http'] and timing.current['subphases']=={}
    assert timing.measure_call('unapproved_name',ordinary,'direct')=='direct'
    assert timing.current['subphases']=={}
    error=ValueError('original-function-error')
    def failing():
        raw[0]=.01;cpu[0]=2_000_000
        raise error
    with pytest.raises(ValueError) as caught:timing.measure_call('yaw_track',failing)
    assert caught.value is error
    assert timing.current['subphases']['yaw_track']==pytest.approx({'calls':1,'elapsed_s':.01,'thread_cpu_s':.002})


def test_borrowed_main_thread_serialization_matches_detached_and_preserves_default_isolation():
    clock=[0.];timing=LoopTiming(clock=lambda:clock[0],capacity=2)
    context=lambda receipts:{'autonomy_reason':'imu_synchronization_timeout',
                             'input_counts':{'scan_points':3200}}
    iteration(timing,clock,[0.,.010,.021,.03],active=True,after=False,context=context)
    borrowed=timing.snapshot(detached=False)
    detached=timing.snapshot()
    assert json.dumps(borrowed,sort_keys=True,allow_nan=False)==json.dumps(detached,sort_keys=True,allow_nan=False)
    assert borrowed['recent_slow_iterations'][0] is timing.slow[0]
    assert borrowed['first_autonomy_failure'] is timing.first_failure
    detached['recent_slow_iterations'][0]['context']['input_counts']['scan_points']=0
    detached['first_autonomy_failure']['event']['context']['autonomy_reason']='changed'
    assert timing.snapshot()['recent_slow_iterations'][0]['context']['input_counts']['scan_points']==3200
    assert timing.snapshot()['first_autonomy_failure']['event']['context']['autonomy_reason']=='imu_synchronization_timeout'


def test_borrowed_records_cannot_be_requested_on_an_http_thread():
    timing=LoopTiming();errors=[]
    def ipc_reader():
        try:timing.snapshot(detached=False)
        except RuntimeError as error:errors.append(str(error))
    worker=threading.Thread(target=ipc_reader);worker.start();worker.join(timeout=1)
    assert not worker.is_alive()
    assert errors==['borrowed_loop_snapshot_requires_adapter_thread']


def callback_context_node(observer):
    return NS(callback_timing=observer,sim_time=3.,clock_seen=0.,
              autonomy=NS(yaw_measurement={},odom={},sensor_wait_reason=None,sensor_wait_since=None,
                          yaw_status={},mission=NS(state='STOPPED',reason='planned_lanes_completed')),
              planar_odometry=NS(pending_wheel=None,pending_since=None,fault=None))


def test_callback_ring_freezes_only_unexpected_stop_and_is_stored_once_outside_slow_events():
    observer=Mock()
    observer.freeze.return_value={'schema':'ubts-callback-timing-v1','events':[{'topic':'/sim/imu'}]}
    node=callback_context_node(observer)
    context=lambda receipts:capture_context(node,include_receipts=receipts)
    clock=[0.];timing=LoopTiming(clock=lambda:clock[0])
    iteration(timing,clock,[0.,.010,.021,.03],active=True,after=False,context=context)
    observer.freeze.assert_not_called()
    assert timing.first_failure is None
    node.autonomy.mission.reason='imu_synchronization_timeout'
    clock[0]=.1;timing.begin(active=True)
    clock[0]=.110;timing.after_spin()
    clock[0]=.121;timing.after_step(active=False,context=context)
    # The exact diagnostic is captured before an HTTP cleanup changes reason.
    node.autonomy.mission.reason='autonomy_integration_end'
    clock[0]=.130;timing.finish(active=False,had_snapshot=True,context=context)
    observer.freeze.assert_called_once_with('imu_synchronization_timeout')
    result=timing.snapshot()
    assert result['first_autonomy_failure']['reason']=='imu_synchronization_timeout'
    assert result['first_autonomy_failure']['callback_timing']==observer.freeze.return_value
    assert json.dumps(result).count('"events"')==1
    assert 'callback_timing_at_stop' not in json.dumps(result)
    iteration(timing,clock,[.2,.21,.221,.23],active=False,after=False,context=context)
    observer.freeze.assert_called_once()
    # Detached public snapshots retain the existing mutation-isolation contract.
    result['first_autonomy_failure']['callback_timing']['events'][0]['topic']='changed'
    assert timing.snapshot()['first_autonomy_failure']['callback_timing']['events'][0]['topic']=='/sim/imu'


def test_failed_callback_freeze_preserves_original_stop_context_without_exception_details():
    observer=Mock();observer.freeze.side_effect=ValueError('private-diagnostic-payload')
    node=callback_context_node(observer);node.autonomy.mission.reason='imu_synchronization_timeout'
    captured=capture_context(node,include_receipts=True)
    assert captured['autonomy_reason']=='imu_synchronization_timeout'
    assert captured['callback_timing_error']=='ValueError'
    assert 'wheel_receipts' in captured and 'feedback_ages' in captured
    assert 'private-diagnostic-payload' not in json.dumps(captured,allow_nan=False)


def test_writer_snapshot_published_trees_remain_unchanged_across_future_loops_and_stop():
    clock=[0.];timing=LoopTiming(clock=lambda:clock[0],capacity=2)
    iteration(timing,clock,[0.,.01,.03,.04])
    captured=timing.writer_snapshot();before=json.dumps(captured,sort_keys=True)
    assert captured['recent_slow_iterations'][0] is timing.slow[0]
    iteration(timing,clock,[.1,.11,.13,.14],active=True,after=False,
              context=lambda receipts:{'autonomy_reason':'imu_synchronization_timeout'})
    fault_snapshot=timing.writer_snapshot();fault_before=json.dumps(fault_snapshot,sort_keys=True)
    for start in (.2,.3,.4):iteration(timing,clock,[start,start+.01,start+.03,start+.04],active=False,after=False)
    assert json.dumps(captured,sort_keys=True)==before
    assert json.dumps(fault_snapshot,sort_keys=True)==fault_before
    assert fault_snapshot['first_autonomy_failure'] is timing.first_failure


def test_dispatch_watchdog_and_sleep_cpu_aggregate_without_double_counting_parent():
    from callback_drain import drain_ready,idle_after_empty
    raw=[0.];cpu=[0];timing=LoopTiming(clock=lambda:raw[0],cpu_clock=lambda:cpu[0],slow_s=.001)
    calls=[]
    def advance(wall,compute):raw[0]+=wall;cpu[0]+=int(compute*1e9)
    def guard():calls.append('guard');advance(.001,.0001)
    def spin(*,timeout_sec):assert timeout_sec==0;calls.append('spin');advance(.003,.0002)
    def sleep(delay):assert delay==.001;calls.append('sleep');advance(.002,.00001)
    timing.begin(active=True)
    result=drain_ready(NS(spin_once=spin),lambda:0,guard,raw_clock=lambda:raw[0],observer=timing.measure_call)
    idle_after_empty(result,guard,sleep=sleep,observer=timing.measure_call)
    timing.after_spin();timing.after_step(active=True,context=lambda _:{});
    timing.finish(active=True,had_snapshot=False,context=lambda _:{});
    event=timing.snapshot()['recent_slow_iterations'][0]
    assert calls==['guard','spin','guard','guard','sleep','guard']
    assert event['spin_s']==pytest.approx(.009)
    assert event['thread_cpu_s']['spin_s']==pytest.approx(.00061)
    stages=event['subphases']
    totals=lambda name:{k:v for k,v in stages[name].items() if k!='max_call'}
    assert totals('executor_spin_once')==pytest.approx({'calls':1,'elapsed_s':.003,'thread_cpu_s':.0002})
    assert totals('drain_watchdog')==pytest.approx({'calls':4,'elapsed_s':.004,'thread_cpu_s':.0004})
    assert totals('idle_sleep')==pytest.approx({'calls':1,'elapsed_s':.002,'thread_cpu_s':.00001})
    assert stages['executor_spin_once']['max_call']==pytest.approx(
        {'started_raw_s':.001,'ended_raw_s':.004,'elapsed_s':.003,'thread_cpu_s':.0002})
    assert timing.snapshot()['dispatch_subphase_nesting']['spin_s']==['executor_spin_once','drain_watchdog','idle_sleep']


def test_safety_nested_timings_preserve_main_thread_scope_and_failure():
    raw=[0.];cpu=[0];timing=LoopTiming(clock=lambda:raw[0],cpu_clock=lambda:cpu[0])
    timing.begin(active=True)
    for name in ('executor_spin_once','drain_watchdog','idle_sleep','safety_lock_wait','safety_body'):
        worker=threading.Thread(target=lambda:timing.measure_call(name,lambda:None))
        worker.start();worker.join(timeout=1)
        assert not worker.is_alive()
    assert timing.current['subphases']=={}
    error=ValueError('safety failed');calls=[]
    def body():raw[0]+=.004;cpu[0]+=200_000;calls.append('body');raise error
    def guarded():
        acquired=timing.start_stage('safety_lock_wait')
        raw[0]+=.010;cpu[0]+=100_000
        timing.end_stage('safety_lock_wait',acquired)
        return timing.measure_call('safety_body',body)
    with pytest.raises(ValueError) as caught:timing.measure_call('drain_watchdog',guarded)
    assert caught.value is error and calls==['body']
    stages=timing.current['subphases']
    totals=lambda name:{k:v for k,v in stages[name].items() if k!='max_call'}
    assert totals('drain_watchdog')==pytest.approx({'calls':1,'elapsed_s':.014,'thread_cpu_s':.0003})
    assert totals('safety_lock_wait')==pytest.approx({'calls':1,'elapsed_s':.010,'thread_cpu_s':.0001})
    assert totals('safety_body')==pytest.approx({'calls':1,'elapsed_s':.004,'thread_cpu_s':.0002})
    assert stages['drain_watchdog']['max_call']==pytest.approx(
        {'started_raw_s':0.,'ended_raw_s':.014,'elapsed_s':.014,'thread_cpu_s':.0003})
    assert timing.snapshot()['dispatch_subphase_nesting']['drain_watchdog']==['safety_lock_wait','safety_body']


def test_dispatch_retains_only_largest_call_interval_after_many_calls():
    raw=[0.];cpu=[0];timing=LoopTiming(clock=lambda:raw[0],cpu_clock=lambda:cpu[0])
    timing.begin(active=True)
    def call(delay):raw[0]+=delay;cpu[0]+=100_000
    for delay in [.001]*20+[.098]+[.001]*20:
        timing.measure_call('executor_spin_once',call,delay)
    stage=timing.current['subphases']['executor_spin_once']
    assert stage['calls']==41 and len(stage)==4
    assert stage['max_call']==pytest.approx(
        {'started_raw_s':.020,'ended_raw_s':.118,'elapsed_s':.098,'thread_cpu_s':.0001})


@pytest.mark.parametrize('boundary',['start','end'])
@pytest.mark.parametrize('clock_name',['clock','cpu_clock'])
@pytest.mark.parametrize('body_raises',[False,True])
def test_dispatch_clock_failure_never_skips_callable_or_replaces_exception(boundary,clock_name,body_raises):
    raw=[0.];timing=LoopTiming(clock=lambda:raw[0],cpu_clock=lambda:0)
    timing.begin(active=True);original=ValueError('original watchdog');calls=[]
    def broken():raise RuntimeError('diagnostic clock failure')
    def body():
        calls.append('called')
        if boundary=='end':setattr(timing,clock_name,broken)
        if body_raises:raise original
        return 42
    if boundary=='start':setattr(timing,clock_name,broken)
    if body_raises:
        with pytest.raises(ValueError) as caught:timing.measure_call('drain_watchdog',body)
        assert caught.value is original
    else:assert timing.measure_call('drain_watchdog',body)==42
    assert calls==['called'] and timing.stage_observation_errors==1
    assert timing.current['subphases']=={}
