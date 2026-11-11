"""Deterministic bounded observations; no ROS runtime or actuator required."""
import inspect
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parents[1]/'dashboard'))
from callback_timing import CallbackTiming


@pytest.fixture
def observed():
    raw, cpu, real = [10.], [100_000_000], [1_700_000_000_000_000_000]
    timing = CallbackTiming(raw_clock=lambda: raw[0], cpu_clock=lambda: cpu[0], realtime_clock=lambda: real[0])
    return timing, raw, cpu, real


def message(sec=2, ns=42):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=ns)), payload='private-body')


def test_wrapper_requires_two_arguments_and_preserves_exact_return_once(observed):
    timing, raw, cpu, real = observed
    calls = []; sentinel = object(); msg = message()
    def callback(value):
        calls.append(value); raw[0] += .04; cpu[0] += 5_000_000
        return sentinel
    wrapped = timing.wrap('/sim/imu', callback)
    with pytest.raises(TypeError): inspect.signature(wrapped).bind(msg)
    inspect.signature(wrapped).bind(msg, {})
    assert not hasattr(wrapped, '__wrapped__') and wrapped._observed_callback is callback
    assert wrapped(msg, {'received_timestamp': 123}) is sentinel and calls == [msg]
    frozen = timing.freeze('imu_synchronization_timeout'); event = frozen['events'][0]
    assert event['source_stamp_ns'] == 2_000_000_042 and event['rmw_received_ns'] == 123
    assert event['entry_raw_s'] == 10. and event['exit_raw_s'] == pytest.approx(10.04)
    assert event['elapsed_s'] == pytest.approx(.04) and event['thread_cpu_s'] == pytest.approx(.005)
    assert event['entry_realtime_ns'] == real[0] and not event['callback_raised']
    assert 'private-body' not in json.dumps(frozen)


def test_original_exception_identity_and_single_execution_survive_diagnostic_failure(observed, monkeypatch):
    timing, *_ = observed; calls = []; original = ValueError('private-error')
    def callback(msg): calls.append(msg); raise original
    def broken(*args): raise RuntimeError('private-diagnostic')
    monkeypatch.setattr(timing, '_finish', broken)
    msg = message()
    with pytest.raises(ValueError) as caught: timing.wrap('/sim/imu', callback)(msg, {})
    assert caught.value is original and calls == [msg]
    assert timing.snapshot()['diagnostic_errors'] == 1
    assert 'private-' not in json.dumps(timing.snapshot())


def test_callback_exception_is_recorded_without_its_message(observed):
    timing, *_ = observed; original = KeyError('private-token')
    def callback(_): raise original
    with pytest.raises(KeyError) as caught: timing.measure_call('/sim/imu', callback, message(), None)
    assert caught.value is original
    frozen = timing.freeze('imu_synchronization_timeout')
    assert frozen['events'][0]['callback_raised'] and 'private-token' not in json.dumps(frozen)


def test_128_event_bound_small_healthy_status_and_immutable_first_stop(observed):
    timing, raw, *_ = observed
    for i in range(140):
        raw[0] = 10.+i*.01
        timing.measure_call('/sim/imu', lambda m: None, message(i), {})
    healthy = timing.snapshot()
    assert healthy['recorded'] == 140 and healthy['topics']['/sim/imu']['count'] == 140
    assert 'events' not in healthy and len(timing.events) == 128
    first = timing.freeze('imu_synchronization_timeout')
    assert first['overwritten'] == 12 and len(first['events']) == 128
    assert first['events'][0]['sequence'] == 13 and first['coverage_raw_s'] == pytest.approx(1.27)
    first['events'][0]['topic'] = 'caller_mutation'
    raw[0] = 20.; timing.measure_call('/clock', lambda m: None, NS(clock=NS(sec=9,nanosec=7)), {})
    preserved = timing.freeze('later_stop')
    assert preserved['reason'] == 'imu_synchronization_timeout'
    assert preserved['events'][0]['topic'] == '/sim/imu' and preserved['recorded'] == 140
    assert timing.snapshot()['recorded'] == 141 and timing.snapshot()['frozen']


def test_off_thread_and_unknown_topic_call_once_without_diagnostics(observed):
    timing, *_ = observed; calls = []; msg = message()
    callback = lambda m: calls.append(m) or 'returned'
    result = []
    worker = threading.Thread(target=lambda: result.extend([timing.wrap('/sim/imu', callback)(msg, {}), timing.freeze('off_thread')]))
    worker.start(); worker.join(timeout=1)
    assert not worker.is_alive() and result == ['returned', None] and calls == [msg]
    assert timing.measure_call('/unknown', callback, msg, {}) == 'returned'
    assert timing.measure_call({}, callback, msg, {}) == 'returned'
    assert calls == [msg, msg, msg] and timing.recorded == 0


@pytest.mark.parametrize('info', [None, {}, {'received_timestamp': 0, 'source_timestamp': -1},
    {'received_timestamp': float('nan'), 'source_timestamp': True,
     'publication_sequence_number': 2**64-1, 'reception_sequence_number': -1}])
def test_absent_or_invalid_metadata_is_unknown_and_callback_still_runs_once(observed, info):
    timing, *_ = observed; calls = []
    timing.measure_call('/sim/imu', calls.append, message(), info)
    event = timing.freeze('test_stop')['events'][0]
    assert len(calls) == 1
    assert all(event[key] is None for key in ('rmw_source_ns', 'rmw_received_ns', 'publication_seq', 'reception_seq'))
    json.dumps(event, allow_nan=False)


def test_object_metadata_and_clock_header_remain_separate_domains(observed):
    timing, *_ = observed
    info = NS(source_timestamp=1_000_000_000, received_timestamp=1_100_000_000,
              publication_sequence_number=0, reception_sequence_number=19)
    timing.measure_call('/clock', lambda m: None, NS(clock=NS(sec=3, nanosec=1)), info)
    event = timing.freeze('test_stop')['events'][0]
    assert event['source_stamp_ns'] == 3_000_000_001
    assert event['rmw_source_ns'] == 1_000_000_000 and event['rmw_received_ns'] == 1_100_000_000
    assert event['publication_seq'] == 0 and event['reception_seq'] == 19
    assert 'queue_latency_s' not in event


def test_broken_metadata_properties_do_not_change_callback(observed):
    timing, *_ = observed
    class Broken:
        def __getattr__(self, key): raise ValueError('private-metadata')
    called = []
    assert timing.measure_call('/sim/imu', lambda m: called.append(m) or 17, Broken(), Broken()) == 17
    frozen = timing.freeze('test_stop')
    assert len(called) == 1 and frozen['events'][0]['source_stamp_ns'] is None
    assert 'private-metadata' not in json.dumps(frozen, allow_nan=False)


def test_invalid_or_backward_diagnostic_clocks_do_not_change_callback(observed):
    timing, raw, cpu, real = observed
    raw[0] = float('nan'); real[0] = -1
    def callback(_): raw[0] = 9.; cpu[0] -= 1; return 42
    assert timing.measure_call('/sim/imu', callback, message(), {}) == 42
    event = timing.freeze('test_stop')['events'][0]
    assert event['entry_raw_s'] is None and event['elapsed_s'] is None
    assert event['entry_realtime_ns'] is None and event['thread_cpu_s'] is None
    assert timing.snapshot()['invalid_clock_records'] == 1
    json.dumps(timing.freeze('another_stop'), allow_nan=False)


@pytest.mark.parametrize('kwargs', [{'capacity':0}, {'capacity':129}, {'capacity':True},
    {'topics':()}, {'topics':('/x','/x')}, {'topics':tuple('/'+str(i) for i in range(8))}])
def test_invalid_capacity_or_topic_bounds_rejected(kwargs):
    with pytest.raises(ValueError): CallbackTiming(**kwargs)


def test_snapshot_caller_mutation_and_unsafe_reason_do_not_leak(observed):
    timing, *_ = observed
    healthy = timing.snapshot(); healthy['topics']['/clock']['count'] = 999
    assert timing.snapshot()['topics']['/clock']['count'] == 0
    frozen = timing.freeze('private exception with spaces')
    assert frozen['reason'] == 'unexpected_stop' and frozen['events'] == []
    assert frozen['coverage_raw_s'] is None


def test_scan_stages_measure_raw_and_cpu_once_without_healthy_payload_growth(observed):
    timing,raw,cpu,_=observed;calls=[];sentinel=object()
    def work(value):
        calls.append(value);raw[0]+=.025;cpu[0]+=2_000_000
        return sentinel
    def scan(msg):
        assert timing.measure_stage('scan_telemetry',work,msg) is sentinel
        assert timing.measure_stage('scan_visualization',work,msg) is sentinel
        return sentinel
    msg=message();assert timing.measure_call('/sim/scan',scan,msg,{}) is sentinel
    assert calls==[msg,msg] and timing.active_scan is None
    event=timing.freeze('test_stop')['events'][0]
    assert set(event['scan_subphases'])=={'scan_telemetry','scan_visualization'}
    first=event['scan_subphases']['scan_telemetry']
    assert first==pytest.approx({'calls':1,'entry_raw_s':10.,'exit_raw_s':10.025,
                                 'elapsed_s':.025,'thread_cpu_s':.002})
    assert 'scan_subphases' not in json.dumps(timing.snapshot())
    timing.measure_call('/sim/imu',lambda m:None,msg,{})
    assert 'scan_subphases' not in timing.events[-1]


def test_scan_stage_exception_preserves_original_and_always_resets_context(observed):
    timing,raw,cpu,_=observed;calls=[];original=ValueError('private-stage-error')
    def bad(msg):
        calls.append(msg);raw[0]+=.01;cpu[0]+=1_000_000;raise original
    def scan(msg):return timing.measure_stage('scan_telemetry',bad,msg)
    msg=message()
    with pytest.raises(ValueError) as caught:timing.measure_call('/sim/scan',scan,msg,{})
    assert caught.value is original and calls==[msg] and timing.active_scan is None
    event=timing.freeze('test_stop')['events'][0]
    assert event['callback_raised'] and event['scan_subphases']['scan_telemetry']['calls']==1
    assert 'private-stage-error' not in json.dumps(event)


def test_stage_offthread_outside_callback_unknown_and_nested_are_plain_single_calls(observed):
    timing,raw,cpu,_=observed;calls=[]
    def work(label):calls.append(label);return label
    assert timing.measure_stage('scan_telemetry',work,'outside')=='outside'
    def scan(msg):
        result=[]
        worker=threading.Thread(target=lambda:result.append(timing.measure_stage('scan_telemetry',work,'worker')))
        worker.start();worker.join(timeout=1)
        assert not worker.is_alive() and result==['worker']
        assert timing.measure_stage('unapproved',work,'unknown')=='unknown'
        def outer():return timing.measure_stage('scan_visualization',work,'nested-stage')
        assert timing.measure_stage('scan_telemetry',outer)=='nested-stage'
        def nested_callback(_):return timing.measure_stage('scan_republish',work,'nested-callback')
        assert timing.measure_call('/sim/scan',nested_callback,msg,{})=='nested-callback'
        assert timing.measure_stage('scan_control_cache',work,'restored')=='restored'
    timing.measure_call('/sim/scan',scan,message(),{})
    assert calls==['outside','worker','unknown','nested-stage','nested-callback','restored']
    assert timing.recorded==1 and timing.active_scan is None
    assert set(timing.events[0]['scan_subphases'])=={'scan_telemetry','scan_control_cache'}


def test_invalid_stage_clock_is_observational_and_strict_json_remains_valid(observed):
    timing,raw,cpu,_=observed;calls=[]
    def work():calls.append(1);raw[0]=float('nan');return 37
    def scan(_):return timing.measure_stage('scan_telemetry',work)
    assert timing.measure_call('/sim/scan',scan,message(),{})==37
    assert calls==[1] and timing.events[0]['scan_subphases']=={}
    assert timing.invalid_clock_records==2 and timing.active_scan is None
    json.dumps(timing.freeze('test_stop'),allow_nan=False)


def test_imu_stages_separate_cpu_from_wall_and_keep_healthy_payload_small(observed):
    timing,raw,cpu,_=observed;calls=[];sentinel=object()
    def work(name):
        calls.append(name);raw[0]+=.02;cpu[0]+=1_000_000;return sentinel
    def imu(_):
        for name in ('imu_estimator','imu_message_prepare','imu_publish_touch'):
            assert timing.measure_stage(name,work,name) is sentinel
        return sentinel
    assert timing.measure_call('/sim/imu',imu,message(),{}) is sentinel
    assert calls==['imu_estimator','imu_message_prepare','imu_publish_touch']
    healthy=timing.snapshot()
    assert 'subphases' not in json.dumps(healthy) and 'events' not in healthy
    event=timing.freeze('test_stop')['events'][0]
    assert 'scan_subphases' not in event and len(event['imu_subphases'])==3
    for stage in event['imu_subphases'].values():
        assert stage['calls']==1 and stage['elapsed_s']==pytest.approx(.02)
        assert stage['thread_cpu_s']==pytest.approx(.001)
    assert event['elapsed_s']==pytest.approx(.06) and event['thread_cpu_s']==pytest.approx(.003)


def test_imu_stage_allowlist_nested_offthread_and_exception_preserve_one_call(observed):
    timing,raw,cpu,_=observed;calls=[];original=ValueError('private-imu-error')
    def work(name):calls.append(name);return name
    assert timing.measure_stage('imu_estimator',work,'outside')=='outside'
    def imu(msg):
        result=[]
        worker=threading.Thread(target=lambda:result.append(timing.measure_stage('imu_estimator',work,'worker')))
        worker.start();worker.join(timeout=1)
        assert not worker.is_alive() and result==['worker']
        timing.measure_stage('scan_republish',work,'wrong-topic')
        def nested():return timing.measure_stage('imu_message_prepare',work,'nested-stage')
        timing.measure_stage('imu_estimator',nested)
        timing.measure_call('/sim/scan',lambda _:timing.measure_stage('imu_publish_touch',work,'nested-callback'),msg,{})
        def bad():calls.append('bad');raw[0]+=.01;cpu[0]+=500_000;raise original
        return timing.measure_stage('imu_publish_touch',bad)
    with pytest.raises(ValueError) as caught:timing.measure_call('/sim/imu',imu,message(),{})
    assert caught.value is original and timing.active_scan is None
    assert calls==['outside','worker','wrong-topic','nested-stage','nested-callback','bad']
    assert timing.recorded==1
    event=timing.freeze('test_stop')['events'][0]
    assert set(event['imu_subphases'])=={'imu_estimator','imu_publish_touch'}
    assert event['callback_raised'] and event['imu_subphases']['imu_publish_touch']['calls']==1
    assert 'private-imu-error' not in json.dumps(event)


def test_imu_stage_names_cannot_attach_to_scan_event(observed):
    timing,*_=observed;calls=[]
    timing.measure_call('/sim/scan',lambda _:timing.measure_stage('imu_estimator',calls.append,17),message(),{})
    assert calls==[17] and timing.events[0]['scan_subphases']=={}
    assert 'imu_subphases' not in timing.events[0]
