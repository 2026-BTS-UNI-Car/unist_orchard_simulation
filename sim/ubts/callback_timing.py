"""Bounded ROS callback observations, never clocks for motion authorization.

RAW durations and main-thread CPU are comparable only within their own clocks.
RMW timestamps are opaque middleware metadata. Entry REALTIME is retained for
offline clock validation, not subtracted from RAW or used to infer queue latency.
Callback entry is after rclpy takes/deserializes a message, before its body/lock.
"""
from collections import deque
import copy
import math
import threading
import time

from scripts.sim_clock import raw_now


DEFAULT_TOPICS = ('/clock', '/sim/scan', '/sim/imu', '/sim/wheel_odom',
                  '/joint_states', '/ground_truth/odom')
SCHEMA = 'ubts-callback-timing-v1'
SCAN_STAGES = frozenset(('scan_republish', 'scan_control_cache',
                         'scan_telemetry', 'scan_visualization'))
IMU_STAGES = frozenset(('imu_estimator', 'imu_message_prepare', 'imu_publish_touch'))
CALLBACK_STAGES = {'/sim/scan': SCAN_STAGES, '/sim/imu': IMU_STAGES}


def _integer(value, maximum=2**63-1, minimum=0):
    return value if isinstance(value, int) and not isinstance(value, bool) and minimum <= value <= maximum else None


def _raw(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0 else None


def _read_clock(clock, validate):
    try:
        return validate(clock())
    except Exception:
        return None


def _source_stamp(topic, message):
    try:
        stamp = message.clock if topic == '/clock' else message.header.stamp
        sec, ns = _integer(stamp.sec), _integer(stamp.nanosec, maximum=999_999_999)
        return _integer(sec*1_000_000_000+ns) if sec is not None and ns is not None else None
    except Exception:
        return None


def _info_value(info, name, *, sequence=False):
    try:
        value = info.get(name) if isinstance(info, dict) else getattr(info, name, None)
        # RMW's unsupported sequence sentinel and absent/zero timestamps are
        # unknown, not real receipt times or proof of a continuous stream.
        return _integer(value, maximum=2**64-2 if sequence else 2**63-1,
                        minimum=0 if sequence else 1)
    except Exception:
        return None


class CallbackTiming:
    def __init__(self, *, raw_clock=raw_now, cpu_clock=time.thread_time_ns,
                 realtime_clock=time.time_ns, capacity=128, topics=DEFAULT_TOPICS):
        topics = tuple(topics)
        if (not isinstance(capacity, int) or isinstance(capacity, bool) or not 1 <= capacity <= 128
                or not 1 <= len(topics) <= 7
                or any(not isinstance(topic, str) or not 1 <= len(topic) <= 128 for topic in topics)
                or len(set(topics)) != len(topics)):
            raise ValueError('invalid_callback_timing_bounds')
        self.raw_clock, self.cpu_clock, self.realtime_clock = raw_clock, cpu_clock, realtime_clock
        self.thread_id = threading.get_ident()
        self.events = deque(maxlen=capacity)
        self.recorded = self.invalid_clock_records = self.diagnostic_errors = 0
        self.frozen = None
        self.active_scan = None
        self.topics = {topic: {'count': 0, 'max_elapsed_s': 0., 'max_thread_cpu_s': 0.,
                              'last_source_stamp_ns': None, 'last_entry_raw_s': None}
                       for topic in topics}

    def wrap(self, topic, callback):
        # Do not use functools.wraps: rclpy inspects this signature and must see
        # two REQUIRED arguments to supply MessageInfo alongside the message.
        def observed(message, info):
            return self.measure_call(topic, callback, message, info)
        observed._observed_callback = callback
        return observed

    def measure_call(self, topic, callback, message, info):
        if threading.get_ident() != self.thread_id or not isinstance(topic, str) or topic not in self.topics:
            return callback(message)
        if self.active_scan is not None:
            # An unexpected nested callback remains part of the outer body,
            # but cannot attach its stages to that body's observation.
            previous=self.active_scan;self.active_scan=None
            try:return callback(message)
            finally:self.active_scan=previous
        entry = _read_clock(self.raw_clock, _raw)
        cpu_start = _read_clock(self.cpu_clock, _integer)
        real_entry = _read_clock(self.realtime_clock, _integer)
        # Read only the original header and tiny numeric metadata. No message
        # body, image, ranges, pose or control credentials are retained.
        source_stamp = _source_stamp(topic, message)
        metadata = {
            'rmw_source_ns': _info_value(info, 'source_timestamp'),
            'rmw_received_ns': _info_value(info, 'received_timestamp'),
            'publication_seq': _info_value(info, 'publication_sequence_number', sequence=True),
            'reception_seq': _info_value(info, 'reception_sequence_number', sequence=True)}
        failed = False
        scan_context={'stages':{},'in_stage':False,'allowed':CALLBACK_STAGES[topic]} if topic in CALLBACK_STAGES else None
        self.active_scan=scan_context
        try:
            return callback(message)
        except BaseException:
            failed = True
            raise
        finally:
            # An observation failure must never replace a callback exception
            # or cause a callback to execute for a second time.
            try:
                self._finish(topic, entry, cpu_start, real_entry, source_stamp, metadata, failed,
                             scan_context['stages'] if scan_context is not None else None)
            except Exception:
                self.diagnostic_errors += 1
            finally:self.active_scan=None

    def measure_stage(self, name, function, *args, **kwargs):
        """Bounded non-overlapping scan/IMU phases; never retain results.

        Direct calls, other callbacks, other threads, and nested phase helpers
        execute normally with no diagnostic work. No timestamps drive control.
        """
        current=self.active_scan
        if (threading.get_ident()!=self.thread_id or current is None or current['in_stage']
                or not isinstance(name,str) or name not in current['allowed']):
            return function(*args,**kwargs)
        entry=_read_clock(self.raw_clock,_raw);cpu_start=_read_clock(self.cpu_clock,_integer)
        current['in_stage']=True
        try:return function(*args,**kwargs)
        finally:
            try:
                end=_read_clock(self.raw_clock,_raw);cpu_end=_read_clock(self.cpu_clock,_integer)
                if entry is not None and end is not None and end>=entry and cpu_start is not None and cpu_end is not None and cpu_end>=cpu_start:
                    stage=current['stages'].setdefault(name,{'calls':0,'entry_raw_s':entry,
                                                            'exit_raw_s':end,'elapsed_s':0.,'thread_cpu_s':0.})
                    stage['calls']+=1;stage['exit_raw_s']=end
                    stage['elapsed_s']+=end-entry;stage['thread_cpu_s']+=(cpu_end-cpu_start)*1e-9
                else:self.invalid_clock_records+=1
            except Exception:self.diagnostic_errors+=1
            finally:current['in_stage']=False

    def _finish(self, topic, entry, cpu_start, real_entry, source_stamp, metadata, failed, scan_stages):
        end = _read_clock(self.raw_clock, _raw)
        cpu_end = _read_clock(self.cpu_clock, _integer)
        elapsed = end-entry if entry is not None and end is not None and end >= entry else None
        cpu = (cpu_end-cpu_start)*1e-9 if cpu_start is not None and cpu_end is not None and cpu_end >= cpu_start else None
        if elapsed is None or cpu is None:
            self.invalid_clock_records += 1
        self.recorded += 1
        event = {'sequence': self.recorded, 'topic': topic, 'source_stamp_ns': source_stamp,
                 'entry_raw_s': entry, 'exit_raw_s': end, 'elapsed_s': elapsed,
                 'thread_cpu_s': cpu, 'entry_realtime_ns': real_entry,
                 'callback_raised': failed, **metadata}
        if scan_stages is not None:
            event['imu_subphases' if topic=='/sim/imu' else 'scan_subphases']=scan_stages
        self.events.append(event)
        summary = self.topics[topic]
        summary['count'] += 1
        if elapsed is not None: summary['max_elapsed_s'] = max(summary['max_elapsed_s'], elapsed)
        if cpu is not None: summary['max_thread_cpu_s'] = max(summary['max_thread_cpu_s'], cpu)
        summary['last_source_stamp_ns'] = source_stamp
        summary['last_entry_raw_s'] = entry

    def snapshot(self):
        """Small healthy status; the ring is never serialized on this path."""
        return {'schema': SCHEMA, 'duration_clock': 'CLOCK_MONOTONIC_RAW',
                'cpu_clock': 'time.thread_time_ns', 'cpu_scope': 'adapter_main_thread_only',
                'entry_realtime_clock': 'CLOCK_REALTIME', 'rmw_clock': 'opaque_middleware_nanoseconds',
                'callback_entry_after_take_and_deserialization': True,
                'capacity': self.events.maxlen, 'recorded': self.recorded,
                'invalid_clock_records': self.invalid_clock_records,
                'diagnostic_errors': self.diagnostic_errors, 'frozen': self.frozen is not None,
                'topics': {topic: dict(summary) for topic, summary in self.topics.items()}}

    def freeze(self, reason):
        """Caller invokes on the first unexpected stop; later calls cannot replace it.

        The detached result is for the existing first-failure record only.
        A full ring covers a variable time span, not necessarily the200ms wait.
        """
        if threading.get_ident() != self.thread_id:
            return None
        if self.frozen is None:
            events = list(self.events)
            first, last = (events[0]['entry_raw_s'], events[-1]['exit_raw_s']) if events else (None, None)
            coverage = last-first if first is not None and last is not None and last >= first else None
            reason = reason if isinstance(reason, str) and 1 <= len(reason) <= 96 and all(c.isascii() and (c.isalnum() or c == '_') for c in reason) else 'unexpected_stop'
            self.frozen = {**self.snapshot(), 'frozen': True, 'reason': reason,
                           'coverage_raw_s': coverage, 'overwritten': max(0, self.recorded-len(events)),
                           'events': copy.deepcopy(events)}
        return copy.deepcopy(self.frozen)
