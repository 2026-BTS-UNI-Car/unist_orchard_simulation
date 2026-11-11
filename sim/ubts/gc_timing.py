"""Bounded, observational CPython cyclic-GC intervals; never a control clock.

Python 3.12 documents gc.callbacks start/stop phases and generation, collected,
uncollectable counters: https://docs.python.org/3.12/library/gc.html#gc.callbacks
This hook does not request collections, inspect objects, or change GC policy.
RAW includes descheduling/finalizer waits; CPU belongs to the collecting thread,
which may be the snapshot writer rather than the adapter. Overlap is evidence of
timing only, not proof that GC held the GIL throughout or caused a deadline miss.
"""
from collections import deque
import gc
import math
import threading
import time

from scripts.sim_clock import raw_now


def _integer(value, maximum=2**63-1):
    return value if type(value) is int and 0 <= value <= maximum else None


def _raw(value):
    return value if type(value) in (int, float) and math.isfinite(value) and value >= 0 else None


class GCTiming:
    """One collector slot,64 ring records and at most3 retained generation peaks.

    CPython serializes cyclic collections. No mutex is acquired in the callback:
    collection can interrupt a thread already holding an application lock. The
    hook never releases the GIL deliberately. Snapshots copy scalar dictionaries
    and a bounded deque; their counters are diagnostic, not transactional state.
    """
    def __init__(self, *, raw_clock=raw_now, cpu_clock=time.thread_time_ns,
                 capacity=64, callbacks=None):
        if type(capacity) is not int or not 1 <= capacity <= 64:
            raise ValueError('invalid_gc_timing_capacity')
        self.raw_clock=raw_clock;self.cpu_clock=cpu_clock
        self.callbacks=gc.callbacks if callbacks is None else callbacks
        self.main_thread_id=threading.get_ident()
        self.events=deque(maxlen=capacity);self.active=None;self.frozen=None
        self.recorded=0;self.invalid_records=0;self.diagnostic_errors=0
        self.reentrant_callbacks=0;self._in_callback=False;self.installed=False
        self.generations=[{'count':0,'collected':0,'uncollectable':0,
                           'max_elapsed_s':0.,'max_thread_cpu_s':0.,'largest_raw_event':None} for _ in range(3)]
        # Keep one exact bound-method object for identity-safe removal.
        self.callback=self._callback

    def install(self):
        try:
            if not any(callback is self.callback for callback in self.callbacks):
                self.callbacks.append(self.callback)
            self.installed=True
        except Exception:
            self.diagnostic_errors+=1

    def close(self):
        try:
            for index in range(len(self.callbacks)-1,-1,-1):
                if self.callbacks[index] is self.callback:
                    del self.callbacks[index]
            self.installed=False;self.active=None
        except Exception:
            self.diagnostic_errors+=1

    def _callback(self, phase, info):
        if self._in_callback:
            self.reentrant_callbacks+=1
            return
        self._in_callback=True
        try:
            if phase not in ('start','stop') or type(info) is not dict:
                self.invalid_records+=1;self.active=None
                return
            generation=_integer(info.get('generation'),2)
            stamp=_raw(self.raw_clock());cpu=_integer(self.cpu_clock())
            thread_id=threading.get_ident();native_id=threading.get_native_id()
            if generation is None or stamp is None or cpu is None:
                self.invalid_records+=1;self.active=None
                return
            if phase=='start':
                if self.active is not None:self.invalid_records+=1
                self.active={'generation':generation,'started_raw_s':stamp,
                             'started_thread_cpu_ns':cpu,'thread_id':thread_id,
                             'native_thread_id':native_id,'adapter_thread':thread_id==self.main_thread_id}
                return
            start=self.active;self.active=None
            collected=_integer(info.get('collected'));uncollectable=_integer(info.get('uncollectable'))
            if (start is None or start['generation']!=generation or start['thread_id']!=thread_id
                    or start['native_thread_id']!=native_id or collected is None or uncollectable is None):
                self.invalid_records+=1
                return
            elapsed=_raw(stamp-start['started_raw_s']);cpu_ns=_integer(cpu-start['started_thread_cpu_ns'])
            if elapsed is None or cpu_ns is None:
                self.invalid_records+=1
                return
            self.recorded+=1
            record={key:value for key,value in start.items() if key!='started_thread_cpu_ns'}
            record.update(sequence=self.recorded,ended_raw_s=stamp,elapsed_s=elapsed,
                          thread_cpu_s=cpu_ns*1e-9,collected=collected,uncollectable=uncollectable)
            # Published rows are immutable; eviction never mutates old snapshots.
            self.events.append(record)
            old=self.generations[generation]
            self.generations[generation]={'count':old['count']+1,'collected':old['collected']+collected,
                'uncollectable':old['uncollectable']+uncollectable,
                'max_elapsed_s':max(old['max_elapsed_s'],elapsed),
                'max_thread_cpu_s':max(old['max_thread_cpu_s'],record['thread_cpu_s']),
                'largest_raw_event':record if old['largest_raw_event'] is None or elapsed>old['max_elapsed_s']
                                    else old['largest_raw_event']}
        except Exception:
            # Diagnostics may not interrupt the allocating callback or replace
            # an original control exception while GC runs during unwinding.
            self.diagnostic_errors+=1;self.active=None
        finally:
            self._in_callback=False

    @staticmethod
    def _generation_snapshot(value):
        peak=value['largest_raw_event']
        return {**value,'largest_raw_event':dict(peak) if peak is not None else None}

    def snapshot(self):
        """Small healthy status: one scalar peak record per generation, no ring."""
        active=self.active
        return {'schema':'ubts-gc-timing-v1','duration_clock':'CLOCK_MONOTONIC_RAW',
                'cpu_clock':'time.thread_time_ns','cpu_scope':'collecting_thread',
                'capacity':self.events.maxlen,'installed':self.installed,
                'recorded':self.recorded,'overwritten':max(0,self.recorded-self.events.maxlen),
                'invalid_records':self.invalid_records,'diagnostic_errors':self.diagnostic_errors,
                'reentrant_callbacks':self.reentrant_callbacks,
                'generations':[self._generation_snapshot(value) for value in self.generations],
                'active_collection':dict(active) if active is not None else None}

    def freeze(self, reason):
        """Detached first-failure ring, captured only by the adapter thread."""
        if threading.get_ident()!=self.main_thread_id:return None
        if self.frozen is None:
            events=[dict(event) for event in tuple(self.events)]
            self.frozen={**self.snapshot(),'reason':reason[:96] if type(reason) is str else None,
                'events':events,'coverage_raw_s':events[-1]['ended_raw_s']-events[0]['started_raw_s'] if events else 0.}
        # Rows contain only scalars; the two other nested containers are explicit.
        return {**self.frozen,'events':[dict(event) for event in self.frozen['events']],
                'generations':[self._generation_snapshot(value) for value in self.frozen['generations']],
                'active_collection':dict(self.frozen['active_collection']) if self.frozen['active_collection'] is not None else None}
