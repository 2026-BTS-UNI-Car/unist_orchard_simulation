"""Bounded, single-threaded dispatch through rclpy's public spin_once API.

Keep timeout arguments identical: changing them replaces rclpy's cached ready
iterator and can repeatedly select the first busy subscription. The RAW budget
limits dispatch selection, not an executing callback or OS descheduling.
"""
import math
import time

from scripts.sim_clock import raw_now


def _observe(observer, name, function, *args, **kwargs):
    if observer is None:
        return function(*args, **kwargs)
    return observer(name, function, *args, **kwargs)


def drain_ready(executor, generation, watchdog, *, raw_clock=raw_now,
                max_callbacks=16, budget_s=.008, observer=None):
    if (isinstance(max_callbacks,bool) or not isinstance(max_callbacks,int) or not 1<=max_callbacks<=16
            or isinstance(budget_s,bool) or not isinstance(budget_s,(int,float))
            or not math.isfinite(budget_s) or not 0<budget_s<=.008):
        raise ValueError('invalid_dispatch_bounds')
    def now():
        value=raw_clock()
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise RuntimeError('dispatch_raw_clock_unavailable')
        return value
    started = now()
    callbacks = 0
    empty = False
    _observe(observer, 'drain_watchdog', watchdog)
    for _ in range(max_callbacks):
        if now()-started >= budget_s:
            break
        before = generation()
        _observe(observer, 'executor_spin_once', executor.spin_once, timeout_sec=0)
        _observe(observer, 'drain_watchdog', watchdog)
        if generation() == before:
            empty = True
            break
        callbacks += 1
    return {'callbacks': callbacks, 'empty': empty,
            'elapsed_s': now()-started}


def idle_after_empty(drain, watchdog, *, sleep=time.sleep, observer=None):
    """Yield at most one requested millisecond; recheck safety after waking."""
    if drain['empty'] and drain['callbacks'] == 0:
        _observe(observer, 'drain_watchdog', watchdog)
        _observe(observer, 'idle_sleep', sleep, .001)
        _observe(observer, 'drain_watchdog', watchdog)
