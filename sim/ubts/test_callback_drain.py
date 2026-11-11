"""Deterministic dispatch ordering, bounded saturation and idle behavior."""
from types import SimpleNamespace

import pytest

from callback_drain import drain_ready, idle_after_empty


class Executor:
    def __init__(self, clock, *, cost=.0001, saturated=True):
        self.clock=clock;self.cost=cost;self.saturated=saturated
        self.generation=0;self.timeouts=[];self.topics=[];self.position=0
        self.previous_timeout=None

    def spin_once(self, *, timeout_sec):
        self.timeouts.append(timeout_sec)
        # Model rclpy invalidating its ready iterator on changed arguments.
        if timeout_sec!=self.previous_timeout:self.position=0
        self.previous_timeout=timeout_sec
        self.clock[0]+=self.cost
        if self.saturated:
            self.topics.append(('/clock','/sim/imu','/sim/wheel_odom')[self.position%3])
            self.position+=1;self.generation+=1


def test_constant_arguments_preserve_ready_iterator_fairness_across_bursts():
    clock=[0.];executor=Executor(clock)
    for _ in range(3):
        result=drain_ready(executor,lambda:executor.generation,lambda:None,
                           raw_clock=lambda:clock[0],max_callbacks=2)
        assert result['callbacks']==2 and not result['empty']
    assert executor.topics==['/clock','/sim/imu','/sim/wheel_odom']*2
    assert executor.timeouts==[0]*6


def test_saturated_source_stops_after_count_limit_and_checks_each_callback():
    clock=[0.];executor=Executor(clock);checks=[]
    result=drain_ready(executor,lambda:executor.generation,
                       lambda:checks.append(executor.generation),raw_clock=lambda:clock[0])
    assert result['callbacks']==16 and len(executor.timeouts)==16
    assert checks==list(range(17))


def test_raw_budget_prevents_starting_more_callbacks_but_cannot_preempt_one():
    clock=[0.];executor=Executor(clock,cost=.005)
    result=drain_ready(executor,lambda:executor.generation,lambda:None,raw_clock=lambda:clock[0])
    assert result['callbacks']==2 and result['elapsed_s']==pytest.approx(.010)
    executor=Executor(clock,cost=.25);checks=[]
    result=drain_ready(executor,lambda:executor.generation,lambda:checks.append(clock[0]),raw_clock=lambda:clock[0])
    assert result['callbacks']==1 and checks[-1]-checks[0]==pytest.approx(.25)


def test_empty_burst_yields_one_millisecond_with_guards_on_both_sides():
    clock=[0.];executor=Executor(clock,saturated=False);calls=[]
    result=drain_ready(executor,lambda:executor.generation,lambda:calls.append('guard'),raw_clock=lambda:clock[0])
    idle_after_empty(result,lambda:calls.append('guard'),sleep=lambda delay:calls.append(delay))
    assert result['empty'] and result['callbacks']==0 and executor.timeouts==[0]
    assert calls==['guard','guard','guard',.001,'guard']
    idle_after_empty({'empty':True,'callbacks':1},lambda:pytest.fail('unexpected idle'),sleep=lambda _:pytest.fail('sleep'))


def test_callback_exception_propagates_without_reexecution():
    calls=[];original=ValueError('callback failure')
    def spin(**kwargs):calls.append(kwargs);raise original
    with pytest.raises(ValueError) as caught:
        drain_ready(SimpleNamespace(spin_once=spin),lambda:0,lambda:None,raw_clock=lambda:0.)
    assert caught.value is original and calls==[{'timeout_sec':0}]


def test_missing_raw_clock_fails_closed_before_dispatch():
    with pytest.raises(RuntimeError,match='dispatch_raw_clock_unavailable'):
        drain_ready(SimpleNamespace(spin_once=lambda **kw:pytest.fail('dispatched')),lambda:0,lambda:None,
                    raw_clock=lambda:None)


def test_observer_preserves_order_timeout_count_and_idle_sleep():
    clock=[0.];executor=Executor(clock,saturated=False);calls=[]
    def observe(name, function, *args, **kwargs):
        calls.append(('begin',name))
        try:return function(*args,**kwargs)
        finally:calls.append(('end',name))
    def guard():calls.append('guard')
    result=drain_ready(executor,lambda:executor.generation,guard,raw_clock=lambda:clock[0],observer=observe)
    idle_after_empty(result,guard,sleep=lambda delay:calls.append(('sleep',delay)),observer=observe)
    assert result['empty'] and result['callbacks']==0 and executor.timeouts==[0]
    assert calls==[
        ('begin','drain_watchdog'),'guard',('end','drain_watchdog'),
        ('begin','executor_spin_once'),('end','executor_spin_once'),
        ('begin','drain_watchdog'),'guard',('end','drain_watchdog'),
        ('begin','drain_watchdog'),'guard',('end','drain_watchdog'),
        ('begin','idle_sleep'),('sleep',.001),('end','idle_sleep'),
        ('begin','drain_watchdog'),'guard',('end','drain_watchdog')]


@pytest.mark.parametrize('failure',['watchdog','spin','sleep'])
def test_observer_records_original_exception_without_extra_dispatch(failure):
    calls=[];error=ValueError('original failure')
    def observe(name, function, *args, **kwargs):
        try:return function(*args,**kwargs)
        finally:calls.append(name)
    def guard():
        if failure=='watchdog':raise error
    def spin(**kwargs):
        assert kwargs=={'timeout_sec':0}
        if failure=='spin':raise error
    def sleep(delay):
        assert delay==.001
        raise error
    with pytest.raises(ValueError) as caught:
        result=drain_ready(SimpleNamespace(spin_once=spin),lambda:0,guard,raw_clock=lambda:0.,observer=observe)
        idle_after_empty(result,guard,sleep=sleep,observer=observe)
    assert caught.value is error
    assert calls==({'watchdog':['drain_watchdog'],
                    'spin':['drain_watchdog','executor_spin_once'],
                    'sleep':['drain_watchdog','executor_spin_once','drain_watchdog','drain_watchdog','idle_sleep']}[failure])
