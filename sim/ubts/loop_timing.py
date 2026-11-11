"""Bounded observational timings for the SIM adapter, never a control clock.

RAW elapsed includes OS scheduling/descheduling. Main-thread CPU is recorded
separately; their difference is not exclusively lock or I/O wait. The spin phase
includes bounded nonblocking dispatch, watchdogs and a conditional1ms idle yield.
Their subphases are separate; safety lock/body timings nest inside watchdogs.
Context is copied only on slow loops or the first unexpected autonomy stop;
there is no per-callback logging or I/O.
"""
from collections import deque
import copy
import math
import threading
import time

from scripts.sim_clock import raw_now


def number(value):
    return value if isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) else None


def text(value):
    return value[:96] if isinstance(value,str) else None


def capture_context(node, *, include_receipts=False):
    """Explicit allowlist: never copy control tokens, leases or owner objects."""
    now=time.monotonic()
    auto=node.autonomy;estimator=node.planar_odometry
    yaw=auto.yaw_measurement or {};pose=auto.odom or {}
    def age(stamp):
        return number(now-stamp) if number(stamp) is not None else None
    value={'sim_s':number(node.sim_time),
           'arrival_age_clock':'CLOCK_MONOTONIC',
           'clock_arrival_age_s':age(node.clock_seen),
           'imu_sim_stamp':number(yaw.get('sim_stamp')),
           'imu_arrival_age_s':age(yaw.get('wall_stamp')),
           'pose_sim_stamp':number(pose.get('sim_stamp')),
           'pose_arrival_age_s':age(pose.get('wall_stamp')),
           'pending_wheel_sim_stamp':number(estimator.pending_wheel[0]) if estimator.pending_wheel else None,
           'pending_wait_age_s':age(estimator.pending_since),
           'autonomy_state':text(auto.mission.state),'autonomy_reason':text(auto.mission.reason),
           'wait_reason':text(auto.sensor_wait_reason),'wait_age_s':age(auto.sensor_wait_since),
           'planning_deferred_due_ticks':number(getattr(auto,'planning_deferrals',0)),
           'last_due_tick_deferred':getattr(auto,'last_planning_deferred',False) is True,
           'estimator_fault':text(estimator.fault),
           'input_counts':{'occupied_cells':len(getattr(auto,'occupied',())),
                           'path_points':len(getattr(auto.mission,'path',())),
                           'scan_points':len(getattr(auto,'scan',{}).get('ranges',()))}}
    if include_receipts:
        value['receipt_clock']='CLOCK_MONOTONIC'
        for name in ('wheel_receipts','imu_receipts'):
            value[name]=[{key:number(row.get(key)) for key in ('stamp','receipt_monotonic')}
                         for row in list(getattr(estimator,name,()))[-8:]]
        value['feedback_ages']={key:number(auto.yaw_status.get(key)) for key in
                               ('measurement_sim_stamp','control_sim_stamp','measurement_sim_age_s',
                                'measurement_wall_age_s','pose_sim_age_s','pose_wall_age_s')}
        observer=getattr(node,'callback_timing',None)
        if observer is not None and auto.mission.reason not in LoopTiming.EXPECTED_STOPS:
            try:value['callback_timing_at_stop']=observer.freeze(auto.mission.reason)
            except Exception as error:
                # A diagnostic failure must not replace the safety rejection
                # or discard its already captured sensor receipts/context.
                value['callback_timing_error']=type(error).__name__
        collector=getattr(node,'gc_timing',None)
        if collector is not None and auto.mission.reason not in LoopTiming.EXPECTED_STOPS:
            try:value['gc_timing_at_stop']=collector.freeze(auto.mission.reason)
            except Exception as error:value['gc_timing_error']=type(error).__name__
    return value


class LoopTiming:
    EXPECTED_STOPS=frozenset(('planned_lanes_completed','autonomy_operator_stop',
                              'operator_stop','autonomy_integration_end','manual_override'))

    DISPATCH_STAGES=frozenset(('executor_spin_once','drain_watchdog','idle_sleep',
                              'safety_lock_wait','safety_body'))
    STAGES=frozenset(('step_lock_wait','snapshot_lock_wait','map_ready','mission_step',
                     'yaw_track','corrected_guard','drive_wheels')) | DISPATCH_STAGES

    def __init__(self, clock=raw_now, *, cpu_clock=time.thread_time_ns, capacity=16, slow_s=.020):
        if not 1<=capacity<=16 or not math.isfinite(slow_s) or slow_s<=0:
            raise ValueError('invalid_loop_timing_bounds')
        self.clock=clock;self.cpu_clock=cpu_clock;self.thread_id=threading.get_ident()
        self.slow_s=slow_s;self.slow=deque(maxlen=capacity)
        self.iterations=0;self.invalid_iterations=0;self.previous_end=None;self.previous_active=False
        self.stage_observation_errors=0
        self.first_failure=None
        self.maxima={key:0. for key in ('loop_s','between_iterations_s','spin_s','step_s','snapshot_s')}
        self.cpu_maxima={key:0. for key in ('loop_s','spin_s','step_s','snapshot_s')}
        self.current=None

    def begin(self, *, active, wait_since=None, wait_reason=None):
        self.current={'start':self.clock(),'cpu_start':self.cpu_clock(),'active_before':bool(active),
                      'subphases':{},
                      'wait_at_start':{'since_monotonic':number(wait_since),'reason':text(wait_reason)}}

    def after_spin(self):
        self.current['spin_end']=self.clock()
        self.current['cpu_spin_end']=self.cpu_clock()

    def after_step(self, *, active, context):
        self.current['step_end']=self.clock()
        self.current['cpu_step_end']=self.cpu_clock()
        # Snapshot filesystem I/O can yield to the HTTP cleanup thread. Copy
        # the rejection now, before cleanup can replace it with a normal STOP.
        transitioned=(self.current['active_before'] or self.previous_active) and not active
        if transitioned and self.first_failure is None:
            self.current['stop_context']=self._context(context,True)
            # Retain the callback ring once, outside the event/slow-loop ring:
            # these events are otherwise copied into two first-failure views.
            self.current['callback_timing_at_stop']=self.current['stop_context'].pop('callback_timing_at_stop',None)
            self.current['gc_timing_at_stop']=self.current['stop_context'].pop('gc_timing_at_stop',None)

    @staticmethod
    def _context(callback, receipts):
        try:return callback(receipts)
        except Exception as error:return {'diagnostic_error':type(error).__name__}

    def start_stage(self, name):
        # IPC threads may call the same control object. Their timing must not
        # contaminate the adapter thread's iteration or CPU measurements.
        if threading.get_ident()!=self.thread_id or self.current is None or name not in self.STAGES:
            return None
        try:return self.clock(),self.cpu_clock()
        except Exception:
            self.stage_observation_errors+=1
            return None

    def end_stage(self, name, started):
        if started is None or self.current is None or threading.get_ident()!=self.thread_id:return
        try:
            ended=self.clock();cpu_ended=self.cpu_clock()
            if name not in self.STAGES or any(number(v) is None for v in (*started,ended,cpu_ended)):return
            elapsed=ended-started[0];cpu=(cpu_ended-started[1])*1e-9
            if elapsed<0 or cpu<0:return
            stage=self.current['subphases'].setdefault(name,{'calls':0,'elapsed_s':0.,'thread_cpu_s':0.})
            stage['calls']+=1;stage['elapsed_s']+=elapsed;stage['thread_cpu_s']+=cpu
            # One maximum interval per dispatch stage correlates with the existing
            # callback ring; there is no growing per-call trace or message content.
            if name in self.DISPATCH_STAGES and elapsed>stage.get('max_call',{}).get('elapsed_s',-1.):
                stage['max_call']={'started_raw_s':started[0],'ended_raw_s':ended,
                                   'elapsed_s':elapsed,'thread_cpu_s':cpu}
        except Exception:
            # Optional diagnostics must not skip a watchdog or replace its
            # exception while the control stack unwinds through finally.
            self.stage_observation_errors+=1

    def measure_call(self, name, function, *args, **kwargs):
        started=self.start_stage(name)
        try:return function(*args,**kwargs)
        finally:self.end_stage(name,started)

    def finish(self, *, active, had_snapshot, context):
        """context(include_receipts) is lazy and must be the allowlisted helper."""
        current=self.current;end=self.clock();cpu_end=self.cpu_clock();self.iterations+=1
        self.current=None
        transitioned=((current['active_before'] or self.previous_active) and not active) or 'stop_context' in current
        self.previous_active=bool(active)
        values=[current.get(key) for key in ('start','spin_end','step_end')]+[end]
        if any(number(value) is None for value in values) or values!=sorted(values):
            self.invalid_iterations+=1;self.previous_end=None
            return
        start,spin_end,step_end,end=values
        durations={'loop_s':end-start,'between_iterations_s':max(0.,start-self.previous_end) if self.previous_end is not None else 0.,
                   'spin_s':spin_end-start,'step_s':step_end-spin_end,'snapshot_s':end-step_end if had_snapshot else 0.}
        cpu_values=[current.get(key) for key in ('cpu_start','cpu_spin_end','cpu_step_end')]+[cpu_end]
        cpu_durations=None
        if all(number(value) is not None for value in cpu_values) and cpu_values==sorted(cpu_values):
            cs,ci,ct,ce=cpu_values
            cpu_durations={'loop_s':(ce-cs)*1e-9,'spin_s':(ci-cs)*1e-9,
                           'step_s':(ct-ci)*1e-9,'snapshot_s':(ce-ct)*1e-9 if had_snapshot else 0.}
            for key,value in cpu_durations.items():self.cpu_maxima[key]=max(self.cpu_maxima[key],value)
        self.previous_end=end
        for key,value in durations.items():self.maxima[key]=max(self.maxima[key],value)
        is_slow=max(durations.values())>=self.slow_s
        if not is_slow and not (transitioned and self.first_failure is None):return
        observed=current.get('stop_context')
        if observed is None:observed=self._context(context,False)
        event={'iteration':self.iterations,'started_raw_s':start,'ended_raw_s':end,
               **durations,'thread_cpu_s':cpu_durations,'subphases':copy.deepcopy(current['subphases']),
               'wait_at_iteration_start':current['wait_at_start'],'context':observed}
        if is_slow:self.slow.append(event)
        reason=observed.get('autonomy_reason')
        if transitioned and self.first_failure is None and reason not in self.EXPECTED_STOPS:
            self.first_failure={'reason':reason,'iteration':self.iterations,
                                'event':copy.deepcopy(event),'recent_slow_iterations':copy.deepcopy(list(self.slow)),
                                'callback_timing':current.get('callback_timing_at_stop'),
                                'gc_timing':current.get('gc_timing_at_stop')}

    def snapshot(self, *, detached=True):
        """Return detached records by default; callers may safely retain/edit them.

        detached=False borrows immutable-after-publication event dictionaries.
        It is only for immediate JSON serialization on the adapter main thread:
        do not retain, mutate, or pass borrowed records to another thread. The
        loop cannot append/freeze another event until that serialization ends.
        """
        if not detached and threading.get_ident()!=self.thread_id:
            raise RuntimeError('borrowed_loop_snapshot_requires_adapter_thread')
        records=list(self.slow)
        failure=self.first_failure
        if detached:
            records=copy.deepcopy(records)
            failure=copy.deepcopy(failure)
        return {'schema':'ubts-adapter-loop-timing-v2','duration_clock':'CLOCK_MONOTONIC_RAW',
                'cpu_clock':'time.thread_time_ns','cpu_scope':'adapter_main_thread_only',
                'elapsed_includes_os_scheduling':True,'spin_includes_wait_s':0.,
                'dispatch_budget_raw_s':.008,'dispatch_max_callbacks':16,'empty_idle_sleep_s':.001,
                'slow_threshold_s':self.slow_s,'capacity':self.slow.maxlen,
                'iterations':self.iterations,'invalid_clock_iterations':self.invalid_iterations,
                'stage_observation_errors':self.stage_observation_errors,
                'max_elapsed_s':dict(self.maxima),'max_thread_cpu_s':dict(self.cpu_maxima),
                'subphases_overlap_parent_phase':True,
                'dispatch_subphase_nesting':{'spin_s':['executor_spin_once','drain_watchdog','idle_sleep'],
                                             'drain_watchdog':['safety_lock_wait','safety_body']},
                'recent_slow_iterations':records,
                'first_autonomy_failure':failure}

    def writer_snapshot(self):
        """Main-thread capture for the single read-only snapshot writer.

        New outer dictionaries/lists and maxima copies own this generation.
        Slow-event and first-failure trees are never mutated after publication:
        append/eviction replaces references, and the failure is first-only.
        The worker may retain and JSON-encode these records, but must never edit
        them. This does not relax the detached default for ordinary callers.
        """
        return self.snapshot(detached=False)
