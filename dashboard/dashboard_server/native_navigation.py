"""Latest-only isolated SIM observer, independent of registration and HTTP.

No control transport is imported. Expensive perception cannot block mapping;
replaced observations are counted and source gaps invalidate confirmation.
"""
import copy
import hashlib
import math
import multiprocessing
import os
from pathlib import Path
import threading
import time

from scripts.sim_clock import raw_now, freshness
from native_navigation import NativeNavigationObserver, SimNavigationProfile


def _observation_process(profile, connection):
    observer=NativeNavigationObserver(profile);generation=None
    try:
        connection.send(('ready',None))
        while True:
            request=connection.recv()
            if request is None:return
            token,item=request
            if generation!=token:observer.reset();generation=token
            scan,result,epoch,receipt_ns,source_ns,gyro=item
            try:
                now=raw_now()
                if now is None:raise ValueError('independent_receipt_clock_unavailable')
                state=observer.observe(scan,result,epoch=epoch,receipt_ns=receipt_ns,
                    receipt_now_ns=round(now*1e9),source_stamp_ns=source_ns,yaw_rate_rad_s=gyro)
            except Exception as exc:state=observer.invalidate('observer_exception:'+type(exc).__name__)
            connection.send((token,state))
    except (EOFError,OSError):return
    finally:connection.close()


def modeled_profile():
    import yaml
    path = Path(__file__).resolve().parents[2]/'sim/ubts/rover.yaml'
    raw = path.read_bytes(); config = yaml.safe_load(raw)
    vehicle, laser = config['vehicle'], config['lidar']
    return SimNavigationProfile(vehicle['original_length_m'], vehicle['original_width_m'],
        laser['x_m'], laser['y_m'], 0., .15, .15, vehicle['acceleration_m_s2'],
        True, hashlib.sha256(raw).hexdigest())


class NativeNavigationWorker:
    def __init__(self, *, start_worker=True, profile=None, clock=raw_now, separate_process=None):
        self.observer = NativeNavigationObserver(profile or modeled_profile())
        self.clock = clock
        self.lock = threading.Lock(); self.wake = threading.Event()
        self.stopping = threading.Event(); self.pending = None; self.generation = 0
        self.dropped = 0; self.processed = 0; self.processing = False
        self.receipt = None; self.state = self.observer._denied('waiting_native_mapping')
        self.thread = None
        self.process=None;self.pipe=None
        if separate_process is None:separate_process=os.environ.get('SIM_NAVIGATION_PROCESS')=='1'
        if separate_process:
            context=multiprocessing.get_context('spawn');self.pipe,child=context.Pipe()
            self.process=context.Process(target=_observation_process,args=(self.observer.profile,child),
                                         daemon=True,name='native-navigation-calculation')
            self.process.start();child.close()
            if not self.pipe.poll(5.) or self.pipe.recv()!=('ready',None):
                self.process.terminate();self.process.join(3)
                raise RuntimeError('observation_process_startup_failed')
        if start_worker:
            self.thread = threading.Thread(target=self._run, daemon=True, name='native-navigation-observer')
            self.thread.start()

    def invalidate(self, reason):
        # A worker may still finish a calculation, but cannot commit it after
        # Stop/reset/session loss. Only that worker mutates its observer state.
        with self.lock:
            self.generation += 1; self.pending = None; self.receipt = None
            self.state = self.observer._denied(reason)
        self.wake.set()

    def submit(self, scan, result, *, epoch, row, yaw_rate_rad_s=None):
        age = freshness(row, .2)
        if not age['fresh']:
            self.invalidate('native_scan_receipt_expired'); return
        stamp = scan.get('header', {}).get('stamp', {})
        if (type(stamp.get('sec')) is not int or type(stamp.get('nanosec')) is not int
                or stamp['sec'] < 0 or not 0 <= stamp['nanosec'] < 10**9):
            self.invalidate('invalid_native_stamp'); return
        item = (copy.deepcopy(scan), copy.deepcopy(result), epoch,
                round(row['updated_monotonic_raw']*1e9),
                stamp['sec']*10**9+stamp['nanosec'], yaw_rate_rad_s)
        with self.lock:
            if self.stopping.is_set():return
            if self.pending is not None:self.dropped += 1
            self.pending = (self.generation, item)
        self.wake.set()

    def process_one(self):
        with self.lock:
            selected, self.pending = self.pending, None
            generation = self.generation
            self.processing = selected is not None
        if getattr(self, '_observer_generation', None) != generation:
            self.observer.reset(); self._observer_generation = generation
        if selected is None:return
        token, (scan, result, epoch, receipt_ns, source_ns, gyro) = selected
        started = time.perf_counter()
        try:
            now = self.clock()
            if now is None or not math.isfinite(now):raise ValueError('independent_receipt_clock_unavailable')
            if self.process is not None:
                if not self.process.is_alive():raise ValueError('observation_process_unavailable')
                self.pipe.send((token,selected[1]));deadline=time.monotonic()+2.
                while not self.pipe.poll(.05):
                    if self.stopping.is_set():raise ValueError('observer_closed')
                    if time.monotonic()>deadline:
                        self.process.terminate();raise ValueError('observation_process_deadline')
                returned,state=self.pipe.recv()
                if returned!=token:raise ValueError('observation_generation_mismatch')
            else:
                state = self.observer.observe(scan, result, epoch=epoch,
                    receipt_ns=receipt_ns, receipt_now_ns=round(now*1e9),
                    source_stamp_ns=source_ns, yaw_rate_rad_s=gyro)
        except Exception as exc:
            # A perception fault must not kill its worker or the live mapper.
            state = self.observer.invalidate('observer_exception:'+type(exc).__name__)
        with self.lock:
            self.processing = False
            if token != self.generation or self.stopping.is_set():return
            self.processed += 1; self.receipt = receipt_ns*1e-9
            self.state = dict(state, total_ms=(time.perf_counter()-started)*1000)

    def _run(self):
        while not self.stopping.is_set():
            self.wake.wait(.1); self.wake.clear()
            self.process_one()

    def snapshot(self):
        with self.lock:
            state = copy.deepcopy(self.state); now = self.clock()
            age = None if self.receipt is None or now is None else now-self.receipt
            fresh = age is not None and 0 <= age < .2
            if not fresh:
                state.update(state='HOLD', reason='observation_expired' if self.receipt is not None
                    else state['reason'], proposed_linear_mps=0., proposed_yaw_rate_rad_s=0.)
                if 'feedback' in state:state['feedback'].update(valid=False, reason='observation_expired')
            return dict(state, observation_fresh=fresh, receipt_age_s=age,
                queued=int(self.pending is not None), processing=self.processing,
                replaced_observations=self.dropped, processed_observations=self.processed,
                generation=self.generation, worker_running=bool(self.thread and self.thread.is_alive()),
                calculation_process_isolated=self.process is not None,
                simulated=True)

    def close(self):
        self.invalidate('observer_closed'); self.stopping.set(); self.wake.set()
        if self.thread:self.thread.join(3)
        if self.process is not None:
            if self.process.is_alive():self.process.terminate()
            self.process.join(3);self.pipe.close()
