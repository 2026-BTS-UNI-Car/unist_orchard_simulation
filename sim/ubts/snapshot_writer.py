"""Single bounded status writer; no sensor/control state or producer clocks.

submit() transfers read-only ownership of a detached snapshot bundle. Callers
must not mutate its dictionaries afterwards. The adapter explicitly copies live
mutable fields; published diagnostic records have a separate immutable contract.
There is at most one in-flight bundle and one pending latest bundle. JSON and
filesystem work happens only in the worker, never while holding its queue lock.
"""
import threading


FILES=frozenset(('rover_bridge_status.json','simulation_status.json','autonomy_status.json',
                 'camera_driver_status.json','imu_driver_status.json','gps_bridge_status.json'))


class LatestSnapshotWriter:
    def __init__(self, save):
        self.save=save;self.condition=threading.Condition()
        self.pending=None;self.inflight=False;self.closing=False
        self.submitted=self.completed=self.coalesced=self.written_files=self.failures=0
        self.last_error=None
        self.thread=threading.Thread(target=self._run,name='ubts-status-writer',daemon=True)
        self.thread.start()

    def submit(self, payloads):
        if not isinstance(payloads,dict) or not payloads or any(name not in FILES for name in payloads):
            raise ValueError('invalid_snapshot_files')
        # Copy the small file table, not the already detached payload trees.
        bundle=dict(payloads)
        with self.condition:
            if self.closing:return False
            self.submitted+=1
            if self.pending is not None:self.coalesced+=1
            self.pending=bundle
            self.condition.notify()
        return True

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.pending is not None or self.closing)
                if self.pending is None:return
                bundle=self.pending;self.pending=None;self.inflight=True
            for name,payload in bundle.items():
                try:
                    self.save(name,payload)
                except Exception as error:
                    with self.condition:
                        self.failures+=1
                        self.last_error={'file':name,'error':type(error).__name__}
                else:
                    with self.condition:self.written_files+=1
            with self.condition:
                self.completed+=1;self.inflight=False

    def status(self):
        with self.condition:
            return {'schema':'ubts-snapshot-writer-v1','submitted':self.submitted,
                    'completed':self.completed,'coalesced':self.coalesced,
                    'pending_bundles':int(self.pending is not None),'inflight_bundles':int(self.inflight),
                    'written_files':self.written_files,'failures':self.failures,
                    'last_error':dict(self.last_error) if self.last_error else None,
                    'closing':self.closing,'running':self.thread.is_alive()}

    def close(self, timeout=.5):
        """Call only after the motor stop, outside all control locks.

        Flush the latest pending bundle when possible; a blocked filesystem must
        not make shutdown unbounded. The daemon may still finish that old write,
        whose original producer timestamps are never refreshed by this worker.
        """
        with self.condition:self.closing=True;self.condition.notify()
        self.thread.join(timeout=timeout)
        return not self.thread.is_alive()
