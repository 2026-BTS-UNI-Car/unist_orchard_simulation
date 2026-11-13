"""Bounded mapping ownership and live-first scheduling, without motor access.

Readers never acquire generation authority. One recorded worker may calculate
at a time; live tracking preempts new recorded work at scan boundaries. A
disconnected replay writer expires. Explicit Stop revokes in-flight requests
until a subsequent user Play sends an explicit resume flag.
"""
import math
from pathlib import Path
import threading
import time
import uuid

from .archive_mapping import ArchiveMapping
from .occupancy_map import read_json
from .resumable_playback import ResumablePlaybackMapper


class MappingConflict(ValueError):
    pass


class MappingDeferred(MappingConflict):
    """A read that must wait (HTTP 503 + Retry-After), not a failure."""
    retry_after_s=5


UNSPECIFIED=object()


class ManagedArchiveMapping(ArchiveMapping):
    controlled = True

    def __init__(self,root,live=None,lease_seconds=15.,start_monitor=True):
        super().__init__(root)
        self.lock=threading.RLock();self.live=live;self.lease_seconds=lease_seconds
        self.leases={};self.suspended=set();self.notes={};self.changed={};self.generations={}
        self.read_slots=threading.BoundedSemaphore(2)
        # C2 (S14): building a map window is the long read; it gets its own
        # slot so short status reads are not refused while one is built.
        self.window_slot=threading.BoundedSemaphore(1)
        self.read_wait_s=1.0
        self._shutdown=threading.Event();self.monitor=None
        if start_monitor:
            self.monitor=threading.Thread(target=self._monitor,daemon=True,name='mapping-job-monitor')
            self.monitor.start()

    def _monitor(self):
        while not self._shutdown.wait(.5):self.expire()

    def _live_busy(self):
        return bool(self.live and getattr(self.live,'tracking',False))

    def _lease_client(self,control):
        if control is None:return 'legacy'
        import re
        if (not isinstance(control,dict) or not isinstance(control.get('client'),str)
                or not re.fullmatch(r'[a-zA-Z0-9-]{1,64}',control['client'])
                or type(control.get('sequence')) is not int or not 0<=control['sequence']<=2**53-1):
            raise ValueError('invalid_playback_control_sequence')
        return control['client']

    def _retire(self,archive_id,reason,timeout=5.):
        worker=self.playback_mappers.get(archive_id)
        if worker:
            worker.close()
            if worker.thread:worker.thread.join(timeout=timeout)
            if worker.running:raise MappingConflict('mapping_worker_stopping_retry')
        self.leases.pop(archive_id,None);self.notes[archive_id]=reason

    def expire(self,now=None):
        now=time.monotonic() if now is None else now
        with self.lock:
            for name,lease in list(self.leases.items()):
                if now-lease['heartbeat']>self.lease_seconds:
                    worker=self.playback_mappers.get(name)
                    if worker:worker.close()
                    self.leases.pop(name,None);self.suspended.add(name)
                    self.generations[name]=uuid.uuid4().hex
                    self.notes[name]='replay_client_disconnected'

    def new_map(self,archive_id,expected_epoch=UNSPECIFIED,client=None):
        path=self.directory(archive_id)
        with self.lock:
            history=read_json(path/'integrated_map_history.json')
            if expected_epoch is not UNSPECIFIED and expected_epoch!=history.get('epoch'):
                raise MappingConflict('mapping_epoch_changed_refresh_before_action')
            lease=self.leases.get(archive_id)
            if lease and lease['client']!=client:
                raise MappingConflict('another_device_controls_this_recording')
            self._retire(archive_id,'new_map')
            if archive_id not in self.playback_mappers and len(self.playback_mappers)>=2:
                for name,worker in list(self.playback_mappers.items()):
                    if worker.running and not worker.closed:continue
                    self._retire(name,'worker_slot_released');self.playback_mappers.pop(name,None)
                if len(self.playback_mappers)>=2:raise MappingConflict('mapping_worker_slots_busy')
            # No asynchronous old writer survives the pointer replacement.
            result=super().new_map(archive_id)
            worker=self.playback_mappers[archive_id];worker.allow_compute=lambda:not self._live_busy()
            self.suspended.discard(archive_id);self.notes.pop(archive_id,None)
            self.generations[archive_id]=uuid.uuid4().hex
            return {**result,'mapping_generation':self.generations[archive_id]}

    def playback(self,archive_id,t_s,playing,control=None,resume=False,generation=None):
        path=self.directory(archive_id);client=self._lease_client(control)
        if not isinstance(playing,bool):raise ValueError('invalid_playback_state')
        try:value=float(t_s)
        except (TypeError,ValueError):raise ValueError('invalid_map_time')
        duration=float(read_json(path/'metadata.json').get('duration_s') or 0)
        if not math.isfinite(value) or not 0<=value<=duration+.5:raise ValueError('invalid_map_time')
        with self.lock:
            self.expire()
            current_generation=self.generations.setdefault(archive_id,uuid.uuid4().hex)
            if generation is not None and generation!=current_generation:
                raise MappingConflict('mapping_control_revoked_refresh')
            lease=self.leases.get(archive_id)
            if lease and lease['client']!=client:
                raise MappingConflict('another_device_controls_this_recording')
            if archive_id in self.suspended:
                if not resume:raise MappingConflict('mapping_stopped_press_play_to_resume')
                if not playing or generation!=current_generation:
                    raise MappingConflict('mapping_control_revoked_refresh')
                self.suspended.discard(archive_id)
            history=read_json(path/'integrated_map_history.json')
            if history.get('complete') and history.get('playback_driven'):
                return dict(ok=True,playback_driven=True,completed=True,map_epoch=history['epoch'],mapping_generation=current_generation)
            if not playing and lease is None:
                return dict(ok=True,playback_driven=bool(history.get('playback_driven')),
                    mapping_paused=True,observer_only=True,mapping_generation=current_generation)
            if playing and not history.get('playback_driven'):
                self.new_map(archive_id);history=read_json(path/'integrated_map_history.json')
            if not history.get('playback_driven'):return dict(ok=True,playback_driven=False)
            worker=self.playback_mappers.get(archive_id)
            if worker and worker.closed:
                self._retire(archive_id,'restarting');worker=None
            # A different active recording is never silently killed by a poll.
            for name,other in self.playback_mappers.items():
                if name!=archive_id and other.running and not other.closed:
                    raise MappingConflict('another_recording_mapping_active_stop_it_first')
            if worker is None:
                for name,other in list(self.playback_mappers.items()):
                    if name!=archive_id:
                        self._retire(name,'worker_slot_released');self.playback_mappers.pop(name,None)
                worker=ResumablePlaybackMapper(path,history);self.playback_mappers[archive_id]=worker
            worker.allow_compute=lambda:not self._live_busy()
            result=worker.advance(value,playing,control)
            if not result.get('stale_control_ignored'):
                self.leases[archive_id]=dict(client=client,heartbeat=time.monotonic(),epoch=history['epoch'])
            return {**result,'waiting_live_priority':self._live_busy(),'mapping_generation':self.generations[archive_id],
                'lease_seconds':self.lease_seconds,'control_owned':True}

    def regenerate(self,archive_id):
        # Automatic bulk generation is replaced by a prepared causal history.
        # User Play drives it, preventing hidden full-record CPU jobs at Stop.
        self.directory(archive_id)
        return self.new_map(archive_id)

    def stop_job(self,archive_id,expected_epoch=None):
        path=self.directory(archive_id)
        with self.lock:
            if expected_epoch is not None and expected_epoch!=read_json(path/'integrated_map_history.json').get('epoch'):
                raise MappingConflict('mapping_epoch_changed_refresh_before_action')
            self.suspended.add(archive_id);self.generations[archive_id]=uuid.uuid4().hex
            self._retire(archive_id,'operator_stopped')
        return dict(ok=True,state='stopped',partial_map_preserved=True,raw_source_preserved=True,mapping_generation=self.generations[archive_id])

    def jobs(self,client=None):
        now=time.monotonic();rows=[]
        with self.lock:
            for name,worker in self.playback_mappers.items():
                h=worker.history;lease=self.leases.get(name);processed=float(h.get('processed_t_s') or 0)
                previous=self.changed.get(name)
                if previous is None or processed!=previous[0]:self.changed[name]=(processed,now)
                active=worker.running and not worker.closed
                stage='complete' if h.get('complete') else 'failed' if worker.error else 'stopping' if worker.closed and worker.running else 'stopped' if worker.closed else 'waiting_live_priority' if self._live_busy() and worker.playing else 'paused' if not worker.playing else 'waiting_cursor' if h.get('waiting_for_cursor') else getattr(worker,'worker_stage','localizing')
                rows.append(dict(id='archive:'+name,kind='archive',priority=1,recording_id=name,
                    epoch=h['epoch'],state=stage,stage=stage,active=active,processing=active and worker.playing and not self._live_busy(),
                    processed_t_s=processed,requested_t_s=worker.target,duration_s=h.get('duration_s'),
                    input_scans=h.get('input_scans',0),scan_count=len(h.get('events',[])),
                    rejected_scans=h.get('rejected_scans',0),motion_prior_scans=h.get('motion_prior_scans',0),
                    last_progress_age_s=now-self.changed[name][1],error=worker.error,reason=self.notes.get(name),
                    lease_remaining_s=None if lease is None else max(0.,self.lease_seconds-(now-lease['heartbeat'])),
                    control_owned=bool(lease and lease['client']==client),controller_present=bool(lease)))
        return rows

    def status(self,*args,**kwargs):
        if not self.read_slots.acquire(timeout=self.read_wait_s):
            raise MappingConflict('mapping_read_capacity_busy')
        try:
            result=super().status(*args,**kwargs)
        finally:
            self.read_slots.release()
        name=args[0] if args else kwargs['archive_id']
        if result.get('playback_driven'):
            job=next((r for r in self.jobs() if r['recording_id']==name),None)
            result={**result,'mapping_job':job,'mapping_generation':self.generations.setdefault(name,uuid.uuid4().hex)}
        return result

    def window(self,*args,**kwargs):
        name=args[0] if args else kwargs['archive_id']
        if not self.window_slot.acquire(timeout=self.read_wait_s):
            raise MappingDeferred('mapping_read_capacity_busy')
        try:
            result=super().window(*args,**kwargs)
            return {**result,'mapping_generation':self.generations.setdefault(name,uuid.uuid4().hex)}
        finally:
            self.window_slot.release()

    def close(self):
        self._shutdown.set()
        if self.monitor:self.monitor.join(timeout=1)
        super().close()


class MappingOperations:
    def __init__(self,live,archives):
        self.live=live;self.archives=archives;self.lock=threading.Lock()
        self.last_resource=(0.,0.);self.resources={}

    def status(self,client=None):
        now=time.monotonic()
        with self.lock:
            if now-self.last_resource[0]>=2:
                try:
                    import resource
                    r=resource.getrusage(resource.RUSAGE_SELF);c=resource.getrusage(resource.RUSAGE_CHILDREN)
                    import os
                    ticks=os.sysconf('SC_CLK_TCK');children=[];pending=[]
                    for task in Path('/proc/self/task').glob('*/children'):
                        pending.extend(task.read_text().split())
                    seen=set()
                    while pending:
                        pid=pending.pop()
                        if pid in seen:continue
                        seen.add(pid)
                        try:
                            # Read process times only, never command arguments.
                            fields=Path('/proc',pid,'stat').read_text().rsplit(')',1)[1].split()
                            children.append((int(fields[11])+int(fields[12]))/ticks)
                            for task in Path('/proc',pid,'task').glob('*/children'):
                                pending.extend(task.read_text().split())
                        except (OSError,ValueError,IndexError):pass
                    cpu=r.ru_utime+r.ru_stime+c.ru_utime+c.ru_stime+sum(children)
                    elapsed=now-self.last_resource[0]
                    percent=None if not self.last_resource[0] else max(0.,(cpu-self.last_resource[1])/elapsed*100)
                    rss=None
                    for row in Path('/proc/self/status').read_text().splitlines():
                        if row.startswith('VmRSS:'):rss=int(row.split()[1])*1024
                    self.resources=dict(cpu_percent_one_core=percent,rss_bytes=rss,scope='dashboard_and_mapper_children',
                        running_child_cpu_included=True,running_child_count=len(children))
                    self.last_resource=(now,cpu)
                except (ImportError,OSError,ValueError):self.resources=dict(scope='unavailable')
            resources=dict(self.resources)
        return dict(ok=True,live_first=True,max_recorded_workers=1,resources=resources,
            jobs=[self.live.task_status(),*self.archives.jobs(client)],autonomy_output_enabled=False)

    def stop(self,job_id,expected_epoch=None):
        if job_id=='live':
            self.live.action('stop',expected_epoch=expected_epoch)
            return dict(ok=True,state='stopped',partial_map_preserved=True)
        if isinstance(job_id,str) and job_id.startswith('archive:'):
            return self.archives.stop_job(job_id[8:],expected_epoch)
        raise ValueError('unknown_mapping_job')
