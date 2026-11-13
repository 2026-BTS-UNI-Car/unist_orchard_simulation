"""Persist a causal map only as far as an authenticated replay clock requests.

Raw recordings are immutable. New map switches a derived pointer; previous
epochs remain recoverable. A paused clock never requests future localization.
"""
import bisect
from datetime import datetime
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid
from collections import deque,OrderedDict
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from .occupancy_map import atomic_json,read_json
from .integrated_archive import (rows,digest,localization_fingerprint,raster_fingerprint)
from .integrated_mapping import integrate
from .raw_mapping.continuous_localization import ContinuousLocalization
from .raw_mapping.raw_localization import source_stamp
from .raw_mapping.gnss_constraints import UniqueGNSS,robust_rigid_alignment
from .tile_transport import StreamingOccupancyMap


class SourceCursorLocalizer:
    """Ordered raw prefix only; independent of raster/storage and HTTP work."""
    def __init__(self):
        self.engine=ContinuousLocalization();self.gnss=UniqueGNSS()
        self.imus=[];self.stamps=[];self.pending_imus=[];self.pairs=[];self.basis=None
        self.input_scans=0;self.note=dict(state='no_usable_fix')

    def consume(self,batch):
        engine=self.engine;result=None
        for row in batch:
            topic=row['topic'];message=row['message']
            if topic=='/imu/data_raw':
                self.pending_imus.append(message)
                i=bisect.bisect_right(self.stamps,source_stamp(message))
                self.imus.insert(i,message);self.stamps.insert(i,source_stamp(message))
                if len(self.imus)>64:self.imus.pop(0);self.stamps.pop(0)
            elif topic=='/gps/data':
                sample=json.loads(message['data']);factor=self.gnss.add(sample)
                if factor:engine.gps(sample)
                if factor and engine.previous:
                    self.pairs.append((engine.pose[:2].copy(),factor))
                    if self.basis is None and len(self.pairs)>=8:
                        fit=robust_rigid_alignment(np_array([p for p,f in self.pairs]),[f for p,f in self.pairs])
                        if fit['accepted']:self.basis=fit
                    if self.basis:
                        local=(np_array(factor.xy)-self.basis['translation'])@self.basis['rotation']
                        scale=math.hypot(factor.scale_m,2*float(np_array(local).dot(np_array(local)))**.5*self.basis['heading_uncertainty_lower_bound_rad'])
                        engine.gps_prior(factor.stamp,local,scale)
                    self.note=dict(state='position_consistency_validation' if self.basis else 'waiting_for_motion_baseline',epochs=len(self.gnss.accepted),reported_hacc_m=factor.reported_hacc_m)
            elif topic=='/scan':
                stamp=source_stamp(message);index=bisect.bisect_right(self.stamps,stamp)-1
                self.pending_imus.sort(key=source_stamp)
                while self.pending_imus and source_stamp(self.pending_imus[0])<=stamp:engine.imu(self.pending_imus.pop(0))
                result=engine.add(message,self.imus[index] if index>=0 else None);self.input_scans+=1
        return result,dict(input_scans=self.input_scans,rejected_scans=engine.rejected,
                          motion_prior_scans=engine.motion_prior_count),self.note


_pipeline_localizer=None
def initialize_localizer_process():
    global _pipeline_localizer
    _pipeline_localizer=SourceCursorLocalizer()


def localize_prefix_batch(batch):
    return _pipeline_localizer.consume(batch)


def persist_json(path,value):
    # Windows readers briefly hold a handle that disallows rename. Keep the
    # previous complete file and retry the atomic replacement; never truncate it.
    for attempt in range(8):
        try:
            atomic_json(path,value);return
        except PermissionError:
            if os.name!='nt' or attempt==7:raise
            time.sleep(.01*(attempt+1))


def new_history(archive,signature):
    archive=Path(archive);source=archive/'mapping_source';manifest=read_json(source/'manifest.json')
    if source.is_symlink() or (source/'sensors.jsonl.gz').is_symlink() or (archive/'integrated_map').is_symlink():
        raise ValueError('invalid_mapping_source_path')
    if not manifest.get('complete') or not (source/'sensors.jsonl.gz').is_file():
        raise ValueError('원시 센서 추출이 준비된 기록에서 새 지도를 만들 수 있습니다.')
    target=archive/'integrated_map_history.json'
    if target.is_file():
        backup=archive/('integrated_map_history-previous-'+digest(target)[:16]+'.json')
        if not backup.exists():backup.write_bytes(target.read_bytes())
    epoch=uuid.uuid4().hex;folder=archive/'integrated_map'/epoch;folder.mkdir(parents=True)
    (folder/'events.jsonl').touch()
    duration=float(read_json(archive/'metadata.json').get('duration_s') or 0)
    value=dict(schema='ubts.integrated_archive.v1',epoch=epoch,source_signature=signature,
        cell_transport='tiles-cells-v1',localization_fingerprint=localization_fingerprint(),
        raster_fingerprint=raster_fingerprint(),hit_endpoints='all_finite_returns',
        free_rays='all_observed_scan_beams',complete=False,events=[],checkpoints=[],
        duration_s=duration,processed_t_s=0.,requested_t_s=0.,progress=0.,segments=1,
        playback_driven=True,waiting_for_cursor=True,mapping_paused=True,pause_policy='freeze_until_resume',tracking='waiting_for_play',input_scans=0,rejected_scans=0,
        source_sensor_sha256=(manifest.get('output_sha256') or {}).get('sensors.jsonl.gz'))
    persist_json(target,value);return value


class PlaybackMapper:
    def __init__(self,archive,history):
        self.archive=Path(archive);self.history=history
        self.condition=threading.Condition();self.target=0.;self.closed=False
        self.running=False;self.error=None;self.playing=False;self.thread=None
        self.control_sequences=OrderedDict()

    def advance(self,time_s,playing,control=None):
        try:value=float(time_s)
        except (ValueError,TypeError):raise ValueError('invalid_map_time')
        if not math.isfinite(value) or not 0<=value<=self.history['duration_s']+.5:
            raise ValueError('invalid_map_time')
        if not isinstance(playing,bool):raise ValueError('invalid_playback_state')
        if control is not None:
            import re
            if (not isinstance(control,dict) or not isinstance(control.get('client'),str) or not re.fullmatch(r'[a-zA-Z0-9-]{1,64}',control['client'])
                    or type(control.get('sequence')) is not int or not 0<=control['sequence']<=2**53-1):
                raise ValueError('invalid_playback_control_sequence')
        with self.condition:
            if control is not None:
                client,sequence=control['client'],control['sequence']
                if sequence<=self.control_sequences.get(client,-1):
                    return dict(ok=True,playback_driven=True,map_epoch=self.history['epoch'],stale_control_ignored=True)
                self.control_sequences[client]=sequence;self.control_sequences.move_to_end(client)
                if len(self.control_sequences)>8:self.control_sequences.popitem(last=False)
            # Pause freezes processing, including an already requested backlog.
            # Retain queued raw scans in order for resume; never skip them.
            self.target=min(value,self.history['duration_s']) if playing else min(value,self.target)
            self.playing=playing
            self.history['mapping_paused']=not playing
            if self.target>self.history.get('processed_t_s',0) and not self.history.get('complete'):
                self.history['waiting_for_cursor']=False
            if not self.error and not self.running and not self.history.get('complete') and self.target>self.history.get('processed_t_s',0):
                self.running=True;self.thread=threading.Thread(target=self._run,daemon=True,name='causal-replay-map');self.thread.start()
            self.condition.notify_all()
        return dict(ok=True,playback_driven=True,map_epoch=self.history['epoch'],
            processed_t_s=self.history.get('processed_t_s',0),requested_t_s=self.target,
            playing=playing,mapping_error=self.error)

    def close(self):
        with self.condition:self.closed=True;self.condition.notify_all()

    def _save(self,log):
        target=self.archive/'integrated_map_history.json'
        if read_json(target).get('epoch')!=self.history['epoch']:
            self.closed=True;return
        log.flush()
        self.history.update(requested_t_s=self.target,progress=min(1,self.history['processed_t_s']/max(.001,self.history['duration_s'])))
        persist_json(target,self.history)

    def _run(self):
        try:
            source=self.archive/'mapping_source';manifest=read_json(source/'manifest.json');file=source/'sensors.jsonl.gz'
            if digest(file)!=self.history['source_sensor_sha256']:
                raise ValueError('unverified_mapping_sensor_source')
            if self.history['localization_fingerprint']!=localization_fingerprint():raise ValueError('mapping_algorithm_changed_new_map_required')
            start=datetime.fromisoformat(read_json(self.archive/'metadata.json')['started_at']).timestamp()
            folder=self.archive/'integrated_map'/self.history['epoch']
            grid=StreamingOccupancyMap(max_beams=450,laser_x=0.,laser_y=0.,conservative_free=True,endpoint_margin_cells=1)
            saved_index=len(self.history['events']);accepted_index=0
            next_checkpoint=-float('inf');last_saved=0.
            # One ordered localization process overlaps with raster/storage.
            # At most two scans from the requested prefix are in flight. No
            # future scan is prefetched or dropped, and pose math is unchanged.
            pipeline=os.environ.get('UBTS_MAP_PIPELINE','0' if os.name=='nt' else '1')=='1'
            executor=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),
                initializer=initialize_localizer_process) if pipeline else None
            localizer=None if pipeline else SourceCursorLocalizer();pending=deque();batch=[]
            with (folder/'events.jsonl').open('ab') as log:
                def wait_for_play(t):
                    with self.condition:
                        saved=False
                        while not self.closed and (not self.playing or t>self.target+1e-6):
                            self.history['waiting_for_cursor']=True
                            if not saved:self._save(log);saved=True
                            self.condition.wait()
                        if self.closed:return False
                        self.history['waiting_for_cursor']=False
                        return True
                def finish(observation):
                    future,message,t=observation
                    if not wait_for_play(t):return
                    result,counters,gnss_note=future.result() if pipeline else future
                    # A pause response waits only for the current raster commit;
                    # no later commit can race past its acknowledgement.
                    with self.condition:
                        if wait_for_play(t):commit(result,counters,gnss_note,message,t)
                def commit(result,counters,gnss_note,message,t):
                    nonlocal accepted_index,next_checkpoint,last_saved
                    stamp=source_stamp(message)
                    self.history.update(**counters,tracking=result['tracking'])
                    if result['accepted'] and result.get('map_eligible',True):
                        previous=grid.scan_count;integrate(grid,message,result['pose'],t);accepted_index+=1
                        if grid.path and result.get('path_break'):grid.path[-1]['break_before']=True
                        if accepted_index>saved_index:
                            packet=grid.tile_packet(None if accepted_index==1 else previous)
                            packet['path']=grid.path[-1:] if grid.path and grid.path[-1].get('stamp')==stamp else []
                            packet.update(ok=True,version=accepted_index,tile_version=accepted_index,segment=0,
                                frame_t_s=t,tile_epoch=self.history['epoch']+':0',map_epoch=self.history['epoch'],
                                preserve_view_on_seek=True,coordinate_frame='continuous_sensor_local_frame',
                                pose_accuracy_validated=False,autonomy_eligible=False,runtime_authority=False,
                                source='causal_recorded_LaserScan_IMU_GNSS',gnss=gnss_note,no_encoder=True,
                                tracking=result['tracking'],playback_driven=True,motion_prior_scans=counters['motion_prior_scans'])
                            encoded=(json.dumps(packet,separators=(',',':'),allow_nan=False)+'\n').encode()
                            self.history['events'].append(dict(t_s=t,offset=log.tell(),bytes=len(encoded),version=accepted_index,segment=0));log.write(encoded)
                            if t>=next_checkpoint or accepted_index==saved_index+1:
                                checkpoint=grid.tile_packet();checkpoint.update({k:v for k,v in packet.items() if k not in ('tiles','path','tiles_reset')})
                                name=f'checkpoint-{len(self.history["checkpoints"]):06d}.json'
                                atomic_json(folder/name,checkpoint);self.history['checkpoints'].append(dict(index=len(self.history['events'])-1,t_s=t,file=name,segment=0));next_checkpoint=t+10
                    self.history['processed_t_s']=max(self.history['processed_t_s'],t)
                    if time.monotonic()-last_saved>.25:
                        self._save(log);last_saved=time.monotonic()
                try:
                    for row in rows(file):
                        t=max(0.,row['bag_time_ns']/1e9-start)
                        if t>self.history['duration_s']+.01:break
                        if t>self.target+1e-6:
                            while pending:finish(pending.popleft())
                        if not wait_for_play(t):return
                        batch.append(row)
                        if row['topic']!='/scan':continue
                        future=executor.submit(localize_prefix_batch,batch) if pipeline else localizer.consume(batch)
                        pending.append((future,row['message'],t));batch=[]
                        if len(pending)>=(2 if pipeline else 1):finish(pending.popleft())
                    while pending:finish(pending.popleft())
                    if self.closed:return
                finally:
                    if executor:executor.shutdown(wait=True,cancel_futures=True)
                self.history.update(complete=True,waiting_for_cursor=True,processed_t_s=self.history['duration_s']);self._save(log)
                grid.save_checkpoint(folder/'native-continuous.json')
        except Exception as error:
            self.error=str(error)[:240];self.history['mapping_error']=self.error
            target=self.archive/'integrated_map_history.json'
            if read_json(target).get('epoch')==self.history['epoch']:persist_json(target,self.history)
        finally:
            with self.condition:self.running=False


def np_array(values):
    import numpy as np
    return np.asarray(values)
