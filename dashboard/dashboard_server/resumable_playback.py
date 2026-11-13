"""Resume recorded sensor prefixes; localization math stays in playback_mapping.

The cursor loop mirrors PlaybackMapper's ordered commit/pause rules. This I/O
extension deliberately does not invalidate already rendered map histories.
"""
import gzip,json,os,time,multiprocessing
from datetime import datetime
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from .playback_mapping import PlaybackMapper,SourceCursorLocalizer,persist_json
from .occupancy_map import read_json,atomic_json
from .integrated_archive import digest,localization_fingerprint
from .tile_transport import StreamingOccupancyMap
from .raw_mapping.raw_localization import source_stamp
from . import playback_mapping as source_module
from .replay_state import export_localizer,restore_localizer,save_state,load_state

def initialize_restored_process(state):
    source_module._pipeline_localizer=restore_localizer(state) if state else SourceCursorLocalizer()

def consume_state(localizer,batch,capture):
    result,counters,note=localizer.consume(batch)
    return result,counters,note,export_localizer(localizer) if capture else None

def localize_state_batch(batch,capture):
    return consume_state(source_module._pipeline_localizer,batch,capture)

def source_rows(path,offset):
    with gzip.open(path,'rb') as stream:
        if offset:stream.seek(offset)
        for line in iter(stream.readline,b''):
            if line.strip():yield json.loads(line),stream.tell()

class ResumablePlaybackMapper(PlaybackMapper):
    def __init__(self,archive,history):
        super().__init__(archive,history)
        self.state_interval=max(.1,min(30.,float(os.environ.get('UBTS_MAP_STATE_INTERVAL_S','2'))))
        self.allow_compute=lambda:True
        self.worker_stage='waiting_cursor'

    def _run(self):
        try:
            with self.condition:
                while not self.closed and not self.allow_compute():
                    self.worker_stage='waiting_live_priority';self.condition.wait(.25)
                if self.closed:return
            self.worker_stage='verifying_source'
            source=self.archive/'mapping_source';manifest=read_json(source/'manifest.json');file=source/'sensors.jsonl.gz'
            if digest(file)!=self.history['source_sensor_sha256']:
                raise ValueError('unverified_mapping_sensor_source')
            if self.history['localization_fingerprint']!=localization_fingerprint():raise ValueError('mapping_algorithm_changed_new_map_required')
            start=datetime.fromisoformat(read_json(self.archive/'metadata.json')['started_at']).timestamp()
            folder=self.archive/'integrated_map'/self.history['epoch']
            grid=StreamingOccupancyMap(max_beams=450,laser_x=0.,laser_y=0.,conservative_free=True,endpoint_margin_cells=1)
            saved_index=len(self.history['events']);accepted_index=0
            restored=load_state(folder,self.history);initial_state=None;raw_offset=0
            if restored:
                meta,initial_state,grid=restored;raw_offset=meta['raw_offset'];accepted_index=grid.scan_count
                self.history['restart_state']=dict(restored=True,t_s=meta['t_s'],input_scans=meta['input_scans'])
            else:self.history['restart_state']=dict(restored=False)
            next_state_t=(restored[0]['t_s'] if restored else 0)+self.state_interval
            next_checkpoint=-float('inf');last_saved=0.
            # One ordered localization process overlaps with raster/storage.
            # At most two scans from the requested prefix are in flight. No
            # future scan is prefetched or dropped, and pose math is unchanged.
            pipeline=os.environ.get('UBTS_MAP_PIPELINE','0' if os.name=='nt' else '1')=='1'
            executor=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),
                initializer=initialize_restored_process,initargs=(initial_state,)) if pipeline else None
            localizer=None if pipeline else restore_localizer(initial_state) if initial_state else SourceCursorLocalizer();pending=deque();batch=[]
            with (folder/'events.jsonl').open('ab') as log:
                def wait_for_play(t):
                    with self.condition:
                        saved=False
                        while not self.closed and (not self.playing or t>self.target+1e-6 or not self.allow_compute()):
                            self.history['waiting_for_cursor']=True
                            self.worker_stage='waiting_live_priority' if not self.allow_compute() else 'paused' if not self.playing else 'waiting_cursor'
                            if not saved:self._save(log);saved=True
                            self.condition.wait(.25)
                        if self.closed:return False
                        self.history['waiting_for_cursor']=False
                        self.worker_stage='localizing'
                        return True
                def finish(observation):
                    future,message,t,offset=observation
                    if not wait_for_play(t):return
                    result,counters,gnss_note,state=future.result() if pipeline else future
                    # A pause response waits only for the current raster commit;
                    # no later commit can race past its acknowledgement.
                    with self.condition:
                        if wait_for_play(t):
                            commit(result,counters,gnss_note,message,t)
                            if state and not self.closed:
                                self._save(log)
                                if not self.closed:
                                    try:save_state(folder,self.history,state,grid,offset,t)
                                    except (OSError,ValueError,TypeError,KeyError) as error:
                                        # Restart acceleration is optional. Preserve mapping
                                        # and the previous valid checkpoint if storage fails.
                                        self.history['restart_state_warning']=str(error)[:160]
                def commit(result,counters,gnss_note,message,t):
                    nonlocal accepted_index,next_checkpoint,last_saved
                    stamp=source_stamp(message)
                    self.history.update(**counters,tracking=result['tracking'])
                    if result['accepted'] and result.get('map_eligible',True):
                        previous=grid.scan_count;source_module.integrate(grid,message,result['pose'],t);accepted_index+=1
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
                    for row,offset in source_rows(file,raw_offset):
                        t=max(0.,row['bag_time_ns']/1e9-start)
                        if t>self.history['duration_s']+.01:break
                        if t>self.target+1e-6:
                            while pending:finish(pending.popleft())
                        if not wait_for_play(t):return
                        batch.append(row)
                        if row['topic']!='/scan':continue
                        capture=t>=next_state_t
                        if capture:next_state_t=t+self.state_interval
                        future=executor.submit(localize_state_batch,batch,capture) if pipeline else consume_state(localizer,batch,capture)
                        pending.append((future,row['message'],t,offset));batch=[]
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
