"""Raw-rate indexed map tiles. Playback never serves a future sensor map."""
import bisect
from datetime import datetime
import gzip
import hashlib
import json
from functools import lru_cache
from pathlib import Path
import threading
import uuid
import numpy as np
from dashboard_server.integrated_mapping import IntegratedMapping, corrected_poses, integrate
from dashboard_server.raw_mapping.raw_localization import source_stamp
from dashboard_server.tile_transport import StreamingOccupancyMap
from dashboard_server.occupancy_map import atomic_json, read_json


def _code_fingerprint(files):
    base=Path(__file__).parent;sha=hashlib.sha256()
    for relative in files:
        sha.update(relative.encode());sha.update((base/relative).read_bytes())
    return sha.hexdigest()


@lru_cache(maxsize=1)
def localization_fingerprint():
    """Derived poses are reusable only with the exact localization algorithm."""
    return _code_fingerprint(('integrated_mapping.py','raw_mapping/raw_localization.py',
        'raw_mapping/raw_point_support.py','raw_mapping/fit_workers.py','raw_mapping/gnss_constraints.py',
        'raw_mapping/continuous_localization.py','playback_mapping.py'))


@lru_cache(maxsize=1)
def raster_fingerprint():
    return _code_fingerprint(('integrated_mapping.py','occupancy_map.py','tile_transport.py','tiled_occupancy.py'))


def digest(path):
    sha=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024),b''):sha.update(block)
    return sha.hexdigest()


def segment_catalog(history, target):
    """Retained local maps available at the cursor; never expose future frames."""
    events = history.get('events', [])
    if '_segment_index' not in history:
        groups = []
        for i, event in enumerate(events):
            if not groups or groups[-1]['segment'] != event['segment']:
                groups.append(dict(segment=event['segment'], first=i, last=i))
            groups[-1]['last'] = i
        history['_segment_index'] = groups
        history['_event_times'] = [e['t_s'] for e in events]
    end = bisect.bisect_right(history['_event_times'], target) - 1
    result = []
    for group in history['_segment_index']:
        if group['first'] > end:
            break
        last = min(group['last'], end)
        result.append(dict(segment=group['segment'], start_s=events[group['first']]['t_s'],
                           end_s=events[last]['t_s'], scan_count=last-group['first']+1))
    return result


def rows(path):
    with gzip.open(path,'rt',encoding='utf-8') as stream:
        for line in stream:yield json.loads(line)


def prepare_trajectory(source, progress):
    path=source/'sensors.jsonl.gz';manifest=read_json(source/'manifest.json')
    signature=digest(path)
    if not manifest.get('complete') or signature!=(manifest.get('output_sha256') or {}).get('sensors.jsonl.gz'):
        raise ValueError('unverified_mapping_sensor_source')
    trajectory=read_json(source/'trajectory.json')
    fingerprint=localization_fingerprint()
    if (trajectory and trajectory.get('source_sensor_sha256')==signature
            and trajectory.get('localization_fingerprint')==fingerprint):
        return trajectory
    imus=[];gps=[]
    for row in rows(path):
        if row['topic']=='/imu/data_raw':imus.append(row['message'])
        if row['topic']=='/gps/data':gps.append(json.loads(row['message']['data']))
    imus.sort(key=source_stamp);stamps=[source_stamp(m) for m in imus]
    engine=IntegratedMapping(background_gnss=False)
    for sample in gps:engine.gps(sample)
    for row in rows(path):
        if row['topic']!='/scan':continue
        stamp=source_stamp(row['message']);index=bisect.bisect_left(stamps,stamp)
        choices=[i for i in (index-1,index) if 0<=i<len(stamps)]
        imu=imus[min(choices,key=lambda i:abs(stamps[i]-stamp))] if choices else None
        engine.add(row['message'],imu,draw=False)
        if engine.input_scans%100==0:progress('localizing',engine.input_scans/max(1,manifest['topic_counts']['/scan']),0)
    segments=engine.old_segments+[dict(segment=engine.segment,records=engine.records)]
    corrected={};diagnostics=[]
    for segment in segments:
        records=segment['records']
        if not records:continue
        poses,proof,_=corrected_poses(records,engine.gnss.accepted)
        frame='fixed_archive_ENU_unqualified' if proof['state']=='GNSS_position_constrained' else f'isolated_local_segment_{segment["segment"]}'
        for row,pose in zip(records,poses):corrected[str(row['stamp'])]=dict(segment=segment['segment'],pose=pose.tolist(),frame=frame)
        diagnostics.append(dict(segment=segment['segment'],gnss=proof,accepted_scans=len(records)))
    result=dict(archive_id=manifest['archive_id'],source_sensor_sha256=signature,localization_fingerprint=fingerprint,corrected=corrected,
                segments=diagnostics,autonomy_eligible=False)
    previous=source/'trajectory.json'
    if previous.is_file():
        backup=source/('trajectory-previous-'+digest(previous)[:16]+'.json')
        if not backup.exists():backup.write_bytes(previous.read_bytes())
    atomic_json(source/'trajectory.json',result)
    return result


def build(archive, signature, progress):
    # A failed rebuild must not replace a usable saved map with partial data.
    target=Path(archive)/'integrated_map_history.json'
    original=target.read_bytes() if target.is_file() else None
    try:
        return _build(archive,signature,progress)
    except Exception:
        if original is not None:
            temporary=target.with_suffix('.restore.tmp');temporary.write_bytes(original);temporary.replace(target)
        elif target.is_file():target.unlink()
        raise


def _build(archive, signature, progress):
    archive=Path(archive);source=archive/'mapping_source';trajectory=prepare_trajectory(source,progress)
    epoch=uuid.uuid4().hex;folder=archive/'integrated_map'/epoch;folder.mkdir(parents=True)
    metadata=read_json(archive/'metadata.json');start=datetime.fromisoformat(metadata['started_at']).timestamp()
    duration=float(metadata.get('duration_s') or 0);events=[];checkpoints=[]
    grid=None;segment=None;next_checkpoint=-float('inf');version=0;grids={}
    history=dict(schema='ubts.integrated_archive.v1',epoch=epoch,source_signature=signature,
                 cell_transport='tiles-cells-v1',
                 localization_fingerprint=trajectory['localization_fingerprint'],
                 raster_fingerprint=raster_fingerprint(),hit_endpoints='all_finite_returns',
                 free_rays='all_observed_scan_beams',
                 complete=False,events=events,checkpoints=checkpoints,duration_s=duration)
    target=archive/'integrated_map_history.json'
    if target.is_file():
        previous=archive/('integrated_map_history-previous-'+digest(target)[:16]+'.json')
        if not previous.exists():previous.write_bytes(target.read_bytes())
    file=folder/'events.jsonl'
    with file.open('wb') as log:
        for row in rows(source/'sensors.jsonl.gz'):
            if row['topic']!='/scan':continue
            scan=row['message'];entry=trajectory['corrected'].get(str(source_stamp(scan)))
            if entry is None:continue
            changed=segment!=entry['segment']
            if changed:
                segment=entry['segment'];grid=StreamingOccupancyMap(max_beams=450,laser_x=0.,laser_y=0.)
                grids[segment]=grid;next_checkpoint=-float('inf')
            previous=grid.scan_count;t_s=max(0,row['bag_time_ns']/1e9-start)
            integrate(grid,scan,entry['pose'],t_s);version+=1
            packet=grid.tile_packet(None if changed else previous)
            # The complete path is recovered from a checkpoint plus increments.
            packet['path']=grid.path[-1:] if grid.path and (version==1 or grid.path[-1].get('stamp')==source_stamp(scan)) else []
            packet.update(ok=True,version=version,tile_version=version,segment=segment,frame_t_s=t_s,
                tile_epoch=f'{epoch}:{segment}',map_epoch=epoch,preserve_view_on_seek=True,
                coordinate_frame=entry['frame'],pose_accuracy_validated=False,autonomy_eligible=False,
                source='recorded_original_LaserScan_raw_IMU_soft_GNSS',runtime_authority=False,
                recording_start_stamp=start,gnss=dict(state='position_constrained' if 'ENU' in entry['frame'] else 'insufficient_motion_baseline'),no_encoder=True)
            encoded=(json.dumps(packet,separators=(',',':'),allow_nan=False)+'\n').encode()
            events.append(dict(t_s=t_s,offset=log.tell(),bytes=len(encoded),version=version,segment=segment))
            log.write(encoded)
            if t_s>=next_checkpoint:
                checkpoint=grid.tile_packet();checkpoint.update({k:v for k,v in packet.items() if k not in ('tiles','path','tiles_reset')})
                name=f'checkpoint-{len(checkpoints):06d}.json'
                atomic_json(folder/name,checkpoint);checkpoints.append(dict(index=len(events)-1,t_s=t_s,file=name,segment=segment))
                next_checkpoint=t_s+10.
                log.flush();history.update(progress=min(1,t_s/max(duration,.001)),processed_t_s=t_s)
                atomic_json(target,history);progress('drawing',history['progress'],t_s)
        log.flush()
    for key,grid in grids.items():grid.save_checkpoint(folder/f'native-segment-{key}.json')
    history.update(complete=True,progress=1,segments=len(grids),processed_t_s=duration,
                   source_sensor_sha256=trajectory['source_sensor_sha256'])
    atomic_json(target,history);progress('complete',1,duration)
    return history


class IndexedTileHistory:
    # C1: windows of a complete (immutable) history are cached. Key: history
    # file identity (mtime_ns, size), start rounded down to 0.5 s, span, codec.
    WINDOW_CACHE_ENTRIES=32
    WINDOW_CACHE_BYTES=64*1024*1024

    def __init__(self):
        self.lock=threading.Lock();self.cache={}
        from collections import OrderedDict
        self.window_cache=OrderedDict();self.window_cache_bytes=0
        self.window_inflight={};self.window_lock=threading.Lock()
        self.window_stats=dict(hits=0,misses=0,shared=0,stored=0,evicted=0)

    def _window_key(self,archive,start,span,codec):
        file=Path(archive)/'integrated_map_history.json'
        try:stat=file.stat()
        except OSError:return None
        with self.lock:
            cached=self.cache.get(str(file))
        if not cached or cached[0]!=stat.st_mtime_ns or not cached[1].get('complete'):return None
        return (str(file),stat.st_mtime_ns,stat.st_size,start,span,codec)

    def cached_window(self,archive,start_s,span_s=12,codec=None):
        """Return a cached window or None, without computing (autonomy priority)."""
        try:start=float(start_s);span=float(span_s)
        except (TypeError,ValueError):return None
        if not np.isfinite(start) or not np.isfinite(span):return None
        start=max(0.,np.floor(start*2)/2)
        try:self._load_history(archive)
        except (OSError,ValueError):return None
        key=self._window_key(archive,start,span,codec)
        with self.window_lock:
            value=self.window_cache.get(key) if key else None
            if value is not None:
                self.window_cache.move_to_end(key);self.window_stats['hits']+=1
                return value[0]
        return None

    def window(self,archive,start_s,span_s=12,codec=None):
        start=float(start_s);span=float(span_s)
        if not np.isfinite(start) or not np.isfinite(span) or not 0<=start<=86400 or not 1<=span<=24:
            raise ValueError('invalid_map_window')
        self._load_history(archive)  # refresh the parsed history (and its completeness) cheaply
        rounded=max(0.,float(np.floor(start*2)/2))
        key=self._window_key(archive,rounded,span,codec)
        if key is None:return self._window(archive,start,span,codec)
        with self.window_lock:
            cached=self.window_cache.get(key)
            if cached is not None:
                self.window_cache.move_to_end(key);self.window_stats['hits']+=1
                return cached[0]
            pending=self.window_inflight.get(key)
            owner=pending is None
            if owner:
                pending=self.window_inflight[key]=dict(event=threading.Event(),value=None,error=None)
                self.window_stats['misses']+=1
            else:self.window_stats['shared']+=1
        if not owner:
            pending['event'].wait(60)
            if pending['error'] is not None:raise pending['error']
            if pending['value'] is None:raise ValueError('map_window_timeout')
            return pending['value']
        try:
            value=self._window(archive,rounded,span,codec)
            pending['value']=value
            size=len(json.dumps(value,separators=(',',':')))
            with self.window_lock:
                if size<=self.WINDOW_CACHE_BYTES//4:
                    self.window_cache[key]=(value,size);self.window_cache_bytes+=size;self.window_stats['stored']+=1
                    while self.window_cache and (len(self.window_cache)>self.WINDOW_CACHE_ENTRIES or
                                                 self.window_cache_bytes>self.WINDOW_CACHE_BYTES):
                        _,(old,old_size)=self.window_cache.popitem(last=False)
                        self.window_cache_bytes-=old_size;self.window_stats['evicted']+=1
            return value
        except BaseException as error:
            pending['error']=error
            raise
        finally:
            with self.window_lock:self.window_inflight.pop(key,None)
            pending['event'].set()

    def _window(self,archive,start,span,codec=None):
        seed=self.status(archive,start)
        if seed is None:return dict(ok=True,available=False)
        history=self.cache[str(Path(archive)/'integrated_map_history.json')][1]
        events=history['events'];end=min(start+span,history['duration_s'])
        selected=[event for event in events if start<event['t_s']<=end]
        folder=Path(archive)/'integrated_map'/history['epoch'];values=[]
        with (folder/'events.jsonl').open('rb') as stream:
            for event in selected:
                stream.seek(event['offset']);values.append(json.loads(stream.read(event['bytes'])))
        coverage=end if history.get('complete') else min(end,history.get('processed_t_s',0))
        result=dict(ok=True,available=True,start_s=start,end_s=coverage,seed=seed,events=values,
                    complete=history.get('complete',False),duration_s=history['duration_s'],
                    segment_catalog=segment_catalog(history, coverage))
        if codec in ('xor2-v1','xor2-deflate-v1'):
            from dashboard_server.window_cells import compact_window
            result=compact_window(result,codec)
        return result

    def _load_history(self,archive):
        file=Path(archive)/'integrated_map_history.json'
        if not file.is_file():return None
        if file.is_symlink() or file.stat().st_size>64*1024*1024:raise ValueError('invalid_integrated_history')
        stamp=file.stat().st_mtime_ns
        with self.lock:
            cached=self.cache.get(str(file))
            if cached and cached[0]==stamp:history=cached[1]
            else:
                history=read_json(file);self.cache[str(file)]=(stamp,history)
                if len(self.cache)>16:self.cache.pop(next(iter(self.cache)))
        return history

    def status(self,archive,t_s,since=None,tile_epoch=None):
        history=self._load_history(archive)
        if history is None:return None
        if history.get('schema')!='ubts.integrated_archive.v1':return None
        outdated=(history.get('localization_fingerprint')!=localization_fingerprint()
                  or history.get('raster_fingerprint')!=raster_fingerprint())
        import re
        if not re.fullmatch(r'[0-9a-f]{32}',str(history.get('epoch'))):raise ValueError('invalid_integrated_epoch')
        events=history.get('events',[]);times=[e['t_s'] for e in events]
        target=float(t_s) if t_s is not None else float('inf')
        index=bisect.bisect_right(times,target)-1
        if index<0:return dict(ok=True,version=0,scan_count=0,map_epoch=history['epoch'],
                               preserve_view_on_seek=True,frame_t_s=0,tiles=[],tiles_reset=True,
                               needs_regeneration=outdated,playback_driven=history.get('playback_driven',False),
                               history_available=True,tracking=history.get('tracking'),
                               mapping_error=history.get('mapping_error'),
                               state='outdated' if outdated else 'saved' if history.get('complete') else 'regenerating',
                               generation=dict(stage='complete' if history.get('complete') else 'drawing',
                                   processed_t_s=history.get('processed_t_s',0),duration_s=history['duration_s'],waiting_for_cursor=history.get('waiting_for_cursor')))
        segment=events[index]['segment'];epoch=f'{history["epoch"]}:{segment}'
        folder=Path(archive)/'integrated_map'/history['epoch']
        try:since=int(since)
        except (TypeError,ValueError):since=-1
        incremental=(tile_epoch==epoch and 0<=since<=events[index]['version'])
        first=bisect.bisect_right([e['version'] for e in events],since) if incremental else 0
        packet={};tiles={};path=[]
        if not incremental:
            checkpoint=[c for c in history['checkpoints'] if c['index']<=index and c['segment']==segment][-1]
            packet=read_json(folder/checkpoint['file']);first=checkpoint['index']+1
            tiles={(t['x'],t['y']):t for t in packet['tiles']};path=list(packet['path'])
        with (folder/'events.jsonl').open('rb') as stream:
            for event in events[first:index+1]:
                stream.seek(event['offset']);value=json.loads(stream.read(event['bytes']))
                if value.get('tiles_reset'):tiles={};path=[]
                tiles.update({(t['x'],t['y']):t for t in value['tiles']});path.extend(value.get('path',[]));packet=value
            if not packet:
                event=events[index];stream.seek(event['offset']);packet=json.loads(stream.read(event['bytes']))
        return {**packet,'tiles':list(tiles.values()),'tiles_reset':not incremental,
                'path':path,'path_append':incremental,'tile_version':events[index]['version'],
                'map_epoch':history['epoch'],'tile_epoch':epoch,'segments':history.get('segments'),
                'segment_catalog':segment_catalog(history, target),
                'state':'outdated' if outdated else 'saved' if history.get('complete') else 'regenerating',
                'generation':dict(stage='complete' if history.get('complete') else 'drawing',
                    progress=history.get('progress',0),processed_t_s=history.get('processed_t_s',0),duration_s=history['duration_s'],waiting_for_cursor=history.get('waiting_for_cursor')),
                'history_available':True,'needs_regeneration':outdated,
                'hit_endpoints':history.get('hit_endpoints','sampled_returns'),
                'free_rays':history.get('free_rays','sampled_scan_beams'),
                'localization_fingerprint':history.get('localization_fingerprint'),
                'raster_fingerprint':history.get('raster_fingerprint'),
                'playback_driven':history.get('playback_driven',False),
                'tracking':history.get('tracking',packet.get('tracking')) if target>=history.get('processed_t_s',0) else packet.get('tracking'),
                'mapping_error':history.get('mapping_error')}
