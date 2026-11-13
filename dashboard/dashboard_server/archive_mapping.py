"""Persist renderer results beside raw archives, without rewriting raw data."""
import base64
import json
import math
from pathlib import Path
import re
import threading
import uuid
from urllib.request import Request, urlopen

from dashboard_server.occupancy_map import atomic_json, read_json


def renderer_snapshot(png,meta,archive_id):
    import cv2
    import numpy as np
    image=cv2.imdecode(np.frombuffer(png,dtype=np.uint8),cv2.IMREAD_COLOR)
    if image is None or image.shape[0]*image.shape[1]>4_194_304:raise ValueError('render_size_exceeded')
    # Renderer bounds are [minX,maxX,minY,maxY], not an XYXY bounding box.
    xmin,xmax,ymin,ymax=meta['render_world_bounds_m']
    h,w=image.shape[:2];resolution=(xmax-xmin)/w
    if resolution<=0 or abs(resolution-(ymax-ymin)/h)>.001:raise ValueError('non_square_grid_scale')
    # Retain the existing renderer's permanent age gradient, showing its rover
    # marker in red consistently with the live dashboard.
    green=((image==[94,197,34]).all(axis=2)|(image==[61,128,21]).all(axis=2))
    image[green]=[67,50,240]
    image=cv2.flip(image,0)
    # This renderer's observed probability palette is gray=245-220*p and
    # unknown=146. Red path/rover decorations are not occupancy. Supply bounds
    # directly; the browser's legacy 25/127/245 palette cannot decode this map.
    occupied=(image[:,:,0]<=102)&(image[:,:,0]==image[:,:,1])&(image[:,:,1]==image[:,:,2])
    ys,xs=np.nonzero(occupied)
    bounds=None if not len(xs) else dict(minX=xmin+int(xs.min())*resolution,minY=ymin+int(ys.min())*resolution,
                                        maxX=xmin+(int(xs.max())+1)*resolution,maxY=ymin+(int(ys.max())+1)*resolution)
    ok,encoded=cv2.imencode('.png',image)
    if not ok:raise ValueError('render_encode_failed')
    return dict(ok=True,scan_count=int(meta.get('frame_index',0))+1,version=int(meta.get('frame_index',0))+1,map_epoch='archive-'+archive_id,
                coordinate_frame='archive_unqualified_lidar_imu_display',path=[],pose=None,
                runtime_authority=False,source='raw_recorded_sensors',renderer_metadata=meta,
                grid=dict(width=w,height=h,resolution=resolution,origin=dict(x=xmin,y=ymin),encoding='png',row_order='min_y_first',
                          occupied_bounds=bounds,occupied_bounds_source='renderer_probability_gray_threshold_p_ge_0_65',image_base64=base64.b64encode(encoded).decode()))


class ArchiveMapping:
    def __init__(self, root, renderer='http://127.0.0.1:8810'):
        self.root=Path(root);self.renderer=renderer.rstrip('/')
        self.lock=threading.Lock();self.running=set();self.errors={};self.progress={}
        from dashboard_server.integrated_archive import IndexedTileHistory
        self.tile_history=IndexedTileHistory()
        self.playback_mappers={}

    def close(self):
        with self.lock:workers=list(self.playback_mappers.values())
        for worker in workers:worker.close()
        for worker in workers:
            if worker.thread:worker.thread.join(timeout=5)

    def new_map(self,archive_id):
        path=self.directory(archive_id)
        from dashboard_server.playback_mapping import new_history
        from dashboard_server.resumable_playback import ResumablePlaybackMapper as PlaybackMapper
        with self.lock:
            if self.running:raise ValueError('기존 지도 작업 완료 후 다시 시도하세요.')
            if len(self.playback_mappers)>=2 and archive_id not in self.playback_mappers:
                for other in self.playback_mappers.values():other.close()
                self.playback_mappers.clear()
            previous=self.playback_mappers.pop(archive_id,None)
            if previous:previous.close()
            history=new_history(path,self.signature(path))
            self.playback_mappers[archive_id]=PlaybackMapper(path,history)
            self.errors.pop(archive_id,None);self.progress.pop(archive_id,None)
        return dict(ok=True,state='waiting_for_play',map_epoch=history['epoch'],playback_driven=True)

    def playback(self,archive_id,t_s,playing,control=None):
        path=self.directory(archive_id)
        from dashboard_server.resumable_playback import ResumablePlaybackMapper as PlaybackMapper
        history=read_json(path/'integrated_map_history.json')
        # Old segmented recordings convert once, on Play. Keep their previous
        # pointer/epochs recoverable and never open a new frame during tracking.
        if playing is True and not history.get('playback_driven') and (path/'mapping_source/sensors.jsonl.gz').is_file():
            self.new_map(archive_id)
        with self.lock:
            mapper=self.playback_mappers.get(archive_id)
            history=mapper.history if mapper and not mapper.closed else read_json(path/'integrated_map_history.json')
            if not history.get('playback_driven'):return dict(ok=True,playback_driven=False)
            if mapper and mapper.closed:
                if mapper.thread:mapper.thread.join(timeout=2)
                if mapper.running:raise ValueError('이전 지도 작업 정리 중입니다. 잠시 후 다시 시도하세요.')
                mapper=None
            if mapper is None:
                if len(self.playback_mappers)>=2:
                    for other in self.playback_mappers.values():other.close()
                    self.playback_mappers.clear()
                mapper=PlaybackMapper(path,history);self.playback_mappers[archive_id]=mapper
            for name,other in self.playback_mappers.items():
                if name!=archive_id and other.running:other.close()
            return mapper.advance(t_s,playing,control)

    def directory(self, archive_id):
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}',archive_id) or archive_id in ('.','..'):
            raise ValueError('invalid_archive_id')
        path=self.root/archive_id
        if not path.is_dir() or path.is_symlink():raise ValueError('archive_not_found')
        return path

    def signature(self, path):
        result={}
        for name in ('metadata.json','timeline.json','gps.jsonl','motor.jsonl'):
            source=path/name
            if source.is_file():
                s=source.stat();result[name]=[s.st_size,s.st_mtime_ns]
        return result

    def status(self, archive_id, t_s=None, since=None, transport=None, tile_since=None, tile_epoch=None):
        path=self.directory(archive_id)
        target=None
        if t_s is not None:
            try:target=float(t_s)
            except (TypeError,ValueError):raise ValueError('invalid_map_time')
            if not math.isfinite(target) or not 0<=target<=86400:raise ValueError('invalid_map_time')
        with self.lock:
            running=archive_id in self.running;error=self.errors.get(archive_id)
            progress=dict(self.progress.get(archive_id,{}) )
            mapper=self.playback_mappers.get(archive_id)
        def cursor_progress(indexed):
            if indexed.get('playback_driven'):
                active=bool(mapper and not mapper.closed and mapper.running and
                            mapper.history.get('epoch')==indexed.get('map_epoch'))
                indexed['generation']={**indexed.get('generation',{}),'worker_active':active,
                    'mapping_paused':not active or not mapper.playing,
                    'waiting_for_cursor':not active or not mapper.playing or bool(mapper.history.get('waiting_for_cursor'))}
            return indexed
        if transport=='tiles':
            indexed=self.tile_history.status(path,target,tile_since,tile_epoch)
            if indexed is not None:
                indexed=cursor_progress(indexed)
                return {**indexed,'regeneration_error':error,'state':'regenerating' if running else indexed.get('state','saved'),
                        'generation':progress or indexed.get('generation',{})}
        if read_json(path/'integrated_map_history.json').get('playback_driven'):
            # A stale legacy final PNG must not reappear after a new-map reset.
            indexed=self.tile_history.status(path,target,tile_since,tile_epoch)
            if indexed is not None:return cursor_progress(indexed)
        history=read_json(path/'dashboard_occupancy_grid_history.json')
        value={};needs_regeneration=False
        if history:
            if history.get('source_signature')!=self.signature(path):
                needs_regeneration=True
            else:
                choices=[r for r in history.get('snapshots',[]) if target is None or float(r['t_s'])<=target]
                if choices:
                    row=choices[-1];epoch=history.get('epoch','');filename=row.get('file','')
                    if not re.fullmatch(r'[0-9a-f]{32}',epoch) or not re.fullmatch(r'snapshot-[0-9]{6}\.json',filename):
                        raise ValueError('invalid_map_history')
                    value=read_json(path/'dashboard_occupancy_grid_history'/epoch/filename)
                progress=progress or {'stage':'complete' if history.get('complete') else 'interrupted',
                                      'progress':history.get('progress',0),'processed_t_s':history.get('processed_t_s',0),
                                      'duration_s':history.get('duration_s',0)}
                if not value:value=dict(map_epoch=history.get('epoch'),preserve_view_on_seek=True,scan_count=0,version=0)
        else:
            legacy=read_json(path/'dashboard_occupancy_grid.json')
            # A historical final PNG cannot represent the map at an earlier
            # playback time. Keep the original and ask for indexed regeneration.
            duration=float(read_json(path/'metadata.json').get('duration_s') or 0)
            if target is None or target>=duration:value=legacy
            elif legacy:needs_regeneration=True
        token=f"{value.get('map_epoch')}:{value.get('version')}"
        if value.get('grid') and since==token:
            value={**value,'grid':{k:v for k,v in value['grid'].items() if k!='image_base64'}}
        state='regenerating' if running else 'outdated' if needs_regeneration else 'saved' if history.get('complete') else 'interrupted' if history else 'saved' if value else 'not_generated'
        return {**value,'ok':True,'state':state,
                'generation':progress,'history_available':bool(history),'needs_regeneration':needs_regeneration,
                'requested_t_s':target,'regeneration_error':error}

    def regenerate(self, archive_id):
        self.directory(archive_id)
        with self.lock:
            if archive_id in self.running:return {'ok':True,'state':'regenerating'}
            # One CPU-intensive archive render at a time; live streams retain
            # their separate latest-only readers and hardware owners.
            if self.running:raise ValueError('다른 지도를 만드는 중입니다. 완료 후 다시 시도하세요')
            self.running.add(archive_id);self.errors.pop(archive_id,None)
            self.progress[archive_id]={'stage':'localizing','progress':0,'processed_t_s':0}
        threading.Thread(target=self._render,args=(archive_id,),daemon=True,name='archive-map').start()
        return {'ok':True,'state':'regenerating'}

    def window(self,archive_id,start_s,span_s=12,codec=None):
        return self.tile_history.window(self.directory(archive_id),start_s,span_s,codec)

    def _render(self, archive_id):
        session_id=None
        try:
            path=self.directory(archive_id);signature=self.signature(path)
            if (path/'mapping_source/sensors.jsonl.gz').is_file():
                from dashboard_server.integrated_archive import build
                def progress(stage,ratio,time_s):
                    with self.lock:self.progress[archive_id]=dict(stage=stage,progress=ratio,processed_t_s=time_s,
                        duration_s=float(read_json(path/'metadata.json').get('duration_s') or 0))
                build(path,signature,progress)
                return
            epoch=uuid.uuid4().hex
            folder=path/'dashboard_occupancy_grid_history'/epoch;folder.mkdir(parents=True)
            history=dict(schema='orchard.archive_map_history.v1',epoch=epoch,archive_id=archive_id,
                         source_signature=signature,complete=False,snapshots=[],progress=0,processed_t_s=0)
            atomic_json(path/'dashboard_occupancy_grid_history.json',history)
            body=json.dumps({'archive_id':archive_id}).encode()
            with urlopen(Request(self.renderer+'/api/v1/sessions',data=body,headers={'Content-Type':'application/json'}),timeout=180) as response:
                session=json.load(response)
            session_id=session['session_id']
            duration=float(session.get('duration_s') or 0)
            if not math.isfinite(duration) or not 0<duration<=86400:raise ValueError('invalid_archive_duration')
            history['duration_s']=duration
            # Every skipped LiDAR scan is still integrated by the renderer. PNG
            # checkpoints are bounded to ~240 plus the first/final frame.
            stride=max(2.,math.ceil(duration/240))
            history['snapshot_interval_s']=stride
            times=[float(t) for t in session.get('frame_times_s',[]) if 0<=float(t)<=duration]
            targets=[times[0] if times else 0.]+list(sample_times(stride,duration,stride))+[duration]
            previous_frame=-1
            for target in targets:
                with urlopen(self.renderer+f'/api/v1/sessions/{session_id}/frame.png?t_s={target}',timeout=180) as response:
                    png=response.read(16*1024*1024+1)
                    meta=json.loads(response.headers['X-OGM-Meta'])
                if len(png)>16*1024*1024:raise ValueError('render_png_too_large')
                frame=int(meta['frame_index']);frame_t=float(meta['frame_t_s'])
                if frame_t>target+1e-6:raise ValueError('renderer_returned_future_map')
                if frame==previous_frame:continue
                previous_frame=frame
                value=renderer_snapshot(png,meta,archive_id)
                value.update(map_epoch=epoch,frame_t_s=frame_t,preserve_view_on_seek=True,
                             source_signature=signature,renderer_fingerprint=session.get('archive_fingerprint'))
                filename=f'snapshot-{len(history["snapshots"]):06d}.json'
                atomic_json(folder/filename,value)
                history['snapshots'].append(dict(t_s=frame_t,file=filename,version=value['version']))
                history.update(progress=min(1,target/duration),processed_t_s=frame_t)
                if self.signature(path)!=signature:raise ValueError('archive_changed_during_mapping')
                atomic_json(path/'dashboard_occupancy_grid_history.json',history)
                with self.lock:self.progress[archive_id]={'stage':'drawing','progress':history['progress'],
                                                        'processed_t_s':frame_t,'duration_s':duration}
            if not history['snapshots']:raise ValueError('no_map_snapshots')
            atomic_json(path/'dashboard_occupancy_grid.json',value)
            history.update(complete=True,progress=1)
            atomic_json(path/'dashboard_occupancy_grid_history.json',history)
            with self.lock:self.progress[archive_id]={'stage':'complete','progress':1,'processed_t_s':frame_t,'duration_s':duration}
        except Exception as error:
            with self.lock:
                self.errors[archive_id]=str(error)[:240]
                self.progress[archive_id]={**self.progress.get(archive_id,{}),'stage':'failed'}
        finally:
            if session_id:
                try:
                    with urlopen(Request(self.renderer+f'/api/v1/sessions/{session_id}',method='DELETE'),timeout=5):pass
                except OSError:pass
            with self.lock:self.running.discard(archive_id)


def sample_times(start,stop,step):
    # Avoid an extra NumPy dependency for the history index itself.
    while start<stop:
        yield start
        start+=step
