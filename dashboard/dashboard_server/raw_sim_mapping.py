"""Continuous SIM mapping from native LiDAR, IMU and GNSS observations only.

Source clocks, field-tested numerical core and retained signed cells match the
actual dashboard. This display/archive mapper never publishes a planning map
or certifies autonomy. The legacy wheel-based mapper remains available for
historical archives but cannot enter this source-only estimator.
"""
import collections
from contextlib import nullcontext
import gzip
import json
import math
from pathlib import Path
import re
import socket
import threading
import time
import uuid

from scripts.sim_clock import freshness
from scripts.native_stage_timing import Recorder
from .sensor_mapping.continuous_mapping import ContinuousMapping
from .sensor_mapping.raw_mapping.raw_localization import source_stamp
from .sensor_mapping.occupancy_map import atomic_json, read_json
from .sensor_mapping.tile_transport import StreamingOccupancyMap
from .sim_mapping import SimulationMapping
from .native_navigation import NativeNavigationWorker

SCHEMA = 'ubts.raw_sim_sensor_source.v1'
WAKE_SCHEMA = 'ubts.raw_sim_sensor_wake.v1'
MAX_SOURCE_WAIT_MS = 20
ALGORITHM = 'sim-native-continuous-LiDAR-IMU-quality-GNSS-v1'


def source_wake_supported(response):
    value = response.get('native_journal_wake') if isinstance(response, dict) else None
    return bool(type(value) is dict and set(value) == {'schema','max_wait_ms','closed'}
        and value.get('schema') == WAKE_SCHEMA
        and type(value.get('max_wait_ms')) is int and value['max_wait_ms'] == MAX_SOURCE_WAIT_MS
        and value.get('closed') is False)


def read_source(path, session, cursor, follow_tail=False, stage_timing=None, wait_ms=0):
    if type(wait_ms) is not int or not 0 <= wait_ms <= MAX_SOURCE_WAIT_MS:
        raise ValueError('invalid_bounded_journal_wait')
    recorder=stage_timing;start=recorder.now() if recorder else None
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(1.5)
        connection.connect(str(path))
        if recorder:recorder.observe('client_rpc_connect',start,recorder.now())
        wire_start=recorder.now() if recorder else None
        request = dict(schema=SCHEMA, session=session, cursor=cursor,follow_tail=follow_tail)
        if wait_ms:request['wait_ms']=wait_ms
        connection.sendall(json.dumps(request).encode()+b'\n')
        with connection.makefile('rb') as stream:
            data = stream.readline(8*1024*1024+1)
    if not data.endswith(b'\n') or len(data) > 8*1024*1024:
        raise ValueError('raw_source_response_size')
    received=recorder.now() if recorder else None
    if recorder:recorder.observe('client_rpc_wire',wire_start,received)
    result=json.loads(data)
    if recorder:recorder.observe('client_rpc_decode',received,recorder.now())
    return result


class RawSimulationMapping(SimulationMapping):
    def __init__(self, source_socket='/run/ubts-sim/sensor-source.sock',
                 state_dir='/run/ubts-sim/raw-mapping', archive_dir='/runtime/recordings',
                 start_worker=True, stage_timing=None):
        # Do not construct/start the legacy latest wheel-pose mailbox worker.
        self.stage_timing=stage_timing if isinstance(stage_timing,Recorder) else Recorder(stage_timing)
        from .sim_native_owner_proxy import optional_archive_priority
        self.native_priority = optional_archive_priority()
        self.source_socket = Path(source_socket)
        self.state_dir, self.archive_dir = Path(state_dir), Path(archive_dir)
        self.lock = threading.RLock()
        self.processing_lock = threading.RLock();self.wakeup = threading.Event()
        self.engine = ContinuousMapping()
        self.map = self.engine.map
        self.map_epoch = uuid.uuid4().hex
        self.localization_epoch = None
        self.active = False;self.state = 'idle';self.version = 0
        self.last_seq = None;self.last_stamp = None;self.last_seen = 0.
        self.session = None;self.cursor = 0;self.scans = collections.deque()
        self.native_pending = collections.deque(maxlen=512);self.native_loss = 0
        self.error = None;self.archive = None;self.persistence = {}
        self._packet_key = None;self._packet = None
        self._regenerating = set();self._stop = threading.Event();self.thread = None;self.source_thread = None
        self.input_scans = 0;self.source_loss = 0;self.source_gap = None
        self.processing=False;self.source_polled=None;self.last_progress=None;self.jobs_by_id={}
        self.navigation = NativeNavigationWorker(start_worker=start_worker)
        self._publish_snapshot()
        self._restore_saved_map()
        if start_worker:
            self.source_thread = threading.Thread(target=self._receive_source,daemon=True,name='raw-sim-source')
            self.source_thread.start()
            self.thread = threading.Thread(target=self._run, daemon=True, name='raw-sim-map')
            self.thread.start()

    def _publish_planning(self):
        # A sensor-local map cannot replace the wheel/odom frame used by the
        # legacy controller. Controller output remains independent and gated.
        return

    def _publish_snapshot(self, refresh_cells=True):
        # IMU/GNSS ingestion and source polling change status, not the grid.
        # Reuse the completed immutable cell generation until a scan changes
        # it. Never re-encode/re-list the whole retained map for each IMU batch.
        publishing=self.stage_timing.now()
        key = (self.map, self.map.scan_count)
        if refresh_cells and key != self._packet_key:
            tile_start=self.stage_timing.now()
            packet = self.map.tile_packet()
            self.stage_timing.observe('snapshot_tiles',tile_start,self.stage_timing.now())
            packet['tiles'] = [dict(t, revision=self.map.tile_versions.get((t['x'], t['y']), 0))
                               for t in packet['tiles']]
            self._packet_key = key;self._packet = packet
        packet = self._packet
        self._snapshot = dict(ok=True, simulated=True, active=self.active,
            state=self.state, version=self.version, algorithm=ALGORITHM,
            map_epoch=self.map_epoch, tile_epoch=self.map_epoch,
            localization_epoch=self.localization_epoch, **packet, **self.engine.info(),
            input_scans=self.input_scans, source_loss=self.source_loss,
            source_gap=self.source_gap, persistence=dict(self.persistence))
        self.stage_timing.observe('snapshot_publish',publishing,self.stage_timing.now())
        timing=self.stage_timing.snapshot()
        if timing is not None:self._snapshot['diagnostic']['stage_timing']=dict(timing,source_process=getattr(self,'source_stage_counters',None))
        self._cell_snapshot = self._snapshot

    def snapshot(self, since=None, transport=None, tile_since=None, tile_epoch=None):
        # Completed immutable generations; HTTP does not wait on registration.
        data = self._cell_snapshot
        timing=self.stage_timing.snapshot()
        if timing is not None:data=dict(data,diagnostic=dict(data['diagnostic'],stage_timing=dict(timing,source_process=getattr(self,'source_stage_counters',None))))
        try:revision = int(tile_since) if tile_epoch == data['tile_epoch'] else -1
        except (ValueError, TypeError):revision = -1
        full = not 0 <= revision <= data['tile_version']
        return dict(data, tiles=data['tiles'] if full else
                    [t for t in data['tiles'] if t['revision'] > revision],
                    tiles_reset=full, scan_age_s=self._age(), error=self.error,
                    navigation=self.navigation.snapshot())

    def _restore_saved_map(self):
        manifest=self.state_dir/'last_map.json'
        saved = read_json(manifest)
        if not saved:
            if manifest.exists():
                self.error='saved_map_restore: invalid_manifest'
                self.state='reset_detected';self._publish_snapshot()
            return
        try:
            from .sensor_mapping.tiled_checkpoint import digest
            epoch = saved['map_epoch']
            if not re.fullmatch(r'[0-9a-f]{32}', epoch):raise ValueError('invalid_saved_map_epoch')
            path = self.state_dir/'last_native_map.json'
            if digest(path) != saved['persistence']['manifest_sha256']:
                raise ValueError('saved_map_generation_mismatch')
            grid = StreamingOccupancyMap.load_checkpoint(path)
            if grid.scan_count != saved['scan_count']:raise ValueError('saved_map_generation_mismatch')
            if not grid.scan_count:return
            # Rebuild display indexes without modifying signed log odds.
            for key, tile in grid._tiles.items():
                grid.dirty_bounds.add(key)
                grid.tile_versions[key] = grid.scan_count
            self.map = grid;self.map_epoch = epoch
            self.version = int(saved['version']);self.state = 'reset_detected'
            self.persistence = dict(saved['persistence'], restored=True, requires_new_localization=True)
            self._rebuild_classes(grid)
            self._publish_snapshot()
        except (OSError, ValueError, TypeError, KeyError, EOFError) as exc:
            self.error = 'saved_map_restore: '+str(exc)[:160]
            self.state='reset_detected'
            self._publish_snapshot()

    @staticmethod
    def _rebuild_classes(grid):
        import numpy as np
        for key, tile in grid._tiles.items():
            states = grid.cell_states(tile);grid.tile_classes[key] = states
            ys, xs = np.nonzero(states == 2)
            if len(xs):grid.object_bounds[key] = (key[0]*64+int(xs.min()), key[1]*64+int(ys.min()),
                                                key[0]*64+int(xs.max())+1, key[1]*64+int(ys.max())+1)
        grid.dirty_bounds.clear()

    def _receive_source(self):
        wake_supported = False
        while True:
            waiting=self.stage_timing.now()
            active = bool(self.active or self.archive)
            delay = (0. if wake_supported else .02) if active else .3
            if self._stop.wait(delay):break
            self.stage_timing.observe('source_loop_wait',waiting,self.stage_timing.now())
            try:
                timing={'stage_timing':self.stage_timing} if self.stage_timing.enabled else {}
                # Recheck after the existing wait so start/stop cannot turn an
                # idle metadata-only read into an active cursor substitution.
                active = bool(self.active or self.archive)
                response=read_source(self.source_socket,self.session,self.cursor,
                    follow_tail=not active,wait_ms=MAX_SOURCE_WAIT_MS if active else 0,**timing)
                wake_supported = active and source_wake_supported(response)
                # Owner shutdown must not admit a late response after close.
                # A map STOP alone still uses the original ingest lifecycle.
                if self._stop.is_set():break
                self.ingest_source(response)
            except (OSError, ValueError, TypeError, KeyError) as exc:
                wake_supported = False
                self.error = str(exc)[:160]

    def _run(self):
        while not self._stop.is_set():
            wait_start=self.stage_timing.now()
            self.wakeup.wait(.01 if self.active else .3);self.wakeup.clear()
            self.stage_timing.observe('run_loop_wait',wait_start,self.stage_timing.now())
            try:self.process_pending()
            except Exception as exc:
                # A calculation failure must not kill the processing thread
                # or leave a previously submitted observation usable.
                self.navigation.invalidate('native_mapping_failed')
                self.error=type(exc).__name__+': '+str(exc)[:160]

    def ingest_source(self, response):
        ingest_entry=self.stage_timing.now()
        server_entry=self.stage_timing.wire_tick(response.get('native_stage_rpc'),'server_entry')
        selected=self.stage_timing.wire_tick(response.get('native_stage_rpc'),'journal_read_done')
        self.stage_timing.observe('server_handler_to_ingest',server_entry,ingest_entry)
        self.source_stage_counters=self.stage_timing.source_snapshot(response.get('native_stage_source_counters'))
        if response.get('schema') != SCHEMA or not freshness(response, 2.)['fresh']:
            raise ValueError('raw_source_stale_or_invalid')
        rows = response.get('rows')
        if not isinstance(rows, list) or len(rows) > 128:raise ValueError('raw_source_rows_size')
        session = response.get('session')
        if not isinstance(session, str) or not re.fullmatch(r'[0-9a-f]{32}', session):
            raise ValueError('raw_source_session')
        lifecycle_change=self.session is not None and (session!=self.session or response.get('overflow'))
        with self.processing_lock if lifecycle_change else nullcontext(), self.lock:
            self.source_polled=time.monotonic()
            source_changed=self.session is not None and session != self.session
            if source_changed:
                self.navigation.invalidate('native_source_session_changed')
                self.active = False;self.state = 'reset_detected';self.scans.clear()
                self.native_pending.clear()
            if response.get('overflow'):
                self.navigation.invalidate('native_source_overflow')
                self.source_loss += 1;self.active = False;self.state = 'reset_detected'
                self.scans.clear();self.source_gap = 'retained_source_overflow_new_map_required'
                self.native_pending.clear()
                self.cursor = response.get('first_seq', 1)-1
            if self.session != session:self.cursor = response.get('first_seq', 1)-1
            if lifecycle_change:self.stage_timing.clear()
            self.session = session;self.localization_epoch = session
            if response.get('follow_tail'):
                # A successful metadata-only read also recovers source health.
                # Otherwise a transient idle RPC timeout stays visible forever.
                if not (self.error or '').startswith('saved_map_restore:'):self.error=None
                if source_changed:self._publish_snapshot(refresh_cells=False)
                self.cursor=response['latest_seq'];return
            for row in rows:
                seq = row.get('seq')
                if type(seq) is not int or seq != self.cursor+1:raise ValueError('raw_source_out_of_order')
                self.cursor = seq
                topic, message = row['topic'], row['message']
                if topic not in ('/scan', '/imu/data_raw', '/gps/data'):
                    raise ValueError('raw_source_topic_not_allowed')
                if not freshness(row, 2.)['fresh']:
                    self.source_gap = 'source_observation_expired';continue
                if self.archive:self._archive_row(dict(row, session=self.session))
                if self.state == 'reset_detected' or not self.active:continue
                if topic in ('/imu/data_raw','/gps/data'):
                    if len(self.native_pending)==self.native_pending.maxlen:self.native_loss+=1
                    self.native_pending.append((topic,message))
                    if topic=='/imu/data_raw' and self.stage_timing.enabled:
                        try:self.stage_timing.note_imu((self.session,self._source_stamp_ns(message)),self.stage_timing.now())
                        except (ValueError,KeyError,TypeError):pass
                else:
                    if len(self.scans) >= 8:
                        # Prefer a fresh measured pose to seconds of queued
                        # history. Keep the fixed frame and old cells; any
                        # missing travel still needs observed re-localization.
                        self.source_loss += 1;discarded=self.scans.popleft()
                        self.stage_timing.discard((self.session,discarded[3].get('seq')))
                    self.stage_timing.admit((self.session,seq),row.get('native_stage_timing'),ingest_entry,self.stage_timing.now(),selected)
                    self.scans.append((message, time.monotonic(), self.active, row))
            self._publish_snapshot(refresh_cells=False)
            self.wakeup.set()

    @staticmethod
    def _native_stamp(topic,message):
        if topic=='/imu/data_raw':return source_stamp(message)
        stamp=message['stamp'];return stamp['sec']+stamp['nanosec']*1e-9

    @staticmethod
    def _source_stamp_ns(message):
        stamp=message['header']['stamp'];sec,nsec=stamp['sec'],stamp['nanosec']
        if type(sec) is not int or type(nsec) is not int or sec<0 or not 0<=nsec<10**9:
            raise ValueError('invalid_native_source_stamp')
        return sec*10**9+nsec

    def process_pending(self, force=False):
        lock_start=self.stage_timing.now()
        with self.processing_lock:
            self.stage_timing.observe('processing_lock_wait',lock_start,self.stage_timing.now())
            while self.scans:
                with self.lock:
                    if not self.scans:return
                    if not force and len(self.scans)>1:
                        # Live reception can outrun fitting under GUI/media load.
                        # Process the newest source scan instead of spending
                        # the next cycle on an obsolete backlog. Native IMU
                        # chronology and raw archives remain complete; skipped
                        # scan counts are explicit, never synthetic poses.
                        skipped=len(self.scans)-1
                        newest=self.scans[-1]
                        if self.stage_timing.enabled:
                            for discarded in list(self.scans)[:-1]:self.stage_timing.discard((self.session,discarded[3].get('seq')))
                        self.scans.clear();self.scans.append(newest)
                        self.source_loss+=skipped
                    scan, received, draw, row = self.scans[0]
                    trace=self.stage_timing.find((self.session,row.get('seq')))
                    if not force and time.monotonic()-received > .35:
                        self.navigation.invalidate('native_mapping_deadline_missed')
                        self.scans.popleft();self.source_loss+=1
                        self.stage_timing.discard((self.session,row.get('seq')))
                        self.source_gap='live_scan_deadline_missed';continue
                # Brief pairing grace: later receipt of IMU at an earlier/equal
                # sensor stamp must enter before scan localization, not after.
                pair_check=self.stage_timing.now()
                if not force and time.monotonic()-received < .15:
                    stamp_ns = self._source_stamp_ns(scan)
                    # Only wait when a causal paired IMU has not arrived.
                    # A future IMU cannot make an earlier scan ready. The
                    # core still independently validates pairing and clocks.
                    # Integer native time preserves the exact 50ms boundary.
                    # Float subtraction can turn an already-arrived causal
                    # sample into 50.00000000007ms and wait the full grace.
                    causal = [self._source_stamp_ns(m) for (topic, t),m in self.engine.pending.items()
                              if topic == '/imu/data_raw']
                    causal += [self._source_stamp_ns(m) for m in self.engine.source.imus]
                    with self.lock:
                        causal += [self._source_stamp_ns(m) for topic,m in self.native_pending
                                   if topic=='/imu/data_raw']
                    causal = [t for t in causal if t<=stamp_ns]
                    if not causal or stamp_ns-max(causal)>50_000_000:
                        if trace is not None:
                            if 'first_pair_wait' not in trace.marks:self.stage_timing.mark(trace,'first_pair_wait')
                            trace.pair_state='waiting_existing_grace'
                        self.stage_timing.observe('pair_check',pair_check,self.stage_timing.now())
                        return
                    if trace is not None:trace.pair_state='causal_ready'
                    self.stage_timing.pair_ready(trace,(self.session,max(causal)))
                else:
                    if trace is not None:trace.pair_state='forced' if force else 'grace_elapsed'
                    self.stage_timing.mark(trace,'pair_ready')
                self.stage_timing.observe('pair_check',pair_check,self.stage_timing.now())
                with self.lock:
                    if not self.scans or self.scans[0][3] is not row:continue
                    self.scans.popleft();stamp = source_stamp(scan)
                    if self.last_stamp is not None and stamp <= self.last_stamp:
                        if stamp < self.last_stamp:
                            self.navigation.invalidate('native_source_clock_reversed')
                            self.active = False;self.state = 'reset_detected'
                            self.source_gap = 'native_source_clock_reversed_new_map_required'
                            self.scans.clear();self.stage_timing.clear();break
                        self.stage_timing.discard((self.session,row.get('seq')))
                        continue
                    self.last_stamp = stamp;engine=self.engine;epoch=self.map_epoch
                    native=[];future=collections.deque(maxlen=512)
                    for topic,m in self.native_pending:
                        (native if self._native_stamp(topic,m)<=stamp else future).append((topic,m))
                    self.native_pending=future
                    self.processing=True
                try:
                    native_start=self.stage_timing.now()
                    for topic,m in native:
                        if topic=='/imu/data_raw':engine.imu(m)
                        else:engine.gps(m)
                    self.stage_timing.observe('native_ingest',native_start,self.stage_timing.now())
                    self.stage_timing.mark(trace,'numeric_begin')
                    submit_age=None
                    def localized(result):
                        nonlocal submit_age
                        self.stage_timing.mark(trace,'localized_enter')
                        submit_age=freshness(row,2.).get('age_s')
                        # Enqueue only. Perception runs in its independent
                        # worker while display raster integration completes.
                        self.navigation.submit(scan,result,epoch=epoch,row=row,
                            yaw_rate_rad_s=engine.localizer.gyro_rate)
                        self.stage_timing.mark(trace,'localized_exit')
                    result=engine.add(scan,None,draw=draw and self.active,on_localized=localized)
                    self.stage_timing.mark(trace,'numeric_return')
                    with self.lock:
                        self.input_scans += 1;self.last_seen = time.monotonic()
                        self.error = None;self.source_gap = None
                        if self.active:self.state = 'drawing' if result['accepted'] and result.get('map_eligible',True) else 'waiting_geometry'
                        engine.diagnostic['source_to_map_s']=freshness(row,2.).get('age_s')
                        engine.diagnostic['observer_submit_receipt_age_s']=submit_age
                        engine.diagnostic['native_inbox_dropped']=self.native_loss
                        if result['accepted'] and result.get('map_eligible',True):self.last_progress=time.monotonic()
                        self.version += 1
                        self._publish_snapshot()
                        self.stage_timing.mark(trace,'published')
                        self.stage_timing.finish(trace,'completed')
                except Exception as exc:
                    self.navigation.invalidate('native_mapping_failed')
                    with self.lock:self.error=type(exc).__name__+': '+str(exc)[:160]
                    self.stage_timing.finish(trace,'error')
                    raise
                finally:self.processing=False
                # Poll source again before fitting the next queued scan, so
                # delayed IMU receipt can pair by its original sensor stamp.
                # Draining a whole batch starves source ingestion and Stop.
                if not force:return

    @property
    def tracking(self):return self.active

    def task_status(self):
        now=time.monotonic()
        return dict(id='live',kind='live',priority=0,epoch=self.map_epoch,state=self.state,
                    active=self.active,tracking=self.active,processing=self.processing,
                    stage='registering_scan' if self.processing else 'waiting_sensors' if self.active else 'idle',
                    input_scans=self.input_scans,scan_count=self.map.scan_count,queue_scans=len(self.scans),
                    dropped_scans=self.source_loss,reason=self.error or self.source_gap,
                    last_progress_age_s=None if self.last_progress is None else now-self.last_progress,
                    diagnostic=dict(self.engine.diagnostic),runtime_authority=False)

    def action(self, action, transport=None,expected_epoch=None):
        with self.processing_lock,self.lock:
            if expected_epoch is not None and expected_epoch!=self.map_epoch:
                return dict(ok=False,error='mapping_epoch_changed_refresh_before_action')
            if action == 'reset':
                self.navigation.invalidate('new_map_requested')
                # Only this explicit action creates a new local frame.
                self.engine.closed = True;self.engine = ContinuousMapping();self.map = self.engine.map
                self.map_epoch = uuid.uuid4().hex;self.scans.clear();self.last_stamp = None
                self.native_pending.clear()
                self.active = False;self.state = 'idle';self.persistence = {};self.input_scans = 0
                self.error = None;self.source_gap = None;self.source_loss=0;self.native_loss=0
                self.stage_timing.clear()
            elif action == 'start':
                if self.state == 'reset_detected':return {**self.snapshot(), 'ok': False, 'error': 'new_map_required'}
                self.active = True;self.state = 'drawing'
            elif action == 'stop':
                self.navigation.invalidate('mapping_stopped')
                self.active = False
                if self.state != 'reset_detected':self.state = 'paused'
                # No already queued scan may cast rays after this response.
                self.scans.clear()
                self.native_pending.clear()
                self.stage_timing.clear()
            else:return dict(ok=False, error='unknown_mapping_action')
            self.version += 1;self._publish_snapshot()
            if action in ('stop', 'reset'):self._save_native_map()
        return self.snapshot(transport=transport)

    def _archive_row(self, row):
        item = self.archive
        text = json.dumps(dict(row, recording_t_s=max(0., time.monotonic()-item['start_monotonic'])),
                          allow_nan=False, separators=(',', ':'))+'\n'
        item['bytes'] += len(text.encode())
        if item['bytes'] > 512*1024*1024:item['limit_reached'] = True;return
        item['file'].write(text)

    def begin_archive(self, recording_id):
        with self.lock:
            if self.archive:self.finish_archive()
            path = self._archive_path(recording_id)
            if (path/'native_mapping_sensors.jsonl.gz').exists():
                return dict(ok=False, error='mapping_archive_already_exists')
            self.archive = dict(id=recording_id, path=path, bytes=0, started_at=time.time(),
                start_monotonic=time.monotonic(), start_stamp=self.last_stamp,
                file=gzip.open(path/'native_mapping_sensors.jsonl.gz', 'wt', encoding='utf8', compresslevel=1))
            atomic_json(path/'mapping_config.json', dict(algorithm=ALGORITHM, source_only=True,
                frame='recording_native_sensor_local_frame', PWM_used_as_odometry=False))
            return dict(ok=True, recording_id=recording_id, source_only=True)

    def finish_archive(self):
        with self.lock:
            if not self.archive:return dict(ok=True, state='idle')
            item, self.archive = self.archive, None;item['file'].close()
        # Rebuild an independent recording-local frame, never embed the live
        # frame/wheel pose. It can be regenerated from the archived raw stream.
        return dict(ok=True,state='waiting_for_map_request',recording_id=item['id'],source_only=True)

    def regenerate(self, recording_id, background=True):
        try:path = self._archive_path(recording_id)
        except ValueError as exc:return dict(ok=False, error=str(exc))
        if not (path/'native_mapping_sensors.jsonl.gz').is_file():
            return dict(ok=False, error='native_sensor_archive_required_legacy_pose_not_used')
        with self.lock:
            if self.archive and self.archive['id'] == recording_id:return dict(ok=False, error='recording_still_active')
            if recording_id in self._regenerating:return dict(ok=True, state='regenerating')
            if self._regenerating:return dict(ok=False,error='another_recording_mapping_active_stop_it_first')
            self._regenerating.add(recording_id)
            self.jobs_by_id[recording_id]=dict(id='archive:'+recording_id,kind='archive',priority=1,
                recording_id=recording_id,epoch=uuid.uuid4().hex,state='waiting_live_priority' if self.active else 'localizing',
                active=True,processing=False,processed_t_s=0.,requested_t_s=0.,scan_count=0,
                cancel=threading.Event(),thread=None,last_progress=time.monotonic())
        if background:
            thread=threading.Thread(target=self._regenerate, args=(recording_id, path), daemon=True,
                                   name='native-sim-archive-map')
            self.jobs_by_id[recording_id]['thread']=thread;thread.start()
            return dict(ok=True, state='regenerating', recording_id=recording_id)
        return self._regenerate(recording_id, path)

    def _regenerate(self, recording_id, path):
        job=self.jobs_by_id[recording_id]
        engine=ContinuousMapping()
        try:
            pending = [];used = 0;session = None;last_scan = None
            def scan_row(row):
                nonlocal last_scan
                while not job['cancel'].is_set() and not self._stop.is_set() and (self.active or
                        self.native_priority is not None and self.native_priority()):
                    job['state']='waiting_live_priority';job['processing']=False;job['cancel'].wait(.1)
                if job['cancel'].is_set() or self._stop.is_set():raise ValueError('operator_stopped')
                job['state']='localizing';job['processing']=True
                value = source_stamp(row['message'])
                if last_scan is not None and value < last_scan:raise ValueError('native_archive_clock_reset')
                last_scan = value
                engine.add(row['message'], None, t_s=row['recording_t_s'])
                job.update(processing=False,scan_count=engine.map.scan_count,processed_t_s=row['recording_t_s'],last_progress=time.monotonic())
            with gzip.open(path/'native_mapping_sensors.jsonl.gz', 'rt', encoding='utf8') as source:
                for line in source:
                    used += len(line.encode())
                    if used > 512*1024*1024:raise ValueError('mapping_archive_size_limit')
                    row = json.loads(line)
                    if session is not None and row.get('session', session) != session:raise ValueError('native_archive_session_changed')
                    session = row.get('session', session)
                    while pending and row['recording_t_s']-pending[0]['recording_t_s'] >= .06:scan_row(pending.pop(0))
                    if row['topic'] == '/imu/data_raw':engine.imu(row['message'])
                    elif row['topic'] == '/gps/data':engine.gps(row['message'])
                    elif row['topic'] == '/scan':pending.append(row)
                    else:raise ValueError('native_archive_topic_not_allowed')
            for row in pending:scan_row(row)
            result = {**engine.map.snapshot(), **engine.info(), 'ok':True, 'simulated':True,
                'recording_id':recording_id, 'state':'archived', 'active':False,
                'algorithm':ALGORITHM, 'reproducible':True,
                'recording_time_base':'wall_elapsed_seconds', 'native_tiles':self._archive_tiles(engine.map, path)}
            atomic_json(path/'occupancy_grid.json', result)
            (path/'mapping_error.json').unlink(missing_ok=True)
            return result
        except (OSError, ValueError, TypeError, KeyError, EOFError) as exc:
            result = dict(ok=False, state='regeneration_failed', error=str(exc))
            if str(exc)=='operator_stopped' and engine.map.scan_count:
                partial={**engine.map.tile_packet(),**engine.info(), 'ok':True, 'state':'partial',
                         'simulated':True,'recording_id':recording_id,'active':False,'map_epoch':job['epoch']}
                partial['native_tiles']=self._archive_tiles(engine.map,path)
                atomic_json(path/'occupancy_grid.json',partial)
            atomic_json(path/'mapping_error.json', result);return result
        finally:
            job.update(active=False,processing=False,state='stopped' if job['cancel'].is_set() else 'failed' if (path/'mapping_error.json').exists() else 'complete')
            with self.lock:self._regenerating.discard(recording_id)

    def jobs(self,client=None):
        now=time.monotonic()
        return [{k:v for k,v in job.items() if k not in ('thread','cancel','last_progress')} |
                dict(last_progress_age_s=now-job['last_progress']) for job in list(self.jobs_by_id.values())]

    def stop_job(self,recording_id,expected_epoch=None):
        job=self.jobs_by_id.get(recording_id)
        if not job:return dict(ok=False,error='unknown_mapping_job')
        if expected_epoch is not None and expected_epoch!=job['epoch']:
            return dict(ok=False,error='mapping_epoch_changed_refresh_before_action')
        job['cancel'].set()
        if job['thread']:job['thread'].join(timeout=5)
        if job['active']:return dict(ok=False,error='mapping_worker_stopping_retry')
        return dict(ok=True,state='stopped',partial_map_preserved=True,raw_source_preserved=True)

    def archive_snapshot(self,recording_id):
        # Native recorder rows have no wheel-derived live map object.
        try:path=self._archive_path(recording_id)
        except ValueError as exc:return dict(ok=False,error=str(exc))
        if self.archive and self.archive['id']==recording_id:
            return dict(ok=True,state='recording',active=True,scan_count=0)
        result=read_json(path/'occupancy_grid.json') or dict(ok=True,scan_count=0,state='waiting_for_map_request')
        if recording_id in self._regenerating:result={**result,'state':'regenerating'}
        return result

    def close(self):
        self.navigation.close()
        self._stop.set()
        self.wakeup.set()
        for job in self.jobs_by_id.values():job['cancel'].set()
        if self.thread:self.thread.join(timeout=3)
        if self.source_thread:self.source_thread.join(timeout=3)
        for job in self.jobs_by_id.values():
            if job['thread']:job['thread'].join(timeout=5)
        if self.archive:
            item,self.archive=self.archive,None;item['file'].close()
