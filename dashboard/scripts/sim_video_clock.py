"""Future SIM-only native video clocks; no ROS, device or network access.

Never apply this policy to a legacy recording or to actual-Jetson reception
timestamps. Native camera header PTS and raw ROSbag reception stamps remain
distinct evidence. Missing/ambiguous anchors are errors, never guessed offsets.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import queue
import struct
import threading


CONTRACT_SCHEMA='ubts.sim_video_timing.v1'
ANCHOR_SCHEMA='ubts.sim_video_anchor.v1'
CLOCK_SCHEMA='ubts.sim_video_clock.v1'
DOMAIN='ros_camera_header_ns'
TOPIC='/camera/image_raw'
ANCHOR_FILE='native_camera_clock.jsonl'
MAX_LINE_BYTES=512
MAX_ANCHOR_BYTES=128*1024*1024
MAX_ANCHOR_ROWS=1_000_000
MAX_NS=(1<<63)-1
MAX_SAFE_INTEGER=(1<<53)-1


def new_contract():
    return dict(schema=CONTRACT_SCHEMA,domain=DOMAIN,source_topic=TOPIC,
        source_message_type='sensor_msgs/msg/Image',receipt_steady_clock='CLOCK_MONOTONIC_RAW',
        host_timeline_origin='recording.started_monotonic_wall_epoch',
        anchors_file=ANCHOR_FILE,bag_receipt_policy='preserved_unmodified',
        header_policy='strictly_monotonic_no_drop_no_retime',
        playback_clock='host_wall_elapsed_unchanged',
        anchor_identity='exact_native_camera_header_ns',
        max_host_link_gap_s=1.,max_source_link_gap_s=.25,
        max_wall_steady_delta_error_s=.025)


def timing_contract(metadata):
    """Explicit future policy only. Absence retains legacy receipt timing."""
    value=metadata.get('video_timing_contract')
    if value is None:return None
    if not isinstance(value,dict) or value!=new_contract():
        raise ValueError('unsupported_sim_video_timing_contract')
    if (metadata.get('rosbag_timebase') or {}).get('domain')!='ros_simulation_time':
        raise ValueError('sim_video_contract_requires_simulation_bag_clock')
    return value


def camera_header_ns(serialized):
    """Read only the CDR1 ROS Header time; do not deserialize image payloads."""
    data=memoryview(serialized)
    if len(data)<12:raise ValueError('camera_cdr_header_truncated')
    identifier=bytes(data[:2])
    if identifier not in (b'\x00\x00',b'\x00\x01'):
        raise ValueError('camera_cdr_encapsulation_unsupported')
    sec,nanosec=struct.unpack_from('<iI' if identifier==b'\x00\x01' else '>iI',data,4)
    if sec<0 or nanosec>=1_000_000_000:raise ValueError('camera_header_stamp_invalid')
    return sec*1_000_000_000+nanosec


def _ns(value,reason):
    if type(value) is not int or not 0<=value<=MAX_NS:raise ValueError(reason)
    return value


def _validate_anchor(row):
    if not isinstance(row,dict) or row.get('schema')!=ANCHOR_SCHEMA or row.get('topic')!=TOPIC:
        raise ValueError('sim_video_anchor_schema_invalid')
    for name in ('camera_header_ns','receipt_wall_ns','receipt_steady_ns','host_elapsed_ns'):
        _ns(row.get(name),'sim_video_anchor_time_invalid')
    if row['host_elapsed_ns']>MAX_SAFE_INTEGER:
        raise ValueError('sim_video_host_elapsed_not_safe_integer')
    return row


class AnchorWriter:
    """One bounded append worker; callbacks retain times, never image bytes."""
    def __init__(self,path,host_origin_ns,*,queue_capacity=256,
                 max_rows=MAX_ANCHOR_ROWS,max_bytes=MAX_ANCHOR_BYTES):
        if type(queue_capacity) is not int or not 1<=queue_capacity<=4096:
            raise ValueError('sim_video_anchor_queue_bound_invalid')
        if type(max_rows) is not int or not 1<=max_rows<=MAX_ANCHOR_ROWS:
            raise ValueError('sim_video_anchor_row_bound_invalid')
        if type(max_bytes) is not int or not MAX_LINE_BYTES<=max_bytes<=MAX_ANCHOR_BYTES:
            raise ValueError('sim_video_anchor_byte_bound_invalid')
        self.origin=_ns(host_origin_ns,'sim_video_host_origin_invalid')
        self.path=Path(path);self.stream=self.path.open('xb')
        self.pending=queue.Queue(maxsize=queue_capacity)
        self.lock=threading.Lock();self.closed=threading.Event()
        self.max_rows=max_rows;self.max_bytes=max_bytes
        self.enqueued=0;self.written=0;self.bytes=0;self.error=None;self.previous=None;self.first=None
        self.worker=threading.Thread(target=self._work,name='sim-video-clock',daemon=True)
        self.worker.start()

    def fail(self,reason):
        with self.lock:self.error=self.error or str(reason)

    def submit(self,header_ns,wall_ns,steady_ns):
        try:
            header_ns=_ns(header_ns,'sim_video_camera_header_invalid')
            wall_ns=_ns(wall_ns,'sim_video_receipt_wall_invalid')
            steady_ns=_ns(steady_ns,'sim_video_receipt_steady_invalid')
            if wall_ns<self.origin:raise ValueError('sim_video_host_clock_before_origin')
            if wall_ns-self.origin>MAX_SAFE_INTEGER:raise ValueError('sim_video_host_elapsed_not_safe_integer')
            row=dict(schema=ANCHOR_SCHEMA,topic=TOPIC,camera_header_ns=header_ns,
                receipt_wall_ns=wall_ns,receipt_steady_ns=steady_ns,host_elapsed_ns=wall_ns-self.origin)
            encoded=(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n').encode()
            if len(encoded)>MAX_LINE_BYTES:raise ValueError('sim_video_anchor_line_bound')
        except (ValueError,TypeError) as error:
            with self.lock:self.error=self.error or str(error)
            return False
        with self.lock:
            if self.closed.is_set() or self.error:return False
            if self.previous is not None and any(a<=b for a,b in zip(
                    (header_ns,wall_ns,steady_ns),self.previous)):
                self.error='sim_video_anchor_clock_nonmonotonic';return False
            if self.enqueued>=self.max_rows:
                self.error='sim_video_anchor_row_bound';return False
            try:self.pending.put_nowait(encoded)
            except queue.Full:
                self.error='sim_video_anchor_queue_overflow';return False
            self.previous=(header_ns,wall_ns,steady_ns)
            if self.first is None:self.first=self.previous
            self.enqueued+=1
            return True

    def _work(self):
        try:
            while not self.closed.is_set() or not self.pending.empty():
                try:line=self.pending.get(timeout=.05)
                except queue.Empty:continue
                try:
                    if self.bytes+len(line)>self.max_bytes:
                        with self.lock:self.error=self.error or 'sim_video_anchor_byte_bound'
                        continue
                    self.stream.write(line)
                    with self.lock:self.bytes+=len(line);self.written+=1
                finally:self.pending.task_done()
            self.stream.flush()
        except OSError as error:
            with self.lock:self.error=self.error or 'sim_video_anchor_write_failed:'+str(error)
        finally:self.stream.close()

    def status(self):
        with self.lock:
            return dict(schema=ANCHOR_SCHEMA,file=ANCHOR_FILE,host_origin_epoch_ns=self.origin,
                enqueued_rows=self.enqueued,written_rows=self.written,bytes=self.bytes,
                pending_rows=self.pending.qsize(),queue_capacity=self.pending.maxsize,
                coverage='submitted_callbacks_only_until_native_export_count_verification',
                camera_header_range_ns=[self.first[0],self.previous[0]] if self.first else None,
                host_elapsed_range_ns=[self.first[1]-self.origin,self.previous[1]-self.origin] if self.first else None,
                native_source_duration_ns=self.previous[0]-self.first[0] if self.first else None,
                callback_host_duration_ns=self.previous[1]-self.first[1] if self.first else None,
                error=self.error,complete=self.closed.is_set() and not self.worker.is_alive()
                and not self.error and self.written==self.enqueued)

    def close(self,timeout_s=2.):
        if not isinstance(timeout_s,(int,float)) or not math.isfinite(timeout_s) or not 0<timeout_s<=10:
            raise ValueError('sim_video_anchor_flush_timeout_invalid')
        self.closed.set();self.worker.join(timeout_s)
        if self.worker.is_alive():
            with self.lock:self.error=self.error or 'sim_video_anchor_flush_timeout'
        return self.status()


def load_anchors(path,descriptor):
    """Finished bounded source only. The exporter must bind its SHA separately."""
    if not isinstance(descriptor,dict) or descriptor.get('complete') is not True or descriptor.get('error'):
        raise ValueError('sim_video_anchors_not_complete')
    path=Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size>MAX_ANCHOR_BYTES:
        raise ValueError('sim_video_anchor_source_invalid')
    origin=_ns(descriptor.get('host_origin_epoch_ns'),'sim_video_host_origin_invalid')
    rows=[];previous=None
    with path.open('rb') as source:
        while True:
            line=source.readline(MAX_LINE_BYTES+1)
            if not line:break
            if len(line)>MAX_LINE_BYTES or not line.endswith(b'\n'):
                raise ValueError('sim_video_anchor_line_invalid')
            row=_validate_anchor(json.loads(line));current=tuple(row[k] for k in
                ('camera_header_ns','receipt_wall_ns','receipt_steady_ns'))
            if previous is not None and any(a<=b for a,b in zip(current,previous)):
                raise ValueError('sim_video_anchor_clock_nonmonotonic')
            if row['receipt_wall_ns']-origin!=row['host_elapsed_ns']:
                raise ValueError('sim_video_anchor_host_origin_mismatch')
            rows.append(row);previous=current
            if len(rows)>MAX_ANCHOR_ROWS:raise ValueError('sim_video_anchor_row_bound')
    if not rows or len(rows)!=descriptor.get('written_rows') or len(rows)!=descriptor.get('enqueued_rows'):
        raise ValueError('sim_video_anchor_count_mismatch')
    if path.stat().st_size!=descriptor.get('bytes'):
        raise ValueError('sim_video_anchor_size_mismatch')
    if (descriptor.get('camera_header_range_ns')!=[rows[0]['camera_header_ns'],rows[-1]['camera_header_ns']]
            or descriptor.get('host_elapsed_range_ns')!=[rows[0]['host_elapsed_ns'],rows[-1]['host_elapsed_ns']]
            or descriptor.get('native_source_duration_ns')!=rows[-1]['camera_header_ns']-rows[0]['camera_header_ns']
            or descriptor.get('callback_host_duration_ns')!=rows[-1]['host_elapsed_ns']-rows[0]['host_elapsed_ns']):
        raise ValueError('sim_video_anchor_duration_mismatch')
    return rows


def clock_mapping(frames,anchors,pts_ms):
    """Exact header association plus explicit valid piecewise link intervals."""
    if not frames or len(frames)!=len(pts_ms):raise ValueError('sim_video_clock_frame_count')
    by_header={row['camera_header_ns']:row for row in anchors}
    if len(by_header)!=len(anchors):raise ValueError('sim_video_anchor_header_ambiguous')
    points=[];links=[];previous=None;previous_header=None
    for frame,pts in zip(frames,pts_ms):
        header=_ns(frame.get('stamp_ns'),'sim_video_frame_header_invalid')
        if previous_header is not None and header<=previous_header:
            raise ValueError('sim_video_frame_header_nonmonotonic')
        if type(pts) is not int or not 0<=pts<=MAX_SAFE_INTEGER or (points and pts<=points[-1][1]):
            raise ValueError('sim_video_pts_nonmonotonic')
        row=by_header.get(header)
        if row is None:raise ValueError('sim_video_frame_anchor_missing')
        _validate_anchor(row)
        if previous is not None:
            wall_dt=row['receipt_wall_ns']-previous['receipt_wall_ns']
            steady_dt=row['receipt_steady_ns']-previous['receipt_steady_ns']
            host_dt=row['host_elapsed_ns']-previous['host_elapsed_ns']
            if min(wall_dt,steady_dt,host_dt)<=0:
                raise ValueError('sim_video_anchor_clock_nonmonotonic')
            source_dt=header-previous_header
            links.append(bool(host_dt<=1_000_000_000 and source_dt<=250_000_000
                and abs(wall_dt-steady_dt)<=25_000_000))
        points.append([row['host_elapsed_ns'],pts]);previous=row;previous_header=header
    return dict(schema=CLOCK_SCHEMA,host_domain='host_wall_elapsed_unchanged',
        video_domain=DOMAIN,source_topic=TOPIC,points_ns_ms=points,valid_links=links,
        coverage_host_ns=[points[0][0],points[-1][0]],coverage_video_ms=[points[0][1],points[-1][1]],
        outside_policy='unsynchronized_no_extrapolation',
        float_query_roundoff_tolerance=dict(host_ns=.5,video_ms=.000001),
        terminal_policy='native_hold_has_no_invented_host_anchor')


def write_clock_assets(mapping,output,*,chunk_points=512):
    """Generated clock chunks only; bounded browser fetches and no overwrite."""
    if type(chunk_points) is not int or not 2<=chunk_points<=512:
        raise ValueError('sim_video_clock_chunk_bound_invalid')
    points=mapping['points_ns_ms'];links=mapping['valid_links']
    if not points or len(points)>MAX_ANCHOR_ROWS or len(links)!=len(points)-1:
        raise ValueError('sim_video_clock_links_invalid')
    output=Path(output);names=[];chunks=[]
    for start in range(0,max(1,len(points)-1),chunk_points-1):
        stop=min(len(points),start+chunk_points)
        name=f'clock-{len(chunks):05d}.json'
        chunk=dict(schema=CLOCK_SCHEMA,points_ns_ms=points[start:stop],valid_links=links[start:stop-1])
        data=json.dumps(chunk,separators=(',',':'),allow_nan=False).encode()
        if len(data)>64*1024:raise ValueError('sim_video_clock_chunk_byte_bound')
        with (output/name).open('xb') as stream:stream.write(data)
        names.append(name);chunks.append(dict(asset=name,point_count=stop-start,
            host_range_ns=[points[start][0],points[stop-1][0]],
            video_range_ms=[points[start][1],points[stop-1][1]]))
    index={k:v for k,v in mapping.items() if k not in ('points_ns_ms','valid_links')}
    index.update(chunks=chunks,point_count=len(points),chunk_points_bound=chunk_points,
        chunk_bytes_bound=64*1024,overlap_policy='one_identical_boundary_anchor')
    data=json.dumps(index,separators=(',',':'),allow_nan=False).encode()
    if len(data)>1024*1024:raise ValueError('sim_video_clock_index_byte_bound')
    with (output/'clock-index.json').open('xb') as stream:stream.write(data)
    return ['clock-index.json',*names]


def map_clock(mapping,value_s,*,direction='host_to_video'):
    """Pure reference for the browser: no constant offset/extrapolated match."""
    if type(value_s) not in (int,float) or not math.isfinite(value_s):return None
    if direction not in ('host_to_video','video_to_host'):raise ValueError('sim_video_clock_direction')
    points=mapping['points_ns_ms'];links=mapping['valid_links']
    if not points or len(links)!=len(points)-1:raise ValueError('sim_video_clock_links_invalid')
    source_index,target_index,source_scale,target_scale=(0,1,1e9,1e3) if direction=='host_to_video' else (1,0,1e3,1e9)
    value=value_s*source_scale
    tolerance=.5 if direction=='host_to_video' else .000001
    if value<points[0][source_index]-tolerance or value>points[-1][source_index]+tolerance:return None
    import bisect
    index=bisect.bisect_left([p[source_index] for p in points],value)
    # Decimal seconds→integer ns/ms multiplication can land just outside an
    # exact anchor. At most sub-nanosecond roundoff may snap to that anchor;
    # this never bridges a missing link or invents a terminal host sample.
    for neighbor in (index,index-1):
        if 0<=neighbor<len(points) and abs(value-points[neighbor][source_index])<=tolerance:
            return points[neighbor][target_index]/target_scale
    if index==0 or index==len(points) or not links[index-1]:return None
    a,b=points[index-1],points[index]
    fraction=(value-a[source_index])/(b[source_index]-a[source_index])
    return (a[target_index]+fraction*(b[target_index]-a[target_index]))/target_scale
