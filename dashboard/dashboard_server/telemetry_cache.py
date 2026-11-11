"""Small derived archive telemetry, independent of LiDAR/video downloads."""
import json
import os
import threading
import gzip
import hashlib
from datetime import datetime


def light_segment(cache_dir,index):
    directory=cache_dir/'segments';target=directory/f'telemetry-{index:06d}.json'
    if target.is_file():return json.loads(target.read_text(encoding='utf-8'))
    source=directory/f'segment-{index:06d}.json'
    if not source.is_file():return None
    value=json.loads(source.read_text(encoding='utf-8'))
    result={key:value[key] for key in ('imu','gps','motor','sync','duration_s','coordinate_frame','segment') if key in value}
    # Only derived playback-cache files are written. Each writer has its own
    # temporary path, so concurrent seeks cannot publish partial JSON.
    temporary=target.with_suffix(f'.{os.getpid()}.{threading.get_ident()}.tmp')
    try:
        temporary.write_text(json.dumps(result,separators=(',',':'),allow_nan=False),encoding='utf-8')
        temporary.replace(target)
    except OSError:
        try:temporary.unlink(missing_ok=True)
        except OSError:pass
    return result


def raw_telemetry(archive,cache_root):
    """Recover a missing preview timeline from the verified raw sensor archive.

    Only a derived display cache is published; the broken original is preserved.
    Use bag receipt time, matching the map and video playback clock exactly.
    """
    source=archive/'mapping_source/sensors.jsonl.gz';manifest_file=archive/'mapping_source/manifest.json'
    if not source.is_file() or not manifest_file.is_file():return None
    manifest=json.loads(manifest_file.read_text());expected=(manifest.get('output_sha256') or {}).get('sensors.jsonl.gz')
    if not manifest.get('complete') or not expected:return None
    metadata=json.loads((archive/'metadata.json').read_text());start=datetime.fromisoformat(metadata['started_at']).timestamp()
    target=cache_root/'raw-telemetry'/archive.name/(expected+'-v1.json')
    if target.is_file():return json.loads(target.read_text())
    sha=hashlib.sha256()
    with source.open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024),b''):sha.update(block)
    if sha.hexdigest()!=expected:raise ValueError('unverified_raw_telemetry')
    from dashboard_server.imu_processing import quaternion_to_euler_deg
    frames=[]
    with gzip.open(source,'rt') as stream:
        for line in stream:
            row=json.loads(line)
            if row.get('topic')!='/imu/data_raw':continue
            message=row['message'];covariance=message.get('orientation_covariance',[-1])
            euler=quaternion_to_euler_deg(message['orientation'].get(k) for k in ('x','y','z','w')) if covariance[0]!=-1 else {}
            raw={k:message[k] for k in ('angular_velocity','linear_acceleration')}
            raw.update(frame_id=message['header']['frame_id'],stamp=message['header']['stamp'],euler_deg=euler)
            frames.append(dict(t_s=max(0,row['bag_time_ns']/1e9-start),raw=raw,stamp=raw['stamp']))
    frames.sort(key=lambda f:f['t_s'])
    result=dict(duration_s=metadata['duration_s'],imu=dict(frames=frames),sync=dict(imu_frames=len(frames)),
                source='verified_raw_sensor_archive',source_sensor_sha256=expected,preview_timeline_missing=True)
    target.parent.mkdir(parents=True,exist_ok=True)
    temporary=target.with_suffix(f'.{os.getpid()}.{threading.get_ident()}.tmp')
    temporary.write_text(json.dumps(result,separators=(',',':'),allow_nan=False));temporary.replace(target)
    return result
