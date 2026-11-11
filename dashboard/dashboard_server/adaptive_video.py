"""Generated VOD sidecars, with an explicit allowlist and source provenance."""
import json,re
from pathlib import Path

MIMES={'.m3u8':'application/vnd.apple.mpegurl','.m4s':'video/iso.segment','.mp4':'video/mp4',
       '.json':'application/json'}
CLOCK_NAME=re.compile(r'^(?:clock-index|clock-[0-9]{5})\.json$')


def clock_info(value,assets):
    """Explicit SIM clock declaration; never guess an offset for this domain."""
    clock=value.get('playback_clock')
    if clock is None:
        return {} if not any(CLOCK_NAME.fullmatch(name) for name in assets) else None
    if (not isinstance(clock,dict) or clock.get('schema')!='ubts.sim_video_clock.v1'
            or clock.get('index')!='clock-index.json' or clock.get('index') not in assets
            or clock.get('host_domain')!='host_wall_elapsed_unchanged'
            or clock.get('video_domain')!='ros_camera_header_ns'
            or clock.get('synchronization')!='piecewise_exact_native_header_anchors'
            or value.get('frame_timing')!='ros_camera_header_ns_VFR'
            or value.get('time_base')!='ros_camera_header_ns'
            or value.get('timeline_offset_s') is not None):return None
    for name in ('coverage_host_ns','coverage_video_ms'):
        span=clock.get(name)
        if (not isinstance(span,list) or len(span)!=2 or
                any(type(v) is not int or not 0<=v<2**53 for v in span) or span[1]<span[0]):return None
    return {key:clock[key] for key in ('schema','host_domain','video_domain','synchronization',
                                       'coverage_host_ns','coverage_video_ms')}


def manifest(archive):
    archive=Path(archive).resolve()
    source=archive/'adaptive_video/adaptive_video.json'
    try:
        if source.resolve().parent.parent!=archive or source.parent.is_symlink():return None
        if source.stat().st_size>1024*1024:return None
        value=json.loads(source.read_text(encoding='utf-8'))
        if value.get('schema')!='orchard.adaptive_video.v1' or not value.get('complete') or value.get('archive_id')!=archive.name:return None
        assets=value.get('assets')
        if not isinstance(assets,dict) or not 1<=len(assets)<=20000:return None
        for name,row in assets.items():
            if not isinstance(name,str) or not (re.fullmatch(r'(?:[0-9]{2,4}/)?[A-Za-z0-9_-]+\.(?:m3u8|m4s|mp4)',name) or CLOCK_NAME.fullmatch(name)):return None
            if not isinstance(row,dict) or not isinstance(row.get('bytes'),int) or not 0<row['bytes']<=1024**3:return None
            if not re.fullmatch(r'[0-9a-f]{64}',str(row.get('sha256',''))):return None
            if CLOCK_NAME.fullmatch(name) and row['bytes']>(1024*1024 if name=='clock-index.json' else 64*1024):return None
        for name in (value.get('master'),value.get('fallback')):
            if name not in assets or asset_path(archive,value,name) is None:return None
        clock=clock_info(value,assets)
        if clock is None:return None
        if clock and any(asset_path(archive,value,name) is None for name in assets if CLOCK_NAME.fullmatch(name)):return None
        return value
    except (OSError,ValueError,TypeError):return None


def asset_path(archive,value,name):
    if name not in value.get('assets',{}):return None
    root=(Path(archive)/'adaptive_video').resolve()
    if root.parent!=Path(archive).resolve() or root.name!='adaptive_video':return None
    path=(root/name).resolve()
    try:
        if root not in path.parents or not path.is_file() or path.stat().st_size!=value['assets'][name]['bytes']:return None
        return path
    except OSError:return None


def public_info(archive,value):
    prefix=f'/api/recordings/{Path(archive).name}/adaptive-video/'
    version=value['assets'][value['master']]['sha256'][:16]
    result={**{k:value.get(k) for k in ('source_topic','source_frames','source_width','source_height','fps','duration_s','timeline_offset_s','frame_timing','variants')},
            'manifest_url':prefix+value['master']+'?v='+version,
            'fallback_url':prefix+value['fallback']+'?v='+version}
    clock=clock_info(value,value['assets'])
    if clock:
        clock_version=value['assets']['clock-index.json']['sha256'][:16]
        result.update(playback_clock=clock,
            clock_index_url=prefix+'clock-index.json?v='+clock_version,
            clock_asset_base_url=prefix,clock_asset_version=clock_version)
    return result
