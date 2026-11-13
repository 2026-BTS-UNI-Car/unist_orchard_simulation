"""Generated VOD sidecars, with an explicit allowlist and source provenance."""
import json,re
from pathlib import Path

MIMES={'.m3u8':'application/vnd.apple.mpegurl','.m4s':'video/iso.segment','.mp4':'video/mp4'}


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
            if not re.fullmatch(r'(?:1080-original/)?(?:[0-9]{2,4}/)?[A-Za-z0-9_-]+\.(?:m3u8|m4s|mp4)',name):return None
            limit=16*1024**3 if name.endswith('native.mp4') else 64*1024**2
            if not isinstance(row,dict) or not isinstance(row.get('bytes'),int) or not 0<row['bytes']<=limit:return None
            if not re.fullmatch(r'[0-9a-f]{64}',str(row.get('sha256',''))):return None
        for name in (value.get('master'),value.get('fallback')):
            if name not in assets or asset_path(archive,value,name) is None:return None
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
    return {**{k:value.get(k) for k in ('source_topic','source_frames','source_width','source_height','fps','duration_s','timeline_offset_s','frame_timing','variants')},
            'manifest_url':prefix+value['master']+'?v='+version,
            'fallback_url':prefix+value['fallback']+'?v='+version,
            'alternative_sources':[{**source,
                'manifest_url':prefix+source['master']+'?v='+value['assets'][source['master']]['sha256'][:16],
                'fallback_url':prefix+source['fallback']+'?v='+value['assets'][source['fallback']]['sha256'][:16]}
                for source in value.get('alternative_sources',[])
                if source.get('master') in value['assets'] and source.get('fallback') in value['assets']]}
