#!/usr/bin/env python3
"""Validate and atomically install a generated sidecar; preserve raw archives."""
import argparse,hashlib,json,re,shutil,uuid,ctypes,sys
from pathlib import Path

MEDIA_NAME=re.compile(r'(?:1080-original/)?(?:[0-9]{2,4}/)?[A-Za-z0-9_-]+\.(?:m3u8|m4s|mp4)')
CLOCK_NAME=re.compile(r'(?:clock-index|clock-[0-9]{5})\.json')


def validate_clock_assets(package,manifest,archive):
    """Only a verified future SIM declaration may install bounded clock JSON."""
    assets=manifest['assets'];names={name for name in assets if CLOCK_NAME.fullmatch(name)}
    clock=manifest.get('playback_clock')
    if clock is None:
        if names:raise ValueError('undeclared_clock_assets')
        return set()
    try:from scripts.sim_video_clock import ANCHOR_FILE,CLOCK_SCHEMA,DOMAIN,MAX_SAFE_INTEGER,TOPIC,timing_contract
    except ModuleNotFoundError as error:
        if error.name!='scripts':raise
        from sim_video_clock import ANCHOR_FILE,CLOCK_SCHEMA,DOMAIN,MAX_SAFE_INTEGER,TOPIC,timing_contract
    if (not isinstance(clock,dict) or clock.get('schema')!=CLOCK_SCHEMA
            or clock.get('index')!='clock-index.json' or 'clock-index.json' not in names
            or clock.get('host_domain')!='host_wall_elapsed_unchanged'
            or clock.get('video_domain')!=DOMAIN
            or clock.get('synchronization')!='piecewise_exact_native_header_anchors'
            or manifest.get('frame_timing')!='ros_camera_header_ns_VFR'
            or manifest.get('time_base')!=DOMAIN or manifest.get('timeline_offset_s') is not None):
        raise ValueError('invalid_clock_declaration')
    metadata=json.loads((archive/'metadata.json').read_text())
    if timing_contract(metadata) is None:raise ValueError('clock_source_contract_missing')
    descriptor=metadata.get('video_clock_anchors') or {}
    if descriptor.get('complete') is not True or descriptor.get('error'):
        raise ValueError('sim_video_anchors_not_complete')
    source_hash=manifest.get('source_sha256',{}).get(ANCHOR_FILE)
    source=archive/ANCHOR_FILE
    if (not re.fullmatch(r'[0-9a-f]{64}',str(source_hash or '')) or source.is_symlink()
            or clock.get('source_anchor_sha256')!=source_hash or sha(source)!=source_hash):
        raise ValueError('clock_source_anchor_changed')
    def read_clock(name,limit):
        path=(package/name).resolve(strict=True)
        if package not in path.parents or (package/name).is_symlink() or not 0<path.stat().st_size<=limit:
            raise ValueError('clock_asset_bound_or_path')
        return json.loads(path.read_text())
    index=read_clock('clock-index.json',1024*1024)
    if (index.get('schema')!=CLOCK_SCHEMA or index.get('host_domain')!=clock['host_domain']
            or index.get('video_domain')!=DOMAIN or index.get('source_topic')!=TOPIC
            or index.get('outside_policy')!='unsynchronized_no_extrapolation'
            or index.get('terminal_policy')!='native_hold_has_no_invented_host_anchor'
            or index.get('overlap_policy')!='one_identical_boundary_anchor'
            or type(index.get('chunk_points_bound')) is not int or not 2<=index['chunk_points_bound']<=512
            or index.get('chunk_bytes_bound')!=64*1024):raise ValueError('invalid_clock_index')
    chunks=index.get('chunks')
    if not isinstance(chunks,list) or not 1<=len(chunks)<=20000:raise ValueError('invalid_clock_chunks')
    if names!={'clock-index.json',*(f'clock-{i:05d}.json' for i in range(len(chunks)))}:
        raise ValueError('clock_asset_set_mismatch')
    first=previous=None;count=0
    for i,row in enumerate(chunks):
        if not isinstance(row,dict) or row.get('asset')!=f'clock-{i:05d}.json':raise ValueError('invalid_clock_chunk_entry')
        chunk=read_clock(row['asset'],64*1024);points=chunk.get('points_ns_ms');links=chunk.get('valid_links')
        if (chunk.get('schema')!=CLOCK_SCHEMA or not isinstance(points,list)
                or not 1<=len(points)<=index['chunk_points_bound'] or not isinstance(links,list)
                or len(links)!=len(points)-1 or any(type(link) is not bool for link in links)):
            raise ValueError('invalid_clock_chunk')
        if any(not isinstance(point,list) or len(point)!=2 or
               any(type(n) is not int or not 0<=n<=MAX_SAFE_INTEGER for n in point) for point in points):
            raise ValueError('invalid_clock_point')
        if any(any(a>=b for a,b in zip(left,right)) for left,right in zip(points,points[1:])):
            raise ValueError('clock_points_nonmonotonic')
        if (row.get('point_count')!=len(points) or row.get('host_range_ns')!=[points[0][0],points[-1][0]]
                or row.get('video_range_ms')!=[points[0][1],points[-1][1]]):raise ValueError('clock_chunk_range_mismatch')
        if previous is not None and previous!=points[0]:raise ValueError('clock_chunk_overlap_mismatch')
        if first is None:first=points[0]
        count+=len(points)-(i>0);previous=points[-1]
    if (not 1<=count<=1_000_000 or type(index.get('point_count')) is not int or index['point_count']!=count
            or manifest.get('source_frames')!=count
            or first[1]!=0):raise ValueError('clock_source_frame_count_mismatch')
    for name,column in (('coverage_host_ns',0),('coverage_video_ms',1)):
        if index.get(name)!=[first[column],previous[column]] or clock.get(name)!=index[name]:
            raise ValueError('clock_coverage_mismatch')
    return {ANCHOR_FILE}


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()


def exchange_generated_directories(first,second):
    if sys.platform!='linux':raise ValueError('atomic_generated_upgrade_requires_linux')
    libc=ctypes.CDLL(None,use_errno=True)
    exchange=getattr(libc,'renameat2',None)
    if exchange is None:raise ValueError('atomic_exchange_unavailable')
    if exchange(-100,os_bytes(first),-100,os_bytes(second),2)!=0:
        raise OSError(ctypes.get_errno(),'atomic_generated_exchange_failed')

def os_bytes(path):return str(path).encode(sys.getfilesystemencoding())

def install(package,root,replace_manifest_sha256=None):
    package=package.resolve(strict=True);root=root.resolve(strict=True)
    m=json.loads((package/'adaptive_video.json').read_text());name=m['archive_id']
    if not m.get('complete') or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',name):raise ValueError('invalid_adaptive_package')
    archive=(root/name).resolve(strict=True)
    if archive.parent!=root or (root/name).is_symlink():raise ValueError('invalid_archive_path')
    clock_sources=validate_clock_assets(package,m,archive)
    protected={n:sha(archive/n) for n in ('metadata.json','gps.jsonl','motor.jsonl',*clock_sources) if (archive/n).is_file()}
    if any(m['source_sha256'].get(n)!=h for n,h in protected.items()):raise ValueError('source_sidecar_changed')
    for n,sig in m['source_signatures'].items():
        p=(archive/n).resolve(strict=True)
        if archive not in p.parents or [p.stat().st_size,p.stat().st_mtime_ns]!=sig:raise ValueError('source_bag_changed')
    target=archive/'adaptive_video'
    if target.is_symlink():raise ValueError('adaptive_sidecar_symlink')
    if target.exists():
        old=json.loads((target/'adaptive_video.json').read_text())
        if old==m and all(sha(target/n)==v['sha256'] for n,v in m['assets'].items()):return dict(archive_id=name,state='already_installed')
        # Resuming an older batch must preserve a verified newer rendition.
        ancestry=old.get('generated_upgrade_manifest_ancestry_sha256',[])
        if not isinstance(ancestry,list) or len(ancestry)>16 or any(not re.fullmatch(r'[0-9a-f]{64}',str(v)) for v in ancestry):
            raise ValueError('invalid_generated_ancestry')
        if sha(package/'adaptive_video.json') in ancestry+[old.get('generated_upgrade_source_manifest_sha256')] and all(
            n==m.get('master') or old.get('assets',{}).get(n)==v for n,v in m['assets'].items()) and all(
            sha(target/n)==v['sha256'] for n,v in old['assets'].items()):
            return dict(archive_id=name,state='already_installed_upgraded')
        if not re.fullmatch(r'[0-9a-f]{64}',str(replace_manifest_sha256 or '')) or sha(target/'adaptive_video.json')!=replace_manifest_sha256:
            raise ValueError('existing_adaptive_sidecar_requires_review')
        if m.get('generated_upgrade_source_manifest_sha256')!=replace_manifest_sha256:raise ValueError('generated_upgrade_baseline_mismatch')
        if any(sha(target/n)!=v['sha256'] for n,v in old['assets'].items()):raise ValueError('generated_baseline_asset_changed')
        backup=archive/('.adaptive_video-backup-'+replace_manifest_sha256[:16])
        if backup.exists():raise ValueError('generated_backup_already_exists')
    for n,v in m['assets'].items():
        if not (MEDIA_NAME.fullmatch(n) or CLOCK_NAME.fullmatch(n)):raise ValueError('invalid_asset_name')
        p=(package/n).resolve(strict=True)
        if package not in p.parents or p.stat().st_size!=v['bytes'] or sha(p)!=v['sha256']:raise ValueError('invalid_asset_hash')
    staging=archive/('.adaptive_video-'+uuid.uuid4().hex+'.partial');staging.mkdir()
    for n in m['assets']:
        p=package/n
        dest=staging/n;dest.parent.mkdir(exist_ok=True);shutil.copyfile(p,dest)
    shutil.copyfile(package/'adaptive_video.json',staging/'adaptive_video.json')
    if any(sha(archive/n)!=h for n,h in protected.items()):raise ValueError('raw_sidecar_drift_during_install')
    if target.exists():
        if sha(target/'adaptive_video.json')!=replace_manifest_sha256:raise ValueError('generated_baseline_drift_during_install')
        exchange_generated_directories(staging,target);staging.rename(backup)
        state='upgraded'
    else:staging.rename(target);state='installed'
    return dict(archive_id=name,state=state,source_frames=m['source_frames'],assets=len(m['assets']),raw_sidecars_preserved=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('package',type=Path);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--replace-generated-sha256')
    a=p.parse_args();print(json.dumps(install(a.package,a.root,a.replace_generated_sha256)),flush=True)
