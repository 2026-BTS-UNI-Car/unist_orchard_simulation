#!/usr/bin/env python3
"""Build private VOD proxies from a verified native-camera export ZIP.

No archive/metadata replacement. The package is installed as a generated
sidecar only after source hashes and playback have been verified. Never raises
source capture FPS or upscales its resolution. FFmpeg is an offline dependency.
"""
import argparse,hashlib,json,math,os,re,subprocess,time,zipfile
from pathlib import Path
from datetime import datetime
try:
    from scripts.hls_bandwidth import hls_bandwidth_metrics,normalize_generated_target_duration
    from scripts.native_jpeg_vfr import frame_timing,mux_native_jpeg
    from scripts.sim_video_clock import ANCHOR_FILE,DOMAIN,TOPIC,timing_contract,load_anchors,clock_mapping,write_clock_assets
except ModuleNotFoundError:
    from hls_bandwidth import hls_bandwidth_metrics,normalize_generated_target_duration
    from native_jpeg_vfr import frame_timing,mux_native_jpeg
    from sim_video_clock import ANCHOR_FILE,DOMAIN,TOPIC,timing_contract,load_anchors,clock_mapping,write_clock_assets


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()


def vfr_encoder_timing(ffmpeg='ffmpeg'):
    """Select an explicit terminal packet duration without resampling PTS.

    FFmpeg 6 rejects output -r combined with VFR. Its setts duration option
    provides the same 1-ms terminal sample hint as the legacy nominal clock,
    while MP4 materializes all preceding sample durations from actual DTS.
    Probe the public option rather than guessing from a version string.
    """
    probe=subprocess.run([ffmpeg,'-hide_banner','-h','bsf=setts'],
                         capture_output=True,text=True,timeout=10)
    if probe.returncode:
        raise RuntimeError('FFmpeg VFR timing capability probe failed')
    common=['-fps_mode','vfr','-enc_time_base','1:1000']
    if re.search(r'^\s+-duration\s+<string>(?:\s|$)',probe.stdout,re.MULTILINE):
        return common+['-bsf:v','setts=duration=0.001/TB'],dict(
            terminal_duration_mode='setts_packet_duration',encoder_nominal_rate_hz=None)
    # FFmpeg 4.4 accepts this nominal clock with -vsync vfr. It forwards the
    # captured PTS unchanged; it does not synthesize 1,000 captured frames/s.
    return ['-r','1000',*common],dict(
        terminal_duration_mode='legacy_vfr_nominal_clock',encoder_nominal_rate_hz=1000)


def run(args,log,cwd=None):
    # Ubuntu ROS Iron uses FFmpeg 4.4, whereas the isolated encoder has 6.x.
    # Keep variable source timestamps on both; never force constant 30 FPS.
    if '-fps_mode' in args:
        probe=subprocess.run([args[0],'-hide_banner','-h','full'],capture_output=True,text=True)
        if '-fps_mode' not in probe.stdout:
            args=['-vsync' if item=='-fps_mode' else item for item in args]
    import shutil
    if shutil.which('nice'):args=['nice','-n','10',*args]
    start=0
    try:
        with log.open('ab') as f:
            start=f.tell()
            subprocess.run(args,check=True,stdout=f,stderr=subprocess.STDOUT,cwd=cwd)
    except subprocess.CalledProcessError as error:
        # A temporary smoke archive may be removed when its test fails. Keep
        # the actual encoder diagnosis in the worker traceback/job error before
        # that happens, without reading old successful attempts or a huge log.
        try:
            with log.open('rb') as f:
                end=f.seek(0,os.SEEK_END)
                f.seek(max(start,end-4096))
                tail=f.read(4096).decode('utf8',errors='replace').strip()
        except OSError:
            tail='encoder log unavailable'
        raise RuntimeError(f'FFmpeg failed (exit {error.returncode}): '
                           f'{tail or "no encoder diagnostic output"}') from error


def build(source,output,ffmpeg='ffmpeg',archive_metadata=None,vfr_input=None):
    source=source.resolve(strict=True);output=output.resolve()
    if output.exists():raise ValueError('new_output_directory_required')
    output.mkdir(parents=True);framesdir=output/'frames';framesdir.mkdir()
    started=time.monotonic()
    with zipfile.ZipFile(source) as bundle:
        m=json.loads(bundle.read('manifest.json'));frames=m['frames']
        if m.get('schema')!='ubts.native_camera_export.v1' or not m.get('complete') or not frames:
            raise ValueError('incomplete_camera_export')
        if archive_metadata is None or digest(archive_metadata)!=m['source_sha256']['metadata.json']:
            raise ValueError('verified_archive_metadata_required_for_sensor_sync')
        metadata=json.loads(archive_metadata.read_text())
        contract=timing_contract(metadata)
        timestamp_domain=DOMAIN if contract else 'bag_receipt_ns'
        if contract:
            if m.get('video_timing_contract')!=contract or m.get('timestamp_domain')!=DOMAIN or m['source_topic']!=TOPIC:
                raise ValueError('sim_video_export_contract_mismatch')
            anchor_source=archive_metadata.parent/ANCHOR_FILE
            anchor_sha=m['source_sha256'].get(ANCHOR_FILE)
            if not anchor_sha or digest(anchor_source)!=anchor_sha:
                raise ValueError('sim_video_verified_anchors_required')
            anchors=load_anchors(anchor_source,metadata.get('video_clock_anchors'))
            bag_offset=None
        else:
            if m.get('timestamp_domain') not in (None,'bag_receipt_ns'):
                raise ValueError('unverified_video_timestamp_domain')
            origin=datetime.fromisoformat(metadata['started_at'])
            if origin.tzinfo is None:raise ValueError('archive_origin_timezone_required')
            bag_offset=m['bag_start_ns']/1e9-origin.timestamp()
            if not math.isfinite(bag_offset) or not 0<=bag_offset<=60:raise ValueError('archive_camera_clock_mismatch')
        previous=-math.inf
        for row in frames:
            name=row['file'];stamp=float(row['t_s'])
            if not re.fullmatch(r'frames/[0-9]{7}\.jpg',name) or not math.isfinite(stamp) or stamp<=previous:
                raise ValueError('invalid_camera_frame_index')
            previous=stamp;data=bundle.read(name)
            if len(data)!=row['bytes'] or hashlib.sha256(data).hexdigest()!=row['sha256']:raise ValueError('frame_hash_mismatch')
            (output/name).write_bytes(data)
    # FFmpeg 4.4 cannot parse concat's `option framerate` directive. Remux
    # verified JPEG packets into an explicit millisecond VFR input by default;
    # never fall back to 25-Hz concat timestamps. The old explicit concat mode
    # is retained for existing FFmpeg 6 tooling, not selected automatically.
    vfr_input=vfr_input or os.environ.get('UBTS_MEDIA_VFR_INPUT','pyav')
    if vfr_input not in ('pyav','concat'):raise ValueError('invalid_media_vfr_input')
    timing=frame_timing(frames,timestamp_domain)
    intervals=[frames[i+1]['t_s']-frames[i]['t_s'] for i in range(len(frames)-1)]
    last_dt=timing['last_dt_s']
    import cv2
    threads=str(max(1,min(2,int(os.environ.get('UBTS_MEDIA_THREADS','2')))))
    image=cv2.imread(str(output/frames[0]['file']));height,width=image.shape[:2]
    if vfr_input=='pyav':
        native_input=output/'native_frames.mkv'
        input_info=mux_native_jpeg(frames,output,native_input,width,height,timestamp_domain)
        input_args=['-i',str(native_input)]
    else:
        concat=['ffconcat version 1.0']
        for i,row in enumerate(frames):
            concat.extend([f"file '{row['file']}'",'option framerate 1000',
                f"duration {(intervals[i] if i<len(intervals) else last_dt):.9f}"])
        concat.append(f"file '{frames[-1]['file']}'");concat.append('option framerate 1000')
        playlist=output/'frames.ffconcat';playlist.write_text('\n'.join(concat)+'\n')
        input_args=['-safe','0','-f','concat','-i',str(playlist)]
        input_info=dict(kind='ffconcat_explicit_millisecond_rate',timestamp_quantum_s=.001,
                        last_frame_policy='repeat_last_at_terminal_pts')
    encoder_timing,timing_info=vfr_encoder_timing(ffmpeg)
    prefix=[ffmpeg,'-hide_banner','-loglevel','error','-nostdin','-n','-threads',threads,
        *input_args,'-an','-c:v','libx264','-preset','fast',
        '-threads',threads,'-pix_fmt','yuv420p','-bf','0',*encoder_timing,
        '-level:v','4.1','-force_key_frames','expr:gte(t,n_forced*2)']
    # Both public capability branches preserve VFR timestamps and a 1-ms
    # terminal repeat presentation, including perfectly uniform sources.
    input_info.update(**timing_info,terminal_presentation_duration_s=.001,
                      b_frames=0,hls_timing='VFR_MP4_sample_durations_then_stream_copy')
    if m.get('rotation_deg')==180:prefix+=['-vf','hflip,vflip']
    # Full source-sized high-quality MP4 is the fallback and top ABR variant.
    native=output/'native.mp4'
    run(prefix+['-crf','18','-maxrate','6000k' if height>=1080 else '3000k','-bufsize','12000k' if height>=1080 else '6000k','-movflags','+faststart',str(native)],output/'encoder.log')
    rates=[(height,6000000 if height>=1080 else 3000000,None),(720,2500000,22),(480,1100000,24),(360,450000,25),(240,180000,28)]
    variants=[];assetnames=['native.mp4'];playback_clock=None
    if contract:
        mapping=clock_mapping(frames,anchors,timing['pts_ms'])
        assetnames+=write_clock_assets(mapping,output)
        playback_clock=dict(schema=mapping['schema'],index='clock-index.json',
            source_anchor_sha256=anchor_sha,host_domain=mapping['host_domain'],video_domain=DOMAIN,
            synchronization='piecewise_exact_native_header_anchors',
            coverage_host_ns=mapping['coverage_host_ns'],coverage_video_ms=mapping['coverage_video_ms'])
    for h,rate,crf in rates:
        if h>height or any(v['height']==h for v in variants):continue
        w=(int(width*h/height)//2)*2;folder=output/str(h);folder.mkdir()
        variant=native
        if crf is not None:
            # FFmpeg 4.4's direct HLS encode sums nominal packet durations,
            # even with exact VFR PTS, producing e.g. EXTINF=.013 for .8 s.
            # MP4 first materializes actual sample deltas, then stream-copy
            # HLS preserves them without a second encode. No B frames means
            # PTS=DTS, avoiding the muxer's negative-DTS timeline shift.
            variant=output/f'variant-{h}.mp4'
            run([ffmpeg,'-hide_banner','-loglevel','error','-nostdin','-n','-threads',threads,
                 '-i',str(native),'-an','-vf',f'scale={w}:{h}','-c:v','libx264','-preset','fast',
                 '-threads',threads,'-crf',str(crf),'-maxrate',str(rate),'-bufsize',str(rate*2),
                 '-pix_fmt','yuv420p','-bf','0',*encoder_timing,
                 '-level:v','4.1','-force_key_frames','expr:gte(t,n_forced*2)',str(variant)],output/'encoder.log')
        args=[ffmpeg,'-hide_banner','-loglevel','error','-nostdin','-n','-threads',threads,
              '-i',str(variant),'-an','-c:v','copy']
        args+=['-f','hls','-hls_time','4','-hls_playlist_type','vod','-hls_segment_type','fmp4',
               '-hls_flags','independent_segments','-hls_fmp4_init_filename','init.mp4',
               '-hls_segment_filename','segment-%05d.m4s','index.m3u8']
        run(args,output/'encoder.log',cwd=folder)
        target_info=normalize_generated_target_duration(folder)
        names=[p.relative_to(output).as_posix() for p in folder.iterdir() if p.is_file()];assetnames+=names
        metrics=hls_bandwidth_metrics(folder)
        variants.append(dict(height=h,width=w,playlist=f'{h}/index.m3u8',**target_info,**metrics))
        if variant!=native:variant.unlink()
    master=['#EXTM3U','#EXT-X-VERSION:7','#EXT-X-INDEPENDENT-SEGMENTS']
    for v in sorted(variants,key=lambda v:v['height']):
        # The decoder reads the actual AVC configuration in init.mp4. Do not
        # fabricate a profile/level CODECS string for arbitrary source sizes.
        master.extend([f"#EXT-X-STREAM-INF:BANDWIDTH={v['bandwidth']},AVERAGE-BANDWIDTH={v['average_bandwidth']},RESOLUTION={v['width']}x{v['height']}",v['playlist']])
    (output/'master.m3u8').write_text('\n'.join(master)+'\n');assetnames.append('master.m3u8')
    duration=frames[-1]['t_s']-frames[0]['t_s']+last_dt
    public=dict(schema='orchard.adaptive_video.v1',complete=True,archive_id=m['archive_id'],
        source_topic=m['source_topic'],source_sha256=m['source_sha256'],source_signatures=m['source_signatures'],camera_export_sha256=digest(source),
        source_frames=len(frames),source_width=width,source_height=height,fps=(len(frames)-1)/(duration-last_dt) if len(frames)>1 else 1/last_dt,
        timeline_offset_s=None if contract else bag_offset+frames[0]['t_s'],duration_s=duration,
        frame_timing='ros_camera_header_ns_VFR' if contract else 'recorded_camera_receipt_VFR',
        vfr_input=input_info,
        time_base=DOMAIN if contract else 'archive_metadata.started_at',
        **(dict(playback_clock=playback_clock) if contract else {}),
        master='master.m3u8',fallback='native.mp4',variants=variants,
        assets={name:dict(bytes=(output/name).stat().st_size,sha256=digest(output/name)) for name in sorted(assetnames)},
        build_elapsed_s=time.monotonic()-started)
    (output/'adaptive_video.json').write_text(json.dumps(public,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in public.items() if k not in ('assets','source_sha256')}),flush=True)
    return public


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ffmpeg',default='ffmpeg');p.add_argument('--archive-metadata',type=Path,required=True)
    p.add_argument('--vfr-input',choices=('pyav','concat'))
    a=p.parse_args();build(a.source,a.output,a.ffmpeg,a.archive_metadata,a.vfr_input)
