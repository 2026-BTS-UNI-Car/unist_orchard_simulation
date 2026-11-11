"""Lossless JPEG packet mux for the public PyAV 8 / FFmpeg 4.4 API.

Only generated files are written. JPEG bytes are verified and remuxed; no
decoded frame is re-encoded by this helper. PyAV 8 Packet.duration is read-only:
packet presentation timestamps use a millisecond time base, and a repeated
last JPEG at the terminal PTS explicitly preserves the existing last-frame hold.
"""
from fractions import Fraction
import hashlib
import io
import math
from pathlib import Path
import re


TIME_BASE=Fraction(1,1000)


def frame_timing(frames,timestamp_domain='bag_receipt_ns'):
    if timestamp_domain not in ('bag_receipt_ns','ros_camera_header_ns'):
        raise ValueError('unsupported_native_video_timestamp_domain')
    if not frames:raise ValueError('camera_frames_required')
    times=[];native=[]
    for row in frames:
        value=row.get('t_s')
        if type(value) not in (float,int) or not math.isfinite(value) or value<0:
            raise ValueError('invalid_camera_frame_time')
        if times and value<=times[-1]:raise ValueError('camera_frame_time_nonmonotonic')
        times.append(float(value))
        ns=row.get('stamp_ns' if timestamp_domain=='ros_camera_header_ns' else 'bag_time_ns')
        if timestamp_domain=='ros_camera_header_ns' and ns is None:
            raise ValueError('explicit_camera_header_time_required')
        if ns is not None:
            if type(ns) is not int or ns<0 or (native and ns<=native[-1]):
                raise ValueError('invalid_native_camera_time')
            native.append(ns)
    if native and len(native)!=len(frames):raise ValueError('mixed_native_camera_time')
    if native:
        relative=[ns-native[0] for ns in native]
        if any(abs(ns*1e-9-(t-times[0]))>2e-6 for ns,t in zip(relative,times)):
            raise ValueError('camera_native_time_mismatch')
        intervals=[native[i+1]-native[i] for i in range(len(native)-1)]
        last_ns=sorted(intervals)[len(intervals)//2] if intervals else 66_666_667
        pts=[(ns+500_000)//1_000_000 for ns in relative]
        terminal=(relative[-1]+last_ns+500_000)//1_000_000
        last_dt=last_ns*1e-9
    else:
        relative=[t-times[0] for t in times]
        intervals=[times[i+1]-times[i] for i in range(len(times)-1)]
        last_dt=sorted(intervals)[len(intervals)//2] if intervals else 1/15
        pts=[math.floor(t*1000+.5) for t in relative]
        terminal=math.floor((relative[-1]+last_dt)*1000+.5)
    if any(b<=a for a,b in zip(pts,pts[1:])) or terminal<=pts[-1]:
        # Do not move two actual source endpoints apart to fabricate monotonic
        # time. Thirty-FPS native recordings comfortably exceed this quantum.
        raise ValueError('camera_pts_collapsed_at_millisecond_resolution')
    return dict(pts_ms=pts,terminal_pts_ms=terminal,last_dt_s=last_dt,
                duration_s=times[-1]-times[0]+last_dt,
                max_rounding_error_s=max(abs(p*.001-(t-times[0])) for p,t in zip(pts,times)))


def _verified_frame(root,row):
    name=row.get('file')
    if not isinstance(name,str) or not re.fullmatch(r'frames/[0-9]{7}\.jpg',name):
        raise ValueError('invalid_camera_frame_path')
    path=root/name
    if path.is_symlink() or root not in path.resolve(strict=True).parents:
        raise ValueError('camera_frame_path_escape')
    count=row.get('bytes')
    if type(count) is not int or not 0<count<=64*1024*1024 or path.stat().st_size!=count:
        raise ValueError('camera_frame_size_mismatch')
    data=path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=row.get('sha256'):
        raise ValueError('camera_frame_hash_mismatch')
    return data


def mux_native_jpeg(frames,root,output,width,height,timestamp_domain='bag_receipt_ns'):
    import av
    root=Path(root).resolve(strict=True);output=Path(output)
    plan=frame_timing(frames,timestamp_domain)
    def input_container(data):
        # Demux a SINGLE JPEG, not a filename sequence. Its duration/time-base
        # and keyframe flag come from the demuxer, using APIs available in 8.x.
        return av.open(io.BytesIO(data),'r',format='image2pipe',
                       options={'framerate':'1000','threads':'1'})
    first=_verified_frame(root,frames[0])
    with input_container(first) as template,output.open('xb') as stream_file:
        input_stream=template.streams.video[0]
        if (input_stream.width,input_stream.height)!=(width,height):
            raise ValueError('camera_frame_dimensions_mismatch')
        with av.open(stream_file,'w',format='matroska') as container:
            stream=container.add_stream(template=input_stream)
            stream.time_base=TIME_BASE
            for index,row in enumerate([*frames,frames[-1]]):
                data=first if index==0 else _verified_frame(root,row)
                with input_container(data) as source:
                    source_stream=source.streams.video[0]
                    if (source_stream.width,source_stream.height)!=(width,height):
                        raise ValueError('camera_frame_dimensions_mismatch')
                    packets=[packet for packet in source.demux(source_stream) if packet.size]
                    if len(packets)!=1:raise ValueError('one_native_jpeg_packet_required')
                    packet=packets[0]
                    if bytes(packet)!=data:raise ValueError('native_jpeg_packet_bytes_changed')
                    # FFmpeg 4.4 image2pipe may leave duration unknown (0).
                    # Presentation intervals are still explicit in adjacent
                    # PTS; do not try an unavailable PyAV 8 duration setter.
                    if (packet.time_base!=TIME_BASE or packet.duration not in (None,0,1)
                            or source_stream.average_rate!=1000 or not packet.is_keyframe):
                        raise ValueError('native_jpeg_demux_timing_contract')
                    packet.stream=stream
                    packet.pts=packet.dts=plan['pts_ms'][index] if index<len(frames) else plan['terminal_pts_ms']
                    packet.time_base=TIME_BASE
                    container.mux(packet)
    return dict(kind='pyav_mjpeg_matroska',pyav_version=av.__version__,
                timestamp_domain=timestamp_domain,
                source_jpeg_bytes_preserved=True,time_base=[1,1000],
                timestamp_quantum_s=.001,max_rounding_error_s=plan['max_rounding_error_s'],
                last_frame_policy='repeat_last_at_terminal_pts',last_frame_hold_s=plan['last_dt_s'],
                terminal_pts_s=plan['terminal_pts_ms']*.001,terminal_packet_duration_s=None,
                source_packets=len(frames),muxed_packets=len(frames)+1)
