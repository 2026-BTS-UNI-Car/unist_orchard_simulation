#!/usr/bin/env python3
"""Pictures and a short clip of a RUNNING steering stack (compose.ubts-steer.yaml). Real runs only, no mock-ups.

The material is for the documents (docs/steering/figures). Every value of the stack is a placeholder, so a caption
says "자리값 기반 SIM 결과" and never field performance. Nothing here starts or stops the stack and nothing talks to
the vehicle: start the stack with tools/steer_sim/stack.py first.

  camera   JPEG frames of the camera WebSocket bridge of the sim container (ws://127.0.0.1:8770/ws/camera, the stream
           the dashboard relays), read with "docker exec" in <project>-sim-1 for --seconds:
             python -I tools/steer_sim/capture_media.py camera --project ubts-steer-<job> --seconds 20 --out DIR
           writes DIR/frames/000000.jpg ... and DIR/FRAMES.json (count, receive times, size).
  clip     an H.264 MP4 of those frames (host ffmpeg; --fps defaults to the measured frame rate, --every N keeps every
           N-th frame, --max-mb refuses a file above the limit):
             python -I tools/steer_sim/capture_media.py clip --frames DIR --out FILE.mp4 [--every 2] [--max-mb 5]
  still    one frame as PNG (host ffmpeg):
             python -I tools/steer_sim/capture_media.py still --frames DIR --index 120 --out FILE.png
  page     a screenshot of a dashboard page with a headless browser (host specific: the playwright package and an
           installed Chrome). Capture the rover-control page only: the overview page shows addresses of the PC.
             python -I tools/steer_sim/capture_media.py page --out FILE.png \
                 [--url http://127.0.0.1:18090/rover-control] [--wait-s 10] [--width 1440] [--height 1500]

Check every file by eye before it goes into the repository: no user name, no address, no coordinates.
"""
import argparse
import io
import json
import subprocess
import sys
import tarfile
from pathlib import Path

CAMERA_WS = 'ws://127.0.0.1:8770/ws/camera'
DASHBOARD_PAGE = 'http://127.0.0.1:18090/rover-control'
LOCAL_PAGE_PREFIXES = ('http://127.0.0.1:', 'http://localhost:')

# Runs inside the sim container (python3 -B -): frames of the camera WebSocket as a tar stream on stdout.
INNER = r'''
import asyncio, base64, io, json, struct, sys, tarfile, time
import websockets
URL, SECONDS, MAGIC = sys.argv[1], float(sys.argv[2]), b'UBCF'

def frame_of(raw):
    if isinstance(raw, (bytes, bytearray)):
        data = bytes(raw)
        if not data.startswith(MAGIC) or len(data) < len(MAGIC) + 5:
            return None
        offset = len(MAGIC) + 1
        (size,) = struct.unpack('!I', data[offset:offset + 4])
        return data[offset + 4 + size:]
    try:
        meta = json.loads(raw)
    except (TypeError, ValueError):
        return None
    if not isinstance(meta, dict) or meta.get('type') != 'frame' or meta.get('data') is None:
        return None
    return base64.b64decode(meta['data'])

async def run():
    frames = []
    async with websockets.connect(URL, max_size=8 * 1024 * 1024, open_timeout=8.) as ws:
        t0 = time.monotonic()
        while time.monotonic() - t0 < SECONDS:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=2.)
            except asyncio.TimeoutError:
                continue
            jpeg = frame_of(raw)
            if jpeg and jpeg.startswith(b'\xff\xd8'):
                frames.append((round(time.monotonic() - t0, 4), jpeg))
    return frames

frames = asyncio.run(run())
out = tarfile.open(fileobj=sys.stdout.buffer, mode='w|')
def add(name, data):
    info = tarfile.TarInfo(name)
    info.size = len(data)
    out.addfile(info, io.BytesIO(data))
add('FRAMES.json', json.dumps(dict(url=URL, seconds=SECONDS, count=len(frames),
                                   receive_s=[t for t, _ in frames])).encode('utf-8'))
for i, (_, jpeg) in enumerate(frames):
    add('frames/%06d.jpg' % i, jpeg)
out.close()
'''


def sim_container(project):
    return '%s-sim-1' % project


def camera_command(project, seconds, docker='docker', url=CAMERA_WS):
    return [docker, 'exec', '-i', sim_container(project), 'python3', '-B', '-', url, '%g' % seconds]


def unpack_frames(tar_bytes, out_dir):
    """Write the frames of the tar stream to out_dir; the summary (count, receive times, measured rate)."""
    out_dir = Path(out_dir)
    (out_dir / 'frames').mkdir(parents=True, exist_ok=True)
    summary, written = {}, 0
    with tarfile.open(fileobj=io.BytesIO(tar_bytes), mode='r|') as tar:
        for member in tar:
            data = tar.extractfile(member).read()
            name = member.name
            if name == 'FRAMES.json':
                summary = json.loads(data.decode('utf-8'))
            elif name.startswith('frames/') and name.endswith('.jpg') and '/' not in name[7:] and '..' not in name:
                (out_dir / name).write_bytes(data)
                written += 1
    times = summary.get('receive_s') or []
    span = times[-1] - times[0] if len(times) > 1 else 0.
    summary.update(written=written, measured_fps=round((len(times) - 1) / span, 2) if span > 0 else None)
    (out_dir / 'FRAMES.json').write_bytes((json.dumps(summary, indent=1, sort_keys=True) + '\n').encode('utf-8'))
    return summary


def frame_files(frames_dir):
    return sorted((Path(frames_dir) / 'frames').glob('*.jpg'))


def clip_command(list_file, out, fps, ffmpeg='ffmpeg', crf=30):
    return [ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'concat', '-safe', '0', '-r', '%g' % fps,
            '-i', str(list_file), '-an', '-c:v', 'libx264', '-preset', 'slow', '-crf', str(crf),
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-metadata', 'title=', str(out)]


def do_camera(args):
    command = camera_command(args.project, args.seconds, docker=args.docker)
    done = subprocess.run(command, input=INNER.encode('utf-8'), capture_output=True, timeout=args.seconds + 60.)
    if done.returncode != 0:
        sys.stderr.write(done.stderr.decode('utf-8', 'replace')[-2000:])
        return done.returncode or 1
    summary = unpack_frames(done.stdout, args.out)
    print(json.dumps(dict((k, summary.get(k)) for k in ('count', 'written', 'measured_fps', 'seconds'))))
    return 0 if summary.get('written') else 1


def do_clip(args):
    files = frame_files(args.frames)[args.first:args.last][::max(1, args.every)]
    if not files:
        sys.stderr.write('no frames\n')
        return 1
    summary = json.loads((Path(args.frames) / 'FRAMES.json').read_bytes().decode('utf-8'))
    fps = args.fps or (summary.get('measured_fps') or 30.) / max(1, args.every)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    list_file = Path(args.frames) / 'clip_list.txt'
    list_file.write_bytes(''.join("file '%s'\n" % f.resolve().as_posix() for f in files).encode('utf-8'))
    done = subprocess.run(clip_command(list_file, out, fps, ffmpeg=args.ffmpeg, crf=args.crf))
    if done.returncode != 0:
        return done.returncode
    size_mb = out.stat().st_size / 1e6
    print(json.dumps(dict(out=out.name, frames=len(files), fps=round(fps, 2), seconds=round(len(files) / fps, 1),
                          size_mb=round(size_mb, 2))))
    if size_mb > args.max_mb:
        sys.stderr.write('the clip is above --max-mb: raise --crf or --every\n')
        return 1
    return 0


def do_still(args):
    files = frame_files(args.frames)
    if not files:
        sys.stderr.write('no frames\n')
        return 1
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    source = files[max(0, min(len(files) - 1, args.index))]
    return subprocess.run([args.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-i', str(source),
                           '-frames:v', '1', str(out)]).returncode


def do_page(args):
    if not args.url.startswith(LOCAL_PAGE_PREFIXES):
        sys.stderr.write('only a page of this PC (127.0.0.1 / localhost) is captured\n')
        return 2
    from playwright.sync_api import sync_playwright
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(channel=args.channel, headless=True)
        page = browser.new_page(viewport=dict(width=args.width, height=args.height))
        page.goto(args.url, wait_until='domcontentloaded', timeout=30000)
        page.wait_for_timeout(int(args.wait_s * 1000))
        if args.selector:
            page.locator(args.selector).first.screenshot(path=str(out))
        else:
            page.screenshot(path=str(out), full_page=args.full_page)
        browser.close()
    print(json.dumps(dict(out=out.name, bytes=out.stat().st_size)))
    return 0


def build_parser():
    p = argparse.ArgumentParser(prog='capture_media', description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='command', required=True)
    c = sub.add_parser('camera')
    c.add_argument('--project', required=True)
    c.add_argument('--seconds', type=float, default=20.)
    c.add_argument('--out', required=True)
    c.add_argument('--docker', default='docker')
    c.set_defaults(run=do_camera)
    c = sub.add_parser('clip')
    c.add_argument('--frames', required=True)
    c.add_argument('--out', required=True)
    c.add_argument('--fps', type=float, default=0.)
    c.add_argument('--every', type=int, default=1)
    c.add_argument('--first', type=int, default=None)
    c.add_argument('--last', type=int, default=None)
    c.add_argument('--crf', type=int, default=30)
    c.add_argument('--max-mb', type=float, default=5.)
    c.add_argument('--ffmpeg', default='ffmpeg')
    c.set_defaults(run=do_clip)
    c = sub.add_parser('still')
    c.add_argument('--frames', required=True)
    c.add_argument('--index', type=int, default=0)
    c.add_argument('--out', required=True)
    c.add_argument('--ffmpeg', default='ffmpeg')
    c.set_defaults(run=do_still)
    c = sub.add_parser('page')
    c.add_argument('--url', default=DASHBOARD_PAGE)
    c.add_argument('--out', required=True)
    c.add_argument('--wait-s', type=float, default=10.)
    c.add_argument('--width', type=int, default=1440)
    c.add_argument('--height', type=int, default=1500)
    c.add_argument('--full-page', action='store_true')
    c.add_argument('--selector', default=None, help='capture this element only (CSS selector)')
    c.add_argument('--channel', default='chrome')
    c.set_defaults(run=do_page)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.run(args)


if __name__ == '__main__':
    sys.exit(main())
