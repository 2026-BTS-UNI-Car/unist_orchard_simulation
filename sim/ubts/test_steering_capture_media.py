"""capture_media.py (tools/steer_sim): the parts that need no Docker, browser or ffmpeg."""
import importlib.util
import io
import json
import tarfile
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def load():
    spec = importlib.util.spec_from_file_location('capture_media', ROOT / 'tools' / 'steer_sim' / 'capture_media.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def tar_of(entries):
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode='w|') as tar:
        for name, data in entries:
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
    return buf.getvalue()


class CaptureMediaTest(unittest.TestCase):
    def test_camera_command_reads_the_sim_container_of_the_project(self):
        m = load()
        command = m.camera_command('ubts-steer-job', 12.5)
        self.assertEqual(command[:4], ['docker', 'exec', '-i', 'ubts-steer-job-sim-1'])
        self.assertEqual(command[-2:], [m.CAMERA_WS, '12.5'])
        compile(m.INNER, 'inner', 'exec')

    def test_unpack_writes_frames_only_and_measures_the_rate(self):
        m = load()
        jpeg = bytes([0xFF, 0xD8, 0xFF, 0xD9])
        summary = dict(url=m.CAMERA_WS, seconds=1., count=3, receive_s=[0., .5, 1.])
        data = tar_of([('FRAMES.json', json.dumps(summary).encode('utf-8')), ('frames/000000.jpg', jpeg),
                       ('frames/000001.jpg', jpeg), ('frames/000002.jpg', jpeg), ('frames/../escape.jpg', jpeg),
                       ('other.txt', b'x')])
        with tempfile.TemporaryDirectory() as tmp:
            result = m.unpack_frames(data, tmp)
            self.assertEqual(result['written'], 3)
            self.assertEqual(result['measured_fps'], 2.)
            self.assertEqual([f.name for f in m.frame_files(tmp)], ['000000.jpg', '000001.jpg', '000002.jpg'])
            self.assertFalse((Path(tmp) / 'escape.jpg').exists())
            self.assertFalse((Path(tmp) / 'other.txt').exists())

    def test_clip_is_h264_without_audio(self):
        m = load()
        command = m.clip_command('list.txt', 'out.mp4', 15.)
        self.assertIn('libx264', command)
        self.assertIn('-an', command)
        self.assertEqual(command[command.index('-pix_fmt') + 1], 'yuv420p')

    def test_page_refuses_a_page_that_is_not_local(self):
        m = load()
        args = m.build_parser().parse_args(['page', '--url', 'http://example.invalid/', '--out', 'x.png'])
        self.assertEqual(args.run(args), 2)


if __name__ == '__main__':
    unittest.main()
