"""check_assets.py (tools/steer_sim): the asset manifest, size limits, source tags, metadata, coordinates and links.

Every case builds a small repository tree in a temporary folder (standard library only, no Pillow). Coordinate-shaped
numbers and absolute paths are assembled at run time so that this file itself holds none of them.
"""
import hashlib
import importlib.util
import json
import struct
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TAG = '[SIM · 자리값]'


def load():
    spec = importlib.util.spec_from_file_location('check_assets', ROOT / 'tools' / 'steer_sim' / 'check_assets.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def chunk(ctype, data):
    body = ctype + data
    return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xffffffff)


def png(width=4, height=2, extra=()):
    raw = b''.join(b'\0' + b'\x80\x80\x80' * width for _ in range(height))
    parts = [b'\x89PNG\r\n\x1a\n', chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))]
    parts += [chunk(t, d) for t, d in extra]
    parts += [chunk(b'IDAT', zlib.compress(raw)), chunk(b'IEND', b'')]
    return b''.join(parts)


def gif(width=1, height=1, comment=False):
    out = b'GIF89a' + struct.pack('<HHBBB', width, height, 0x80, 0, 0) + b'\0\0\0\xff\xff\xff'
    out += b'\x21\xff\x0bNETSCAPE2.0\x03\x01\x00\x00\x00'
    if comment:
        out += b'\x21\xfe\x03abc\x00'
    out += b'\x2c' + struct.pack('<HHHHB', 0, 0, width, height, 0) + b'\x02\x02\x44\x01\x00'
    return out + b'\x3b'


def jpeg(width=8, height=4, app1=False):
    out = b'\xff\xd8\xff\xe0' + struct.pack('>H', 16) + b'JFIF\0\x01\x01\0\0\x01\0\x01\0\0'
    if app1:
        out += b'\xff\xe1' + struct.pack('>H', 8) + b'Exif\0\0'
    out += b'\xff\xc0' + struct.pack('>HBHHB', 11, 8, height, width, 1) + b'\x01\x11\x00'
    return out + b'\xff\xd9'


class Tree:
    """A minimal repository: docs/assets with one PNG (and more on request), the manifest and a README."""

    def __init__(self, folder):
        self.root = Path(folder)
        self.entries = []
        self.add('docs/assets/rover/model.png', png(), 'image')
        self.readme = '# t\n\n![model](docs/assets/rover/model.png)\n\n그림 1. 모델 %s\n' % TAG

    def add(self, rel, data, kind, role=None, checked=True, **over):
        path = self.root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        digest = hashlib.sha256(data).hexdigest()
        m = load()
        width, height, _ = m.media_metadata(path, data)
        entry = dict(path=rel, kind=kind, bytes=len(data), sha256=digest, width=width, height=height, tag=TAG,
                     caption='시험용 그림 설명입니다', origin=dict(source='<runs>/x.png'),
                     privacy=dict(metadata='none', visual_check=dict(date='2026-11-13', sha256=digest) if checked
                                  else None))
        if role:
            entry['role'] = role
        entry.update(over)
        self.entries.append(entry)
        return entry

    def write(self, manifest_extra=''):
        manifest = dict(schema='ubts.sim_repo.assets.v1', note='시험' + manifest_extra, assets=self.entries)
        (self.root / 'docs' / 'assets' / 'ASSETS.json').write_bytes(
            json.dumps(manifest, ensure_ascii=False).encode('utf-8'))
        (self.root / 'README.md').write_bytes(self.readme.encode('utf-8'))

    def check(self):
        self.write()
        return load().check(self.root, origin_tokens=None)


def coordinate_text():
    lat = 33.0 + 2.5 + 0.123456     # inside the lat-like band, 6 decimals
    lon = 124.0 + 3.0 + 0.654321    # inside the lon-like band
    return '%.6f, %.6f' % (lat, lon)


class CheckAssetsTest(unittest.TestCase):
    def run_case(self, edit):
        with tempfile.TemporaryDirectory() as folder:
            tree = Tree(folder)
            edit(tree)
            return tree.check()

    def assertProblem(self, report, text):
        self.assertFalse(report['ok'], report)
        self.assertTrue(any(text in p for p in report['problems']), report['problems'])

    def test_a_clean_tree_passes(self):
        report = self.run_case(lambda t: None)
        self.assertTrue(report['ok'], report['problems'])
        self.assertEqual(report['summary']['assets'], 1)

    def test_an_image_over_the_limit_is_refused(self):
        big = png(extra=[(b'zzZz', b'\0' * 1_000_000)])     # an ancillary chunk makes the file 1 MB
        report = self.run_case(lambda t: t.add('docs/assets/rover/big.png', big, 'image'))
        self.assertProblem(report, 'the image limit is < 1000000')

    def test_a_file_missing_from_the_manifest_is_refused(self):
        def edit(t):
            (t.root / 'docs' / 'assets' / 'world').mkdir(parents=True)
            (t.root / 'docs' / 'assets' / 'world' / 'stray.png').write_bytes(png())
        self.assertProblem(self.run_case(edit), 'docs/assets/world/stray.png: not in docs/assets/ASSETS.json')

    def test_a_listed_file_that_is_gone_is_refused(self):
        def edit(t):
            t.add('docs/assets/rover/gone.png', png(), 'image')
            (t.root / 'docs' / 'assets' / 'rover' / 'gone.png').unlink()
        self.assertProblem(self.run_case(edit), 'gone.png): file missing')

    def test_an_image_without_its_source_tag_next_to_it_is_refused(self):
        def edit(t):
            t.readme = '# t\n\n![model](docs/assets/rover/model.png)\n\n그림 1. 모델\n\n\n\n출처 %s\n' % TAG
        self.assertProblem(self.run_case(edit), 'has no source tag')

    def test_an_unknown_tag_is_refused(self):
        def edit(t):
            t.entries[0]['tag'] = '[SIM]'
        self.assertProblem(self.run_case(edit), "tag '[SIM]' is not one of the source tags")

    def test_coordinate_shaped_numbers_are_refused_in_the_manifest_and_the_documents(self):
        def in_caption(t):
            t.entries[0]['caption'] = '출발점 ' + coordinate_text()
        self.assertProblem(self.run_case(in_caption), 'ASSETS.json: coordinate-shaped text')

        def in_readme(t):
            t.readme += '\n위치 ' + coordinate_text() + '\n'
        self.assertProblem(self.run_case(in_readme), 'README.md: coordinate-shaped text')

    def test_a_local_absolute_path_is_refused(self):
        def edit(t):
            t.readme += '\n자료: ' + 'D' + ':' + '/Develop/x/y.png\n'
        self.assertProblem(self.run_case(edit), 'README.md: local absolute path')

        def in_manifest(t):
            t.entries[0]['origin'] = dict(source='C' + ':' + '\\runs\\x.png')
        self.assertProblem(self.run_case(in_manifest), 'ASSETS.json: local absolute path')

    def test_a_changed_file_needs_a_new_visual_check(self):
        def edit(t):
            t.entries[0]['privacy']['visual_check']['sha256'] = '0' * 64
        self.assertProblem(self.run_case(edit), 'no visual check of this exact file')

    def test_a_wrong_hash_or_size_is_refused(self):
        def edit(t):
            t.entries[0]['sha256'] = 'f' * 64
            t.entries[0]['width'] = 99
        report = self.run_case(edit)
        self.assertProblem(report, 'sha256 differs from the file')
        self.assertProblem(report, 'size 99x2 recorded, 4x2 in the file')

    def test_metadata_blocks_that_can_hold_private_data_are_refused(self):
        self.assertProblem(self.run_case(lambda t: t.add('docs/assets/rover/text.png',
                                                         png(extra=[(b'tEXt', b'Author\0someone')]), 'image')),
                           'metadata png_chunk:tEXt')
        self.assertProblem(self.run_case(lambda t: t.add('docs/assets/video/c.gif', gif(comment=True), 'animation')),
                           'metadata gif_comment')
        self.assertProblem(self.run_case(lambda t: t.add('docs/assets/world/e.jpg', jpeg(app1=True), 'image')),
                           'metadata jpeg_exif_or_xmp')

    def test_clean_gif_and_jpeg_pass_with_their_sizes(self):
        def edit(t):
            t.add('docs/assets/video/ok.gif', gif(3, 2), 'animation')
            t.add('docs/assets/world/ok.jpg', jpeg(8, 4), 'image')
        report = self.run_case(edit)
        self.assertTrue(report['ok'], report['problems'])

    def test_the_readme_hero_must_be_a_small_narrow_animation(self):
        def edit(t):
            t.add('docs/assets/video/wide.gif', gif(1280, 720), 'animation', role='readme_hero')
        self.assertProblem(self.run_case(edit), 'the README hero must be an animation')

    def test_names_and_folders_follow_the_rule(self):
        def edit(t):
            t.add('docs/assets/misc/Bad Name.png', png(), 'image')
        self.assertProblem(self.run_case(edit), 'path must be docs/assets/')

    def test_links_and_anchors(self):
        def broken(t):
            t.readme += '\n[설계](docs/NOPE_KO.md)\n'
        self.assertProblem(self.run_case(broken), 'README.md:7: broken link docs/NOPE_KO.md')

        def anchors(t):
            doc = t.root / 'docs' / 'A_KO.md'
            doc.parent.mkdir(parents=True, exist_ok=True)
            doc.write_bytes('# 문서\n\n## 2. 빠른 시작 (로컬)\n\n```\n# 코드 안의 줄\n```\n'.encode('utf-8'))
            t.readme += '\n[시작](docs/A_KO.md#2-빠른-시작-로컬) [없음](docs/A_KO.md#코드-안의-줄)\n'
        report = self.run_case(anchors)
        self.assertProblem(report, 'no heading for #코드-안의-줄')
        self.assertFalse(any('빠른-시작-로컬' in p for p in report['problems']), report['problems'])

    def test_text_in_brackets_of_an_older_topic_document_is_only_a_warning(self):
        def edit(t):
            doc = t.root / 'docs' / 'steering' / 'OLD_KO.md'
            doc.parent.mkdir(parents=True, exist_ok=True)
            doc.write_bytes('위치 P[n](7.3절)과 [옛 문서](MISSING_KO.md)\n'.encode('utf-8'))
        report = self.run_case(edit)
        self.assertProblem(report, 'docs/steering/OLD_KO.md:1: broken link MISSING_KO.md')
        self.assertTrue(any('7.3절' in w for w in report['warnings']), report['warnings'])

    def test_the_total_of_all_media_under_docs_is_limited(self):
        m = load()
        with tempfile.TemporaryDirectory() as folder:
            tree = Tree(folder)
            tree.write()
            saved = m.TOTAL_MAX_BYTES
            try:
                m.TOTAL_MAX_BYTES = 10
                report = m.check(tree.root, origin_tokens=None)
            finally:
                m.TOTAL_MAX_BYTES = saved
        self.assertFalse(report['ok'])
        self.assertTrue(any('media total' in p for p in report['problems']), report['problems'])


if __name__ == '__main__':
    unittest.main()
