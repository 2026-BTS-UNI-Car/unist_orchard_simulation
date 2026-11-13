#!/usr/bin/env python3
"""Check the documentation assets and the Markdown files of this repository (docs/DOCS_POLICY_KO.md).

  python -I tools/steer_sim/check_assets.py [--root PATH] [--json]

Reads only; writes nothing. Standard library only. Exit 0 = no problem, 1 = problems (listed).

What is checked
  docs/assets/ASSETS.json (the asset manifest) against docs/assets/:
    - every file under docs/assets/ except the manifest is listed, every listed file exists;
    - path, folder and name rule (docs/assets/<folder>/<name>.<ext>, lower-case ASCII, digits, '_');
    - bytes and sha256 as recorded; width and height as recorded (PNG, GIF, JPEG headers);
    - kind matches the extension (image, animation, video) and the size limit of the kind;
    - a source tag from the closed set (TAGS), a caption, an origin (a source file or a command);
    - the visual check: privacy.visual_check.sha256 is the sha256 of the file as it is now (a changed file needs a
      new look), and the file carries no metadata that can hold private data (PNG text/EXIF chunks, JPEG EXIF/XMP/
      IPTC/comment segments, GIF comments or unknown application blocks, MP4 location atoms);
    - an entry with role readme_hero: an animation of at most HERO_MAX_BYTES and HERO_MAX_WIDTH pixels wide.
  Every image or video file under docs/: the per-file limit of its kind; the total of all of them <= TOTAL_MAX_BYTES.
  Markdown (README.md and every *.md under docs/):
    - relative links and images point to files that exist (and a '#anchor' to a heading of that file); in the topic
      folders a missing target that does not look like a path (e.g. 'P[n](7.3)' in a formula) is a warning only;
    - an image of docs/assets/ is followed (same line or the next CAPTION_LINES lines) by the source tag of its
      manifest entry;
    - the new documents (README.md, docs/*.md directly under docs/, docs/assets/**) and the manifest: no local
      absolute path (drive letter, UNC, home folders) and no coordinate-shaped numbers (the A16 rules of
      sim/ubts/field_world/coord_lint.py: latlon_pair, yaml_origin when the maps folder is configured, gnss_path).
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_REL = 'docs/assets/ASSETS.json'
SCHEMA = 'ubts.sim_repo.assets.v1'
ASSET_DIR = 'docs/assets'
FOLDERS = ('rover', 'world', 'sensors', 'dashboard', 'video', 'diagrams', 'photos')
NAME_RE = re.compile(r'^[a-z0-9]+(?:_[a-z0-9]+)*\.(png|jpg|gif|webp|svg|mp4|webm)$')
KIND_OF_EXT = {'.png': 'image', '.jpg': 'image', '.jpeg': 'image', '.webp': 'image', '.svg': 'image',
               '.gif': 'animation', '.mp4': 'video', '.webm': 'video', '.mov': 'video'}
LIMIT_OF_KIND = {'image': 1_000_000, 'animation': 10_000_000, 'video': 10_000_000}   # bytes, '<' (strictly less)
TOTAL_MAX_BYTES = 50_000_000                                                         # all media under docs/, '<='
HERO_MAX_BYTES = 5_000_000
HERO_MAX_WIDTH = 640
TAGS = (re.compile(r'^\[SIM · 자리값\]$'), re.compile(r'^\[현장 기록 · \d{4}-\d{2}-\d{2}\]$'), re.compile(r'^\[도식\]$'),
        re.compile(r'^\[사진\]$'), re.compile(r'^\[계획 · 미구현\]$'))
CAPTION_LINES = 3
ABS_PATH_RE = re.compile(r'(?<![\w])[A-Za-z]:[\\/]|\\\\[A-Za-z0-9]|(?<![\w])/(?:home|Users|mnt/[a-z])/', re.I)
LINK_RE = re.compile(r'(!?)\[([^\]]*)\]\(([^)\s]+)(?:\s+"[^"]*")?\)')
SKIP_DIRS = ('.git', '__pycache__', '.pytest_cache', 'node_modules')
PATH_LIKE_RE = re.compile(r'^(?:\.{0,2}/)?[\w.-]+(?:/[\w.-]+)*(?:\.[A-Za-z0-9]{1,5}|/)(?:#[\w-]*)?$')


def sha256_of(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


# --- image headers and metadata ----------------------------------------------------------------------------------

def png_info(data: bytes):
    """(width, height, [chunk types]) or None when not a PNG."""
    if data[:8] != b'\x89PNG\r\n\x1a\n':
        return None
    pos, chunks, size = 8, [], None
    while pos + 8 <= len(data):
        length, ctype = struct.unpack('>I4s', data[pos:pos + 8])
        ctype = ctype.decode('latin-1')
        chunks.append(ctype)
        if ctype == 'IHDR':
            size = struct.unpack('>II', data[pos + 8:pos + 16])
        pos += 12 + length
        if ctype == 'IEND':
            break
    return (size[0], size[1], chunks) if size else None


def gif_info(data: bytes):
    """(width, height, [problems]) or None when not a GIF. Walks the blocks: comment extensions and application
    extensions other than the loop blocks are problems."""
    if data[:6] not in (b'GIF87a', b'GIF89a'):
        return None
    width, height, flags = struct.unpack('<HHB', data[6:11])
    pos = 13
    if flags & 0x80:
        pos += 3 * (1 << ((flags & 7) + 1))
    problems = []

    def skip_sub_blocks(p):
        while p < len(data):
            n = data[p]
            p += 1
            if n == 0:
                return p
            p += n
        return p

    while pos < len(data):
        b = data[pos]
        if b == 0x3B:                                   # trailer
            break
        if b == 0x21:                                   # extension
            label = data[pos + 1]
            if label == 0xFE:
                problems.append('gif_comment')
            if label == 0xFF:
                app = data[pos + 3:pos + 3 + data[pos + 2]]
                if app not in (b'NETSCAPE2.0', b'ANIMEXTS1.0'):
                    problems.append('gif_application_block:%s' % app.decode('latin-1', 'replace'))
            pos = skip_sub_blocks(pos + 2)
            continue
        if b == 0x2C:                                   # image descriptor
            iflags = data[pos + 9]
            pos += 10
            if iflags & 0x80:
                pos += 3 * (1 << ((iflags & 7) + 1))
            pos = skip_sub_blocks(pos + 1)              # LZW minimum code size, then data
            continue
        problems.append('gif_bad_block')
        break
    return width, height, problems


def jpeg_info(data: bytes):
    """(width, height, [problems]) or None when not a JPEG."""
    if data[:2] != b'\xff\xd8':
        return None
    pos, size, problems = 2, None, []
    while pos + 4 <= len(data):
        if data[pos] != 0xFF:
            problems.append('jpeg_bad_marker')
            break
        marker = data[pos + 1]
        if marker in (0xD9, 0xDA):                      # end, start of scan: no more headers
            break
        length = struct.unpack('>H', data[pos + 2:pos + 4])[0]
        body = data[pos + 4:pos + 2 + length]
        if marker == 0xE1:
            problems.append('jpeg_exif_or_xmp')
        elif marker == 0xED:
            problems.append('jpeg_iptc')
        elif marker == 0xFE:
            problems.append('jpeg_comment')
        elif marker in (0xC0, 0xC1, 0xC2, 0xC3) and size is None:
            height, width = struct.unpack('>HH', body[1:5])
            size = (width, height)
        pos += 2 + length
    return (size[0], size[1], problems) if size else (None, None, problems + ['jpeg_no_size'])


def media_metadata(path: Path, data: bytes):
    """(width, height, [problems]) for the formats we can read; (None, None, [...]) otherwise."""
    ext = path.suffix.lower()
    if ext == '.png':
        info = png_info(data)
        if info is None:
            return None, None, ['not_a_png']
        width, height, chunks = info
        bad = sorted({c for c in chunks if c in ('tEXt', 'zTXt', 'iTXt', 'eXIf')})
        return width, height, ['png_chunk:%s' % c for c in bad]
    if ext == '.gif':
        info = gif_info(data)
        return info if info else (None, None, ['not_a_gif'])
    if ext in ('.jpg', '.jpeg'):
        info = jpeg_info(data)
        return info if info else (None, None, ['not_a_jpeg'])
    if ext in ('.mp4', '.mov'):
        problems = [name for name, atom in (('mp4_location_xyz', b'\xa9xyz'), ('mp4_location_loci', b'loci'))
                    if atom in data]
        return None, None, problems
    return None, None, []


# --- Markdown ------------------------------------------------------------------------------------------------------

def github_anchors(text: str):
    """Anchors GitHub makes for the headings of one Markdown text (lower case, punctuation dropped, spaces -> '-',
    repeats get -1, -2, ...). Headings inside ``` fences are not headings."""
    anchors, seen, fence = set(), {}, False
    for line in text.splitlines():
        if line.lstrip().startswith('```'):
            fence = not fence
            continue
        if fence:
            continue
        m = re.match(r'^(#{1,6})\s+(.*?)\s*#*\s*$', line)
        if not m:
            continue
        title = re.sub(r'`([^`]*)`', r'\1', m.group(2))
        title = re.sub(r'\[([^\]]*)\]\([^)]*\)', r'\1', title)
        slug = ''.join(ch for ch in title.lower() if ch.isalnum() or ch in '-_ ').replace(' ', '-')
        n = seen.get(slug, 0)
        anchors.add(slug if n == 0 else '%s-%d' % (slug, n))
        seen[slug] = n + 1
    for m in re.finditer(r'<a\s+(?:id|name)="([^"]+)"', text):
        anchors.add(m.group(1))
    return anchors


def markdown_links(text: str):
    """(line number, is_image, target) of every inline link outside ``` fences and `code`."""
    out, fence = [], False
    for number, line in enumerate(text.splitlines(), 1):
        if line.lstrip().startswith('```'):
            fence = not fence
            continue
        if fence:
            continue
        plain = re.sub(r'`[^`]*`', '', line)
        for m in LINK_RE.finditer(plain):
            out.append((number, m.group(1) == '!', m.group(3)))
    return out


def markdown_files(root: Path):
    files = [root / 'README.md'] if (root / 'README.md').is_file() else []
    docs = root / 'docs'
    if docs.is_dir():
        for path in sorted(docs.rglob('*.md')):
            if not any(part in SKIP_DIRS for part in path.relative_to(root).parts):
                files.append(path)
    return files


def own_document(rel: str) -> bool:
    """The documents of the asset and front-page work (README, docs/*.md, docs/assets/**) get the stricter text
    rules; the topic folders (docs/steering, ...) keep their own history."""
    parts = rel.split('/')
    return rel == 'README.md' or (parts[0] == 'docs' and (len(parts) == 2 or parts[1] == 'assets'))


# --- coordinates ---------------------------------------------------------------------------------------------------

def load_coord_lint(root: Path):
    path = root / 'sim' / 'ubts' / 'field_world' / 'coord_lint.py'
    if not path.is_file():
        path = ROOT / 'sim' / 'ubts' / 'field_world' / 'coord_lint.py'
    spec = importlib.util.spec_from_file_location('ubts_coord_lint_for_assets', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def coordinate_findings(lint, text: str, origin_tokens):
    counts = lint.lint_text(text, origin_tokens)
    return {k: counts[k] for k in ('latlon_pair', 'yaml_origin', 'gnss_path') if counts.get(k)}


# --- the check -----------------------------------------------------------------------------------------------------

def check(root: Path = ROOT, origin_tokens='configured'):
    """dict(ok, problems [str], summary). origin_tokens: 'configured' = the 10/10 map origins when the maps folder is
    configured (coord_lint default), or a token dict / None for tests."""
    root = Path(root)
    problems, warnings = [], []
    summary = dict(assets=0, asset_bytes=0, media_files=0, media_bytes=0, markdown_files=0, links=0)
    lint = load_coord_lint(root)
    tokens = None if origin_tokens == 'configured' else origin_tokens
    if origin_tokens == 'configured':
        sim_dir = str(root / 'sim' / 'ubts')     # coord_lint reads the configured maps folder through closed_loop
        added = sim_dir not in sys.path
        if added:
            sys.path.insert(0, sim_dir)
        try:
            tokens = lint._yaml_origin_tokens()
        finally:
            if added:
                sys.path.remove(sim_dir)
    summary['yaml_origin_checked'] = tokens is not None

    # manifest
    manifest_path = root / MANIFEST_REL
    entries = []
    if not manifest_path.is_file():
        problems.append('%s: missing' % MANIFEST_REL)
    else:
        raw = manifest_path.read_bytes()
        try:
            manifest = json.loads(raw.decode('utf-8'))
        except (UnicodeDecodeError, ValueError) as exc:
            manifest = None
            problems.append('%s: not JSON (%s)' % (MANIFEST_REL, exc))
        if manifest is not None:
            if manifest.get('schema') != SCHEMA:
                problems.append('%s: schema is %r, expected %r' % (MANIFEST_REL, manifest.get('schema'), SCHEMA))
            entries = manifest.get('assets') or []
            if not isinstance(entries, list):
                problems.append('%s: assets is not a list' % MANIFEST_REL)
                entries = []
            text = raw.decode('utf-8')
            if ABS_PATH_RE.search(text):
                problems.append('%s: local absolute path' % MANIFEST_REL)
            found = coordinate_findings(lint, text, tokens)
            if found:
                problems.append('%s: coordinate-shaped text %s' % (MANIFEST_REL, found))

    listed = {}
    for i, entry in enumerate(entries):
        where = '%s assets[%d]' % (MANIFEST_REL, i)
        if not isinstance(entry, dict) or not isinstance(entry.get('path'), str):
            problems.append('%s: no path' % where)
            continue
        rel = entry['path']
        where = '%s (%s)' % (MANIFEST_REL, rel)
        if rel in listed:
            problems.append('%s: listed twice' % where)
        listed[rel] = entry
        parts = rel.split('/')
        if len(parts) != 4 or '/'.join(parts[:2]) != ASSET_DIR or parts[2] not in FOLDERS or not NAME_RE.match(parts[3]):
            problems.append('%s: path must be %s/<%s>/<lower_case_name>.<ext>' % (where, ASSET_DIR, '|'.join(FOLDERS)))
        path = root / rel
        if not path.is_file():
            problems.append('%s: file missing' % where)
            continue
        data = path.read_bytes()
        summary['assets'] += 1
        summary['asset_bytes'] += len(data)
        kind = KIND_OF_EXT.get(path.suffix.lower())
        if entry.get('kind') != kind:
            problems.append('%s: kind %r, the extension says %r' % (where, entry.get('kind'), kind))
        if entry.get('bytes') != len(data):
            problems.append('%s: bytes %r recorded, %d on disk' % (where, entry.get('bytes'), len(data)))
        digest = sha256_of(data)
        if entry.get('sha256') != digest:
            problems.append('%s: sha256 differs from the file' % where)
        width, height, meta = media_metadata(path, data)
        for problem in meta:
            problems.append('%s: metadata %s' % (where, problem))
        if width is not None and (entry.get('width'), entry.get('height')) != (width, height):
            problems.append('%s: size %sx%s recorded, %sx%s in the file' % (where, entry.get('width'),
                                                                           entry.get('height'), width, height))
        tag = entry.get('tag')
        if not isinstance(tag, str) or not any(p.match(tag) for p in TAGS):
            problems.append('%s: tag %r is not one of the source tags' % (where, tag))
        caption = entry.get('caption')
        if not isinstance(caption, str) or len(caption.strip()) < 8:
            problems.append('%s: no caption' % where)
        origin = entry.get('origin') or {}
        if not (origin.get('source') or origin.get('command')):
            problems.append('%s: no origin (source or command)' % where)
        visual = (entry.get('privacy') or {}).get('visual_check') or {}
        if visual.get('sha256') != digest or not visual.get('date'):
            problems.append('%s: no visual check of this exact file (privacy.visual_check.sha256/date)' % where)
        if entry.get('role') == 'readme_hero':
            if kind != 'animation' or len(data) > HERO_MAX_BYTES or (width or 0) > HERO_MAX_WIDTH:
                problems.append('%s: the README hero must be an animation <= %d bytes and <= %d px wide'
                                % (where, HERO_MAX_BYTES, HERO_MAX_WIDTH))

    asset_root = root / ASSET_DIR
    if asset_root.is_dir():
        for path in sorted(asset_root.rglob('*')):
            if path.is_file():
                rel = path.relative_to(root).as_posix()
                if rel != MANIFEST_REL and rel not in listed:
                    problems.append('%s: not in %s' % (rel, MANIFEST_REL))

    # every media file under docs/
    docs = root / 'docs'
    if docs.is_dir():
        for path in sorted(docs.rglob('*')):
            kind = KIND_OF_EXT.get(path.suffix.lower())
            if not path.is_file() or kind is None:
                continue
            size = path.stat().st_size
            summary['media_files'] += 1
            summary['media_bytes'] += size
            if size >= LIMIT_OF_KIND[kind]:
                problems.append('%s: %d bytes, the %s limit is < %d' % (path.relative_to(root).as_posix(), size,
                                                                         kind, LIMIT_OF_KIND[kind]))
    if summary['media_bytes'] > TOTAL_MAX_BYTES:
        problems.append('docs/: media total %d bytes > %d' % (summary['media_bytes'], TOTAL_MAX_BYTES))

    # Markdown
    tag_of = {rel: entry.get('tag') for rel, entry in listed.items()}
    anchor_cache = {}
    for md in markdown_files(root):
        rel_md = md.relative_to(root).as_posix()
        text = md.read_bytes().decode('utf-8', 'replace')
        lines = text.splitlines()
        summary['markdown_files'] += 1
        if own_document(rel_md):
            if ABS_PATH_RE.search(re.sub(r'https?://\S+', '', text)):
                problems.append('%s: local absolute path' % rel_md)
            found = coordinate_findings(lint, text, tokens)
            if found:
                problems.append('%s: coordinate-shaped text %s' % (rel_md, found))
        for number, is_image, target in markdown_links(text):
            if re.match(r'^[a-z][a-z0-9+.-]*:', target, re.I):     # http:, https:, mailto:, ...
                continue
            summary['links'] += 1
            file_part, _, anchor = target.partition('#')
            dest = md if not file_part else (md.parent / file_part)
            try:
                dest = dest.resolve()
                dest_rel = dest.relative_to(root.resolve()).as_posix()
            except ValueError:
                problems.append('%s:%d: link leaves the repository: %s' % (rel_md, number, target))
                continue
            if not dest.exists():
                message = '%s:%d: broken link %s' % (rel_md, number, target)
                if own_document(rel_md) or PATH_LIKE_RE.match(target):
                    problems.append(message)
                else:      # e.g. P[n](7.3) in a formula of an older topic document: renders as a link, not a path
                    warnings.append(message + ' (not path-like: probably text in brackets followed by parentheses)')
                continue
            if anchor and dest.suffix.lower() == '.md':
                if dest not in anchor_cache:
                    anchor_cache[dest] = github_anchors(dest.read_bytes().decode('utf-8', 'replace'))
                if anchor not in anchor_cache[dest]:
                    problems.append('%s:%d: no heading for #%s in %s' % (rel_md, number, anchor, dest_rel))
            if is_image and dest_rel in tag_of:
                window = '\n'.join(lines[number - 1:number + CAPTION_LINES])
                if not tag_of[dest_rel] or tag_of[dest_rel] not in window:
                    problems.append('%s:%d: image %s has no source tag %s next to it'
                                    % (rel_md, number, dest_rel, tag_of[dest_rel]))
    return dict(ok=not problems, problems=problems, warnings=warnings, summary=summary)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Check docs/assets, the asset manifest and the Markdown links.')
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)
    report = check(args.root)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=1))
    else:
        s = report['summary']
        print('assets %d (%d bytes); media under docs/ %d files, %d bytes (limit %d); markdown %d files, %d links; '
              'map origins checked %s' % (s['assets'], s['asset_bytes'], s['media_files'], s['media_bytes'],
                                          TOTAL_MAX_BYTES, s['markdown_files'], s['links'], s['yaml_origin_checked']))
        for problem in report['problems']:
            print('  ' + problem)
        for warning in report['warnings']:
            print('  warning: ' + warning)
        print('OK' if report['ok'] else 'PROBLEMS %d' % len(report['problems']))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
