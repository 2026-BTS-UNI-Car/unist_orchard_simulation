"""A16 coordinate-leak lint (design 1.4 rule 7, 5.2, 7.2 A16) for world outputs, run outputs and source files.

A16 (judged; ok = these three counts are 0; matched text is never written out):
  latlon_pair     two decimal numbers with >= 6 decimals within 60 characters of each other, one in the lat-like
                  band [33, 39.5] and the other in the lon-like band [124, 132] (either order): the number form of a
                  GNSS position near the recording sites. Field world writers keep W / raster-local metres at <= 5
                  decimals in SDF text, so their own numbers cannot form such a pair.
  yaml_origin     the origin numbers of the 10/10 map YAML files (read-only from the configured maps folder; kept in
                  memory): each value as written (6 decimals, with or without the sign) as a whole number token, and
                  the (x, y) pair rounded to 3, 4 or 5 decimals with both within 80 characters; 'checked' is false
                  when the maps folder is not configured
  gnss_path       the GNSS data folders of the 10/10 data (the GNSS check folder, the private GNSS folder and the
                  drive extract folder)
Strict rule for run outputs (2026-11-09 review RW-07; judged with --strict / strict=True, reported otherwise):
  latlon_pair_strict  every pair of decimal numbers with >= 4 decimals (about 10 m and finer) within 200 characters,
                  one in each band (not only neighbouring numbers): catches 4-5 decimal pairs and pairs with other
                  numbers between them. World outputs (SDF, manifest) keep the non-strict judged rule because their W
                  and raster-local metres at 5 decimals could form such a pair; run outputs (results, logs) are
                  checked strictly (a hit is inspected by hand: a SIM-local value can fall in both bands).
Extra (report, extra_ok): GNSS field and topic words that legitimate code may name to remove them (the two
degree-coordinate field names and their short keys, the GPS topic prefix, the navigation-satellite plugin word, the
GNSS sentence format name, the SDF spherical coordinates element). World outputs are expected to have 0 of them too.
The patterns are assembled from parts so that this file does not contain them. Binary files are skipped and listed.
Which files are read: by default the files whose suffix is in TEXT_SUFFIXES (others count as binary). With
all_text=True / --all-text the suffix is ignored: every file is read unless its first ALL_TEXT_PROBE_BYTES bytes hold
a NUL byte (.js, .mjs, .css, .html, .ino, Dockerfile, files without a suffix, ...), every skipped file is listed
(skipped_binary_files is not cut at 50) and .git / __pycache__ folders are not entered. Use it for a whole
repository tree.

CLI: python -B -m field_world.coord_lint <file or folder> ... [--json] [--strict] [--all-text]   (exit 0 = A16 ok)
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

LAT_BAND = (33., 39.5)
LON_BAND = (124., 132.)
PAIR_WINDOW = 60
ORIGIN_PAIR_WINDOW = 80
STRICT_DECIMALS = 4
STRICT_WINDOW = 200
TEXT_SUFFIXES = ('.sdf', '.json', '.jsonl', '.txt', '.log', '.md', '.py', '.yaml', '.yml', '.csv', '.sh', '.xml',
                 '.world', '.cfg', '.ini', '.toml', '.cc', '.h', '.cpp', '.ps1', '.bat')
ALL_TEXT_PROBE_BYTES = 8192
ALL_TEXT_SKIP_DIRS = ('.git', '__pycache__')
JUDGED = ('latlon_pair', 'yaml_origin', 'gnss_path')
STRICT = ('latlon_pair_strict',)
EXTRA = ('gnss_word',)
_NUMBER = re.compile(r'(?<![\w.])-?\d{1,3}\.\d{6,}(?![\w.])')
_NUMBER_STRICT = re.compile(r'(?<![\w.])-?\d{1,3}\.\d{%d,}(?![\w.])' % STRICT_DECIMALS)
_GNSS_PATHS = ('gnss' + '_check', 'gnss' + '-private', 'drive' + '-extract')
_GNSS_WORDS = re.compile('(?i)' + '|'.join((r'\blati' + r'tude', r'\blongi' + r'tude', r'\blat' + r'_deg\b',
                                            r'\blon' + r'_deg\b', '"' + 'lat' + '"', '"' + 'lon' + '"',
                                            '/' + 'gps/', 'nav' + 'sat', r'\bnm' + r'ea\b',
                                            'spherical' + '_coordinates')))
_YAML_CACHE = {}


def origin_tokens_from_values(pairs):
    """Lint tokens of origin (x, y) pairs: exact = the 6-decimal texts (either sign), pairs = (x, y) texts rounded to
    3, 4 and 5 decimals (signed; flagged when both appear within ORIGIN_PAIR_WINDOW characters)."""
    exact, pair_tokens = set(), []
    for x, y in pairs:
        for value in (x, y):
            text = '%.6f' % value
            exact.update({text, text.lstrip('-')})
        for digits in (3, 4, 5):
            pair_tokens.append(('%.*f' % (digits, x), '%.*f' % (digits, y)))
    return dict(exact=exact, pairs=pair_tokens)


def _yaml_origin_tokens():
    """Lint tokens of the map YAML origins (None when the maps folder is not configured). Values stay in memory."""
    if 'tokens' in _YAML_CACHE:
        return _YAML_CACHE['tokens']
    tokens = None
    try:
        from closed_loop import external_paths
        root = external_paths.configured_path('maps_1010_root', 'dir') / 'offline' / 'maps'
        pairs = []
        for yaml_path in sorted(root.glob('*/*_offline.yaml')):
            for line in yaml_path.read_text(encoding='utf-8').splitlines():
                key, _, value = line.partition(':')
                if key.strip() == 'origin':
                    items = value.strip().strip('[]').split(',')
                    pairs.append((float(items[0]), float(items[1])))
        tokens = origin_tokens_from_values(pairs) if pairs else None
    except (OSError, ValueError, ImportError):
        tokens = None
    _YAML_CACHE['tokens'] = tokens
    return tokens


def _token_positions(text, token):
    out, start = [], 0
    while True:
        pos = text.find(token, start)
        if pos < 0:
            return out
        before = text[pos - 1] if pos > 0 else ''
        after = text[pos + len(token)] if pos + len(token) < len(text) else ''
        if not (before.isdigit() or before == '.') and not (after.isdigit() or after == '.'):
            out.append(pos)
        start = pos + 1


def strict_pairs(text, window=STRICT_WINDOW):
    """Pairs of >= STRICT_DECIMALS-decimal numbers, one in each band, within window characters (all pairs)."""
    import bisect
    lat, lon = [], []
    for m in _NUMBER_STRICT.finditer(text):
        value = float(m.group())
        if LAT_BAND[0] <= value <= LAT_BAND[1]:
            lat.append(m.start())
        if LON_BAND[0] <= value <= LON_BAND[1]:
            lon.append(m.start())
    count = 0
    for pos in lat:
        count += bisect.bisect_right(lon, pos + window) - bisect.bisect_left(lon, pos - window)
    return count


def lint_text(text, origin_tokens=None):
    """Counts of the rules for one text (origin_tokens: None -> the configured YAML origins; yaml_origin None when
    they are not available)."""
    numbers = [(m.start(), float(m.group())) for m in _NUMBER.finditer(text)]
    pairs = 0
    for (p1, v1), (p2, v2) in zip(numbers, numbers[1:]):
        if p2 - p1 > PAIR_WINDOW:
            continue
        lat_lon = LAT_BAND[0] <= v1 <= LAT_BAND[1] and LON_BAND[0] <= v2 <= LON_BAND[1]
        lon_lat = LON_BAND[0] <= v1 <= LON_BAND[1] and LAT_BAND[0] <= v2 <= LAT_BAND[1]
        pairs += int(lat_lon or lon_lat)
    lower = text.lower()
    tokens = _yaml_origin_tokens() if origin_tokens is None else origin_tokens
    origin = None
    if tokens is not None:
        origin = sum(len(_token_positions(text, token)) for token in tokens['exact'])
        for tx, ty in tokens['pairs']:
            xs = _token_positions(text, tx)
            if not xs:
                continue
            ys = _token_positions(text, ty)
            origin += sum(1 for a in xs for b in ys if abs(a - b) <= ORIGIN_PAIR_WINDOW)
    return dict(latlon_pair=pairs, yaml_origin=origin, gnss_path=sum(lower.count(w) for w in _GNSS_PATHS),
                gnss_word=len(_GNSS_WORDS.findall(text)), latlon_pair_strict=strict_pairs(text))


def _entries(items, all_text=False):
    for item in items:
        if hasattr(item, 'text') and hasattr(item, 'name'):
            yield str(item.name), item.text
            continue
        path = Path(item)
        if path.is_dir():
            for sub in sorted(path.rglob('*')):
                if all_text and any(part in ALL_TEXT_SKIP_DIRS for part in sub.relative_to(path).parts):
                    continue
                if sub.is_file():
                    yield from _entries([sub], all_text)
            continue
        if all_text:
            data = path.read_bytes()
            yield str(path), (None if b'\0' in data[:ALL_TEXT_PROBE_BYTES] else data.decode('utf-8', 'replace'))
            continue
        if path.suffix.lower() not in TEXT_SUFFIXES:
            yield str(path), None
            continue
        yield str(path), path.read_bytes().decode('utf-8', 'replace')


def lint_files(items, origin_tokens=None, strict=False, all_text=False):
    """dict(ok (A16), extra_ok, totals, findings, files_checked, skipped_binary, yaml_origin_checked, rules) over files,
    folders or text probes (objects with .name and .text). strict: the strict pair rule is judged too (run outputs).
    all_text: read every file without a NUL byte in its first bytes, whatever its suffix (module docstring)."""
    tokens = _yaml_origin_tokens() if origin_tokens is None else origin_tokens
    rows, skipped = [], []
    judged = JUDGED + (STRICT if strict else ())
    totals = {key: 0 for key in JUDGED + STRICT + EXTRA}
    checked = 0
    for name, text in _entries(items, all_text):
        if text is None:
            skipped.append(name)
            continue
        checked += 1
        counts = lint_text(text, tokens)
        for key in totals:
            totals[key] += int(counts[key] or 0)
        if any(counts[k] for k in totals):
            rows.append(dict(file=name, **counts))
    mode = dict(all_text=True) if all_text else {}      # the default report keeps its keys (world manifests embed it)
    return dict(ok=not any(totals[k] for k in judged), extra_ok=not any(totals[k] for k in EXTRA), totals=totals,
                findings=rows, files_checked=checked, skipped_binary=len(skipped),
                skipped_binary_files=skipped if all_text else skipped[:50], **mode,
                yaml_origin_checked=tokens is not None, judged_rules=list(judged), extra_rules=list(EXTRA),
                strict=bool(strict),
                rules=dict(latlon_pair='>= 6 decimals, one in band %s and one in band %s within %d chars'
                           % (LAT_BAND, LON_BAND, PAIR_WINDOW),
                           latlon_pair_strict='>= %d decimals, every pair one in each band within %d chars (judged '
                                              'with strict)' % (STRICT_DECIMALS, STRICT_WINDOW),
                           yaml_origin='10/10 map YAML origin values (6 decimals) or (x, y) pairs (3-5 decimals, %d '
                                       'chars)' % ORIGIN_PAIR_WINDOW,
                           gnss_path='the GNSS data folders of the 10/10 data',
                           gnss_word='GNSS field / topic / element words (report)'))


def main(argv=None):
    parser = argparse.ArgumentParser(description='A16 coordinate-leak lint (design 7.2 A16).')
    parser.add_argument('paths', nargs='+', type=Path)
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--strict', action='store_true', help='judge the strict pair rule (run outputs)')
    parser.add_argument('--all-text', action='store_true',
                        help='read every file without a NUL byte in its first bytes, whatever its suffix')
    args = parser.parse_args(argv)
    report = lint_files(args.paths, strict=args.strict, all_text=args.all_text)
    if args.json:
        print(json.dumps(report, indent=1))
    else:
        print('coord_lint totals %s, findings %d, files %d, skipped binary %d, yaml origin checked %s -> A16 %s, '
              'extra %s' % (report['totals'], len(report['findings']), report['files_checked'],
                            report['skipped_binary'], report['yaml_origin_checked'],
                            'OK' if report['ok'] else 'FINDINGS', 'OK' if report['extra_ok'] else 'words found'))
        for row in report['findings']:
            print('  %s: %s' % (row['file'], {k: row[k] for k in report['totals'] if row[k]}))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
