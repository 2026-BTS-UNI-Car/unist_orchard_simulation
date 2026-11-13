#!/usr/bin/env python3
"""Copy the dashboard of the dashboard repository into dashboard/ at one fixed commit, and check that copy (D-13).

The web dashboard (server and page) of the simulation is the dashboard repository's code, not a fork of it. This tool
takes three folders of one commit of that repository, byte for byte as git stores them, and writes them under
dashboard/ of this repository together with a source manifest (dashboard/DASHBOARD_SOURCE.json: the repository, the
commit, every file's sha256, size and git blob id). Running it again with the same commit gives the same files and the
same manifest (no time stamps). The simulation's own additions are not in these folders: they are plugged into the
dashboard's extension points from dashboard_sim/ (docs/steering/SIM_DASHBOARD_KO.md).

  python -I tools/steer_sim/sync_dashboard.py --source <dashboard repository> --commit <rev>       copy, write manifest
  python -I tools/steer_sim/sync_dashboard.py --source <dashboard repository> --commit <rev> --dry-run
  python -I tools/steer_sim/sync_dashboard.py --check [--source <dashboard repository>]            check only

Copied folders (SYNCED_ROOTS): dashboard_server/, static/, config/ of the dashboard repository, all files except the
prefixes in EXCLUDED (recorded with their reason in the manifest). Every file
under dashboard/<root>/ that the commit does not have is removed (the earlier simulation copy of those folders).
Everything else under dashboard/ stays as it is: scripts/ (the sensor bridges, the recorder and the steering host code
that the simulation runs in its own containers; their origin and status are in the manifest note and the document),
.gitattributes and UPSTREAM.md.

--check (no git needed for the first part): every file of the manifest is on disk with its sha256 and size, and no
other file is under the copied folders (exit 1 otherwise). With --source, also: the manifest's commit exists there and
its files are exactly the manifest's (blob ids). The source repository is read only (git ls-tree, git cat-file).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / 'dashboard'
MANIFEST_NAME = 'DASHBOARD_SOURCE.json'
SCHEMA = 'ubts.sim.dashboard_source.v1'
REPOSITORY = '2026-BTS-UNI-Car/unist_orachard_dashboard'
SYNCED_ROOTS = ('config', 'dashboard_server', 'static')
SKIP_NAMES = ('__pycache__', '.pytest_cache')          # local caches under the copied folders, never part of the copy
# Parts of the copied folders that the simulation does not take (prefix -> reason). They are left out of the copy
# and of the manifest; files under them on disk count as extra and are removed by a sync.
EXCLUDED = {
    'dashboard_server/_field10/': 'the hash-pinned field SHADOW bundle of the differential vehicle (dashboard D-14); the '
                                  'steering stack never starts the differential observation that imports it, and the '
                                  'bundle holds recorded-data paths and names that this repository does not carry',
}
NOTE = ('Copy of the dashboard repository (server and page). The files under config/, dashboard_server/ and static/ '
        'are the blobs of the commit above, unchanged; tools/steer_sim/sync_dashboard.py writes and checks them. '
        'dashboard/scripts/ is not part of this copy: the sensor bridges, the recorder and the steering host code that '
        'the simulation runs (docs/steering/SIM_DASHBOARD_KO.md section 3). The simulation additions of the dashboard '
        'are in dashboard_sim/ and are plugged into the extension points DASHBOARD_PROVIDER, DASHBOARD_EXTENSION and '
        'DASHBOARD_EXTRA_STATIC.')
_HEX40 = re.compile(r'[0-9a-f]{40}')


class SyncError(RuntimeError):
    pass


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def git(source: Path, *args: str, data: bytes | None = None) -> bytes:
    result = subprocess.run(['git', '-c', 'core.autocrlf=false', '-C', str(source), *args], input=data,
                            capture_output=True)
    if result.returncode != 0:
        raise SyncError('git %s: %s' % (' '.join(args), result.stderr.decode('utf-8', 'replace').strip()))
    return result.stdout


def resolve_commit(source: Path, rev: str) -> str:
    commit = git(source, 'rev-parse', '--verify', '--quiet', rev + '^{commit}').decode('ascii').strip()
    if not _HEX40.fullmatch(commit):
        raise SyncError('not a commit: %r' % rev)
    return commit


def source_tree(source: Path, commit: str) -> dict[str, dict]:
    """{path: {mode, blob}} of the copied folders at the commit (regular files only)."""
    raw = git(source, 'ls-tree', '-r', '-z', '--full-tree', commit, '--', *SYNCED_ROOTS)
    files = {}
    for entry in raw.split(b'\0'):
        if not entry:
            continue
        meta, _, path = entry.partition(b'\t')
        mode, kind, blob = meta.decode('ascii').split(' ')
        path = path.decode('utf-8')
        if kind != 'blob':
            raise SyncError('%s: a %s in the copied folders (submodules are not copied)' % (path, kind))
        if mode not in ('100644', '100755'):
            raise SyncError('%s: mode %s (symbolic links are not copied)' % (path, mode))
        if any(path.startswith(prefix) for prefix in EXCLUDED):
            continue
        files[path] = dict(mode=mode, blob=blob)
    if not files:
        raise SyncError('commit %s has none of the folders %s' % (commit, ', '.join(SYNCED_ROOTS)))
    return files


def read_blobs(source: Path, blobs: list[str]) -> dict[str, bytes]:
    """The bodies of the blobs (git cat-file --batch: the stored bytes, no line-end conversion)."""
    out = git(source, 'cat-file', '--batch', data=''.join(b + '\n' for b in blobs).encode('ascii'))
    bodies, at = {}, 0
    for blob in blobs:
        end = out.index(b'\n', at)
        header = out[at:end].decode('ascii').split(' ')
        if len(header) != 3 or header[0] != blob or header[1] != 'blob':
            raise SyncError('cat-file: unexpected header %r for %s' % (header, blob))
        size = int(header[2])
        bodies[blob] = out[end + 1:end + 1 + size]
        at = end + 1 + size + 1
    return bodies


def blob_id(data: bytes) -> str:
    return hashlib.sha1(b'blob %d\0' % len(data) + data).hexdigest()


def build_manifest(commit: str, files: dict[str, dict], bodies: dict[str, bytes]) -> dict:
    rows = {}
    for path in sorted(files):
        data = bodies[files[path]['blob']]
        if blob_id(data) != files[path]['blob']:
            raise SyncError('%s: the body does not hash to its blob id' % path)
        rows[path] = dict(sha256=sha256(data), bytes=len(data), blob=files[path]['blob'], mode=files[path]['mode'])
    combined = sha256(''.join('%s %s\n' % (row['sha256'], path) for path, row in rows.items()).encode('utf-8'))
    return dict(schema=SCHEMA, repository=REPOSITORY, commit=commit, roots=list(SYNCED_ROOTS),
                file_count=len(rows), bytes=sum(row['bytes'] for row in rows.values()), files_sha256=combined,
                note=NOTE, excluded=dict(sorted(EXCLUDED.items())), files=rows)


def manifest_bytes(manifest: dict) -> bytes:
    return (json.dumps(manifest, ensure_ascii=False, indent=1, sort_keys=True) + '\n').encode('utf-8')


def disk_files(target: Path) -> list[str]:
    """Files under the copied folders of target (posix paths relative to target), caches left out."""
    found = []
    for root in SYNCED_ROOTS:
        base = target / root
        if not base.exists():
            continue
        for folder, dirs, names in os.walk(base):
            dirs[:] = sorted(d for d in dirs if d not in SKIP_NAMES)
            for name in names:
                if name.endswith('.pyc'):
                    continue
                found.append(Path(folder, name).relative_to(target).as_posix())
    return sorted(found)


def plan(target: Path, manifest: dict) -> dict:
    """What a sync would change: {'add': [...], 'change': [...], 'remove': [...], 'same': n}."""
    wanted = manifest['files']
    present = set(disk_files(target))
    add, change, same = [], [], 0
    for path, row in wanted.items():
        if path not in present:
            add.append(path)
            continue
        data = (target / path).read_bytes()
        if sha256(data) != row['sha256'] or len(data) != row['bytes']:
            change.append(path)
        else:
            same += 1
    remove = sorted(present - set(wanted))
    return dict(add=add, change=change, remove=remove, same=same)


def sync(source: Path, rev: str, target: Path = TARGET, dry_run: bool = False) -> dict:
    commit = resolve_commit(source, rev)
    files = source_tree(source, commit)
    bodies = read_blobs(source, sorted({row['blob'] for row in files.values()}))
    manifest = build_manifest(commit, files, bodies)
    changes = plan(target, manifest)
    manifest_path = target / MANIFEST_NAME
    old = manifest_path.read_bytes() if manifest_path.is_file() else None
    new = manifest_bytes(manifest)
    report = dict(commit=commit, file_count=manifest['file_count'], bytes=manifest['bytes'],
                  files_sha256=manifest['files_sha256'], add=len(changes['add']), change=len(changes['change']),
                  remove=len(changes['remove']), same=changes['same'], manifest_changed=old != new,
                  dry_run=dry_run, removed=changes['remove'])
    if dry_run:
        return report
    for path in changes['remove']:
        (target / path).unlink()
    for path in changes['add'] + changes['change']:
        destination = target / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(bodies[files[path]['blob']])
    for root in SYNCED_ROOTS:                       # folders the removals left empty
        base = target / root
        for folder, dirs, names in sorted(os.walk(base, topdown=False), key=lambda item: -len(item[0])):
            if Path(folder) != base and not any(Path(folder).iterdir()):
                Path(folder).rmdir()
    manifest_path.write_bytes(new)
    after = check(target)
    if not after['ok']:
        raise SyncError('the tree does not match the new manifest: %s' % after)
    return report


def check(target: Path = TARGET, source: Path | None = None) -> dict:
    """The copy against its manifest (and, with source, the manifest against the source commit)."""
    manifest_path = target / MANIFEST_NAME
    try:
        manifest = json.loads(manifest_path.read_bytes().decode('utf-8'))
    except (OSError, ValueError) as exc:
        return dict(ok=False, error='manifest: %s' % exc)
    problems = []
    if manifest.get('schema') != SCHEMA:
        problems.append('schema %r' % manifest.get('schema'))
    if list(manifest.get('roots') or []) != list(SYNCED_ROOTS):
        problems.append('roots %r (this tool copies %r)' % (manifest.get('roots'), list(SYNCED_ROOTS)))
    rows = manifest.get('files') or {}
    combined = sha256(''.join('%s %s\n' % (row['sha256'], path) for path, row in sorted(rows.items())).encode('utf-8'))
    if combined != manifest.get('files_sha256') or len(rows) != manifest.get('file_count'):
        problems.append('manifest totals do not match its file list')
    changes = plan(target, manifest)
    if manifest_path.read_bytes() != manifest_bytes(manifest):
        problems.append('manifest text is not in the canonical form (written by hand?)')
    source_report = None
    if source is not None:
        try:
            files = source_tree(source, manifest['commit'])
            mismatched = sorted(p for p in set(files) | set(rows)
                                if p not in files or p not in rows or files[p]['blob'] != rows[p]['blob']
                                or files[p]['mode'] != rows[p]['mode'])
            source_report = dict(commit=manifest['commit'], mismatched=mismatched)
            if mismatched:
                problems.append('%d files differ from commit %s' % (len(mismatched), manifest['commit'][:12]))
        except SyncError as exc:
            source_report = dict(error=str(exc))
            problems.append(str(exc))
    ok = not problems and not (changes['add'] or changes['change'] or changes['remove'])
    return dict(ok=ok, commit=manifest.get('commit'), file_count=len(rows), same=changes['same'],
                missing=changes['add'], changed=changes['change'], extra=changes['remove'], problems=problems,
                source=source_report)


def main(argv=None) -> int:
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--source', type=Path, help='a clone of the dashboard repository (read only)')
    parser.add_argument('--commit', help='the commit (or a name of it) to copy')
    parser.add_argument('--check', action='store_true', help='check the copy against its manifest only')
    parser.add_argument('--dry-run', action='store_true', help='show what a sync would change, write nothing')
    parser.add_argument('--target', type=Path, default=TARGET, help=argparse.SUPPRESS)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.check:
            report = check(args.target, args.source)
        else:
            if args.source is None or not args.commit:
                parser.error('--source and --commit are needed to copy (or --check)')
            report = sync(args.source, args.commit, args.target, dry_run=args.dry_run)
            report['ok'] = True
    except SyncError as exc:
        report = dict(ok=False, error=str(exc))
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=1))
    elif args.check:
        print('dashboard copy %s: commit %s, %s files, same %s, missing %d, changed %d, extra %d%s'
              % ('OK' if report.get('ok') else 'FAIL', (report.get('commit') or '?')[:12], report.get('file_count'),
                 report.get('same'), len(report.get('missing', [])), len(report.get('changed', [])),
                 len(report.get('extra', [])), '' if report.get('ok') else ': %s' % (report.get('problems')
                                                                                    or report.get('error'))))
        for key in ('missing', 'changed', 'extra'):
            for path in report.get(key, [])[:40]:
                print('  %s %s' % (key, path))
    elif report.get('ok'):
        print('%s commit %s: %d files (%d bytes), add %d, change %d, remove %d, same %d, manifest %s'
              % ('would copy' if report['dry_run'] else 'copied', report['commit'][:12], report['file_count'],
                 report['bytes'], report['add'], report['change'], report['remove'], report['same'],
                 'changed' if report['manifest_changed'] else 'unchanged'))
    else:
        print('FAIL: %s' % report.get('error'))
    return 0 if report.get('ok') else 1


if __name__ == '__main__':
    sys.exit(main())
