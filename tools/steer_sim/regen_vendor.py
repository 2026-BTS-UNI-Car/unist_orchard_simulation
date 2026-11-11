#!/usr/bin/env python3
"""Regenerate the row executor's vendored SIM modules (tools/row_executor/vendor/) byte-exact from the SIM tree.

The executor imports its estimator / controllers / single_row / scan_corridor only from vendor/ (core.load_vendor checks
every sha256 of vendor/SOURCE_MANIFEST.json first; sim_in_loop requires the SIM tree to be byte-identical). After a
change of a vendored SIM-tree file the copy, the manifest and tools/row_executor/SHA256SUMS must be rewritten together
(design 8.3 P4, 6.8):

  python -I tools/steer_sim/regen_vendor.py --worktree             # work-tree bytes, source_commit WORKTREE-UNCOMMITTED
  python -I tools/steer_sim/regen_vendor.py --commit <sha>         # git show <sha>:<source_path>, records git_blob
  options: --add <source_path> <vendored_path>  (repeatable; e.g. sim/ubts/closed_loop/steer_control.py
           closed_loop_copy/steer_control.py), --remove <vendored_path> (repeatable), --dry-run, --json
The vendored packages are closed_loop_copy and orchard_perception_copy (core.VENDOR_PACKAGES): subpackages of the
executor package's vendor folder, imported by core.load_vendor() without putting the folder on sys.path (their names
must not be a SIM-tree or standard-library module name; tools/steer_sim/check_pins.py checks new names).

Steps: copy every listed file (bytes as stored: no newline conversion), refuse files of vendor/ that the new manifest
does not list, rewrite vendor/SOURCE_MANIFEST.json (files sorted by vendored_path, keys sorted, UTF-8, LF), run
tools/row_executor/write_sha256sums.py, then check with the executor's own core.verify_vendor() and
executor_main.verify_sources() in a fresh interpreter. Exit 0 only when everything passed. Run
tools/steer_sim/check_vendor_sync.py afterwards (also before every push).
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'tools' / 'row_executor'
VENDOR = PACKAGE / 'vendor'
MANIFEST = VENDOR / 'SOURCE_MANIFEST.json'
WORKTREE = 'WORKTREE-UNCOMMITTED'
SCHEMA = 'ubts.row_executor.vendor_manifest.v1'
SOURCE_BRANCH = 'main'
SOURCE_REPOSITORY = '2026-BTS-UNI-Car/unist_orchard_simulation'
IMPORT_ROOT = ('tools/row_executor/vendor: the vendor namespace package of row_executor; closed_loop_copy and '
               'orchard_perception_copy are imported as row_executor.vendor.* by row_executor.core.load_vendor (the '
               'folder is never put on sys.path)')
VERIFICATION = ('row_executor.core.load_vendor() recomputes every sha256 before import and refuses on mismatch, imports '
                'the copies under row_executor.vendor (the SIM-tree name orchard_perception points at the vendored copy '
                'only while the vendored estimator is imported) and checks the imported files again; '
                'closed_loop_copy/single_row.py is imported next to the verified controllers module '
                '(core.single_row_module) and its sha256 is checked again at import')


def git(*args, binary=False):
    result = subprocess.run(['git', *args], cwd=str(ROOT), capture_output=True)
    if result.returncode:
        raise SystemExit('git %s failed: %s' % (' '.join(args), result.stderr.decode('utf-8', 'replace').strip()))
    return result.stdout if binary else result.stdout.decode('utf-8').strip()


def blob_id(data):
    """The git blob id of these bytes (what `git hash-object` prints for them without filters)."""
    return hashlib.sha1(b'blob %d\0' % len(data) + data).hexdigest()


def vendor_files_on_disk():
    out = set()
    for root, dirs, files in os.walk(VENDOR):
        dirs[:] = [d for d in dirs if d != '__pycache__' and not d.startswith('.')]
        for name in files:
            rel = os.path.relpath(os.path.join(root, name), VENDOR).replace(os.sep, '/')
            if rel != MANIFEST.name and not name.endswith(('.pyc', '.pyo')):
                out.add(rel)
    return out


def prune_empty_dirs(folder):
    """Remove a vendor sub-folder (and its parents up to vendor/) that holds nothing but bytecode caches."""
    folder = Path(folder)
    while folder != VENDOR and VENDOR in folder.parents:
        cache = folder / '__pycache__'
        if cache.is_dir() and all(p.suffix in ('.pyc', '.pyo') for p in cache.iterdir()):
            for p in cache.iterdir():
                p.unlink()
            cache.rmdir()
        if any(folder.iterdir()):
            return
        folder.rmdir()
        folder = folder.parent


def plan_entries(old, add, remove):
    entries = {e['vendored_path']: e['source_path'] for e in old.get('files', [])}
    for vendored in remove:
        if vendored not in entries:
            raise SystemExit('--remove %s: not in the manifest' % vendored)
        del entries[vendored]
    for source, vendored in add:
        if vendored in entries and entries[vendored] != source:
            raise SystemExit('--add %s: %s already comes from %s' % (vendored, vendored, entries[vendored]))
        entries[vendored] = source
    for vendored, source in entries.items():
        if Path(vendored).is_absolute() or '..' in Path(vendored).parts or '\\' in vendored:
            raise SystemExit('bad vendored path %r' % vendored)
        if not source.startswith('sim/ubts/') or '..' in Path(source).parts:
            raise SystemExit('source %r is not a SIM-tree path (sim/ubts/...)' % source)
    return dict(sorted(entries.items()))


def read_source(source, commit):
    if commit is None:
        path = ROOT / source
        if not path.is_file():
            raise SystemExit('work-tree source missing: %s' % source)
        return path.read_bytes(), None
    data = git('show', '%s:%s' % (commit, source), binary=True)
    blob = git('rev-parse', '%s:%s' % (commit, source))
    if blob_id(data) != blob:
        raise SystemExit('git show bytes of %s do not hash to its blob %s' % (source, blob))
    return data, blob


def verify_in_fresh_interpreter():
    code = ('import json, sys; sys.dont_write_bytecode = True; sys.path.insert(0, sys.argv[1]); '
            'from row_executor import core, executor_main; m = core.verify_vendor(); d = executor_main.verify_sources(); '
            'print(json.dumps(dict(source_commit=m["source_commit"], sums_digest=d)))')
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    result = subprocess.run([sys.executable, '-B', '-c', code, str(ROOT / 'tools')], capture_output=True, text=True,
                            env=env)
    if result.returncode:
        return dict(ok=False, error=result.stderr.strip()[-2000:])
    return dict(ok=True, **json.loads(result.stdout.strip().splitlines()[-1]))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--worktree', action='store_true', help='copy the work-tree bytes (source_commit %s)' % WORKTREE)
    mode.add_argument('--commit', help='copy git show <commit>:<source_path> (records git_blob)')
    p.add_argument('--add', nargs=2, action='append', default=[], metavar=('SOURCE_PATH', 'VENDORED_PATH'))
    p.add_argument('--remove', action='append', default=[], metavar='VENDORED_PATH')
    p.add_argument('--dry-run', action='store_true', help='report only; write nothing')
    p.add_argument('--json', action='store_true')
    a = p.parse_args(argv)
    old_bytes = MANIFEST.read_bytes()
    old = json.loads(old_bytes.decode('utf-8'))
    commit = None if a.worktree else git('rev-parse', '--verify', a.commit + '^{commit}')
    entries = plan_entries(old, a.add, a.remove)
    extra = sorted(vendor_files_on_disk() - set(entries) - set(a.remove))
    if extra:            # checked before anything is written
        raise SystemExit('vendor/ holds files the manifest does not list (delete them or --add them): %s' % extra)
    sources = {vendored: read_source(source, commit) for vendored, source in entries.items()}   # all read first
    files, writes = [], []
    for vendored, source in entries.items():
        data, blob = sources[vendored]
        target = VENDOR / vendored
        current = target.read_bytes() if target.is_file() else None
        files.append(dict(vendored_path=vendored, source_path=source, sha256=hashlib.sha256(data).hexdigest(),
                          bytes=len(data), git_blob=blob if blob is not None else blob_id(data)))
        if current != data:
            writes.append(vendored)
            if not a.dry_run:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(data)
    removed = []
    for vendored in a.remove:
        target = VENDOR / vendored
        if target.is_file():
            removed.append(vendored)
            if not a.dry_run:
                target.unlink()
                prune_empty_dirs(target.parent)
    manifest = dict(
        schema=SCHEMA, source_repository=SOURCE_REPOSITORY,
        source_branch=SOURCE_BRANCH, source_commit=commit or WORKTREE, modified=False,
        copy_method=('git show <commit>:<source_path> (byte-exact, as stored)' if commit else
                     'work-tree bytes of <source_path> (byte-exact; tools/steer_sim/regen_vendor.py --worktree; '
                     'git_blob = the blob id of these bytes, equal to the committed blob once committed unchanged)'),
        import_root=IMPORT_ROOT, verification=VERIFICATION,
        generated_by='tools/steer_sim/regen_vendor.py (%s)' % ('--commit' if commit else '--worktree'),
        files=files,
        # previous_* name the manifest this one replaces, unless that one was itself a work-tree regeneration: then
        # its derivation is kept, so it always names the last vendor that came from a commit
        derivation=(old.get('derivation') if old.get('source_commit') == WORKTREE and old.get('derivation') else
                    dict(previous_source_commit=old.get('source_commit'),
                         previous_manifest_SHA256=hashlib.sha256(old_bytes).hexdigest(),
                         previous_source_branch=old.get('source_branch'),
                         previous_source_repository=old.get('source_repository'),
                         note='rewritten by tools/steer_sim/regen_vendor.py; check with tools/steer_sim/'
                              'check_vendor_sync.py')))
    text = json.dumps(manifest, indent=1, sort_keys=True, ensure_ascii=False) + '\n'
    data = text.encode('utf-8')
    report = dict(mode='commit' if commit else 'worktree', source_commit=manifest['source_commit'],
                  files=len(files), written=writes, removed=removed, dry_run=a.dry_run,
                  manifest_changed=data != old_bytes)
    if not a.dry_run:
        if data != old_bytes:
            MANIFEST.write_bytes(data)
        sums = subprocess.run([sys.executable, '-B', str(PACKAGE / 'write_sha256sums.py')], cwd=str(ROOT),
                              capture_output=True, text=True,
                              env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
        report['write_sha256sums'] = dict(returncode=sums.returncode, out=(sums.stdout + sums.stderr).strip()[-500:])
        report['verify'] = verify_in_fresh_interpreter() if sums.returncode == 0 else dict(ok=False,
                                                                                            error='SHA256SUMS not written')
    ok = a.dry_run or (report['write_sha256sums']['returncode'] == 0 and report['verify']['ok'])
    report['ok'] = bool(ok)
    if a.json:
        print(json.dumps(report, indent=1))
    else:
        print('vendor %s: %d files, written %s, removed %s, manifest %s' % (
            report['source_commit'], len(files), writes or 'none', removed or 'none',
            'rewritten' if report['manifest_changed'] else 'unchanged'))
        if not a.dry_run:
            print('SHA256SUMS: %s' % report['write_sha256sums']['out'])
            print('verify_vendor + verify_sources: %s' % ('OK' if report['verify']['ok'] else report['verify']))
        print('OK' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
