#!/usr/bin/env python3
"""Check that the row executor's vendored SIM modules match the SIM tree and its integrity files (design 6.8, 8.5).

  python -I tools/steer_sim/check_vendor_sync.py [--json] [--no-git]

Fails (exit 1, with a list) when:
  - vendor/SOURCE_MANIFEST.json is unreadable, does not declare modified false, or lists no files;
  - a listed vendored file is missing, or its bytes / sha256 / git_blob differ from the manifest;
  - a listed vendored file differs from its SIM-tree source (source_path, e.g. sim/ubts/closed_loop/estimator.py);
  - vendor/ holds a file the manifest does not list;
  - the manifest names a commit (not WORKTREE-UNCOMMITTED) and git show <commit>:<source_path> differs, or git (present,
    inside this work tree) does not know that commit or that path (skipped only with --no-git or where git is not
    installed or this folder is not a git work tree; reported as a note);
  - tools/row_executor/SHA256SUMS lists missing, extra or changed files, i.e. executor_main.verify_sources() would fail
    (also run for real, with core.verify_vendor(), in a fresh interpreter).
Fix with tools/steer_sim/regen_vendor.py. Reads only; writes nothing.
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
SUMS = PACKAGE / 'SHA256SUMS'
WORKTREE = 'WORKTREE-UNCOMMITTED'


def blob_id(data):
    return hashlib.sha1(b'blob %d\0' % len(data) + data).hexdigest()


def package_files():
    """Same rule as executor_main.source_files: every file except SHA256SUMS, caches and dot names."""
    out = []
    for root, dirs, files in os.walk(PACKAGE):
        dirs[:] = sorted(d for d in dirs if d != '__pycache__' and not d.startswith('.'))
        for name in sorted(files):
            if name == SUMS.name or name.endswith(('.pyc', '.pyo')) or name.startswith('.'):
                continue
            out.append(os.path.relpath(os.path.join(root, name), PACKAGE).replace(os.sep, '/'))
    return sorted(out)


def check_sums():
    problems = []
    try:
        recorded = SUMS.read_bytes().decode('utf-8')
    except OSError as exc:
        return ['SHA256SUMS unreadable: %s' % exc]
    listed = {}
    for line in recorded.splitlines():
        if '  ' not in line:
            problems.append('SHA256SUMS: malformed line %r' % line[:80])
            continue
        digest, rel = line.split('  ', 1)
        listed[rel] = digest
    actual = package_files()
    for rel in sorted(set(listed) - set(actual)):
        problems.append('SHA256SUMS lists a missing file: %s' % rel)
    for rel in sorted(set(actual) - set(listed)):
        problems.append('SHA256SUMS does not list: %s' % rel)
    for rel in sorted(set(actual) & set(listed)):
        data = (PACKAGE / rel).read_bytes()
        if hashlib.sha256(data).hexdigest() != listed[rel]:
            problems.append('SHA256SUMS differs for: %s' % rel)
    expected = ''.join('%s  %s\n' % (hashlib.sha256((PACKAGE / rel).read_bytes()).hexdigest(), rel) for rel in actual)
    if not problems and recorded != expected:
        problems.append('SHA256SUMS text is not the canonical rendering (order or format)')
    return problems


def _git(args, run):
    """A read-only git command in ROOT, or None when git is not installed."""
    try:
        return run(['git', *args], cwd=str(ROOT), capture_output=True)
    except OSError:
        return None


def _text(result):
    return result.stderr.decode('utf-8', 'replace').strip()[:200] if result is not None else 'git not found'


def check_git(manifest, use_git, run=subprocess.run):
    """(problems, note). Only a missing git or a folder that is not a git work tree skips the comparison (a note);
    with git there, a commit or a path git does not know is a problem (review P4R-12). run: subprocess.run (tests
    give a stand-in)."""
    commit = manifest.get('source_commit')
    if commit == WORKTREE:
        return [], 'source_commit %s: the work tree is the source (no commit to compare)' % WORKTREE
    if not use_git:
        return [], 'git comparison skipped (--no-git)'
    inside = _git(['rev-parse', '--is-inside-work-tree'], run)
    if inside is None or inside.returncode or inside.stdout.strip() != b'true':
        return [], 'git comparison not possible: %s' % (_text(inside) if inside is None or inside.returncode
                                                        else 'not a git work tree')
    if not isinstance(commit, str) or not commit:
        return ['manifest source_commit is missing'], 'git: no commit to compare'
    found = _git(['cat-file', '-e', '%s^{commit}' % commit], run)
    if found is None or found.returncode:
        return ['manifest source_commit %s is not a commit of this repository: %s' % (commit, _text(found))], \
            'git does not know the manifest commit'
    problems = []
    for entry in manifest.get('files', []):
        result = _git(['show', '%s:%s' % (commit, entry['source_path'])], run)
        if result is None or result.returncode:
            problems.append('git show %s:%s failed: %s' % (commit, entry['source_path'], _text(result)))
            continue
        vendored = VENDOR / entry['vendored_path']
        if vendored.is_file() and result.stdout != vendored.read_bytes():
            problems.append('vendored %s differs from %s:%s' % (entry['vendored_path'], commit, entry['source_path']))
    return problems, 'compared with git show %s' % commit


def check_real_functions():
    code = ('import json, sys; sys.dont_write_bytecode = True; sys.path.insert(0, sys.argv[1]); '
            'from row_executor import core, executor_main; core.verify_vendor(); executor_main.verify_sources(); '
            'print("ok")')
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    result = subprocess.run([sys.executable, '-B', '-c', code, str(ROOT / 'tools')], capture_output=True, text=True,
                            env=env)
    if result.returncode:
        return ['core.verify_vendor / executor_main.verify_sources failed: %s' % result.stderr.strip()[-600:]]
    return []


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    p.add_argument('--json', action='store_true')
    p.add_argument('--no-git', action='store_true')
    a = p.parse_args(argv)
    problems, notes = [], []
    try:
        manifest = json.loads(MANIFEST.read_bytes().decode('utf-8'))
    except (OSError, ValueError) as exc:
        manifest = None
        problems.append('vendor manifest unreadable: %s' % exc)
    listed = set()
    if manifest is not None:
        if manifest.get('modified') is not False:
            problems.append('vendor manifest does not declare modified: false')
        if not manifest.get('files'):
            problems.append('vendor manifest lists no files')
        for entry in manifest.get('files', []):
            vendored, source = entry.get('vendored_path'), entry.get('source_path')
            listed.add(vendored)
            path = VENDOR / vendored
            if not path.is_file():
                problems.append('vendored file missing: %s' % vendored)
                continue
            data = path.read_bytes()
            if len(data) != entry.get('bytes') or hashlib.sha256(data).hexdigest() != entry.get('sha256'):
                problems.append('vendored file differs from the manifest sha256/bytes: %s' % vendored)
            if entry.get('git_blob') and blob_id(data) != entry['git_blob']:
                problems.append('vendored file differs from the manifest git_blob: %s' % vendored)
            sim = ROOT / source if source else None
            if sim is None or not sim.is_file():
                problems.append('SIM-tree source missing: %s (for %s)' % (source, vendored))
            elif sim.read_bytes() != data:
                problems.append('vendored %s differs from the SIM tree %s' % (vendored, source))
        extra = []
        for root, dirs, files in os.walk(VENDOR):
            dirs[:] = [d for d in dirs if d != '__pycache__' and not d.startswith('.')]
            for name in files:
                rel = os.path.relpath(os.path.join(root, name), VENDOR).replace(os.sep, '/')
                if rel != MANIFEST.name and rel not in listed and not name.endswith(('.pyc', '.pyo')):
                    extra.append(rel)
        problems += ['vendor/ holds a file the manifest does not list: %s' % rel for rel in sorted(extra)]
        git_problems, note = check_git(manifest, not a.no_git)
        problems += git_problems
        notes.append(note)
    problems += check_sums()
    problems += check_real_functions()
    report = dict(ok=not problems, problems=problems, notes=notes,
                  source_commit=None if manifest is None else manifest.get('source_commit'),
                  vendored_files=len(listed))
    if a.json:
        print(json.dumps(report, indent=1))
    else:
        print('vendor %s: %d files' % (report['source_commit'], report['vendored_files']))
        for note in notes:
            print('  note: %s' % note)
        for problem in problems:
            print('  PROBLEM: %s' % problem)
        print('OK' if report['ok'] else 'FAIL')
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
