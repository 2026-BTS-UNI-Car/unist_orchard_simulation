"""Provenance block of the P6a summaries and the P7 result files (audit gap G11; design 7.3 last line, R13).

provenance(code_files=(), profile=None) returns
  source_commit / source_status_sha256   UBTS_STEER_SOURCE_COMMIT / UBTS_STEER_SOURCE_STATUS_SHA256 (the container
                                         rule R13-5); without them the commit comes from a read-only `git rev-parse
                                         HEAD` of this checkout when git is there, the status digest stays None
  profile_id / profile_sha256            the vehicle profile given, else the default profile
  code_sha256                            SHA-256 of each code file given (path relative to sim/ubts when inside it)
  python, numpy, platform, result_kind, field_validated
No GNSS coordinates, map origins or record trajectories are written. Placeholder-based SIM results.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import platform
import subprocess
import sys

UBTS_DIR = Path(__file__).resolve().parents[1]
RESULT_KIND = '자리값 기반 SIM 결과'


def file_sha256(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def git_head():
    """The HEAD commit of this checkout (read-only git), or None."""
    try:
        out = subprocess.run(['git', '-C', str(UBTS_DIR), 'rev-parse', 'HEAD'], capture_output=True, text=True,
                             timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    head = out.stdout.strip()
    return head if out.returncode == 0 and len(head) == 40 else None


def relative(path):
    path = Path(path).resolve()
    try:
        return path.relative_to(UBTS_DIR).as_posix()
    except ValueError:
        return path.name


def provenance(code_files=(), profile=None):
    commit = os.environ.get('UBTS_STEER_SOURCE_COMMIT')
    commit_source = 'env UBTS_STEER_SOURCE_COMMIT'
    if not commit or commit == 'unknown':
        commit, commit_source = git_head(), 'git rev-parse HEAD (read-only)'
    out = dict(source_commit=commit, source_commit_from=commit_source,
               source_status_sha256=os.environ.get('UBTS_STEER_SOURCE_STATUS_SHA256'),
               code_sha256={relative(f): file_sha256(f) for f in code_files},
               python=platform.python_version(), platform=sys.platform, result_kind=RESULT_KIND,
               field_validated=False)
    try:
        import numpy
        out['numpy'] = numpy.__version__
    except ImportError:
        out['numpy'] = None
    try:
        if profile is None:
            from steering.vehicle_profile import load_profile
            profile = load_profile()
        out.update(profile_id=profile.profile_id, profile_sha256=profile.sha256)
    except Exception as exc:            # information only: a missing profile must not stop a result file
        out.update(profile_id=None, profile_sha256=None, profile_error='%s: %s' % (type(exc).__name__, exc))
    return out
