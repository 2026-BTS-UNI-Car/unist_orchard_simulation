#!/usr/bin/env python3
"""Build the native steering lockstep program (C++11; decision R1).

Compiler   argument `compiler`, else env UBTS_STEER_CXX, else `g++` on PATH.
Build dir  argument `build_dir`, else env UBTS_STEER_BUILD_DIR, else `_build/` next to this
           file (git-ignored by the .gitignore in this folder). Binaries are never committed.
Flags      MinGW 32-bit: -std=gnu++11 -O2 -msse2 -mfpmath=sse -static (+ -Wall -Wextra);
           other MinGW: -std=gnu++11 -O2 -static; Linux x86_64: -std=c++11 -O2 (i?86 adds SSE2).
           Windows targets also link with -Wl,--no-insert-timestamp, so the same sources and flags give
           the same binary bytes (the PE header time stamp would differ otherwise).
           Extra flags from env UBTS_STEER_CXXFLAGS (shell-split) are appended.
Sources    the native model and lockstep driver (this folder), the Arduino-style shim (arduino_shim/) and the
           unchanged firmware sources of firmware/ubts_steer_v3 (decision R1), compiled with UBTS_STEER_SIM=1
           and UBTS_STEER_FW_SRC = the firmware build id (ESP protocol 10.5, firmware_build_id()).
Firmware   folder from env UBTS_STEER_FIRMWARE_DIR (must hold ubts_steer_v3.ino), else firmware/ubts_steer_v3 of
           this checkout (repository-relative). When that default folder is missing (for example in an image that
           copies only sim/ubts) the program is built without firmware mode: fw_session_absent.cc replaces
           fw_session.cc, fw_sketch.cc and arduino_shim/sim_mcu.cc, UBTS_STEER_FW_SRC is `none`, the v1 modes and
           the v2 direct_pwm / test_servo modes are unchanged and a firmware-mode startup line exits 2. The result
           and the manifest then carry firmware_build_id None. The firmware unit test needs the sources.
           Env UBTS_STEER_REQUIRE_FIRMWARE=1 (integration runs, a checkout that tracks the firmware): the tests that
           need firmware mode fail instead of skipping when the sources are missing (require_firmware_reason()).
Output     ubts_steer_native-<target>-<key>[.exe] with key = SHA-256 of the sources, the
           compiler version line and target, and the flags; a <name>.json manifest beside it.
           ensure_unit_test_built() builds the native firmware unit test (fw_unit_test.cc) the same way.
Safety     one builder at a time per output (lock file created with O_EXCL; stale after LOCK_STALE_S, longer
           than the compile and smoke-run limits; a PermissionError from a lock file that another process is
           deleting on Windows is retried), compile into a temporary name, smoke-run it (EOF before the
           startup line -> exit 0), then publish with an atomic rename. Other callers reuse the finished binary.
Caching    within one process the compiler probe (per compiler file) and the binary SHA-256 (per file size and
           modification time) are cached, so reusing a built binary costs a few stat calls and the source
           hash (the sources are read every time so that edits are seen). clear_caches() drops the caches.

Usage: python build.py [--compiler PATH] [--build-dir DIR] [--force] [--json] [--unit-test] [--firmware-id]
Standard library only, so it also runs as a plain script.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time
import uuid

NATIVE_DIR = Path(__file__).resolve().parent
SOURCES = ('steer_json.cc', 'steer_config.cc', 'steer_geometry.cc', 'actuator.cc', 'potentiometer.cc',
           'steer_model.cc', 'fw_session.cc', 'fw_sketch.cc', 'arduino_shim/sim_mcu.cc', 'abs_hold8.cc',
           'smooth_r5.cc', 'lockstep_main.cc')
HEADERS = ('steer_json.h', 'steer_config.h', 'steer_geometry.h', 'steer_rng.h', 'steer_wire.h', 'actuator.h',
           'potentiometer.h', 'steer_model.h', 'fw_session.h', 'arduino_shim/Arduino.h', 'arduino_shim/EEPROM.h',
           'arduino_shim/ESP8266WiFi.h', 'arduino_shim/user_interface.h', 'arduino_shim/sim_mcu.h', 'abs_hold8.h',
           'smooth_r5.h')
FIRMWARE_ONLY_SOURCES = ('fw_session.cc', 'fw_sketch.cc', 'arduino_shim/sim_mcu.cc', 'abs_hold8.cc',
                         'smooth_r5.cc')   # need the firmware
ABSENT_SOURCE = 'fw_session_absent.cc'   # replaces FIRMWARE_ONLY_SOURCES when the firmware sources are missing
SHIM_DIR = NATIVE_DIR / 'arduino_shim'
# firmware/ubts_steer_v3 of this work tree (repository-relative: sim/ubts/steering/native -> the tree root)
FIRMWARE_DIR = NATIVE_DIR.parents[3] / 'firmware' / 'ubts_steer_v3'
ENV_FIRMWARE_DIR = 'UBTS_STEER_FIRMWARE_DIR'
ENV_REQUIRE_FIRMWARE = 'UBTS_STEER_REQUIRE_FIRMWARE'
FIRMWARE_REL = 'firmware/ubts_steer_v3'
FIRMWARE_SUFFIXES = ('.ino', '.h', '.cpp', '.c')     # files of the firmware build id (ESP protocol 10.5)
FIRMWARE_SKETCH = 'ubts_steer_v3.ino'                 # compiled through fw_sketch.cc
UNIT_TEST_SOURCES = ('fw_unit_test.cc', 'fw_sketch.cc', 'arduino_shim/sim_mcu.cc')
PROGRAM = 'ubts_steer_native'
UNIT_TEST_PROGRAM = 'ubts_steer_fw_unit'
RECIPE_VERSION = 6   # 2: Windows targets link with -Wl,--no-insert-timestamp; 3: firmware sources and shim;
                     # 4: without firmware mode when the firmware sources are missing (fw_session_absent.cc);
                     # 5: abs_hold8.cc (optional SIM steering law of the startup block steer_law)
                     # 6: smooth_r5.cc (the SMOOTH r5 law of the same block, kind smooth_r5)
ENV_CXX = 'UBTS_STEER_CXX'
ENV_BUILD_DIR = 'UBTS_STEER_BUILD_DIR'
ENV_EXTRA_FLAGS = 'UBTS_STEER_CXXFLAGS'
DEFAULT_BUILD_DIR = NATIVE_DIR / '_build'
COMPILE_TIMEOUT_S = 900.
SMOKE_TIMEOUT_S = 60.
LOCK_STALE_S = COMPILE_TIMEOUT_S + SMOKE_TIMEOUT_S + 60.   # older than any live build: the builder is gone
LOCK_TIMEOUT_S = LOCK_STALE_S + 300.                      # a waiter outlasts a stale lock and breaks it
LOCK_PERMISSION_RETRY_S = 10.   # a lock file being deleted on Windows refuses O_EXCL with PermissionError briefly


class NativeBuildError(RuntimeError):
    """The native program could not be built (no compiler, compile error, lock timeout, smoke run failed)."""


@dataclass(frozen=True)
class Toolchain:
    compiler: str          # absolute path of the C++ driver
    version: str           # first line of `--version`
    target: str            # `-dumpmachine`
    flags: tuple           # compile flags without the per-build defines


@dataclass(frozen=True)
class BuildResult:
    path: str              # the binary
    built: bool            # True if this call compiled it, False if it was reused
    toolchain: Toolchain
    source_sha256: str
    build_key: str
    binary_sha256: str
    seconds: float         # compile time of this call (0 when reused)
    compiler_output: str   # compiler stderr of this call ('' when reused)
    firmware_build_id: str = None   # ESP protocol 10.5 id compiled in; None: built without the firmware sources


_PROBE_CACHE = {}     # (compiler path, size, mtime_ns) -> (version line, target)
_FILE_SHA_CACHE = {}  # (path, size, mtime_ns) -> sha256


def clear_caches():
    """Forget the cached compiler probes and file hashes of this process."""
    _PROBE_CACHE.clear()
    _FILE_SHA_CACHE.clear()


def _stat_key(path):
    st = os.stat(str(path))
    return (os.path.abspath(str(path)), st.st_size, st.st_mtime_ns)


def _run(cmd, timeout):
    try:
        return subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=timeout, check=False)
    except FileNotFoundError as exc:
        raise NativeBuildError('cannot run %r: %s' % (cmd[0], exc)) from None
    except subprocess.TimeoutExpired:
        raise NativeBuildError('timed out after %.0f s: %s' % (timeout, ' '.join(cmd[:3]))) from None


def find_compiler(compiler=None):
    """Absolute path of the C++ compiler (argument, env UBTS_STEER_CXX, or g++ on PATH)."""
    name = compiler or os.environ.get(ENV_CXX) or 'g++'
    path = shutil.which(name)
    if path is None:
        raise NativeBuildError('C++ compiler not found: %r (set %s or put g++ on PATH)' % (name, ENV_CXX))
    return os.path.abspath(path)


def _windows_target(target):
    t = target.lower()
    return 'mingw' in t or 'cygwin' in t or 'windows' in t or 'msys' in t


def default_flags(target):
    t = target.lower()
    common = ['-O2', '-Wall', '-Wextra']
    if _windows_target(t):
        flags = ['-std=gnu++11'] + common + ['-static', '-Wl,--no-insert-timestamp']
        if not t.startswith(('x86_64', 'amd64', 'aarch64', 'arm64')):
            flags += ['-msse2', '-mfpmath=sse']
        return flags
    flags = ['-std=c++11'] + common
    if re.match(r'i[3-6]86', t):
        flags += ['-msse2', '-mfpmath=sse']
    return flags


def toolchain(compiler=None, extra_flags=None):
    """Probe the compiler (version line and target triple) and choose the flags."""
    path = find_compiler(compiler)
    key = _stat_key(path)
    probe = _PROBE_CACHE.get(key)
    if probe is None:
        version = _run([path, '--version'], 120)
        machine = _run([path, '-dumpmachine'], 120)
        if version.returncode != 0 or machine.returncode != 0:
            raise NativeBuildError('compiler probe failed for %s: %s' % (
                path, (version.stderr + machine.stderr).decode('utf-8', 'replace').strip()))
        first = version.stdout.decode('utf-8', 'replace').strip().splitlines()
        probe = (first[0] if first else '', machine.stdout.decode('utf-8', 'replace').strip())
        _PROBE_CACHE[key] = probe
    version_line, target = probe
    extra = list(extra_flags) if extra_flags is not None else shlex.split(os.environ.get(ENV_EXTRA_FLAGS, ''))
    return Toolchain(compiler=path, version=version_line, target=target, flags=tuple(default_flags(target) + extra))


def firmware_dir():
    """The firmware folder: env UBTS_STEER_FIRMWARE_DIR (must hold the sketch), else FIRMWARE_DIR of this checkout."""
    chosen = os.environ.get(ENV_FIRMWARE_DIR)
    if chosen:
        path = Path(chosen).resolve()
        if not (path / FIRMWARE_SKETCH).is_file():
            raise NativeBuildError('%s=%s has no %s' % (ENV_FIRMWARE_DIR, chosen, FIRMWARE_SKETCH))
        return path
    return FIRMWARE_DIR


def firmware_present():
    """True if the firmware sources are there (the program is then built with firmware mode)."""
    return (firmware_dir() / FIRMWARE_SKETCH).is_file()


def firmware_required():
    """True when env UBTS_STEER_REQUIRE_FIRMWARE is set (not '' or '0'): missing firmware sources are an error."""
    return os.environ.get(ENV_REQUIRE_FIRMWARE, '') not in ('', '0')


def missing_firmware_message():
    return 'firmware sources missing: %s (copy %s or set %s)' % (firmware_dir() / FIRMWARE_SKETCH, FIRMWARE_REL,
                                                                 ENV_FIRMWARE_DIR)


def firmware_files():
    """Firmware source files (relative '/' paths, sorted by their bytes): .ino, .h, .cpp, .c of firmware_dir()."""
    root = firmware_dir()
    if not root.is_dir():
        raise NativeBuildError('firmware folder missing: %s (set %s)' % (root, ENV_FIRMWARE_DIR))
    names = []
    for path in root.rglob('*'):
        if path.is_file() and path.suffix in FIRMWARE_SUFFIXES:
            names.append(path.relative_to(root).as_posix())
    names.sort(key=lambda name: name.encode('utf-8'))
    if FIRMWARE_SKETCH not in names:
        raise NativeBuildError('firmware sketch missing: %s' % (root / FIRMWARE_SKETCH))
    return names


def firmware_build_id():
    """ESP protocol 10.5: first 8 uppercase hex of SHA-256 over the firmware files sorted by relative path, each
    contributing path bytes + 0x00 + u64 LE length + file bytes. Same value for the SIM and the real-chip build."""
    digest = hashlib.sha256()
    root = firmware_dir()
    for name in firmware_files():
        data = (root / name).read_bytes()
        digest.update(name.encode('utf-8') + b'\0' + len(data).to_bytes(8, 'little'))
        digest.update(data)
    return digest.hexdigest()[:8].upper()


def firmware_compile_sources():
    """Firmware translation units compiled directly (.cpp, .c); the sketch goes through fw_sketch.cc."""
    return [name for name in firmware_files() if name.endswith(('.cpp', '.c'))]


def source_sha256():
    """SHA-256 over the names and bytes of all native sources and headers and the firmware sources (a marker
    instead of them when they are missing)."""
    digest = hashlib.sha256()
    entries = [(name, NATIVE_DIR / name) for name in SOURCES + HEADERS + UNIT_TEST_SOURCES[:1] + (ABSENT_SOURCE,)]
    present = firmware_present()
    if present:
        root = firmware_dir()
        entries += [('%s/%s' % (FIRMWARE_REL, name), root / name) for name in firmware_files()]
    for name, path in sorted(set(entries)):
        if not path.is_file():
            raise NativeBuildError('native source missing: %s' % path)
        data = path.read_bytes()
        digest.update(('%s\0%d\0' % (name, len(data))).encode('ascii'))
        digest.update(data)
    if not present:
        digest.update(b'firmware sources absent\0')
    return digest.hexdigest()


def build_key(chain, sources):
    payload = json.dumps(dict(recipe=RECIPE_VERSION, version=chain.version, target=chain.target,
                              flags=list(chain.flags), sources=sources), sort_keys=True)
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()[:16]


def binary_name(chain, key, program=PROGRAM):
    target = re.sub(r'[^A-Za-z0-9_.-]+', '_', chain.target) or 'unknown'
    return '%s-%s-%s%s' % (program, target, key, '.exe' if _windows_target(chain.target) else '')


def resolve_build_dir(build_dir=None):
    chosen = build_dir or os.environ.get(ENV_BUILD_DIR) or DEFAULT_BUILD_DIR
    return Path(chosen).resolve()


def _file_sha256_uncached(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def file_sha256(path):
    """SHA-256 of a file, cached in this process per (path, size, modification time)."""
    key = _stat_key(path)
    value = _FILE_SHA_CACHE.get(key)
    if value is None:
        value = _file_sha256_uncached(path)
        _FILE_SHA_CACHE[key] = value
    return value


def _create_exclusive(lock):
    return os.open(str(lock), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)


def _acquire_lock(lock):
    deadline = time.monotonic() + LOCK_TIMEOUT_S
    denied_since = None
    while True:
        try:
            fd = _create_exclusive(lock)
        except PermissionError as exc:
            # Windows: another process is deleting the lock file (delete pending). Retry briefly; a lasting
            # PermissionError is a real access problem and becomes a NativeBuildError.
            now = time.monotonic()
            denied_since = now if denied_since is None else denied_since
            if now - denied_since > LOCK_PERMISSION_RETRY_S or now > deadline:
                raise NativeBuildError('cannot create the build lock %s: %s' % (lock, exc)) from None
            time.sleep(0.02)
            continue
        except FileExistsError:
            denied_since = None
            try:
                if time.time() - lock.stat().st_mtime > LOCK_STALE_S:
                    lock.unlink()
                    continue
            except FileNotFoundError:
                continue   # released meanwhile: try again at once
            except PermissionError:
                pass       # being deleted (Windows): try again after the short sleep below
            if time.monotonic() > deadline:
                raise NativeBuildError('timed out waiting for the build lock %s' % lock) from None
            time.sleep(0.05)
            continue
        try:
            os.write(fd, ('pid %d time %.3f\n' % (os.getpid(), time.time())).encode('ascii'))
        finally:
            os.close(fd)
        return


def _release_lock(lock):
    for _ in range(50):
        try:
            lock.unlink()
            return
        except FileNotFoundError:
            return
        except PermissionError:
            time.sleep(0.02)


def _smoke_run(path):
    """The program must exit 0 at once when stdin ends before the startup line."""
    try:
        proc = subprocess.run([str(path)], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=SMOKE_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise NativeBuildError('smoke run of %s failed: %s' % (path, exc)) from None
    if proc.returncode != 0 or proc.stdout:
        raise NativeBuildError('smoke run of %s: exit %s, stdout %r, stderr %r' % (
            path, proc.returncode, proc.stdout[:80], proc.stderr.decode('utf-8', 'replace').strip()))


def _write_manifest(path, result):
    data = asdict(result)
    data['toolchain']['flags'] = list(result.toolchain.flags)
    data['schema'] = 'ubts.steer_native_build.v1'
    data['sources'] = list(SOURCES)
    data['headers'] = list(HEADERS)
    data['firmware_files'] = (['%s/%s' % (FIRMWARE_REL, name) for name in firmware_files()]
                              if result.firmware_build_id is not None else [])
    manifest = Path(path).with_suffix('.json') if path.endswith('.exe') else Path(path + '.json')
    tmp = manifest.with_name('.%s.%d.%s.tmp' % (manifest.name, os.getpid(), uuid.uuid4().hex[:8]))
    tmp.write_bytes((json.dumps(data, indent=2, sort_keys=True) + '\n').encode('utf-8'))
    os.replace(str(tmp), str(manifest))


def ensure_built(compiler=None, build_dir=None, extra_flags=None, force=False):
    """Path and provenance of a ready binary; compiles it when missing (or when force=True)."""
    return _ensure_program(PROGRAM, SOURCES, compiler, build_dir, extra_flags, force)


def ensure_unit_test_built(compiler=None, build_dir=None, extra_flags=None, force=False):
    """The native firmware unit test (fw_unit_test.cc + firmware + shim); run it with no arguments, exit 0 = pass."""
    return _ensure_program(UNIT_TEST_PROGRAM, UNIT_TEST_SOURCES, compiler, build_dir, extra_flags, force)


def _ensure_program(program, native_sources, compiler, build_dir, extra_flags, force):
    chain = toolchain(compiler, extra_flags)
    present = firmware_present()
    if not present and program != PROGRAM:
        raise NativeBuildError('the firmware unit test needs the firmware sources: %s missing (set %s)' % (
            firmware_dir() / FIRMWARE_SKETCH, ENV_FIRMWARE_DIR))
    sources = source_sha256()
    fw_id = firmware_build_id() if present else None
    if present:
        root = firmware_dir()
        compile_sources = [str(NATIVE_DIR / name) for name in native_sources]
        compile_sources += [str(root / name) for name in firmware_compile_sources()]
        includes = ['-I', str(NATIVE_DIR), '-I', str(SHIM_DIR), '-I', str(root)]
    else:
        compile_sources = [str(NATIVE_DIR / name) for name in native_sources if name not in FIRMWARE_ONLY_SOURCES]
        compile_sources.append(str(NATIVE_DIR / ABSENT_SOURCE))
        includes = ['-I', str(NATIVE_DIR), '-I', str(SHIM_DIR)]
    key = build_key(chain, sources) if program == PROGRAM else build_key(chain, sources + ':' + program)
    out_dir = resolve_build_dir(build_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    exe = out_dir / binary_name(chain, key, program)

    def reused():
        return BuildResult(path=str(exe), built=False, toolchain=chain, source_sha256=sources, build_key=key,
                           binary_sha256=file_sha256(exe), seconds=0., compiler_output='', firmware_build_id=fw_id)

    if exe.is_file() and not force:
        return reused()
    lock = exe.with_name(exe.name + '.lock')
    _acquire_lock(lock)
    tmp = exe.with_name('.%s.%d.%s.tmp%s' % (exe.stem, os.getpid(), uuid.uuid4().hex[:8], exe.suffix))
    try:
        if exe.is_file() and not force:
            return reused()
        cmd = ([chain.compiler] + list(chain.flags)
               + ['-DUBTS_STEER_SRC_ID=h' + sources[:12], '-DUBTS_STEER_FW_SRC=' + (fw_id or 'none'),
                  '-DUBTS_STEER_SIM=1'] + includes + compile_sources + ['-o', str(tmp)])
        start = time.monotonic()
        proc = _run(cmd, COMPILE_TIMEOUT_S)
        seconds = time.monotonic() - start
        output = proc.stderr.decode('utf-8', 'replace')
        if proc.returncode != 0 or not tmp.is_file():
            raise NativeBuildError('compile failed (exit %d) with %s:\n%s' % (proc.returncode, chain.version,
                                                                              output.strip()))
        if program == PROGRAM:
            _smoke_run(tmp)
        try:
            os.replace(str(tmp), str(exe))
        except OSError as exc:
            raise NativeBuildError('cannot publish %s (is it running?): %s' % (exe, exc)) from None
        result = BuildResult(path=str(exe), built=True, toolchain=chain, source_sha256=sources, build_key=key,
                             binary_sha256=file_sha256(exe), seconds=seconds, compiler_output=output,
                             firmware_build_id=fw_id)
        _write_manifest(str(exe), result)
        return result
    finally:
        if tmp.exists():
            try:
                tmp.unlink()
            except OSError:
                pass
        _release_lock(lock)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Build the native steering lockstep program.')
    parser.add_argument('--compiler', help='C++ compiler (default: $%s or g++ on PATH)' % ENV_CXX)
    parser.add_argument('--build-dir', help='output folder (default: $%s or %s)' % (ENV_BUILD_DIR, DEFAULT_BUILD_DIR))
    parser.add_argument('--force', action='store_true', help='rebuild even if the binary exists')
    parser.add_argument('--json', action='store_true', help='print the result as JSON')
    parser.add_argument('--unit-test', action='store_true', help='build the native firmware unit test instead')
    parser.add_argument('--firmware-id', action='store_true',
                        help='print the firmware build id (UBTS_STEER_FW_SRC) and exit; no compiler needed')
    args = parser.parse_args(argv)
    if args.firmware_id:
        try:
            print(firmware_build_id())
        except NativeBuildError as exc:
            print('build failed: %s' % exc, file=sys.stderr)
            return 1
        return 0
    try:
        build = ensure_unit_test_built if args.unit_test else ensure_built
        result = build(compiler=args.compiler, build_dir=args.build_dir, force=args.force)
    except NativeBuildError as exc:
        print('build failed: %s' % exc, file=sys.stderr)
        return 1
    if args.json:
        data = asdict(result)
        data['toolchain']['flags'] = list(result.toolchain.flags)
        print(json.dumps(data, indent=2, sort_keys=True))
    else:
        print('compiler: %s' % result.toolchain.compiler)
        print('version:  %s' % result.toolchain.version)
        print('target:   %s' % result.toolchain.target)
        print('flags:    %s' % ' '.join(result.toolchain.flags))
        print('binary:   %s (%s)' % (result.path, 'built in %.2f s' % result.seconds if result.built else 'reused'))
        print('sha256:   %s' % result.binary_sha256)
        print('firmware: %s' % (result.firmware_build_id or 'none (sources missing: %s; no firmware mode)'
                                % (firmware_dir() / FIRMWARE_SKETCH)))
        if result.compiler_output.strip():
            print('compiler output:\n%s' % result.compiler_output.strip())
    return 0


if __name__ == '__main__':
    sys.exit(main())
