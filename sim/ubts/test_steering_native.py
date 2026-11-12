"""Native steering actuator and potentiometer model (C++, decision R1) and its lockstep link.

The native program is built with the compiler from UBTS_STEER_CXX or g++ on PATH into
UBTS_STEER_BUILD_DIR or a pytest temporary folder (never into the work tree), then driven
through NativeSteeringBackend in direct_pwm and test_servo modes. Expected values come from the
protocol formulas (docs/steering/LOCKSTEP_PROTOCOL_KO.md section 7) and the profile, not from
another model. All values are placeholders: these are placeholder-based SIM checks.
"""
import gc
import json
import math
import os
from pathlib import Path
import re
import statistics
import struct
import subprocess
import sys
import time

import pytest

from steering import backend_api as ba
from steering import geometry
from steering import vehicle_profile as vp
from steering.actuator_link import NativeBackendError, NativeSteeringBackend
from steering.native import build as native_build

DEG = math.pi / 180.
UBTS_DIR = Path(__file__).resolve().parent
WORK_TREE = UBTS_DIR.parents[1]
PROCS = []  # every native child started here; all must be gone at the end

CLEAN = {
    'steer_zero_offset_deg': {'dist': 'uniform', 'low': 0.0, 'high': 0.0},
    'pot_cal_residual_deg': {'dist': 'uniform', 'low': 0.0, 'high': 0.0},
    'pot_noise_sd_counts': 0.0,
    'pot_supply_sag_per_pwm': 0.0,
    'steer_backlash_wander': {'sd_deg': 0.0, 'corr_length_m': 5.0},
    'steer_release_restoring': {'scale': 1.0, 'centering_per_s_per_m_s': 1.0, 'noise_sd_deg': 0.0,
                                'noise_corr_length_m': 1.0},
}
ASYMMETRIC_LIMITS = {'left_wheel_toward_left': 28.0, 'left_wheel_toward_right': 26.0, 'right_wheel_toward_left': 25.0,
                     'right_wheel_toward_right': 27.0}


# --- fixtures and helpers -------------------------------------------------------------------------------------------
@pytest.fixture(scope='session')
def native(tmp_path_factory):
    try:
        native_build.find_compiler()
    except native_build.NativeBuildError as exc:
        pytest.skip('no C++ compiler for the native steering model: %s' % exc)
    build_dir = os.environ.get(native_build.ENV_BUILD_DIR) or str(tmp_path_factory.mktemp('steer_native_build'))
    return native_build.ensure_built(build_dir=build_dir)


@pytest.fixture(scope='session')
def profile():
    # UBTS_STEER_TEST_PROFILE: rerun this suite against another profile file (e.g. v0.2 for the A2 / A3(F) / A8
    # re-measurement of ST-10, SIM_ACCEPTANCE amendment 40); default the shipped profile.
    return vp.load_profile(os.environ.get('UBTS_STEER_TEST_PROFILE') or None)


@pytest.fixture
def make(native):
    made = []

    def factory(prof, seed=1, **kwargs):
        backend = NativeSteeringBackend(prof, seed, executable=native.path, **kwargs)
        made.append(backend)
        PROCS.append(backend._proc)
        return backend

    yield factory
    for backend in made:
        backend.close()


def variant(prof, clean=True, **changes):
    overrides = dict(CLEAN) if clean else {}
    if clean and prof.has('belt_pot_play_deg'):
        # schema v3 (profile v0.2): the measurement-side belt play is not in the pot formula of expected_raw; the belt
        # terms have their own tests (test_steering_profile_v3.py). Found by the first v0.2 run (amendment 40 (1)).
        overrides['belt_pot_play_deg'] = 0.0
    overrides.update(changes)
    if not overrides:
        return prof
    return prof.with_overrides(overrides, label='native_test', allow_out_of_range=True)


def run(backend, n, dt=0.001, **kwargs):
    inp = ba.SteerStepInput(dt_s=dt, mode=backend.mode, **kwargs)
    return [backend.step(inp) for _ in range(n)]


def pot_gain(cfg):
    pot = cfg['pot']
    return (1 << pot['adc_bits']) * pot['to_steer_ratio'] * (1. + pot['gain_error']) / pot['range_rad']


def round_half_away(x):
    return math.floor(x + .5) if x >= 0 else -math.floor(-x + .5)


def expected_raw(cfg, theta, applied=0., supply=1.):
    """Protocol section 7 raw formula without noise (the expected value, from the specification)."""
    pot = cfg['pot']
    n = 1 << pot['adc_bits']
    center = pot['center_raw']
    r0 = center + pot_gain(cfg) * theta
    r1 = r0 + pot['linearity_frac'] * n * math.sin(2. * math.pi * (r0 - center) / n) if pot['linearity_frac'] else r0
    s_v = supply * (1. - pot['supply_sag_per_pwm'] * abs(applied))
    r2 = r1 if pot['ratiometric'] else s_v * r1
    return min(n - 1, max(0, round_half_away(r2)))


def slope(outputs, start, stop, dt=0.001):
    return (outputs[stop].motor_side_angle_rad - outputs[start].motor_side_angle_rad) / ((stop - start) * dt)


def startup_bytes(prof, mode='direct_pwm', seed=1, edit=None, injections=()):
    cfg = ba.native_startup_config(prof, seed, mode=mode, injections=injections)
    if edit is not None:
        edit(cfg)
    return json.dumps(cfg, separators=(',', ':')).encode('ascii') + b'\n'


def run_native(exe, data, timeout=30):
    proc = subprocess.run([exe], input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
    return proc.returncode, proc.stdout, proc.stderr.decode('ascii', 'replace')


def step_frame(seq, dt=0.01, pwm=0., target=0., speed=0., supply=1., mode=0, en=1, stopped=1, reserved=0,
               serial=b'', serial_len=None, payload_len=None):
    payload = ba.STEP_REQUEST.pack(dt, pwm, target, speed, supply, mode, en, stopped, reserved,
                                   len(serial) if serial_len is None else serial_len) + serial
    return ba.HEADER.pack(b'USTP', 1, ba.FRAME_STEP_REQUEST, seq,
                          len(payload) if payload_len is None else payload_len) + payload


def pid_alive(pid):
    if os.name == 'nt':
        import ctypes
        kernel32 = ctypes.windll.kernel32
        kernel32.OpenProcess.restype = ctypes.c_void_p
        handle = kernel32.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
        if not handle:
            return False
        try:
            code = ctypes.c_ulong()
            ok = kernel32.GetExitCodeProcess(ctypes.c_void_p(handle), ctypes.byref(code))
            return bool(ok) and code.value == 259  # STILL_ACTIVE
        finally:
            kernel32.CloseHandle(ctypes.c_void_p(handle))
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:
        with open('/proc/%d/stat' % pid) as handle:
            return handle.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return True


# --- build ------------------------------------------------------------------------------------------------------------
def test_build_result_flags_and_reuse(native):
    chain = native.toolchain
    assert chain.version and chain.target and os.path.isfile(native.path)
    name = os.path.basename(native.path)
    assert name.startswith('ubts_steer_native-') and native.build_key in name
    if chain.target.lower() == 'mingw32':
        assert {'-std=gnu++11', '-O2', '-msse2', '-mfpmath=sse', '-static'} <= set(chain.flags)
    elif 'linux' in chain.target:
        assert '-std=c++11' in chain.flags and '-O2' in chain.flags
    again = native_build.ensure_built(build_dir=os.path.dirname(native.path))
    assert again.path == native.path and not again.built and again.binary_sha256 == native.binary_sha256
    manifest = Path(native.path[:-4] + '.json' if native.path.endswith('.exe') else native.path + '.json')
    data = json.loads(manifest.read_bytes())
    assert data['schema'] == 'ubts.steer_native_build.v1' and data['binary_sha256'] == native.binary_sha256
    assert data['source_sha256'] == native_build.source_sha256()
    # binaries never go into git: the default build folder is ignored inside the native folder
    gitignore = (native_build.NATIVE_DIR / '.gitignore').read_bytes()
    assert b'_build/' in gitignore and native_build.DEFAULT_BUILD_DIR.name == '_build'


def test_build_concurrent_callers_compile_once(native, tmp_path):
    out = tmp_path / 'concurrent'
    code = ('import sys; from steering.native import build as b; r = b.ensure_built(build_dir=sys.argv[1]); '
            'print(r.path); print(r.built)')
    procs = [subprocess.Popen([sys.executable, '-B', '-c', code, str(out)], cwd=str(UBTS_DIR), stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE) for _ in range(3)]
    results = []
    for proc in procs:
        stdout, stderr = proc.communicate(timeout=600)
        assert proc.returncode == 0, stderr.decode('utf-8', 'replace')
        lines = stdout.decode('utf-8').split()
        results.append((lines[0], lines[1]))
    assert len({path for path, _ in results}) == 1
    assert sorted(built for _, built in results) == ['False', 'False', 'True']
    names = sorted(p.name for p in out.iterdir())
    binary = os.path.basename(results[0][0])
    assert binary in names and len(names) == 2  # the binary and its manifest; no lock or temporary file left
    # RI-07: same sources, flags and compiler in another folder -> the same bytes (MinGW: no PE time stamp)
    assert native_build.file_sha256(results[0][0]) == native.binary_sha256


def test_build_errors(tmp_path):
    with pytest.raises(native_build.NativeBuildError, match='compiler not found'):
        native_build.ensure_built(compiler='no-such-compiler-for-steering', build_dir=str(tmp_path))
    try:
        native_build.find_compiler()
    except native_build.NativeBuildError as exc:
        pytest.skip(str(exc))
    with pytest.raises(native_build.NativeBuildError, match='compile failed'):
        native_build.ensure_built(build_dir=str(tmp_path / 'bad'), extra_flags=['-include', 'no_such_header_xyz.h'])
    assert sorted(p.name for p in (tmp_path / 'bad').iterdir()) == []


def test_build_cli_prints_compiler_version(native):
    proc = subprocess.run([sys.executable, '-B', str(native_build.NATIVE_DIR / 'build.py'), '--build-dir',
                           os.path.dirname(native.path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=300)
    text = proc.stdout.decode('utf-8', 'replace')
    assert proc.returncode == 0 and native.toolchain.version in text and 'reused' in text and native.path in text


def test_build_lock_retries_a_lock_file_being_deleted(tmp_path, monkeypatch):
    """RI-05: on Windows O_EXCL on a lock file that another process is deleting raises PermissionError; that is
    retried, and a lasting PermissionError becomes a NativeBuildError (never a raw PermissionError)."""
    real = native_build._create_exclusive
    calls = []

    def flaky(lock):
        calls.append(lock)
        if len(calls) <= 3:
            raise PermissionError(13, 'Permission denied (delete pending)')
        return real(lock)
    monkeypatch.setattr(native_build, '_create_exclusive', flaky)
    lock = tmp_path / 'x.lock'
    native_build._acquire_lock(lock)
    assert len(calls) == 4 and lock.is_file()
    native_build._release_lock(lock)

    def denied(lock):
        raise PermissionError(13, 'Permission denied')
    monkeypatch.setattr(native_build, '_create_exclusive', denied)
    monkeypatch.setattr(native_build, 'LOCK_PERMISSION_RETRY_S', 0.2)
    start = time.monotonic()
    with pytest.raises(native_build.NativeBuildError, match='cannot create the build lock'):
        native_build._acquire_lock(lock)
    assert time.monotonic() - start < 5.
    # a live compile (up to COMPILE_TIMEOUT_S plus the smoke run) never looks stale, and waiters outlast stale locks
    assert native_build.LOCK_STALE_S > native_build.COMPILE_TIMEOUT_S + native_build.SMOKE_TIMEOUT_S
    assert native_build.LOCK_TIMEOUT_S > native_build.LOCK_STALE_S


def test_build_reuse_probes_the_compiler_once(native, monkeypatch):
    """RI-08: reusing a built binary does not run the compiler again or re-hash the binary in one process."""
    native_build.clear_caches()
    runs, hashes = [], []
    real_run, real_hash = native_build._run, native_build._file_sha256_uncached
    monkeypatch.setattr(native_build, '_run', lambda cmd, timeout: runs.append(cmd) or real_run(cmd, timeout))
    monkeypatch.setattr(native_build, '_file_sha256_uncached', lambda path: hashes.append(path) or real_hash(path))
    build_dir = os.path.dirname(native.path)
    first = native_build.ensure_built(build_dir=build_dir)
    for _ in range(3):
        again = native_build.ensure_built(build_dir=build_dir)
        assert again.path == first.path and again.binary_sha256 == first.binary_sha256 == native.binary_sha256
    assert len(runs) == 2 and len(hashes) == 1        # --version and -dumpmachine once, the binary hashed once
    assert not first.built and not again.built


# --- protocol basics --------------------------------------------------------------------------------------------------
def test_ready_and_reset_state(profile, make):
    backend = make(variant(profile))
    assert re.match(r'ubts-steer-native/1 src=h[0-9a-f]{12} cxx=\S+ bits=(32|64)', backend.build_id)
    out = backend.reset(0.)
    assert out.ticks == 0 and out.sim_time_s == 0. and out.applied_steer_pwm == 0. and not out.steer_en
    assert out.delta_eq_rad == pytest.approx(0., abs=1e-15) and out.motor_side_angle_rad == pytest.approx(0., abs=1e-15)
    assert out.pot_raw == backend.config['pot']['center_raw']
    assert out.flags() == ['IN_BACKLASH', 'FREE_WHEELING'] and out.fault_bits == 0
    assert (out.drive_duty_left, out.drive_duty_right, out.drive_en, out.serial_out) == (0., 0., False, b'')
    out = backend.reset(0.2)
    assert out.delta_eq_rad == pytest.approx(0.2, abs=1e-15) and out.motor_side_angle_rad == pytest.approx(0.2, abs=1e-15)
    label = backend.result_label()
    assert label['result_kind'] == vp.RESULT_LABEL_PLACEHOLDER and label['field_validated'] is False
    assert label['backend'] == 'native_lockstep_v1' and label['build_id'] == backend.build_id
    assert label['executable_sha256'] and label['overrides']['label'] == 'native_test'


def test_tick_rule_is_cumulative(profile, make):
    backend = make(variant(profile))
    backend.reset(0.)
    dt = 1. / 120.
    outs = run(backend, 120, dt=dt)
    plant_time, total, expected = 0., 0, []
    for _ in range(120):
        plant_time += dt
        target = round_half_away(plant_time * 1000)
        expected.append(target - total)
        total = target
    assert [o.ticks for o in outs] == expected and set(expected) == {8, 9} and total == 1000
    assert outs[-1].sim_time_s == pytest.approx(1.0, abs=1e-12)
    backend.reset(0.)
    assert [o.ticks for o in run(backend, 50, dt=0.01)] == [10] * 50


def test_zero_tick_steps_return_the_state_unchanged(profile, make):
    backend = make(variant(profile))
    reset = backend.reset(0.1)
    outs = run(backend, 3, dt=2e-4, steer_pwm=1., vehicle_stopped=True)
    assert [o.ticks for o in outs] == [0, 0, 1]
    for out in outs[:2]:
        assert (out.sim_time_s, out.motor_side_angle_rad, out.delta_eq_rad, out.pot_raw, out.applied_steer_pwm,
                out.steer_en) == (reset.sim_time_s, reset.motor_side_angle_rad, reset.delta_eq_rad, reset.pot_raw,
                                  reset.applied_steer_pwm, reset.steer_en)
    assert outs[2].applied_steer_pwm == 1. and outs[2].steer_en and outs[2].sim_time_s == pytest.approx(1e-3)


# --- actuator (direct_pwm) ---------------------------------------------------------------------------------------------
@pytest.mark.parametrize('stopped, rate_name', [(False, 'rate_max_rad_s'), (True, 'rate_standstill_rad_s')])
def test_rate_limit_and_lag(profile, make, stopped, rate_name):
    backend = make(variant(profile))
    cfg = backend.config['actuator']
    rate, tau = cfg[rate_name], cfg['tau_s']
    start = backend.reset(-0.40).motor_side_angle_rad
    outs = run(backend, 1200, steer_pwm=1., vehicle_speed_m_s=0. if stopped else 0.4, vehicle_stopped=stopped)
    assert slope(outs, 899, 1199) == pytest.approx(rate, rel=1e-3)  # A2: rate within 5 %
    t_end = 1.2
    tau_est = t_end - (outs[-1].motor_side_angle_rad - start) / rate
    assert tau_est == pytest.approx(tau, rel=0.01)  # A2: lag within 5 %
    # exact first-order step response of the speed: theta(t) = w (t - tau (1 - exp(-t / tau)))
    for index in (19, 79, 299):
        t = (index + 1) * 1e-3
        assert outs[index].motor_side_angle_rad - start == pytest.approx(rate * (t - tau * (1 - math.exp(-t / tau))),
                                                                          rel=1e-9)
    # the opposite PWM sign moves the other way at the same rate
    backend.reset(0.40)
    back = run(backend, 1200, steer_pwm=-1., vehicle_speed_m_s=0. if stopped else 0.4, vehicle_stopped=stopped)
    assert slope(back, 899, 1199) == pytest.approx(-rate, rel=1e-3)


@pytest.mark.parametrize('stopped, min_name, rate_name', [(False, 'min_pwm', 'rate_max_rad_s'),
                                                          (True, 'min_pwm_standstill', 'rate_standstill_rad_s')])
def test_min_pwm_deadband_and_standstill_friction(profile, make, stopped, min_name, rate_name):
    backend = make(variant(profile))
    cfg = backend.config['actuator']
    min_pwm, rate = cfg[min_name], cfg[rate_name]
    speed = 0. if stopped else 0.4
    start = backend.reset(-0.2).motor_side_angle_rad
    below = run(backend, 500, steer_pwm=0.95 * min_pwm, vehicle_speed_m_s=speed, vehicle_stopped=stopped)
    assert all(o.motor_side_angle_rad == start for o in below) and below[-1].applied_steer_pwm == 0.95 * min_pwm
    backend.reset(-0.2)
    above = run(backend, 300, steer_pwm=1.05 * min_pwm, vehicle_speed_m_s=speed, vehicle_stopped=stopped)
    assert above[-1].motor_side_angle_rad > start  # A2: deadband edge within 5 %
    backend.reset(-0.2)
    u = min_pwm + 0.15
    outs = run(backend, 1000, steer_pwm=u, vehicle_speed_m_s=speed, vehicle_stopped=stopped)
    assert slope(outs, 499, 999) == pytest.approx(rate * (u - min_pwm) / (1 - min_pwm), rel=1e-3)
    if not stopped:
        # extra static friction while stopped: the same PWM that moves a rolling car does not move a standing one
        backend.reset(-0.2)
        still = run(backend, 300, steer_pwm=0.3, vehicle_stopped=True)
        assert all(o.motor_side_angle_rad == start for o in still)
        assert cfg['min_pwm'] < 0.3 < cfg['min_pwm_standstill'] and cfg['rate_standstill_rad_s'] < cfg['rate_max_rad_s']


def test_backlash_play_on_reversal(profile, make):
    backend = make(variant(profile))
    play = backend.config['actuator']['backlash_rad']
    assert play > 0
    backend.reset(0.)
    outs = run(backend, 400, steer_pwm=1., vehicle_stopped=True)
    # first the motor crosses half the play before the link moves
    first = next(i for i, o in enumerate(outs) if o.delta_eq_rad > 1e-12)
    assert outs[first - 1].motor_side_angle_rad <= play / 2 < outs[first].motor_side_angle_rad + 1e-12
    assert 'IN_BACKLASH' in outs[first - 1].flags() and 'IN_BACKLASH' not in outs[-1].flags()
    back = run(backend, 600, steer_pwm=-0.8, vehicle_stopped=True)
    seq = outs + back
    peak = max(range(len(seq)), key=lambda i: seq[i].motor_side_angle_rad)
    held = seq[peak].delta_eq_rad
    moved = next(i for i in range(peak, len(seq)) if seq[i].delta_eq_rad < held - 1e-12)
    travel = seq[peak].motor_side_angle_rad - seq[moved - 1].motor_side_angle_rad
    step_motion = seq[moved - 1].motor_side_angle_rad - seq[moved].motor_side_angle_rad
    assert play - step_motion - 1e-12 <= travel <= play + 1e-12  # A2: backlash within one step (< 5 %)
    assert all('IN_BACKLASH' in seq[i].flags() for i in range(peak + 2, moved - 1))
    assert all(abs(o.motor_side_angle_rad - o.delta_eq_rad) <= play / 2 + 1e-12 for o in seq)


@pytest.mark.parametrize('case', [dict(steer_type='linkage', steer_ackermann_ratio=0.0),
                                  dict(steer_type='linkage', steer_ackermann_ratio=0.5),
                                  dict(steer_type='linkage', steer_ackermann_ratio=1.0),
                                  dict(steer_type='axle_pivot')], ids=['parallel', 'half', 'ideal', 'axle_pivot'])
def test_end_stops_per_wheel_and_equivalent_limit(profile, make, case):
    prof = variant(profile, steer_mech_limit_deg=ASYMMETRIC_LIMITS, steer_lr_asymmetry=0.06,
                   steer_zero_offset_deg={'dist': 'uniform', 'low': 1.2, 'high': 1.2},
                   steer_link_table=[[-40.0, -37.0], [0.0, 0.0], [40.0, 42.0]], **case)
    backend = make(prof)
    g = backend.config['geometry']
    a = backend.config['actuator']
    limits = g['wheel_mech_limit_rad']
    geom = geometry.SteeringGeometry.from_profile(prof)
    backend.reset(0.)
    left = run(backend, 3500, steer_pwm=1., vehicle_stopped=True)
    right = run(backend, 7000, steer_pwm=-1., vehicle_stopped=True)
    for out in left + right:  # A21: no wheel beyond its own mechanical limit, ever
        assert -limits['left_wheel_toward_right'] <= out.delta_left_rad <= limits['left_wheel_toward_left']
        assert -limits['right_wheel_toward_right'] <= out.delta_right_rad <= limits['right_wheel_toward_left']
        assert -g['eq_mech_limit_right_rad'] <= out.delta_eq_rad <= g['eq_mech_limit_left_rad']
    for out in left[::250] + right[::500]:
        expected = geometry.wheel_angles(out.delta_eq_rad, g['steer_type'], g['wheelbase_m'], g['kingpin_width_m'],
                                         g['ackermann_ratio'])
        assert (out.delta_left_rad, out.delta_right_rad) == pytest.approx(expected, abs=1e-12)
    at_left, at_right = left[-1], right[-1]
    assert at_left.delta_eq_rad == pytest.approx(g['eq_mech_limit_left_rad'], abs=1e-12)
    assert at_right.delta_eq_rad == pytest.approx(-g['eq_mech_limit_right_rad'], abs=1e-12)
    assert 'END_STOP_LEFT' in at_left.flags() and 'END_STOP_RIGHT' in at_right.flags()
    # one wheel touches its limit at each equivalent limit (the limit is the largest such angle)
    assert max(at_left.delta_left_rad - limits['left_wheel_toward_left'],
               at_left.delta_right_rad - limits['right_wheel_toward_left']) == pytest.approx(0., abs=1e-9)
    assert max(-at_right.delta_left_rad - limits['left_wheel_toward_right'],
               -at_right.delta_right_rad - limits['right_wheel_toward_right']) == pytest.approx(0., abs=1e-9)
    # the motor side stops half a play beyond the link end stop
    def link_angle(delta):
        y = delta - a['zero_offset_rad']
        y = y / (1 + a['lr_asymmetry']) if y > 0 else y / (1 - a['lr_asymmetry'])
        return geom.link_table.pot_from_wheel(y)
    half = a['backlash_rad'] / 2
    assert at_left.motor_side_angle_rad == pytest.approx(link_angle(g['eq_mech_limit_left_rad']) + half, abs=1e-12)
    assert at_right.motor_side_angle_rad == pytest.approx(link_angle(-g['eq_mech_limit_right_rad']) - half, abs=1e-12)


def test_test_servo_target_beyond_the_limit_stops_at_the_end(profile, make):
    backend = make(variant(profile, steer_mech_limit_deg=ASYMMETRIC_LIMITS), mode='test_servo')
    g = backend.config['geometry']
    limits = g['wheel_mech_limit_rad']
    backend.reset(0.)
    outs = run(backend, 4000, dt=0.01, target_delta_eq_rad=0.7, vehicle_stopped=True)
    assert outs[-1].delta_eq_rad == pytest.approx(g['eq_mech_limit_left_rad'], abs=1e-12)
    assert all(o.delta_left_rad <= limits['left_wheel_toward_left'] and
               o.delta_right_rad <= limits['right_wheel_toward_left'] for o in outs)
    assert outs[-1].applied_steer_pwm == 1.0  # the servo keeps pushing into the stop


def test_free_wheeling_without_self_locking(profile, make):
    backend = make(variant(profile))
    play = backend.config['actuator']['backlash_rad']
    backend.reset(0.25)
    outs = run(backend, 2000, steer_pwm=0., vehicle_speed_m_s=0.5, vehicle_stopped=False)
    assert outs[-1].delta_eq_rad == pytest.approx(0.25 * math.exp(-1.0), rel=1e-9)  # centering 1/m over 1 m
    assert outs[-1].motor_side_angle_rad == pytest.approx(outs[-1].delta_eq_rad + play / 2, abs=1e-12)  # back-driven
    assert all('FREE_WHEELING' in o.flags() for o in outs)
    # EN off is also "no drive torque"; reversing gives the same distance-based relaxation
    backend.reset(0.25)
    off = run(backend, 2000, steer_pwm=0.6, steer_en=False, vehicle_speed_m_s=0.5, vehicle_stopped=False)
    assert off[-1].delta_eq_rad == pytest.approx(outs[-1].delta_eq_rad, abs=1e-15) and off[-1].applied_steer_pwm == 0.
    backend.reset(0.25)
    rev = run(backend, 2000, vehicle_speed_m_s=-0.5, vehicle_stopped=False)
    assert rev[-1].delta_eq_rad == outs[-1].delta_eq_rad
    # standing still: nothing moves
    backend.reset(0.25)
    still = run(backend, 2000, steer_pwm=0., vehicle_stopped=True)
    assert all(o.delta_eq_rad == still[0].delta_eq_rad and o.motor_side_angle_rad == 0.25 for o in still)
    # release scale 0.5 halves the centering rate
    half = make(variant(profile, steer_release_restoring={'scale': 0.5, 'centering_per_s_per_m_s': 1.0,
                                                          'noise_sd_deg': 0.0, 'noise_corr_length_m': 1.0}))
    half.reset(0.25)
    assert run(half, 2000, vehicle_speed_m_s=0.5, vehicle_stopped=False)[-1].delta_eq_rad == pytest.approx(
        0.25 * math.exp(-0.5), rel=1e-9)


def test_self_locking_holds_the_motor_side(profile, make):
    prof = variant(profile, steer_self_locking=True, steer_backlash_wander={'sd_deg': 0.3, 'corr_length_m': 5.0})
    backend = make(prof)
    play = backend.config['actuator']['backlash_rad']
    backend.reset(0.25)
    outs = run(backend, 2000, dt=0.01, steer_pwm=0., vehicle_speed_m_s=0.5, vehicle_stopped=False)
    assert all(o.motor_side_angle_rad == 0.25 for o in outs)  # not back-driven
    assert all(abs(o.delta_eq_rad - 0.25) <= play / 2 + 1e-12 for o in outs)  # link moves only inside the play
    assert statistics.pstdev([o.delta_eq_rad for o in outs]) > 0.02 * DEG
    assert not any('FREE_WHEELING' in o.flags() for o in outs)


def test_backlash_wander_while_moving_only(profile, make):
    backend = make(variant(profile, steer_backlash_wander={'sd_deg': 0.3, 'corr_length_m': 5.0}))
    play = backend.config['actuator']['backlash_rad']
    start = backend.reset(0.1).motor_side_angle_rad
    # PWM 0.1 is below the minimum: the motor holds (drive torque present) and does not move
    moving = run(backend, 2000, dt=0.01, steer_pwm=0.1, vehicle_speed_m_s=0.5, vehicle_stopped=False)
    assert all(o.motor_side_angle_rad == start for o in moving)
    assert all(abs(o.delta_eq_rad - start) <= play / 2 + 1e-12 for o in moving)
    assert statistics.pstdev([o.delta_eq_rad for o in moving]) > 0.02 * DEG
    still = run(backend, 500, dt=0.01, steer_pwm=0.1, vehicle_stopped=True)
    assert len({o.delta_eq_rad for o in still}) == 1
    assert not any('FREE_WHEELING' in o.flags() for o in moving + still)


def test_release_noise_and_centering_statistics(profile, make):
    prof = variant(profile, steer_release_restoring={'scale': 1.0, 'centering_per_s_per_m_s': 1.0,
                                                     'noise_sd_deg': 0.3, 'noise_corr_length_m': 1.0})
    backend = make(prof, seed=5)
    backend.reset(0.1)
    outs = run(backend, 40000, dt=0.01, vehicle_speed_m_s=0.5, vehicle_stopped=False)  # 200 m
    tail = [o.delta_eq_rad for o in outs[20000:]]
    # increments of an OU disturbance (sd 0.3 deg, 1 m) against centering 1/m: stationary sd 0.3/sqrt(2) deg
    assert statistics.pstdev(tail) == pytest.approx(0.3 * DEG / math.sqrt(2.), rel=0.35)
    assert abs(statistics.fmean(tail)) < 0.15 * DEG


def test_actuator_fault_jam_slow_and_driver_dead(profile, make):
    jam = make(variant(profile), injections=[dict(kind='jam', at_s=0.3, params={})])
    jam.reset(-0.3)
    outs = run(jam, 600, steer_pwm=1., vehicle_stopped=True)
    assert outs[298].motor_side_angle_rad < outs[299].motor_side_angle_rad
    assert all(o.motor_side_angle_rad == outs[299].motor_side_angle_rad for o in outs[299:])
    assert all(o.applied_steer_pwm == 1. for o in outs)  # the PWM still goes out
    assert 'INJECTION_ACTIVE' not in outs[298].flags() and 'INJECTION_ACTIVE' in outs[299].flags()

    slow = make(variant(profile), injections=[dict(kind='slow', at_s=0.8, params=dict(rate_scale=0.5))])
    rate = slow.config['actuator']['rate_standstill_rad_s']
    slow.reset(-0.4)
    outs = run(slow, 1600, steer_pwm=1., vehicle_stopped=True)
    assert slope(outs, 599, 799) == pytest.approx(rate, rel=1e-3)
    assert slope(outs, 1399, 1599) == pytest.approx(0.5 * rate, rel=1e-3)

    dead = make(variant(profile), injections=[dict(kind='driver_dead', at_s=0., params={})])
    start = dead.reset(0.).motor_side_angle_rad
    outs = run(dead, 300, steer_pwm=1., vehicle_stopped=True)
    assert all(o.applied_steer_pwm == 0. and o.motor_side_angle_rad == start for o in outs)
    assert all('FREE_WHEELING' in o.flags() for o in outs)


def test_actuator_fault_driver_stuck_on_stops_only_with_en_off(profile, make):
    backend = make(variant(profile), injections=[dict(kind='driver_stuck_on', at_s=0., params=dict(pwm=0.8))])
    cfg = backend.config['actuator']
    backend.reset(-0.3)
    outs = run(backend, 600, steer_pwm=0., vehicle_stopped=True) + run(backend, 200, steer_pwm=-1.,
                                                                        vehicle_stopped=True)
    assert all(o.applied_steer_pwm == 0.8 for o in outs)
    w = cfg['rate_standstill_rad_s'] * (0.8 - cfg['min_pwm_standstill']) / (1 - cfg['min_pwm_standstill'])
    assert slope(outs, 699, 799) == pytest.approx(w, rel=1e-3)
    off = run(backend, 800, steer_pwm=0., steer_en=False, vehicle_stopped=True)
    assert all(o.applied_steer_pwm == 0. and not o.steer_en for o in off)
    # EN off removes the drive: the speed decays with tau, so the motor coasts w * tau and stops
    coast = off[-1].motor_side_angle_rad - outs[-1].motor_side_angle_rad
    assert coast == pytest.approx(w * cfg['tau_s'], rel=1e-3)
    assert abs(off[-1].motor_side_angle_rad - off[-101].motor_side_angle_rad) < 1e-3 * w * 0.1


def test_actuator_fault_polarity_reversed(profile, make):
    backend = make(variant(profile), injections=[dict(kind='polarity_reversed', at_s=0., params={})])
    start = backend.reset(0.).motor_side_angle_rad
    outs = run(backend, 300, steer_pwm=0.6, vehicle_stopped=True)
    assert outs[-1].motor_side_angle_rad < start and outs[-1].applied_steer_pwm == 0.6  # reported before the wiring
    servo = make(variant(profile), mode='test_servo', injections=[dict(kind='polarity_reversed', at_s=0., params={})])
    servo.reset(0.)
    outs = run(servo, 300, dt=0.01, target_delta_eq_rad=5 * DEG, vehicle_stopped=True)
    runaway = next(i for i, o in enumerate(outs) if 'END_STOP_RIGHT' in o.flags())
    assert runaway * 0.01 < 3.0 and outs[-1].applied_steer_pwm == 1.0  # positive feedback runs to the far end


# --- test servo -------------------------------------------------------------------------------------------------------
def test_test_servo_law(profile, make):
    prof = variant(profile)
    backend = make(prof, mode='test_servo')
    servo = backend.config['test_servo']
    period = backend.config['mcu_tick_hz'] // servo['servo_hz']
    table = geometry.SteeringGeometry.from_profile(prof).link_table
    target = 10 * DEG
    previous = backend.reset(0.)
    outs = run(backend, 1500, target_delta_eq_rad=target, vehicle_stopped=True)
    for n, out in enumerate(outs):
        if n % period == 0:
            error = target - table.wheel_from_pot(previous.motor_side_angle_rad)  # pot at the motor side
            magnitude = 0. if abs(error) <= servo['deadband_rad'] else min(
                servo['max_pwm'], max(servo['min_pwm'], servo['kp_per_rad'] * abs(error)))
            assert out.applied_steer_pwm == pytest.approx(math.copysign(magnitude, error), abs=1e-12)
        else:
            assert out.applied_steer_pwm == outs[n - 1].applied_steer_pwm
        previous = out


@pytest.mark.parametrize('moving', [True, False], ids=['moving', 'standing'])
@pytest.mark.parametrize('target_deg', [1.0, 1.5, 2.0, 3.0, 10.0, -2.0])
def test_test_servo_settles_inside_the_deadband_with_the_shipped_profile(profile, make, moving, target_deg):
    """F1: with the shipped servo and actuator values (no override) the test servo reaches its deadband moving
    and standing. With servo_min_pwm_pct 25 (the old placeholder) it stopped 1.67 / 2.33 deg short, and targets of
    1-1.5 deg did not move the motor at all. Standing it then holds with output 0. Moving, the release model
    (non-self-locking, output 0 inside the deadband) pulls the wheels back toward straight, so the test servo
    (no brake hold; the firmware dwell brake is P3) pushes again at the deadband edge: the error stays within the
    deadband plus the drag of one servo period."""
    assert profile.value('servo_min_pwm_pct') > profile.value('steer_min_pwm_standstill_pct')
    backend = make(variant(profile), mode='test_servo')
    deadband = backend.config['test_servo']['deadband_rad']
    play = backend.config['actuator']['backlash_rad']
    backend.reset(0.)
    outs = run(backend, 600, dt=0.01, target_delta_eq_rad=target_deg * DEG, vehicle_speed_m_s=0.4 if moving else 0.,
               vehicle_stopped=not moving)
    final = outs[-1]
    tail = [abs(o.motor_side_angle_rad - target_deg * DEG) for o in outs[-200:]]   # last 2 s
    if moving:
        assert max(tail) <= deadband + 0.05 * DEG and statistics.fmean(tail) < deadband
    else:
        assert max(tail) <= deadband + 1e-9 and final.applied_steer_pwm == 0.
    assert abs(final.delta_eq_rad - final.motor_side_angle_rad) <= play / 2 + 1e-12  # the wheel lags inside the play


def test_test_servo_minimum_at_the_motor_start_stops_short(profile, make):
    """The F1 artefact itself (now reported by profile.warnings()): a servo minimum equal to the moving start PWM."""
    prof = variant(profile, servo_min_pwm_pct=25.0, steer_backlash_deg=0.0)
    assert [w['code'] for w in prof.warnings()] == ['servo_min_pwm_cannot_start_the_motor'] * 2
    backend = make(prof, mode='test_servo')
    backend.reset(0.)
    outs = run(backend, 400, dt=0.01, target_delta_eq_rad=10 * DEG, vehicle_speed_m_s=0.4, vehicle_stopped=False)
    error = 10 * DEG - outs[-1].motor_side_angle_rad
    assert error > backend.config['test_servo']['deadband_rad'] and outs[-1].applied_steer_pwm == 0.25


def test_l0_ideal_servo(profile, make):
    prof = variant(profile, steer_actuator_level='L0')
    backend = make(prof, mode='test_servo')
    rate = backend.config['actuator']['rate_standstill_rad_s']
    backend.reset(0.)
    outs = run(backend, 2500, target_delta_eq_rad=10 * DEG, vehicle_stopped=True)
    assert slope(outs, 99, 299) == pytest.approx(rate, rel=1e-9)  # rate-limited approach
    assert outs[-1].motor_side_angle_rad == pytest.approx(10 * DEG, abs=1e-9)
    assert all(abs(o.applied_steer_pwm) <= 1. for o in outs) and outs[150].applied_steer_pwm == pytest.approx(1.)
    with pytest.raises(NativeBackendError) as info:
        make(prof, mode='direct_pwm')
    assert info.value.exit_code == ba.EXIT_BAD_STARTUP and 'L0' in info.value.stderr_line
    with pytest.raises(NativeBackendError) as info:
        make(prof, mode='test_servo', injections=[dict(kind='driver_dead', at_s=0., params={})])
    assert info.value.exit_code == ba.EXIT_BAD_STARTUP and 'driver fault' in info.value.stderr_line


# --- potentiometer ----------------------------------------------------------------------------------------------------
def test_pot_quantization_and_lsb(profile, make):
    backend = make(variant(profile))
    cfg = backend.config
    g = pot_gain(cfg)
    assert 1 / g == pytest.approx(cfg['pot']['range_rad'] / 1024.)  # LSB = range / (2^bits * ratio)
    backend.reset(-0.05)
    outs = run(backend, 3000, steer_pwm=0.3, vehicle_speed_m_s=0.4, vehicle_stopped=False)
    for out in outs:
        assert out.pot_raw == expected_raw(cfg, out.motor_side_angle_rad)
    raws = [o.pot_raw for o in outs]
    assert all(b - a in (0, 1) for a, b in zip(raws, raws[1:])) and raws[-1] - raws[0] > 20
    changes = [o.motor_side_angle_rad for a, o in zip(outs, outs[1:]) if o.pot_raw != a.pot_raw]
    spans = [b - a for a, b in zip(changes, changes[1:])]
    assert statistics.fmean(spans) == pytest.approx(1 / g, rel=0.01)


def test_pot_location_motor_and_wheel_side(profile, make):
    motor = make(variant(profile, pot_location='motor'))
    wheel = make(variant(profile, pot_location='wheel'))
    outs = {}
    for name, backend in (('motor', motor), ('wheel', wheel)):
        backend.reset(0.)
        outs[name] = run(backend, 400, steer_pwm=1., vehicle_stopped=True) + run(backend, 700, steer_pwm=-0.8,
                                                                                  vehicle_stopped=True)
    cfg = motor.config
    for a, b in zip(outs['motor'], outs['wheel']):
        assert a.motor_side_angle_rad == b.motor_side_angle_rad and a.delta_eq_rad == b.delta_eq_rad
        assert a.pot_raw == expected_raw(cfg, a.motor_side_angle_rad)  # before the backlash
        assert b.pot_raw == expected_raw(cfg, b.delta_eq_rad)  # after the backlash (identity link, no offset)
    # while the motor side crosses the play after the reversal, the motor-side pot moves and the wheel-side pot stays
    seq = outs['motor']
    peak = max(range(len(seq)), key=lambda i: seq[i].motor_side_angle_rad)
    moved = next(i for i in range(peak, len(seq)) if seq[i].delta_eq_rad < seq[peak].delta_eq_rad - 1e-12)
    assert len({o.pot_raw for o in outs['wheel'][peak:moved]}) == 1
    assert outs['motor'][peak].pot_raw - outs['motor'][moved - 1].pot_raw >= 7  # about b * g = 2 deg * 3.8 / deg


@pytest.mark.parametrize('sd', [1.0, 3.0])
def test_pot_noise_sd(profile, make, sd):
    backend = make(variant(profile, pot_noise_sd_counts=sd), seed=21)
    reset = backend.reset(0.1)
    expected_mean = backend.config['pot']['center_raw'] + pot_gain(backend.config) * reset.motor_side_angle_rad
    raws = [o.pot_raw for o in run(backend, 10000, vehicle_stopped=True)]
    assert statistics.pstdev(raws) == pytest.approx(sd, rel=0.10)  # A3: noise sd within 10 %
    assert statistics.fmean(raws) == pytest.approx(expected_mean, abs=0.1)


def test_pot_nonlinearity_gain_and_offset(profile, make):
    backend = make(variant(profile, pot_linearity_pct=2.0, pot_gain_error=0.05, pot_center_raw=600))
    cfg = backend.config
    assert cfg['pot']['linearity_frac'] == 0.02 and cfg['pot']['gain_error'] == 0.05
    reset = backend.reset(0.)
    assert reset.pot_raw == 600
    backend.reset(-0.4)
    outs = run(backend, 2000, steer_pwm=1., vehicle_stopped=True)
    for out in outs[::7]:
        assert out.pot_raw == expected_raw(cfg, out.motor_side_angle_rad)
    # the 2 % term is visible: about 0.02 * 1024 * sin(2 pi * 95 / 1024) = 11 counts near -0.4 rad
    assert max(abs(o.pot_raw - (600 + pot_gain(cfg) * o.motor_side_angle_rad)) for o in outs) > 8


def test_pot_linearity_term_is_mostly_a_local_gain_in_the_steering_range(profile, make):
    """F6 (documented in the protocol section 7 and the profile note): the full-range sine of pot_linearity_pct gives
    a local slope of about 1 + 2 pi * linearity near the centre, i.e. 2 % acts like a 12.6 % gain error there."""
    backend = make(variant(profile, pot_linearity_pct=2.0, pot_noise_sd_counts=0.0, steer_backlash_deg=0.0))
    cfg = backend.config
    gain = pot_gain(cfg)
    slopes = []
    for theta_deg in (5., 10., 15., 20., 25.):
        out = backend.reset(theta_deg * DEG)
        slopes.append((out.pot_raw - 512) / (gain * out.motor_side_angle_rad))
    expected = 1. + 2. * math.pi * 0.02
    assert all(abs(s - expected) < 0.03 for s in slopes), slopes          # quantisation 1 count and the sine curvature
    assert min(slopes) > 1.1                                              # larger than the 5 % gain-error sweep end


def test_pot_supply_droop(profile, make):
    for ratiometric in (False, True):
        backend = make(variant(profile, pot_supply_sag_per_pwm=0.02, adc_ratiometric=ratiometric))
        cfg = backend.config
        theta = backend.reset(10 * DEG).motor_side_angle_rad
        nominal = expected_raw(cfg, theta)
        outs = run(backend, 5, steer_pwm=0.3, supply_scale=0.99, vehicle_stopped=True)  # 0.3 < standstill min PWM
        assert outs[-1].motor_side_angle_rad == theta
        drooped = expected_raw(cfg, theta, applied=0.3, supply=0.99)
        assert outs[-1].pot_raw == drooped
        if ratiometric:
            assert drooped == nominal
        else:
            assert nominal - drooped == round_half_away(theta * pot_gain(cfg) + 512) - round_half_away(
                (theta * pot_gain(cfg) + 512) * 0.99 * (1 - 0.02 * 0.3)) and nominal - drooped >= 8


def pot_case(kind, params, at_s=0.3):
    return [dict(kind=kind, at_s=at_s, params=params)]


def test_pot_faults_open_short_stuck(profile, make):
    for kind, value in (('pot_open', 0), ('pot_short', 1023)):
        backend = make(variant(profile), injections=pot_case(kind, {}))
        backend.reset(0.)
        outs = run(backend, 500, steer_pwm=1., vehicle_stopped=True)
        assert outs[298].pot_raw not in (0, 1023)
        assert all(o.pot_raw == value for o in outs[299:])  # from the sample at t = at_s exactly
        assert 'POT_CLIPPED' not in outs[-1].flags() and 'INJECTION_ACTIVE' in outs[-1].flags()
    stuck = make(variant(profile, pot_noise_sd_counts=1.0), injections=pot_case('pot_stuck', {}))
    stuck.reset(0.)
    outs = run(stuck, 600, steer_pwm=1., vehicle_stopped=True)
    assert all(o.pot_raw == outs[299].pot_raw for o in outs[299:])
    assert outs[-1].motor_side_angle_rad - outs[299].motor_side_angle_rad > 2 * DEG  # the shaft kept moving


def test_pot_faults_noise_jump_slip_drift_reversed(profile, make):
    noisy = make(variant(profile, pot_noise_sd_counts=1.0), seed=4, injections=pot_case('pot_noise_x10',
                                                                                         dict(scale=10.), at_s=0.))
    noisy.reset(0.)
    assert statistics.pstdev([o.pot_raw for o in run(noisy, 5000, vehicle_stopped=True)]) == pytest.approx(10., rel=0.1)

    jump = make(variant(profile), injections=pot_case('pot_jump', dict(counts=40, duration_s=0.05)))
    cfg = jump.config
    jump.reset(0.)
    outs = run(jump, 500, vehicle_stopped=True)
    base = expected_raw(cfg, 0.)
    assert [i for i, o in enumerate(outs) if o.pot_raw != base] == list(range(299, 349))  # 50 samples = 0.05 s
    assert all(o.pot_raw == base + 40 for o in outs[299:349])
    assert 'INJECTION_ACTIVE' in outs[300].flags() and 'INJECTION_ACTIVE' not in outs[-1].flags()

    slip = make(variant(profile), injections=pot_case('pot_slip', dict(offset_rad=0.05)))
    slip.reset(0.)
    outs = run(slip, 400, vehicle_stopped=True)
    assert outs[298].pot_raw == base and outs[299].pot_raw == expected_raw(cfg, 0.05)

    drift = make(variant(profile), injections=pot_case('pot_drift', dict(rate_rad_per_s=0.02), at_s=0.2))
    drift.reset(0.)
    outs = run(drift, 2200, vehicle_stopped=True)
    for index in (199, 700, 2199):
        t = (index + 1) * 1e-3
        assert outs[index].pot_raw == expected_raw(cfg, 0.02 * max(0., t - 0.2))

    reversed_ = make(variant(profile), injections=pot_case('pot_reversed', {}, at_s=0.))
    theta = reversed_.reset(10 * DEG).motor_side_angle_rad
    out = run(reversed_, 3, vehicle_stopped=True)[-1]
    assert out.pot_raw == expected_raw(cfg, -theta) and expected_raw(cfg, -theta) < base < expected_raw(cfg, theta)


def test_pot_clipped_flag(profile, make):
    backend = make(variant(profile, pot_to_steer_ratio=5.0, pot_range_deg=240.0))
    cfg = backend.config
    backend.reset(0.)
    outs = run(backend, 2500, steer_pwm=1., vehicle_stopped=True)
    unclamped = [round_half_away(512 + pot_gain(cfg) * o.motor_side_angle_rad) for o in outs]
    assert any(value > 1023 for value in unclamped)
    for out, value in zip(outs, unclamped):
        assert ('POT_CLIPPED' in out.flags()) == (value > 1023)
        assert out.pot_raw == min(1023, value)


def test_fixed_per_run_values_reach_both_backends(profile, make):
    """RP-04: steer_zero_p15 / pot_offset_m2 style overrides (fixed numbers) work with the native backend as with L0."""
    from steering.l0_servo import L0ServoBackend
    fixed = variant(profile, steer_zero_offset_deg=1.5, pot_cal_residual_deg=-2.0)
    native = make(fixed, mode='test_servo')
    assert native.config['actuator']['zero_offset_rad'] == pytest.approx(1.5 * DEG, abs=1e-15)
    assert native.config['pot']['cal_residual_rad'] == pytest.approx(-2.0 * DEG, abs=1e-15)
    l0 = L0ServoBackend()
    l0.configure(fixed, 1)
    assert l0.zero_offset == pytest.approx(1.5 * DEG, abs=1e-15)
    # the test servo does not know the zero offset: target 0 leaves the wheels at about +1.5 deg
    native.reset(0.)
    final = run(native, 300, dt=0.01, target_delta_eq_rad=0., vehicle_stopped=True)[-1]
    play, deadband = native.config['actuator']['backlash_rad'], native.config['test_servo']['deadband_rad']
    assert abs(final.delta_eq_rad - 1.5 * DEG) <= deadband + play / 2 + 1e-9


# --- determinism (A8) ---------------------------------------------------------------------------------------------------
def scenario_frames(count=3000):
    frames = []
    for seq in range(1, count + 1):
        phase = (seq // 300) % 4
        pwm = (0.9, 0., -0.5, 0.2)[phase]
        frames.append(ba.encode_step_request(seq, ba.SteerStepInput(
            dt_s=0.01, steer_pwm=pwm, steer_en=phase != 1 or seq % 2 == 0, vehicle_speed_m_s=0.45,
            vehicle_stopped=phase == 3, supply_scale=1.0 - 0.01 * phase)))
    return frames


def collect(backend, frames):
    return b''.join(backend._exchange_raw(frame) for frame in frames)


def test_determinism_same_seed_same_bytes(profile, make):
    frames = scenario_frames()
    first = collect(make(profile, seed=11), frames)
    second = collect(make(profile, seed=11), frames)
    other = collect(make(profile, seed=12), frames)
    assert len(first) == len(frames) * (ba.HEADER.size + ba.STEP_RESPONSE.size)
    assert first == second and first != other


def test_reset_restarts_the_noise_streams(profile, make):
    backend = make(profile, seed=3)
    frames = scenario_frames(800)
    payloads = []
    for base in (0, 801):
        reset = backend._exchange_raw(ba.encode_reset_request(base + 1, 0.05))
        steps = [backend._exchange_raw(ba.encode_step_request(base + 1 + seq, ba.decode_step_request(frame)[1]))
                 for seq, frame in enumerate(frames, start=1)]
        payloads.append([reset[ba.HEADER.size:]] + [s[ba.HEADER.size:] for s in steps])
    assert payloads[0] == payloads[1]


# --- protocol errors and exit codes -------------------------------------------------------------------------------------
def assert_exit(result, code):
    returncode, _, stderr = result
    assert returncode == code, stderr
    if code:
        match = re.match(r'steer_native: (\w+): .+', stderr.strip())
        assert match and match.group(1) == ba.EXIT_CODES[code], stderr


def test_exit_ok_cases(native, profile):
    assert run_native(native.path, b'') == (0, b'', '')
    code, stdout, stderr = run_native(native.path, startup_bytes(profile))
    assert code == 0 and stderr == ''
    tick, build_id = ba.decode_ready(stdout, expected_tick_hz=1000)
    assert build_id.startswith('ubts-steer-native/1')


def edit_set(path, value):
    def edit(cfg):
        node = cfg
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value
    return edit


def edit_del(path):
    def edit(cfg):
        node = cfg
        for key in path[:-1]:
            node = node[key]
        del node[path[-1]]
    return edit


@pytest.mark.parametrize('edit', [
    edit_set(('extra',), 1), edit_del(('pot', 'noise_sd_counts')), edit_set(('pot', 'center_raw'), 512.0),
    edit_set(('protocol',), 'ubts.steer_lockstep.v2'), edit_set(('mode',), 'firmware'),
    edit_set(('actuator', 'level'), 'L1b'), edit_set(('actuator', 'level'), 'L0'),
    edit_set(('injections',), [dict(kind='fire', at_s=0., params={})]),
    edit_set(('injections',), [dict(kind='slow', at_s=0., params={})]),
    edit_set(('geometry', 'eq_mech_limit_left_rad'), 0.5), edit_set(('geometry', 'link_table_rad'), [[0., 0.], [0., 1.]]),
    edit_set(('mcu_tick_hz',), 1001), edit_set(('mcu',), []), edit_set(('seed',), -1), edit_set(('seed',), 2 ** 64),
    edit_set(('pot', 'adc_bits'), 12), edit_set(('actuator', 'self_locking'), 0),
    edit_set(('actuator', 'rate_standstill_rad_s'), 0.6), edit_set(('actuator', 'min_pwm_standstill'), 0.2),
    edit_set(('pot', 'range_rad'), 5e-324), edit_set(('pot', 'range_rad'), 1e-6),
], ids=['unknown_key', 'missing_key', 'int_as_float', 'protocol', 'firmware', 'l1b', 'l0_direct', 'injection_kind',
        'injection_params', 'eq_limit', 'link_table', 'tick', 'mcu_type', 'seed_negative', 'seed_big', 'adc_bits',
        'bool_type', 'standstill_rate_above_moving', 'standstill_min_pwm_below_moving', 'pot_gain_not_finite',
        'pot_gain_too_large'])
def test_exit_bad_startup_config(native, profile, edit):
    assert_exit(run_native(native.path, startup_bytes(profile, edit=edit)), ba.EXIT_BAD_STARTUP)


def test_exit_bad_startup_line(native, profile):
    line = startup_bytes(profile)
    for data in (b'{"protocol": 1\n', b'not json\n', line[:-1], line.replace(b'"seed":', b'"seed":1,"seed":'),
                 line.replace(b'"tau_s":0.08', b'"tau_s":NaN'), line.replace(b'"tau_s":0.08', b'"tau_s":008'),
                 line[:-2] + b'} x\n', b'{"profile_id":"' + b'a' * 65536 + b'"}\n',
                 line.replace(b'"front_steer', b'"\xc3\xa9front_steer')):
        assert_exit(run_native(native.path, data), ba.EXIT_BAD_STARTUP)


def test_exit_frame_errors(native, profile):
    line = startup_bytes(profile)
    good = step_frame(1)
    cases = [
        (b'XXXX' + good[4:], ba.EXIT_BAD_HEADER),
        (good[:4] + struct.pack('<H', 2) + good[6:], ba.EXIT_BAD_HEADER),
        (ba.HEADER.pack(b'USTP', 1, ba.FRAME_STEP_RESPONSE, 1, 80) + bytes(80), ba.EXIT_BAD_HEADER),
        (ba.HEADER.pack(b'USTP', 1, 0x0003, 1, 0), ba.EXIT_BAD_HEADER),
        (step_frame(2), ba.EXIT_SEQUENCE),
        (good + step_frame(1), ba.EXIT_SEQUENCE),
        (step_frame(1, payload_len=40)[:16] + bytes(40), ba.EXIT_BAD_LENGTH),
        (ba.HEADER.pack(b'USTP', 1, ba.FRAME_RESET_REQUEST, 1, 16) + bytes(16), ba.EXIT_BAD_LENGTH),
        (step_frame(1, serial=b'abcd', serial_len=0), ba.EXIT_BAD_LENGTH),
        (step_frame(1, en=2), ba.EXIT_BAD_VALUE),
        (step_frame(1, reserved=1), ba.EXIT_BAD_VALUE),
        (step_frame(1, mode=1), ba.EXIT_BAD_VALUE),
        (step_frame(1, dt=0.), ba.EXIT_BAD_VALUE),
        (step_frame(1, dt=float('nan')), ba.EXIT_BAD_VALUE),
        (step_frame(1, pwm=1.5), ba.EXIT_BAD_VALUE),
        (step_frame(1, target=0.1), ba.EXIT_BAD_VALUE),
        (step_frame(1, supply=0.), ba.EXIT_BAD_VALUE),
        (step_frame(1, serial=b'abcd'), ba.EXIT_BAD_VALUE),
        (ba.encode_reset_request(1, 1.0), ba.EXIT_BAD_VALUE),
        (good[:10], ba.EXIT_TRUNCATED),
        (good[:30], ba.EXIT_TRUNCATED),
    ]
    for frames, code in cases:
        result = run_native(native.path, line + frames)
        assert_exit(result, code)
        assert ba.decode_ready(result[1][:ba.HEADER.size + ba.READY.size + struct.unpack_from('<I', result[1], 20)[0]])
    # a good step then EOF at the boundary: one response, exit 0
    code, stdout, stderr = run_native(native.path, line + good)
    ready_len = ba.HEADER.size + ba.READY.size + struct.unpack_from('<I', stdout, 20)[0]
    assert code == 0 and stderr == '' and ba.decode_step_response(stdout[ready_len:], 1).ticks == 10


def test_exit_io_error_when_stdout_is_closed(native, profile):
    proc = subprocess.Popen([native.path], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    PROCS.append(proc)
    proc.stdin.write(startup_bytes(profile))
    proc.stdin.flush()
    ba.read_frame(proc.stdout)
    proc.stdout.close()
    try:
        proc.stdin.write(step_frame(1))
        proc.stdin.close()
    except OSError:
        pass
    assert proc.wait(timeout=30) == ba.EXIT_IO
    assert proc.stderr.read().decode('ascii').startswith('steer_native: io_error:')
    proc.stderr.close()


# --- Python link: exceptions, partial reads, crashes ---------------------------------------------------------------------
def test_python_exception_for_startup_errors(profile, make):
    with pytest.raises(NativeBackendError) as info:
        make(variant(profile), mode='firmware')
    err = info.value
    assert err.exit_code == ba.EXIT_BAD_STARTUP and err.exit_name == 'bad_startup' and err.phase == 'startup'
    assert err.stderr_line.startswith('steer_native: bad_startup: mode: firmware') and 'exit code 2' in str(err)
    with pytest.raises(NativeBackendError) as info:
        make(variant(profile, steer_actuator_level='L1b'))
    assert info.value.exit_code == ba.EXIT_BAD_STARTUP and 'L1b' in info.value.stderr_line


def test_python_exception_for_bad_frames_closes_the_backend(profile, make):
    backend = make(variant(profile))
    backend.reset(0.)
    proc = backend._proc
    with pytest.raises(NativeBackendError) as info:
        backend._exchange_raw(b'XXXX' + step_frame(2)[4:])
    assert info.value.exit_code == ba.EXIT_BAD_HEADER and info.value.phase == 'raw' and not info.value.killed
    assert info.value.stderr_line.startswith('steer_native: bad_header:')
    assert not backend.running and proc.poll() is not None
    with pytest.raises(RuntimeError, match='not_running'):
        backend.step(ba.SteerStepInput(dt_s=0.01))
    other = make(variant(profile))
    with pytest.raises(NativeBackendError) as info:
        other._exchange_raw(step_frame(5))
    assert info.value.exit_code == ba.EXIT_SEQUENCE


def test_invalid_inputs_are_refused_before_sending(profile, make):
    backend = make(variant(profile))
    backend.reset(0.)
    with pytest.raises(ValueError):
        backend.step(ba.SteerStepInput(dt_s=0.01, steer_pwm=1.5))
    with pytest.raises(ValueError):
        backend.step(ba.SteerStepInput(dt_s=0.01, mode='test_servo', target_delta_eq_rad=0.1))
    with pytest.raises(ValueError):
        backend.reset(2.0)
    with pytest.raises(ValueError, match='outside_equivalent_mechanical_limits'):
        backend.reset(backend.config['geometry']['eq_mech_limit_left_rad'] + 1e-6)
    assert backend.running and backend.step(ba.SteerStepInput(dt_s=0.01)).ticks == 10
    assert backend.reset(backend.config['geometry']['eq_mech_limit_left_rad']).flags()[0] == 'END_STOP_LEFT'


def test_child_crash_raises_with_exit_code(profile, make):
    backend = make(variant(profile))
    backend.reset(0.)
    proc = backend._proc
    proc.kill()
    proc.wait(timeout=30)
    with pytest.raises(NativeBackendError) as info:
        for _ in range(3):
            backend.step(ba.SteerStepInput(dt_s=0.01))
    assert info.value.phase == 'step' and info.value.exit_code not in (None, 0) and not backend.running


FAKE_CHILD = r'''
import struct, sys, time
mode = sys.argv[1]
stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
def dribble(data):
    for i in range(0, len(data), 7):
        stdout.write(data[i:i + 7]); stdout.flush(); time.sleep(0.001)
line = stdin.readline()
if mode == 'silent':
    time.sleep(120)
dribble(struct.pack('<4sHHII', b'USTP', 1, 0x8000, 0, 12) + struct.pack('<II', 1000, 4) + b'fake')
while True:
    head = stdin.read(16)
    if len(head) < 16:
        sys.exit(0)
    magic, version, kind, seq, length = struct.unpack('<4sHHII', head)
    stdin.read(length)
    body = struct.pack('<ddddddddIHHHBBI', 0.01 * seq, 0.1, 0.1, 0.1, 0.1, 0.5, 0., 0., 10 if kind == 1 else 0,
                       512, 0, 0, 1, 0, 0)
    frame = struct.pack('<4sHHII', b'USTP', 1, 0x8001 if kind == 1 else 0x8002, seq + (mode == 'badseq'), 80) + body
    if mode == 'hang':
        time.sleep(120)
    if mode == 'crash' and seq == 3:
        stdout.write(frame[:40]); stdout.flush()
        sys.stderr.write('steer_native: internal_error: fake crash\n'); sys.stderr.flush()
        sys.exit(9)
    dribble(frame)
'''


def test_partial_reads_crash_and_bad_responses_with_a_fake_child(profile, tmp_path):
    script = tmp_path / 'fake_child.py'
    script.write_bytes(FAKE_CHILD.encode('ascii'))
    prof = variant(profile)
    ok = NativeSteeringBackend(prof, 1, executable=[sys.executable, '-B', str(script), 'ok'])
    PROCS.append(ok._proc)
    try:
        assert ok.build_id == 'fake' and ok.executable_sha256 is None
        assert ok.reset(0.).ticks == 0
        assert all(ok.step(ba.SteerStepInput(dt_s=0.01)).applied_steer_pwm == 0.5 for _ in range(5))
    finally:
        ok.close()
    assert ok.exit_code == 0
    crash = NativeSteeringBackend(prof, 1, executable=[sys.executable, '-B', str(script), 'crash'])
    PROCS.append(crash._proc)
    crash.reset(0.)
    crash.step(ba.SteerStepInput(dt_s=0.01))
    with pytest.raises(NativeBackendError) as info:
        crash.step(ba.SteerStepInput(dt_s=0.01))
    assert info.value.exit_code == 9 and info.value.stderr_line == 'steer_native: internal_error: fake crash'
    assert 'truncated' in str(info.value) or 'EOF' in str(info.value)
    bad = NativeSteeringBackend(prof, 1, executable=[sys.executable, '-B', str(script), 'badseq'])
    PROCS.append(bad._proc)
    with pytest.raises(NativeBackendError) as info:
        bad.reset(0.)
    assert info.value.killed and 'sequence' in str(info.value) and not bad.running
    assert not info.value.timed_out


@pytest.mark.parametrize('mode, phase', [('hang', 'reset'), ('silent', 'startup')])
def test_a_child_that_stops_answering_is_killed_after_the_response_timeout(profile, tmp_path, mode, phase):
    """RI-04: a child that answers READY and then nothing (or not even READY) must not hang the caller."""
    script = tmp_path / 'fake_child.py'
    script.write_bytes(FAKE_CHILD.encode('ascii'))
    backend = NativeSteeringBackend(mode='direct_pwm', executable=[sys.executable, '-B', str(script), mode],
                                    response_timeout_s=1.0)
    start = time.monotonic()
    with pytest.raises(NativeBackendError) as info:
        backend.configure(variant(profile), 1)
        backend.reset(0.)
    elapsed = time.monotonic() - start
    err = info.value
    assert err.timed_out and err.killed and err.phase == phase and 'no response frame within 1 s' in str(err)
    assert 1.0 <= elapsed < 8.0 and not backend.running
    assert err.exit_code is not None and err.exit_code != 0                 # the child was killed, not left behind
    with pytest.raises(ValueError):
        NativeSteeringBackend(mode='direct_pwm', response_timeout_s=0.)


def test_a_child_that_stops_reading_cannot_block_a_write(profile, tmp_path):
    """RI-04: the same deadline covers a write that blocks on a full pipe because the child stopped reading."""
    script = tmp_path / 'fake_child.py'
    script.write_bytes(FAKE_CHILD.encode('ascii'))
    backend = NativeSteeringBackend(variant(profile), 1, executable=[sys.executable, '-B', str(script), 'hang'],
                                    response_timeout_s=1.0)
    start = time.monotonic()
    with pytest.raises(NativeBackendError) as info:
        backend._exchange_raw(bytes(4 << 20))      # 4 MiB: far more than any pipe buffer; the child sleeps
    assert info.value.timed_out and info.value.phase == 'raw' and time.monotonic() - start < 8.0
    assert not backend.running and info.value.exit_code not in (None, 0)


# --- process lifetime ------------------------------------------------------------------------------------------------
def test_close_is_clean_and_idempotent(profile, make):
    backend = make(variant(profile))
    proc = backend._proc
    backend.reset(0.)
    backend.close()
    assert backend.exit_code == 0 and proc.poll() == 0 and not backend.running
    backend.close()
    with NativeSteeringBackend(variant(profile), 2, executable=backend.executable) as other:
        PROCS.append(other._proc)
        inner = other._proc
        other.reset(0.)
    assert inner.poll() == 0


def test_garbage_collection_stops_the_child(profile, native):
    backend = NativeSteeringBackend(variant(profile), 1, executable=native.path)
    proc = backend._proc
    PROCS.append(proc)
    backend.reset(0.)
    del backend
    gc.collect()
    assert proc.wait(timeout=30) == 0


def test_interpreter_exit_stops_the_child(native):
    script = ('import sys; sys.path.insert(0, sys.argv[1]); from steering import vehicle_profile as vp; '
              'from steering.actuator_link import NativeSteeringBackend; '
              'b = NativeSteeringBackend(vp.load_profile(), 1, executable=sys.argv[2]); b.reset(0.0); '
              'print(b.pid, flush=True)')
    proc = subprocess.run([sys.executable, '-B', '-c', script, str(UBTS_DIR), native.path], stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode('utf-8', 'replace')
    pid = int(proc.stdout.split()[0])
    deadline = time.monotonic() + 15
    while pid_alive(pid) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not pid_alive(pid)


def test_zz_no_leftover_processes():
    assert PROCS
    for proc in PROCS:
        if proc is None:
            continue
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            raise AssertionError('native child %d still running' % proc.pid)
        assert proc.poll() is not None
