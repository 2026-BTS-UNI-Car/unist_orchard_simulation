"""Steered plant <-> steering backend contract (backend_api.py) and the lockstep wire format v1
(docs/steering/LOCKSTEP_PROTOCOL_KO.md): fault bits, step validation, frame layouts and error codes,
startup line and the abstract backend. The fake backend here is a test double, not a model.
"""
import dataclasses
import io
import json
import math
from pathlib import Path
import re
import struct

import pytest

from steering import backend_api as ba
from steering import geometry
from steering import vehicle_profile as vp


@pytest.fixture(scope='module')
def profile():
    return vp.load_profile()


def test_fault_bits():
    assert ba.FAULT_BIT_NAMES == ('NOCAL', 'POT_OPEN', 'POT_SHORT', 'POT_JUMP', 'POT_NOISE', 'POLARITY', 'DRV_STUCK',
                                  'SOFTLIMIT', 'STALL', 'TRACK', 'THERMAL', 'TTL', 'WATCHDOG', 'CRC', 'BOOT')
    for index, name in enumerate(ba.FAULT_BIT_NAMES):
        assert getattr(ba.FaultBit, name) == 1 << index == ba.FAULT_BITS[name]
    assert ba.FAULT_RESERVED_MASK == 0x8000
    mask = ba.fault_mask(['WATCHDOG', 'POT_OPEN'])
    assert mask == ba.FaultBit.POT_OPEN | ba.FaultBit.WATCHDOG and ba.fault_names(mask) == ['POT_OPEN', 'WATCHDOG']
    assert set(ba.LATCHED_FAULTS) <= set(ba.FAULT_BIT_NAMES)
    assert not {'SOFTLIMIT', 'TTL', 'BOOT'} & set(ba.LATCHED_FAULTS)
    for bad in (0x8000, -1, 0x10000, True):
        with pytest.raises(ValueError):
            ba.fault_names(bad)
    with pytest.raises(ValueError):
        ba.fault_mask(['FIRE'])


def test_model_flags():
    for index, name in enumerate(ba.MODEL_FLAG_NAMES):
        assert getattr(ba.ModelFlag, name) == 1 << index == ba.MODEL_FLAGS[name]
    assert ba.model_flag_names(ba.ModelFlag.END_STOP_LEFT | ba.ModelFlag.POT_CLIPPED) == ['END_STOP_LEFT',
                                                                                          'POT_CLIPPED']
    with pytest.raises(ValueError):
        ba.model_flag_names(1 << 6)


def test_injection_kinds_cover_the_profile(profile):
    for kind in profile.value('pot_fault_injections'):
        assert 'pot_' + kind in ba.INJECTION_KINDS
    for kind in ('jam', 'slow', 'driver_dead', 'driver_stuck_on', 'polarity_reversed'):
        assert kind in ba.INJECTION_KINDS


def test_struct_sizes():
    assert (ba.HEADER.size, ba.STEP_REQUEST.size, ba.STEP_RESPONSE.size, ba.RESET_REQUEST.size, ba.READY.size) == (
        16, 48, 80, 8, 8)


@pytest.mark.parametrize('kwargs', [
    dict(mode='servo'), dict(dt_s=0.), dict(dt_s=1.5), dict(dt_s=float('nan')), dict(steer_pwm=1.2),
    dict(steer_pwm=float('inf')), dict(steer_en=1), dict(vehicle_stopped=0), dict(supply_scale=0.),
    dict(supply_scale=2.5), dict(serial_in=b'PING\n'), dict(target_delta_eq_rad=.1),
    dict(mode='test_servo', steer_pwm=.3), dict(mode='test_servo', target_delta_eq_rad=2.), dict(mode='firmware',
                                                                                              steer_en=False),
    dict(mode='firmware', serial_in=b'x' * (ba.MAX_SERIAL_BYTES + 1)), dict(serial_in='PING'),
])
def test_step_input_validation(kwargs):
    base = dict(dt_s=.01)
    base.update(kwargs)
    with pytest.raises(ValueError):
        ba.SteerStepInput(**base).validate()


def test_step_input_valid_examples():
    ba.SteerStepInput(dt_s=.01, steer_pwm=-1., steer_en=False, vehicle_speed_m_s=-.3).validate()
    ba.SteerStepInput(dt_s=.01, mode='test_servo', target_delta_eq_rad=.4, vehicle_stopped=False).validate()
    ba.SteerStepInput(dt_s=1. / 120., mode='firmware', serial_in=b'C 1 250 102 102 -310*4a\n').validate()


def _output(**kw):
    base = dict(sim_time_s=.05, ticks=10, delta_left_rad=.21, delta_right_rad=.19, delta_eq_rad=.2,
                motor_side_angle_rad=.205, pot_raw=560, applied_steer_pwm=-.25, steer_en=True)
    base.update(kw)
    return ba.SteerStepOutput(**base)


def test_step_round_trip():
    inp = ba.SteerStepInput(dt_s=.01, mode='test_servo', target_delta_eq_rad=.2, vehicle_speed_m_s=.4,
                            vehicle_stopped=False, supply_scale=.99)
    frame = ba.encode_step_request(5, inp)
    assert len(frame) == 16 + 48 and frame[:4] == b'USTP'
    assert ba.decode_step_request(frame, expected_seq=5, expected_mode='test_servo') == (5, inp)
    out = _output(model_flags=ba.ModelFlag.IN_BACKLASH)
    frame = ba.encode_step_response(5, out)
    assert len(frame) == 16 + 80
    assert ba.decode_step_response(frame, 5) == out
    serial = ba.SteerStepInput(dt_s=.01, mode='firmware', serial_in=b'PING\n')
    frame = ba.encode_step_request(6, serial)
    assert len(frame) == 16 + 48 + 5 and ba.decode_step_request(frame, 6)[1] == serial
    reply = _output(fault_bits=ba.FaultBit.BOOT, drive_duty_left=.4, drive_duty_right=.4, drive_en=True,
                    serial_out=b'UBTS_STEER_V3 hello\n')
    assert ba.decode_step_response(ba.encode_step_response(6, reply), 6) == reply


def test_payload_layout_is_little_endian_and_fixed():
    inp = ba.SteerStepInput(dt_s=.01, steer_pwm=.5, steer_en=True, vehicle_speed_m_s=.4, vehicle_stopped=False,
                            supply_scale=1.)
    frame = ba.encode_step_request(1, inp)
    assert struct.unpack_from('<4sHHII', frame) == (b'USTP', 1, 0x0001, 1, 48)
    assert struct.unpack_from('<dddddBBBBI', frame, 16) == (.01, .5, 0., .4, 1., 0, 1, 0, 0, 0)
    frame = ba.encode_step_response(1, _output())
    assert struct.unpack_from('<4sHHII', frame) == (b'USTP', 1, 0x8001, 1, 80)
    values = struct.unpack_from('<ddddddddIHHHBBI', frame, 16)
    assert values == (.05, .21, .19, .2, .205, -.25, 0., 0., 10, 560, 0, 0, 1, 0, 0)


def test_reset_and_ready_round_trip():
    frame = ba.encode_reset_request(1, -.15)
    assert len(frame) == 16 + 8 and ba.decode_reset_request(frame, 1) == (1, -.15)
    out = _output(sim_time_s=0., ticks=0)
    assert ba.decode_reset_response(ba.encode_reset_response(1, out), 1) == out
    frame = ba.encode_ready(1000, 'steer-native 0.1 gcc-11.4')
    assert ba.decode_ready(frame, expected_tick_hz=1000) == (1000, 'steer-native 0.1 gcc-11.4')
    assert struct.unpack_from('<4sHHII', frame)[2:4] == (0x8000, 0)
    with pytest.raises(ba.ProtocolError) as info:
        ba.decode_ready(frame, expected_tick_hz=2000)
    assert info.value.exit_code == ba.EXIT_BAD_VALUE
    with pytest.raises(ValueError):
        ba.encode_reset_request(1, math.pi)


def _patched(frame, offset, fmt, value):
    data = bytearray(frame)
    struct.pack_into(fmt, data, offset, value)
    return bytes(data)


def test_frame_errors_and_exit_codes():
    frame = ba.encode_step_request(3, ba.SteerStepInput(dt_s=.01))
    cases = [
        (lambda: ba.decode_step_request(_patched(frame, 0, '<4s', b'XXXX')), ba.EXIT_BAD_HEADER),
        (lambda: ba.decode_step_request(_patched(frame, 4, '<H', 2)), ba.EXIT_BAD_HEADER),
        (lambda: ba.decode_step_request(_patched(frame, 6, '<H', 0x0007)), ba.EXIT_BAD_HEADER),
        (lambda: ba.decode_step_response(frame, 3), ba.EXIT_BAD_HEADER),
        (lambda: ba.decode_step_request(frame, expected_seq=4), ba.EXIT_SEQUENCE),
        (lambda: ba.decode_step_request(frame[:-1]), ba.EXIT_TRUNCATED),
        (lambda: ba.decode_step_request(frame[:10]), ba.EXIT_TRUNCATED),
        (lambda: ba.decode_step_request(frame + b'\x00'), ba.EXIT_BAD_LENGTH),
        (lambda: ba.decode_step_request(_patched(frame, 12, '<I', 40)), ba.EXIT_BAD_LENGTH),
        (lambda: ba.decode_step_request(_patched(frame, 16 + 43, '<B', 1)), ba.EXIT_BAD_VALUE),
        (lambda: ba.decode_step_request(_patched(frame, 16 + 40, '<B', 9)), ba.EXIT_BAD_VALUE),
        (lambda: ba.decode_step_request(_patched(frame, 16 + 41, '<B', 2)), ba.EXIT_BAD_VALUE),
        (lambda: ba.decode_step_request(_patched(frame, 16, '<d', float('nan'))), ba.EXIT_BAD_VALUE),
        (lambda: ba.decode_step_request(_patched(frame, 16 + 8, '<d', 1.5)), ba.EXIT_BAD_VALUE),
        (lambda: ba.decode_step_request(frame, expected_mode='firmware'), ba.EXIT_BAD_VALUE),
        (lambda: ba.decode_step_request(_patched(frame, 16 + 44, '<I', 3)), ba.EXIT_BAD_LENGTH),
    ]
    for call, code in cases:
        with pytest.raises(ba.ProtocolError) as info:
            call()
        assert info.value.exit_code == code, info.value
    response = ba.encode_step_response(3, _output())
    for offset, fmt, value in ((16 + 68, '<H', 1024), (16 + 70, '<H', 0x8000), (16 + 72, '<H', 1 << 9),
                               (16 + 74, '<B', 3), (16 + 40, '<d', 1.5)):
        with pytest.raises(ba.ProtocolError) as info:
            ba.decode_step_response(_patched(response, offset, fmt, value), 3)
        assert info.value.exit_code == ba.EXIT_BAD_VALUE
    assert set(ba.EXIT_CODES) == {0, 2, 3, 4, 5, 6, 7, 8, 9}


def test_read_frame_eof_rules():
    one = ba.encode_step_request(1, ba.SteerStepInput(dt_s=.01))
    two = ba.encode_reset_request(2, 0.)
    stream = io.BytesIO(one + two)
    assert ba.read_frame(stream) == one and ba.read_frame(stream) == two and ba.read_frame(stream) == b''
    for cut in (1, 15, 16, len(one) - 1):
        with pytest.raises(ba.ProtocolError) as info:
            ba.read_frame(io.BytesIO(one[:cut]))
        assert info.value.exit_code == ba.EXIT_TRUNCATED


def test_startup_config(profile):
    config = ba.native_startup_config(profile, 3)
    assert list(config) == ['protocol', 'mode', 'seed', 'mcu_tick_hz', 'profile_id', 'profile_sha256',
                            'profile_overrides', 'geometry', 'actuator', 'pot', 'test_servo', 'mcu', 'injections']
    assert config['protocol'] == ba.PROTOCOL and config['mode'] == 'direct_pwm' and config['mcu_tick_hz'] == 1000
    assert config['profile_sha256'] == profile.sha256 and config['profile_overrides'] is None
    assert config['actuator']['rate_max_rad_s'] == pytest.approx(math.radians(30.))
    assert config['actuator']['min_pwm'] == pytest.approx(.25)
    assert config['actuator']['backlash_rad'] == pytest.approx(math.radians(2.))
    assert config['pot']['range_rad'] == pytest.approx(math.radians(270.))
    assert config['pot']['center_raw'] == 512 and config['pot']['adc_bits'] == 10
    assert config['test_servo']['kp_per_rad'] == pytest.approx(.15 * 180. / math.pi)
    assert config['mcu']['faults']['track_error_rad'] == pytest.approx(math.radians(3.))
    draws = profile.sample(3, ba.PER_RUN_DRAW_NAMES)
    assert config['actuator']['zero_offset_rad'] == pytest.approx(math.radians(draws['steer_zero_offset_deg']))
    assert config['pot']['cal_residual_rad'] == pytest.approx(math.radians(draws['pot_cal_residual_deg']))
    geom = config['geometry']
    assert geom['eq_soft_limit_left_rad'] == pytest.approx(math.radians(profile.value('steer_soft_limit_eq_left_deg')))
    assert geom['eq_mech_limit_left_rad'] > geom['eq_soft_limit_left_rad']
    assert set(geom['wheel_mech_limit_rad']) == set(geometry.WHEEL_LIMIT_KEYS)
    assert ba.native_startup_config(profile, 3) == config
    assert ba.native_startup_config(profile, 4)['actuator']['zero_offset_rad'] != config['actuator']['zero_offset_rad']
    line = ba.startup_line(config)
    assert line.endswith(b'\n') and line.count(b'\n') == 1 and len(line) < ba.MAX_STARTUP_LINE_BYTES
    line.decode('ascii')
    assert json.loads(line) == json.loads(json.dumps(config))


def test_startup_config_options_and_errors(profile):
    servo = ba.native_startup_config(profile, 0, mode='test_servo', mcu_tick_hz=2000)
    assert servo['mode'] == 'test_servo' and servo['mcu_tick_hz'] == 2000
    for kwargs in (dict(mode='servo'), dict(mcu_tick_hz=1200), dict(mcu_tick_hz=1000.)):
        with pytest.raises(ValueError):
            ba.native_startup_config(profile, 0, **kwargs)
    for seed in (-1, True, 1.5, 2 ** 64):
        with pytest.raises(ValueError):
            ba.native_startup_config(profile, seed)
    stressed = profile.with_overrides({'steer_backlash_deg': 4.}, label='steer_backlash_x2')
    config = ba.native_startup_config(stressed, 0)
    assert config['actuator']['backlash_rad'] == pytest.approx(math.radians(4.))
    assert config['profile_overrides']['names'] == ['steer_backlash_deg']


def test_injections():
    good = [dict(kind='pot_open', at_s=1., params={}), dict(kind='slow', at_s=0., params={'rate_scale': .5}),
            dict(kind='pot_jump', at_s=2., params={'counts': 40, 'duration_s': .002}),
            dict(kind='driver_stuck_on', at_s=3., params={'pwm': -.6}),
            dict(kind='pot_slip', at_s=4., params={'offset_rad': math.radians(5.)})]
    assert ba.validate_injections(good) == good
    bad = [dict(kind='pot_melt', at_s=1., params={}), dict(kind='slow', at_s=1., params={}),
           dict(kind='slow', at_s=1., params={'rate_scale': 1.5}), dict(kind='pot_open', at_s=-1., params={}),
           dict(kind='pot_jump', at_s=1., params={'counts': 4.5, 'duration_s': .01}),
           dict(kind='driver_stuck_on', at_s=1., params={'pwm': 2.}), dict(kind='pot_open', params={}),
           dict(kind='pot_open', at_s=1., params={'extra': 1.})]
    for item in bad:
        with pytest.raises(ValueError):
            ba.validate_injections([item])


class _FakeBackend(ba.SteeringBackend):
    """In-memory double: passes every exchange through the wire codec; perfect instant steering."""

    def __init__(self):
        self.closed = False
        self.seq = 0
        self.delta = 0.
        self.time_s = 0.

    def configure(self, profile, seed):
        self.config = ba.native_startup_config(profile, seed)
        self.geom = geometry.SteeringGeometry.from_profile(profile)

    def _out(self, ticks):
        left, right = self.geom.wheel_angles(self.delta)
        return ba.SteerStepOutput(sim_time_s=self.time_s, ticks=ticks, delta_left_rad=left, delta_right_rad=right,
                                  delta_eq_rad=self.delta, motor_side_angle_rad=self.delta, pot_raw=512,
                                  applied_steer_pwm=0., steer_en=True)

    def reset(self, delta_eq_rad=0.):
        self.seq += 1
        _, self.delta = ba.decode_reset_request(ba.encode_reset_request(self.seq, delta_eq_rad), self.seq)
        self.time_s = 0.
        return ba.decode_reset_response(ba.encode_reset_response(self.seq, self._out(0)), self.seq)

    def step(self, inp):
        self.seq += 1
        _, inp = ba.decode_step_request(ba.encode_step_request(self.seq, inp), self.seq)
        ticks = int(round(inp.dt_s * self.config['mcu_tick_hz']))
        self.time_s += ticks / self.config['mcu_tick_hz']
        if inp.mode == 'test_servo':
            self.delta = self.geom.clamp_to_soft(inp.target_delta_eq_rad)
        return ba.decode_step_response(ba.encode_step_response(self.seq, self._out(ticks)), self.seq)

    def close(self):
        self.closed = True


def test_backend_interface(profile):
    with pytest.raises(TypeError):
        ba.SteeringBackend()
    with _FakeBackend() as backend:
        backend.configure(profile, 0)
        assert backend.reset(.1).delta_eq_rad == pytest.approx(.1)
        out = backend.step(ba.SteerStepInput(dt_s=.01, mode='test_servo', target_delta_eq_rad=1.))
        assert out.ticks == 10 and out.sim_time_s == pytest.approx(.01)
        assert out.delta_eq_rad == pytest.approx(math.radians(profile.value('steer_soft_limit_eq_left_deg')))
        assert out.delta_left_rad > out.delta_eq_rad > out.delta_right_rad
    assert backend.closed


# --- per-run values and truth fields --------------------------------------------------------------------------------
def test_startup_config_takes_fixed_per_run_values(profile):
    """RP-04: a scenario that fixes the zero offset / calibration residual (design 5.3) gives the same per-run value
    to the native startup line as to the L0 backend and the plant (VehicleProfile.per_run_value)."""
    fixed = profile.with_overrides({'steer_zero_offset_deg': 1.5}, label='steer_zero_p15')
    fixed = fixed.with_overrides({'pot_cal_residual_deg': -2.0}, label='pot_offset_m2', allow_out_of_range=True)
    for seed in (0, 9):
        config = ba.native_startup_config(fixed, seed)
        assert config['actuator']['zero_offset_rad'] == pytest.approx(math.radians(1.5), abs=1e-15)
        assert config['pot']['cal_residual_rad'] == pytest.approx(math.radians(-2.0), abs=1e-15)
    drawn = ba.native_startup_config(profile, 9)
    assert drawn['actuator']['zero_offset_rad'] == pytest.approx(
        math.radians(profile.per_run_value(9, 'steer_zero_offset_deg')), abs=1e-15)


def test_truth_output_fields():
    names = [f.name for f in dataclasses.fields(ba.SteerStepOutput)]
    assert set(ba.TRUTH_OUTPUT_FIELDS) <= set(names)
    assert 'applied_steer_pwm' in ba.TRUTH_OUTPUT_FIELDS and 'pot_raw' not in ba.TRUTH_OUTPUT_FIELDS
    assert 'fault_bits' not in ba.TRUTH_OUTPUT_FIELDS and 'steer_en' not in ba.TRUTH_OUTPUT_FIELDS


# --- the protocol document against the code (RP-02) --------------------------------------------------------------------
PROTOCOL_DOC = Path(ba.__file__).resolve().parents[3] / 'docs' / 'steering' / 'LOCKSTEP_PROTOCOL_KO.md'


@pytest.fixture(scope='module')
def protocol_doc():
    if not PROTOCOL_DOC.is_file():
        pytest.skip('LOCKSTEP_PROTOCOL_KO.md is not next to this checkout')
    return PROTOCOL_DOC.read_bytes().decode('utf-8')


def _between(text, start, end):
    i = text.index(start)
    return text[i:text.index(end, i + len(start))]


def _first_column_names(block):
    names = []
    for line in block.splitlines():
        if line.startswith('| `'):
            names.extend(re.findall(r'`([^`]+)`', line.split('|')[1]))
    return names


def _hex(text):
    return bytes(int(item, 16) for item in text.split())


def test_protocol_doc_hex_examples_match_the_encoder(protocol_doc):
    block = _between(protocol_doc, '### 4.5', '## 5.')
    step_hex = re.search(r'```\n(.*?)```', block, re.S).group(1)
    expected = ba.encode_step_request(1, ba.SteerStepInput(dt_s=.01, steer_pwm=.5, steer_en=True,
                                                           vehicle_speed_m_s=.4, vehicle_stopped=False,
                                                           supply_scale=1.))
    assert _hex(step_hex) == expected
    reset_hex = re.search(r'RESET 0\.0, seq 1: `([0-9a-f ]+)`', block).group(1)
    assert _hex(reset_hex) == ba.encode_reset_request(1, 0.)


def test_protocol_doc_startup_keys_match_native_startup_config(profile, protocol_doc):
    section = _between(protocol_doc, '## 2.', '## 3.')
    config = ba.native_startup_config(profile, 0)
    tables = (('맨 위 키:', '`geometry`:', None), ('`geometry`:', '`actuator`:', 'geometry'),
              ('`actuator`:', '`pot`:', 'actuator'), ('`pot`:', '`test_servo` (', 'pot'),
              ('`test_servo` (', '`mcu`:', 'test_servo'))
    for start, end, block in tables:
        names = _first_column_names(_between(section, start, end))
        expected = config if block is None else config[block]
        assert sorted(names) == sorted(expected), (block, sorted(set(names) ^ set(expected)))
    limit_row = next(line for line in section.splitlines() if line.startswith('| `wheel_mech_limit_rad`'))
    assert all('`%s`' % key in limit_row for key in geometry.WHEEL_LIMIT_KEYS)
    assert set(config['geometry']['wheel_mech_limit_rad']) == set(geometry.WHEEL_LIMIT_KEYS)


def test_protocol_doc_levels_truth_fields_and_exit_codes(profile, protocol_doc):
    section = _between(protocol_doc, '## 2.', '## 3.')
    level_row = next(line for line in section.splitlines() if line.startswith('| `level`'))
    levels = profile.param('steer_actuator_level')['range']['choices']
    assert all('`%s`' % level in level_row for level in levels)
    assert 'L1b' in level_row and '거부' in level_row                      # the v1 native refuses L1b (exit 2)
    truth = _between(protocol_doc, '참값 칸(', '\n')
    assert all('`%s`' % name in truth for name in ba.TRUTH_OUTPUT_FIELDS)
    codes = {}
    for line in _between(protocol_doc, '## 8.', '## 9.').splitlines():
        match = re.match(r'\| (\d+) \| (\w+) \|', line)
        if match:
            codes[int(match.group(1))] = match.group(2)
    assert codes == ba.EXIT_CODES
