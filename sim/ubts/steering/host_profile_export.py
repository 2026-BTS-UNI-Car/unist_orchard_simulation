"""Export the host-side values of a vehicle profile as a small JSON file for the bridge (P3, design 4.2).

The host bridge (dashboard/scripts/steer_*.py) must not import sim code, so it reads this JSON by path
(steer_rover_bridge.py --host-profile, steer_bridge_core.HostProfile). Schema ubts.steer_host_profile.v1:

  vehicle_profile, hw_profile           arm needs "front_steer"; the banner hw must equal hw_profile
  source_profile                        profile_id, profile_sha256, provenance grade, result kind, overrides
  steer_limit                           equivalent soft limit per direction (rad exact, mrad floored to int)
  calibration                           cal_id and the steer_mrad <-> permille table: the SIM calibration marks of
                                        LOCKSTEP_PROTOCOL_KO.md 11.11 (eq mechanical limit - 1 deg, centre 0);
                                        the expected MCU soft limits in permille and the scale s (permille/deg)
  link, stop, arm, host_faults, servo   timing and thresholds (status rate, C TTL and resend, PING, watchdog,
                                        STOP confirm/resend, holds, status gap, n gaps, track mismatch,
                                        steering_ineffective, deadband, max PWM)
  drive, vehicle                        drive_lr_mode (only equal_duty is supported) and the wheelbase
  provenance                            where every value comes from (profile names and tags, or a document)

All values are placeholders of the profile; a file made from the placeholder profile is for placeholder-based SIM
results only. In the field the calibration table comes from the measured marks (B2, B3), not from this rule.

Usage (from sim/ubts): python -B -m steering.host_profile_export --out <file> [--profile <yaml>] [--cal-id N|any]
Pure Python + the profile loader. No numpy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

from . import geometry
from . import vehicle_profile as vp

SCHEMA = 'ubts.steer_host_profile.v1'
VEHICLE_PROFILE = 'front_steer'
HW_PROFILE = 'front_steer_v0'
MARK_INSET_DEG = 1.0          # LOCKSTEP_PROTOCOL_KO.md 11.11 step 1
PING_HZ = 10.0                # design 4.1 period table: PING 10 Hz while connected
BANNER_TIMEOUT_S = 1.5        # ESP_PROTOCOL_V3_DRAFT_KO.md 10.7 placeholder
ARM_CONFIRM_PERIODS = 3       # proposal: ARM confirmation window = 3 status periods (= host status gap)
DEFAULT_CAL_ID = 1            # LOCKSTEP_PROTOCOL_KO.md 11.11 step 6 (scene default)


class ExportError(ValueError):
    pass


def _round_half_up(x):
    return int(math.floor(x + 0.5))


def _int_ms(value, name):
    if abs(value - round(value)) > 1e-9:
        raise ExportError('%s=%r is not a whole number of ms' % (name, value))
    return int(round(value))


def build(profile, *, cal_id=DEFAULT_CAL_ID, hw_profile=HW_PROFILE):
    """Host profile dict for a VehicleProfile. cal_id None means the host accepts any calibration id."""
    if cal_id is not None and (isinstance(cal_id, bool) or not isinstance(cal_id, int) or not 1 <= cal_id <= 65535):
        raise ExportError('cal_id must be None or an int 1..65535')
    v = profile.value
    if v('drive_lr_mode') != 'equal_duty':
        raise ExportError('drive_lr_mode %r: the host bridge supports equal_duty only (electronic diff is OFF)'
                          % v('drive_lr_mode'))
    geom = geometry.SteeringGeometry.from_profile(profile)
    soft_left, soft_right = geom.soft_limits_eq()
    mech_left, mech_right = geom.mech_limits_eq()
    inset = math.radians(MARK_INSET_DEG)
    mark_left, mark_right = mech_left - inset, mech_right - inset
    if not (mark_left > soft_left > 0 and mark_right > soft_right > 0):
        raise ExportError('calibration marks must lie outside the soft limits')
    scale = 1000.0 / ((math.degrees(mark_left) + math.degrees(mark_right)) / 2.0)
    status_hz = v('status_hz')
    label = profile.result_label()
    stop_ms = {key: _int_ms(v(name) * 1000.0, name) for key, name in (
        ('hold_after_stop_ms', 'steer_hold_after_stop_s'), ('hold_after_disarm_ms', 'steer_hold_after_disarm_s'),
        ('hold_after_loss_ms', 'steer_hold_after_loss_s'))}
    data = {
        'schema': SCHEMA,
        'vehicle_profile': VEHICLE_PROFILE,
        'hw_profile': hw_profile,
        'source_profile': {'profile_id': profile.profile_id, 'profile_sha256': profile.sha256,
                           'provenance_grade': label['provenance_grade'], 'result_kind': label['result_kind'],
                           'field_validated': False, 'overrides': label['overrides']},
        'steer_limit': {'left_rad': soft_left, 'right_rad': soft_right,
                        'left_mrad': int(math.floor(soft_left * 1000.0)),
                        'right_mrad': int(math.floor(soft_right * 1000.0))},
        'calibration': {'cal_id': cal_id, 'mark_left_mrad': mark_left * 1000.0, 'mark_right_mrad': mark_right * 1000.0,
                        'permille_per_deg': scale,
                        'expected_soft_permille': {'left': _round_half_up(1000.0 * soft_left / mark_left),
                                                   'right': -_round_half_up(1000.0 * soft_right / mark_right)},
                        'rule': 'SIM marks: +-(equivalent mechanical limit - %g deg), centre 0 '
                                '(LOCKSTEP_PROTOCOL_KO.md 11.11); field: measured B2/B3 marks' % MARK_INSET_DEG},
        'link': {'baud': v('baud'), 'status_hz': status_hz, 'c_ttl_max_ms': _int_ms(v('c_ttl_max_ms'), 'c_ttl_max_ms'),
                 'c_resend_hz': float(v('c_resend_hz')), 'ping_hz': PING_HZ,
                 'watchdog_ms': _int_ms(v('watchdog_ms'), 'watchdog_ms'), 'banner_timeout_s': BANNER_TIMEOUT_S},
        'stop': dict(stop_ms, confirm_periods=v('stop_confirm_periods'),
                     resend_interval_ms=float(v('stop_resend_interval_ms')), resend_max=v('stop_resend_max'),
                     steer_policy=v('stop_policy_steer'), watchdog_steer_mode=v('steer_watchdog_mode'),
                     ttl_expiry_drive_mode=v('ttl_expiry_drive_mode')),
        'arm': {'confirm_ms': ARM_CONFIRM_PERIODS * 1000.0 / status_hz},
        'host_faults': {'status_gap_ms': float(v('host_status_gap_ms')),
                        'seq_gap_missing': v('host_seq_gap')['missing'],
                        'seq_gap_window_s': float(v('host_seq_gap')['window_s']),
                        'track_error_deg': float(v('host_track_mismatch')['error_deg']),
                        'track_duration_s': float(v('host_track_mismatch')['duration_s']),
                        'ineffective': {key: float(value) for key, value in v('steering_ineffective').items()}},
        'servo': {'deadband_permille': _round_half_up(v('servo_deadband_deg') * scale),
                  'max_pwm': _round_half_up(v('servo_max_pwm_pct') * 255.0 / 100.0)},
        'drive': {'lr_mode': v('drive_lr_mode')},
        'vehicle': {'wheelbase_m': float(v('wheelbase_m'))},
    }
    if profile.has('steer_monitor_stack') and v('steer_monitor_stack') == 'esp32_r5':
        # schema v6: steering_not_following as the vehicle's ESP32 stack counts it (steer_bridge_core TRACK_RULES);
        # without the key (every earlier profile) the host profile keeps its bytes
        data['host_faults']['track_rule'] = 'esp32_adapter'
    data['provenance'] = _provenance(profile)
    check_timing_relations(data)
    return data


def check_timing_relations(data):
    """The relations the host loader checks (steer_bridge_core.HostProfile, HOST_BRIDGE_V3_KO.md 6): a PING at least
    twice per MCU watchdog window, a C resend before the C ttl ends, a status gap and an ARM confirmation of at least
    two status periods. A profile whose values pass one by one but not together is refused here already."""
    link, faults, arm = data['link'], data['host_faults'], data['arm']
    period_ms = 1000.0 / link['status_hz']
    for ok, text in ((1000.0 / link['ping_hz'] <= link['watchdog_ms'] / 2.0, 'PING period above watchdog_ms / 2'),
                     (1000.0 / link['c_resend_hz'] < link['c_ttl_max_ms'], 'C resend period not below c_ttl_max_ms'),
                     (faults['status_gap_ms'] >= 2.0 * period_ms, 'host_status_gap_ms below two status periods'),
                     (arm['confirm_ms'] >= 2.0 * period_ms, 'ARM confirmation below two status periods')):
        if not ok:
            raise ExportError('host profile timing: %s (status_hz %r, watchdog_ms %r, c_ttl_max_ms %r)' % (
                text, link['status_hz'], link['watchdog_ms'], link['c_ttl_max_ms']))


_PROVENANCE_SOURCES = {
    'steer_limit': ('steer_soft_limit_eq_left_deg', 'steer_soft_limit_eq_right_deg'),
    'calibration.mark_left_mrad': ('steer_mech_limit_deg', 'steer_ackermann_ratio', 'wheelbase_m', 'kingpin_width_m',
                                   'steer_type'),
    'calibration.mark_right_mrad': ('steer_mech_limit_deg', 'steer_ackermann_ratio', 'wheelbase_m',
                                    'kingpin_width_m', 'steer_type'),
    'link.baud': ('baud',), 'link.status_hz': ('status_hz',), 'link.c_ttl_max_ms': ('c_ttl_max_ms',),
    'link.c_resend_hz': ('c_resend_hz',), 'link.watchdog_ms': ('watchdog_ms',),
    'stop.confirm_periods': ('stop_confirm_periods',), 'stop.resend_interval_ms': ('stop_resend_interval_ms',),
    'stop.resend_max': ('stop_resend_max',), 'stop.steer_policy': ('stop_policy_steer',),
    'stop.hold_after_stop_ms': ('steer_hold_after_stop_s',), 'stop.hold_after_disarm_ms': ('steer_hold_after_disarm_s',),
    'stop.hold_after_loss_ms': ('steer_hold_after_loss_s',), 'stop.watchdog_steer_mode': ('steer_watchdog_mode',),
    'stop.ttl_expiry_drive_mode': ('ttl_expiry_drive_mode',),
    'host_faults.status_gap_ms': ('host_status_gap_ms',), 'host_faults.seq_gap_missing': ('host_seq_gap',),
    'host_faults.seq_gap_window_s': ('host_seq_gap',), 'host_faults.track_error_deg': ('host_track_mismatch',),
    'host_faults.track_duration_s': ('host_track_mismatch',), 'host_faults.ineffective': ('steering_ineffective',),
    'servo.deadband_permille': ('servo_deadband_deg', 'steer_mech_limit_deg'),
    'servo.max_pwm': ('servo_max_pwm_pct',), 'drive.lr_mode': ('drive_lr_mode',), 'vehicle.wheelbase_m': ('wheelbase_m',),
}
_DOCUMENT_SOURCES = {
    'calibration.cal_id': 'LOCKSTEP_PROTOCOL_KO.md 11.11 step 6 (scene value, default 1) or --cal-id',
    'calibration.rule': 'LOCKSTEP_PROTOCOL_KO.md 11.11',
    'calibration.permille_per_deg': 'LOCKSTEP_PROTOCOL_KO.md 11.11 step 3 (from the marks)',
    'calibration.expected_soft_permille': 'LOCKSTEP_PROTOCOL_KO.md 11.11 step 4 (from the soft limits and marks)',
    'link.ping_hz': 'DESIGN_v1_KO.md 4.1 period table (PING 10 Hz while connected)',
    'link.banner_timeout_s': 'ESP_PROTOCOL_V3_DRAFT_KO.md 10.7 (placeholder 1.5 s)',
    'arm.confirm_ms': 'host proposal: %d status periods' % ARM_CONFIRM_PERIODS,
    'hw_profile': 'ESP_PROTOCOL_V3_DRAFT_KO.md 10.4 (UBTS_STEER_HW compile constant)',
    'vehicle_profile': 'DESIGN_v1_KO.md 4.2 (arm needs vehicle_profile front_steer)',
}


def _provenance(profile):
    out = {}
    for key, names in _PROVENANCE_SOURCES.items():
        out[key] = {'profile_names': list(names), 'tags': sorted({profile.provenance(name) for name in names})}
    for key, text in _DOCUMENT_SOURCES.items():
        out[key] = {'document': text}
    if profile.has('steer_monitor_stack') and profile.value('steer_monitor_stack') == 'esp32_r5':
        out['host_faults.track_rule'] = {'profile_names': ['steer_monitor_stack'],
                                         'tags': [profile.provenance('steer_monitor_stack')]}
    return dict(sorted(out.items()))


def to_bytes(data):
    """Canonical file bytes: UTF-8 JSON, sorted keys, one-space indent, LF line ends, final LF."""
    text = json.dumps(data, ensure_ascii=False, sort_keys=True, indent=1, allow_nan=False)
    return (text + '\n').encode('utf-8')


def sha256(data_bytes):
    return hashlib.sha256(data_bytes).hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser(prog='host_profile_export', description='write the host profile JSON')
    ap.add_argument('--out', required=True, help='output JSON path (outside the work tree)')
    ap.add_argument('--profile', default=None, help='vehicle profile YAML (default: the shipped placeholder)')
    ap.add_argument('--cal-id', default=str(DEFAULT_CAL_ID), help='expected MCU cal_id, or "any"')
    a = ap.parse_args(argv)
    cal_id = None if a.cal_id == 'any' else int(a.cal_id)
    profile = vp.load_profile(a.profile)
    raw = to_bytes(build(profile, cal_id=cal_id))
    out = Path(a.out)
    out.write_bytes(raw)
    print(json.dumps({'out': str(out), 'sha256': sha256(raw), 'bytes': len(raw), 'profile_id': profile.profile_id,
                      'profile_sha256': profile.sha256}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
