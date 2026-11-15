"""The vehicle executor configuration for the ESP32-S3 bridge (Jetson, 2026-11-16; docs/steering/EXECUTOR_ESP32_KO.md).

vehicle_config(profile) builds the row executor configuration (tools/row_executor/config.ExecutorConfig) the Jetson kit
installs: front_steer, single-row mode, SHADOW (motor_output_enabled false), the vehicle control socket
/run/ubts-rover/control.sock, the operator page on 127.0.0.1 (only the dashboard on the same host reaches it), the
ESP32-S3 bridge (steer_bridge_kind esp32_abs_hold) and the values of the measured profile. A value goes in only when
its source is a measurement, the firmware or a definition; every field measurement that is still missing stays null
(the executor reports it as 'unknown' and refuses motor output until it is measured):

  from the profile front_steer_v1_spec_20261116 (provenance in brackets)
    steer_wheelbase_m               wheelbase_m [MEASURED_20261116]
    steer_limit_*_mrad              the host profile soft limit (steering limit 20 deg per wheel [MEASURED_20261116]
                                    minus the soft margin [CONFIG]) = 281 mrad
    steer_rate_limit_*_deg_s        0.8 x the board's step response 0 -> 20 (steer_step_time_measured_s
                                    [MEASURED_20261116], abs_hold.executor_values) = 4.075 deg/s. The rate itself
                                    (steer_measured_rate_*) stays null: the steering GO rule (>= steer_rate_min_deg_s 8,
                                    a placeholder) refuses 5.09 deg/s and the threshold is an open decision (the SIM
                                    re-check used 4.0, PREREG_STEER_FOLLOW_REALDIMS_KO.md)
    steer_servo_deadband_deg        the firmware target tolerance 10 counts [FIRMWARE] in equivalent degrees
    steer_esp32_cal                 the firmware CAL line RIGHT / CENTER / LEFT / MAX_COMMAND [FIRMWARE]
    steer_esp32_mark_deg            the equivalent angle of the command 20 = mechanical limit - 1 deg [SIM_ASSUMED; the
                                    field B3 measurement replaces it]
    lidar_x_in_base_m               [MEASURED_20261116]; lidar_yaw_offset_deg [ESTIMATED, the 10/10 mount value]
    footprint_length_m / _width_m   [SPEC_PENDING, the catalogue values]
    steer_rotation_centre_x_m       rear_axle_x_in_base_m [DEFINITION]
  left null (unknown until measured in the field): breakaway_duty_pct (C2), v_measured_m_s (C3),
    measured_stop_distance_m (C7), steer_measured_rate_*_deg_s (B5 + the GO threshold decision), steer_range_measured
    (B2 / B3), steer_wheel_soft_margin_deg, steer_min_pwm_pct (B4), steer_zero_offset_deg (F1), steer_polarity_checked
    (B0); steer_esp32_require_fw_watchdog stays true (the board firmware of 2026-11-16 has no drive watchdog).

CLI (from sim/ubts): python -B -m steering.vehicle_executor_config --out <file> [--profile <yaml>] [--tuning-file PATH]
--check <file> compares a written file with the generator (exit 1 when it differs).
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

PROFILE_ID = 'front_steer_v1_spec_20261116'
VEHICLE_SOCKET = '/run/ubts-rover/control.sock'
BIND_HOST = '127.0.0.1'
BIND_PORT = 8097
DEFAULT_TUNING_FILE = '/var/lib/row_executor/field_tuning.json'   # the kit's writable copy (the package default bytes)
REPO = Path(__file__).resolve().parents[3]


def _row_executor_config():
    tools = str(REPO / 'tools')
    if tools not in sys.path:
        sys.path.insert(0, tools)
    from row_executor import config as XC
    return XC


def load(profile_path=None):
    from .vehicle_profile import PROFILES_DIR, load_profile
    return load_profile(Path(profile_path) if profile_path else PROFILES_DIR / (PROFILE_ID + '.yaml'))


def vehicle_values(profile, tuning_file=DEFAULT_TUNING_FILE):
    """The ExecutorConfig fields of the vehicle configuration (module docstring) and their sources."""
    from . import abs_hold
    from . import host_profile_export as hpe
    XC = _row_executor_config()
    value = profile.value
    if not abs_hold.active(profile):
        raise ValueError('the profile does not run the ABS-HOLD-8 firmware (steer_fw_mode abs_hold_8)')
    limits = hpe.build(profile, cal_id=1)['steer_limit']
    measured = abs_hold.executor_values(profile)
    fw = value('abs_hold_fw')
    rate_limit = round(XC.STEER_RATE_FRACTION * measured['rate_deg_s'], 3)
    values = dict(
        vehicle_profile=XC.FRONT_STEER, mode=XC.SINGLE_ROW_MODE, require_native_navigation=False, profile='field',
        motor_output_enabled=False, max_yaw_rate_deg_s=3.6,
        socket_path=VEHICLE_SOCKET, bind_host=BIND_HOST, bind_port=BIND_PORT, tuning_file=tuning_file,
        steer_bridge_kind=XC.ESP32_BRIDGE_KIND, steer_allow_sim=False,
        steer_esp32_mark_deg=round(abs_hold.mark_angle_deg(profile), 3),
        steer_esp32_cal=(int(fw['cal_right']), int(fw['cal_center']), int(fw['cal_left']), int(fw['command_max_deg'])),
        steer_esp32_require_fw_watchdog=True,
        steer_wheelbase_m=float(value('wheelbase_m')),
        steer_limit_left_mrad=int(limits['left_mrad']), steer_limit_right_mrad=int(limits['right_mrad']),
        steer_rate_limit_deg_s=rate_limit, steer_rate_limit_standstill_deg_s=rate_limit,
        steer_servo_deadband_deg=measured['deadband_deg'],
        steer_rotation_centre_x_m=float(value('rear_axle_x_in_base_m')),
        lidar_yaw_offset_deg=float(value('lidar_yaw_offset_deg')), lidar_x_in_base_m=float(value('lidar_x_in_base_m')),
        footprint_length_m=float(value('footprint_length_m')), footprint_width_m=float(value('footprint_width_m')))
    sources = dict(
        steer_wheelbase_m=profile.provenance('wheelbase_m'), lidar_x_in_base_m=profile.provenance('lidar_x_in_base_m'),
        lidar_yaw_offset_deg=profile.provenance('lidar_yaw_offset_deg'),
        footprint_length_m=profile.provenance('footprint_length_m'),
        footprint_width_m=profile.provenance('footprint_width_m'),
        steer_rotation_centre_x_m=profile.provenance('rear_axle_x_in_base_m'),
        steer_rate_limit_deg_s='0.8 x board step response (steer_step_time_measured_s %s)'
                               % profile.provenance('steer_step_time_measured_s'),
        steer_servo_deadband_deg='firmware target tolerance (abs_hold_fw %s)' % profile.provenance('abs_hold_fw'),
        steer_esp32_cal='firmware CAL line (abs_hold_fw %s)' % profile.provenance('abs_hold_fw'),
        steer_esp32_mark_deg='SIM_ASSUMED: mechanical limit - 1 deg (field B3 replaces it)',
        steer_limit_left_mrad='host profile soft limit (steering limit and soft margin of the profile)',
        unmeasured='breakaway_duty_pct, v_measured_m_s, measured_stop_distance_m, steer_measured_rate_*, '
                   'steer_range_measured, steer_wheel_soft_margin_deg, steer_min_pwm_pct, steer_zero_offset_deg, '
                   'steer_polarity_checked: null / false until measured in the field')
    return values, sources


def vehicle_config(profile=None, tuning_file=DEFAULT_TUNING_FILE):
    XC = _row_executor_config()
    values, _ = vehicle_values(profile or load(), tuning_file)
    return replace(XC.ExecutorConfig(), **values).validate()


def render(profile=None, tuning_file=DEFAULT_TUNING_FILE):
    XC = _row_executor_config()
    return XC.dumps(vehicle_config(profile, tuning_file))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--profile', default=None)
    p.add_argument('--tuning-file', default=DEFAULT_TUNING_FILE)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--out')
    group.add_argument('--check')
    p.add_argument('--sources', action='store_true', help='print the value sources as JSON')
    args = p.parse_args(argv)
    profile = load(args.profile)
    text = render(profile, args.tuning_file)
    if args.sources:
        print(json.dumps(vehicle_values(profile, args.tuning_file)[1], indent=1, sort_keys=True))
    if args.check:
        same = Path(args.check).read_bytes() == text.encode('utf-8')
        print('%s: %s' % (args.check, 'same as the generator' if same else 'DIFFERS from the generator'))
        return 0 if same else 1
    out = Path(args.out)
    if out.exists():
        print('refused: %s exists' % out, file=sys.stderr)
        return 2
    out.write_bytes(text.encode('utf-8'))
    print('written %s (%d bytes)' % (out, len(text)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
