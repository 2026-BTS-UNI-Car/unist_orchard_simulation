"""Opt-in SIM localization envelopes. No ROS, actuator or world-layout input."""
import json
import math
import os
from pathlib import Path

from encoderless.body_motion import LaserMount
from encoderless.body_velocity import VelocityCalibration
from encoderless.localized_motion import LocalizedMotion, localized_motion_error
from encoderless.shadow_pipeline import LiveSimOperatingProfile

SCHEMA = 'ubts.encoderless.localization.v1'
SOURCE = 'lidar_scanmatch_imu_heading'
LOCALIZATION = SOURCE + '_without_loop_closure'
MAX_ENVELOPE_BYTES = 262144


def runtime_mode(environment=None):
    value = (os.environ if environment is None else environment).get('UBTS_LOCALIZATION_MODE', 'legacy')
    if value not in ('legacy', 'encoderless_observe','native_sensors_observe'):
        raise ValueError('unknown_localization_mode')
    return value


def observed_map_enabled(environment=None):
    environment = os.environ if environment is None else environment
    value = environment.get('UBTS_ENCODERLESS_OBSERVED_MAP', '0')
    if value not in ('0', '1'):
        raise ValueError('unknown_observed_map_mode')
    if value == '1' and runtime_mode(environment) != 'encoderless_observe':
        raise ValueError('observed_map_requires_encoderless_observe')
    return value == '1'


def strict_json(raw, limit=MAX_ENVELOPE_BYTES):
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > limit:
        raise ValueError('bounded_json_required')
    def number(text):
        value = float(text)
        if not math.isfinite(value): raise ValueError('nonfinite_json')
        return value
    def constant(_): raise ValueError('nonfinite_json')
    return json.loads(raw, parse_float=number, parse_constant=constant)


def load_profile(path, run_id):
    """Read explicit externally reviewed assumptions; never inspect world layout.

    The supervisor verifies the fixture/evidence bytes before starting children.
    Different layouts can use the same algorithm with their own compatible
    profile. No layout, wheel data, spawn or truth is passed to the estimator.
    Calibration exists only to satisfy the unchanged inhibited pure pipeline;
    it is not an actuator permit, nor physical calibration.
    """
    value = strict_json(Path(path).read_text(encoding='utf-8'), 32768)
    if value.get('schema') != 'ubts.encoderless.runtime-profile.v1':
        raise ValueError('explicit_runtime_profile_required')
    fields = dict(value['profile'])
    if 'run_id' in fields: raise ValueError('run_id_is_process_identity')
    fields.update(run_id=run_id, mount=LaserMount(**fields['mount']))
    profile = LiveSimOperatingProfile(**fields)
    calibration = VelocityCalibration(**value['inhibited_calibration'])
    return profile, calibration, value


def _label(value): return isinstance(value, str) and 0 < len(value) <= 256
def _ns(value): return type(value) is int and 0 <= value < 2**63
def _finite(value):
    try:return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:return False


def validate_envelope(value):
    """Exact scan/pose endpoint, not a nearest or latest-message association."""
    if (not isinstance(value, dict) or value.get('schema') != SCHEMA
            or value.get('pose_source') != SOURCE or value.get('localization') != LOCALIZATION
            or not all(_label(value.get(k)) for k in ('session_id', 'localization_epoch', 'reason'))
            or type(value.get('localization_valid')) is not bool or not _ns(value.get('seq'))
            or value.get('runtime_authority') is not False or value.get('autonomy_eligible') is not False):
        raise ValueError('invalid_localization_envelope')
    if not value['localization_valid']:
        if 'pose' in value or 'scan' in value:
            raise ValueError('invalid_localization_has_no_pose')
        return value
    if not all(_ns(value.get(k)) for k in ('scan_stamp_ns', 'scan_receipt_ns', 'raw_receipt_sequence')):
        raise ValueError('original_scan_identity_required')
    pose, scan = value.get('pose'), value.get('scan')
    if (not isinstance(pose, dict) or not all(_finite(pose.get(k)) for k in ('x', 'y', 'yaw', 'stamp'))
            or pose.get('stamp_ns') != value['scan_stamp_ns']
            or pose.get('localization_epoch') != value['localization_epoch']
            or pose['stamp'] != value['scan_stamp_ns'] / 1e9):
        raise ValueError('exact_pose_endpoint_required')
    if (not isinstance(scan, dict) or scan.get('stamp_ns') != value['scan_stamp_ns']
            or scan.get('raw_receipt_sequence') != value['raw_receipt_sequence']
            or not all(_finite(scan.get(k)) for k in ('angle_min', 'angle_increment', 'range_min', 'range_max'))
            or scan['angle_increment'] <= 0 or not 0 <= scan['range_min'] < scan['range_max']
            or not isinstance(scan.get('ranges'), list) or not 16 <= len(scan['ranges']) <= 6400
            or any(v is not None and not _finite(v) for v in scan['ranges'])):
        raise ValueError('exact_scan_endpoint_required')
    return value


class PoseAccumulator:
    """One local frame. A geometry gap latches; no implicit global rebase."""
    def __init__(self, profile, epoch):
        self.profile, self.epoch = profile, epoch
        self.pose = (0., 0., 0.)
        self.last = None
        self.fault = None

    def commit(self, motion):
        if self.fault: raise ValueError(self.fault)
        error = ('invalid_localized_motion' if not isinstance(motion, LocalizedMotion)
                 else localized_motion_error(motion, profile=self.profile, clock=motion.evaluated_clock))
        if not error and motion.epoch != self.epoch: error = 'localization_epoch_changed'
        if not error and self.last is not None:
            if (motion.sequence != self.last.sequence + 1
                    or motion.previous_scan_stamp_ns != self.last.scan_stamp_ns):
                error = 'discontinuous_pose_increment'
        if error:
            self.fault = error
            raise ValueError(error)
        x, y, yaw = self.pose
        dx, dy = motion.delta_xy_base_m
        self.pose = (x + math.cos(yaw)*dx - math.sin(yaw)*dy,
                     y + math.sin(yaw)*dx + math.cos(yaw)*dy,
                     math.atan2(math.sin(yaw + motion.increment.delta_yaw_rad),
                                math.cos(yaw + motion.increment.delta_yaw_rad)))
        self.last = motion
        return dict(zip(('x', 'y', 'yaw'), self.pose), stamp_ns=motion.scan_stamp_ns,
                    stamp=motion.scan_stamp_ns/1e9, localization_epoch=self.epoch)


def base_envelope(profile, epoch, seq, valid, reason):
    return dict(schema=SCHEMA, session_id=profile.run_id, localization_epoch=epoch,
                seq=seq, localization_valid=valid, reason=reason, pose_source=SOURCE,
                localization=LOCALIZATION, profile_ref=profile.evidence_ref,
                runtime_authority=False, autonomy_eligible=False)
