"""Sensor-native IMU presentation; no software calibration or rotation.

Native device fusion remains. Historical corrected archive fields are read as
recorded, never recomputed. Euler angles are display-only; raw data is preserved.
"""
from __future__ import annotations
import math
from typing import Any, Iterable

IMU_TIMELINE_SCHEMA = "orchard.imu_timeline.v1"
IMU_PROCESSING_REVISION = "sensor_native_raw_unmodified_v1"


def imu_processing_status() -> dict[str, Any]:
    return {"revision": IMU_PROCESSING_REVISION, "mode": "sensor_native_raw",
            "software_rotation": False, "start_reference": False, "manual_zero": False,
            "gyro_bias_correction": False, "compatibility_topic_semantics": "unchanged_raw_relay",
            "mount_validation": "unverified"}


def imu_zero_removed() -> dict[str, Any]:
    return {"ok": False, "state": "removed", "processing_mode": "sensor_native_raw",
            "error": "Software IMU zero/calibration was removed; raw sensor data is unchanged."}


def quaternion_to_euler_deg(values: Iterable[Any]) -> dict[str, float]:
    try:
        quaternion = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError):
        return {}
    if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
        return {}
    norm = math.hypot(*quaternion)
    if not math.isfinite(norm) or not math.isclose(norm, 1., rel_tol=1e-6, abs_tol=1e-6):
        return {}
    x, y, z, w = quaternion
    return {
        "roll": math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
        "pitch": math.degrees(math.asin(max(-1., min(1., 2*(w*y-z*x))))),
        "yaw": math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))),
    }


def raw_imu_payload(msg: Any) -> dict[str, Any]:
    """Copy native data, preserving -1 unavailable and zero unknown covariance.

    Invalid available values are rejected, never replaced with zero measurements.
    This is transport validation, not noise calibration or mount verification.
    """
    row = {
        "frame_id": str(msg.header.frame_id),
        "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
        "orientation": {axis: float(getattr(msg.orientation, axis)) for axis in ('x','y','z','w')},
        "angular_velocity": {axis: float(getattr(msg.angular_velocity, axis)) for axis in ('x','y','z')},
        "linear_acceleration": {axis: float(getattr(msg.linear_acceleration, axis)) for axis in ('x','y','z')},
        "covariance": {}, "availability": {},
        "source": "ros2_sensor_msgs_imu_raw", "processing": imu_processing_status(),
    }
    if not row['frame_id'] or not 0 <= row['stamp']['nanosec'] < 1_000_000_000:
        raise ValueError('invalid IMU frame or stamp')
    for group in ('orientation','angular_velocity','linear_acceleration'):
        values = row[group]
        covariance = [float(value) for value in getattr(msg, group+'_covariance')]
        if len(covariance) != 9 or not all(math.isfinite(value) for value in covariance):
            raise ValueError('invalid IMU covariance')
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError('non-finite IMU message')
        available = covariance[0] != -1.
        if available and group == 'orientation' and not quaternion_to_euler_deg(values.values()):
            raise ValueError('invalid available IMU quaternion')
        row['covariance'][group] = covariance
        row['availability'][group] = {
            'available': available,
            'uncertainty': 'unavailable' if not available else
                'unknown' if all(value == 0. for value in covariance) else 'reported',
        }
    row['euler_deg'] = (quaternion_to_euler_deg(row['orientation'].values())
                        if row['availability']['orientation']['available'] else {})
    return row
