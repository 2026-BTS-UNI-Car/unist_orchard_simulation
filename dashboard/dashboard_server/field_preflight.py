"""Read-only commissioning evidence. It NEVER grants actuator authority.

Source review and replay consistency are distinct from live accuracy, braking
or sensor mounting validation. Even a filled profile remains observe-only.
The dashboard cannot write this profile or enable motors through this module.
"""
import json
import math
from pathlib import Path


def positive(value):
    return type(value) in (int,float) and math.isfinite(value) and value>0


def report(path, live=None):
    try:profile=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    except (OSError,ValueError):profile={}
    if not isinstance(profile,dict):profile={}
    def section(name):
        value=profile.get(name);return value if isinstance(value,dict) else {}
    checks=[]
    def check(name,passed,reason):
        checks.append(dict(id=name,passed=passed is True,reason=None if passed is True else reason))
    check('profile',profile.get('schema')=='ubts.field_autonomy_preparation.v1'
          and profile.get('mode')=='observe_only','missing_or_invalid_observation_profile')
    check('native_sources',profile.get('wheel_encoders_present') is False
          and profile.get('localization_inputs')==['LiDAR','native_IMU','quality_GNSS']
          and profile.get('PWM_used_as_odometry') is False
          and profile.get('camera_used_for_mapping') is False,'source_contract_not_confirmed')
    body=section('body');mount=section('mount')
    check('body_dimensions',body.get('dimensions_measured') is True
          and positive(body.get('length_m')) and positive(body.get('width_m')),'current_body_dimensions_unmeasured')
    xy=mount.get('laser_xy_in_base_m');rotation=mount.get('imu_to_laser_rotation_rad')
    def vector(v,length,limit):
        return isinstance(v,list) and len(v)==length and all(type(x) in (int,float)
            and math.isfinite(x) and abs(x)<=limit for x in v)
    yaw=mount.get('laser_yaw_in_base_rad')
    check('sensor_extrinsics',mount.get('extrinsics_measured') is True
          and vector(xy,2,2.) and vector(rotation,3,math.pi)
          and type(yaw) in (int,float) and math.isfinite(yaw) and abs(yaw)<=math.pi,
          'current_laser_base_and_IMU_rotation_unmeasured')
    check('planar_envelope',mount.get('planar_envelope_validated') is True,'tilt_and_shock_envelope_unvalidated')
    prefs=section('test_preferences');brake=section('braking')
    check('body_velocity_feedback',prefs.get('body_velocity_feedback_calibrated') is True
          and positive(prefs.get('actuator_percent_limit')) and prefs['actuator_percent_limit']<=100,
          'body_linear_yaw_feedback_and_PWM_response_unvalidated')
    check('physical_emergency_stop',brake.get('physical_emergency_stop_verified') is True,'physical_emergency_stop_unverified')
    check('braking',all(positive(brake.get(k)) for k in ('measured_stop_distance_m','measured_stop_time_s','tested_speed_m_s'))
          and positive(prefs.get('proposed_linear_m_s')) and brake.get('tested_speed_m_s',0)>=prefs.get('proposed_linear_m_s',0),
          'measured_braking_at_requested_speed_required')
    for name in ('october_source_review_complete','september_orchard_review_complete',
                 'current_vehicle_localization_validated','live_latency_under_load_validated',
                 'obstacle_and_unknown_space_stop_validated','row_end_and_turn_space_validated',
                 'manual_takeover_verified','work_boundary_defined'):
        check(name,section('qualification').get(name) is True,name+'_pending')
    if live is not None:
        if not isinstance(live,dict):live={}
        d=live.get('diagnostic');d=d if isinstance(d,dict) else {}
        age=live.get('last_progress_age_s');queue=live.get('queue_scans')
        check('live_mapping_fresh',live.get('active') is True and live.get('state')=='drawing'
              and type(age) in (int,float) and 0<=age<=.2
              and d.get('accepted') is True and type(queue) is int and 0<=queue<=1,
              'live_mapping_not_fresh_and_qualified')
    blocking=[c['id'] for c in checks if not c['passed']]
    return dict(schema='ubts.field_preflight.v1',ok=True,state='PREPARATION_INCOMPLETE' if blocking else 'OBSERVATION_CHECKS_COMPLETE',
        checks=checks,blocking=blocking,runtime_authority=False,autonomy_output_enabled=False,
        motor_command=False,mode='observe_only',last_row_policy='unknown_until_verified_work_boundary',
        proposed_preferences=prefs,statement='관측 준비 점검 · 실차 출력 허가와 별도')
