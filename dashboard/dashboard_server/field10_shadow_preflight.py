"""새 dashboard 이미지의 생성 입력 API 확인. ROS/네트워크/모터를 열지 않는다."""
import json,math
from .field10_shadow import Field10ShadowPlanner
from .autonomy_observation import checked_decision
def check():
    context=dict(source_session='generated_module_check',epoch='generated_epoch',generation=1,
        source_clock_domain='ROS_SYSTEM_TIME',receipt_clock_domain='CLOCK_MONOTONIC_RAW:generated_preflight')
    p=Field10ShadowPlanner();p.reset(context)
    r=dict(context=context,receipt_now_ns=10_001_000_000,source_now_ns=1_700_000_000_001_000_000,
        scan=dict(original_native=True,sequence=1,stamp_ns=1_700_000_000_000_000_000,receipt_ns=10_000_000_000,
            frame_id='laser',ranges=[4.]*64,angle_min=-math.pi,angle_increment=2*math.pi/64,range_min=.05,range_max=12.),
        imu=dict(original_native=True,stamp_ns=1_699_999_999_999_000_000,receipt_ns=9_999_000_000,frame_id='imu_link',
            angular_velocity_xyz=[.01,.02,.03],linear_acceleration_xyz=[.1,.2,9.81]))
    a=checked_decision(p.plan(r));assert a['reason']=='FIELD10_ADAPTER_UNQUALIFIED' and a['decision']=='HOLD'
    assert p.loop.counts['scans_processed']==p.loop.counts['imu_fed']==1
    assert not p.loop.motor_allowed and p.loop.bridge is None and p.loop._stop_transport is None
    assert not p._gate.test_only and p.loop.config.require_native_navigation
    p.stop();assert p.plan(r)['decision']=='STOP' and not p.reanchor_after_expiry(context)
    return dict(schema='ubts.field10_C0_shadow_import_preflight.v1',ok=True,generated_only=True,
        latest_row_module_imported=True,raw_scan_and_IMU_API_processed=True,native_navigation='ADAPTER_UNQUALIFIED',
        motor_output_enabled=False,bridge_created=False,stop_transport_created=False,SDK_ROS_Node_started=False,
        actual_physical_sensor_input_observed=False,physical_autonomy_qualified=False)
if __name__=='__main__':print(json.dumps(check(),sort_keys=True))
