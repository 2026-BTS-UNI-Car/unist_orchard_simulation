"""Configurable field-observed stressors, not a calibrated motor-speed curve.

No encoder/pose/GPS feedback enters this plant model. PWM reporting stays the
requested/ramped electrical command; effective wheel targets are independent
simulation assumptions. Zero output never creates drift or stick-slip motion.

``profile`` selects the response: ``qualitative_stress`` (default, the
coefficients below) or ``field_20261010`` (field_response.py: measured
>=90% duty points, MCU ramp/reversal break, pivot breakaway/stall; low-duty
behaviour still SIM_ASSUMED). The field profile is not validated in Gazebo.
"""
import math

PROFILES = ('qualitative_stress', 'field_20261010')


class ActuatorResponse:
    DEFAULTS=dict(forward_deadband=.12,pivot_deadband=.40,
        moving_turn_efficiency=.55,pivot_efficiency=.75,
        left_gain=.99,right_gain=1.,pivot_stick_slip_fraction=.10,pivot_period_s=.8)

    def __init__(self,config=None,track_m=.56):
        config=config or {};self.enabled=config.get('enabled',False)
        self.profile=config.get('profile','qualitative_stress');self.field=None
        if self.profile not in PROFILES:raise ValueError('unknown_actuator_profile')
        if self.profile=='field_20261010' and self.enabled:
            from field_response import FieldActuatorProfile
            self.field=FieldActuatorProfile(dict(config.get('field_20261010') or {},track_m=float(track_m)))
        self.parameters={k:float(config.get(k,v)) for k,v in self.DEFAULTS.items()}
        p=self.parameters
        if not isinstance(self.enabled,bool) or not all(math.isfinite(v) for v in p.values()):raise ValueError('invalid_actuator_response')
        if not 0<=p['forward_deadband']<p['pivot_deadband']<.8:raise ValueError('invalid_actuator_deadband')
        if not all(0<p[k]<=1 for k in ('moving_turn_efficiency','pivot_efficiency','left_gain','right_gain')):raise ValueError('invalid_actuator_gain')
        if not 0<=p['pivot_stick_slip_fraction']<=.3 or not .2<=p['pivot_period_s']<=4.:raise ValueError('invalid_stick_slip')

    def ramp_step(self,applied,target,dt,maximum):
        """MEASURED MCU ramp for the field profile; None keeps SimMotor's ramp."""
        return None if self.field is None else self.field.ramp_step(applied,target,dt,maximum)

    def wheel_targets(self,left,right,maximum,stamp):
        if not self.enabled:return left,right
        if self.field is not None:return self.field.wheel_targets(left,right,maximum,stamp)
        if not all(math.isfinite(v) for v in (left,right,maximum,stamp)) or maximum<=0:raise ValueError('invalid_actuator_input')
        left,right=max(-maximum,min(maximum,left)),max(-maximum,min(maximum,right))
        pivot=left*right<0; p=self.parameters;dead=p['pivot_deadband'] if pivot else p['forward_deadband']
        def wheel(value,gain):return math.copysign(max(0.,abs(value)/maximum-dead)/(1-dead)*maximum*gain,value)
        l=wheel(left,p['left_gain']);r=wheel(right,p['right_gain'])
        mean=(l+r)/2;difference=(r-l)/2
        if pivot:
            # Command-side breakaway/stick-slip approximation, not extra
            # synthetic IMU acceleration or a claimed measured yaw vibration.
            difference*=p['pivot_efficiency']*(1-p['pivot_stick_slip_fraction']*(.5+.5*math.sin(2*math.pi*stamp/p['pivot_period_s'])))
        else:difference*=p['moving_turn_efficiency']
        return mean-difference,mean+difference

    def status(self):
        if self.field is not None:return self.field.status()
        return dict(enabled=self.enabled,profile=self.profile,parameters=self.parameters,
            provenance='qualitative_field_stress_profile_unmeasured_coefficients',
            calibrated_speed_curve=False,braking_calibrated=False)
