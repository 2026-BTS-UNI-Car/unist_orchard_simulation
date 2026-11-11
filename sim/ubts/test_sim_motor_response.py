"""Exercise the actual SimMotor methods without importing a ROS graph."""
import ast,unittest
from pathlib import Path
from types import SimpleNamespace
from actuator_response import ActuatorResponse

source=Path(__file__).with_name('adapter.py').read_text()
tree=ast.parse(source);motor=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SimMotor')
class Twist:
    def __init__(self):self.linear=SimpleNamespace(x=0);self.angular=SimpleNamespace(z=0)
scope=dict(ActuatorResponse=ActuatorResponse,Twist=Twist)
exec(compile(ast.Module(body=[motor],type_ignores=[]),'adapter.SimMotor','exec'),scope)


class MotorContractTest(unittest.TestCase):
    def test_pwm_is_not_effective_wheel_speed_and_stop_bypasses_ramp(self):
        messages=[];node=SimpleNamespace(sim_time=1.,command=SimpleNamespace(publish=messages.append))
        config=dict(max_wheel_speed_m_s=1.,track_m=.56,acceleration_m_s2=1.5,actuator_response={'enabled':True})
        m=scope['SimMotor'](node,config);m.applied=[.8,.8];m.publish()
        self.assertEqual(m.outputs,dict(D8=204,D5=0,D3=204,D0=0))
        self.assertNotEqual(m.effective,m.applied);self.assertNotEqual(messages[-1].angular.z,0)
        m.hard_stop();self.assertEqual(m.effective,[0.,0.]);self.assertEqual(m.applied,[0.,0.])
        self.assertTrue(all(v==0 for v in m.outputs.values()))
        self.assertEqual(messages[-1].linear.x,0);self.assertEqual(messages[-1].angular.z,0)

    def test_field_profile_uses_measured_mcu_ramp_with_reversal_break(self):
        messages=[];node=SimpleNamespace(sim_time=0.,command=SimpleNamespace(publish=messages.append))
        config=dict(max_wheel_speed_m_s=1.,track_m=.56,acceleration_m_s2=1.5,
                    actuator_response={'enabled':True,'profile':'field_20261010'})
        m=scope['SimMotor'](node,config);m.drive(100,100)
        for _ in range(25):node.sim_time+=.02;m.step()
        self.assertAlmostEqual(m.applied[0],.5,places=6)   # 100%/s, not 1.5 m/s^2
        m.applied=[1.,1.];m.drive(-100,-100)
        for _ in range(50):node.sim_time+=.02;m.step()
        self.assertEqual(m.applied,[0.,0.])                  # faded to zero, now in the dead time
        for _ in range(3):node.sim_time+=.02;m.step()
        self.assertEqual(m.applied,[0.,0.])
        for _ in range(10):node.sim_time+=.02;m.step()
        self.assertLess(m.applied[0],0.)
        self.assertEqual(m.response.status()['profile'],'field_20261010')


if __name__=='__main__':unittest.main()
