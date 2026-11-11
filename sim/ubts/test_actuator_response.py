import unittest
from actuator_response import ActuatorResponse


class ResponseTest(unittest.TestCase):
    def test_zero_and_disabled_are_exact(self):
        plain=ActuatorResponse();self.assertEqual(plain.wheel_targets(.3,-.2,1.,0.),(.3,-.2))
        plant=ActuatorResponse({'enabled':True})
        for t in (0.,.17,100.):self.assertEqual(plant.wheel_targets(0.,0.,1.,t),(0.,0.))
    def test_pivot_requires_more_output_and_forward_drift_is_not_encoder_data(self):
        p=ActuatorResponse({'enabled':True})
        self.assertEqual(p.wheel_targets(.3,-.3,1.,0.),(0.,0.))
        self.assertGreater(sum(p.wheel_targets(.3,.3,1.,0.)),0.)
        a,b=p.wheel_targets(.8,.8,1.,0.);self.assertNotEqual(a,b)
        a,b=p.wheel_targets(.5,1.,1.,0.);self.assertLess(b-a,.5)
        self.assertFalse(p.status()['calibrated_speed_curve']);self.assertFalse(p.status()['braking_calibrated'])
    def test_sign_symmetry_and_bounded_outputs(self):
        p=ActuatorResponse({'enabled':True})
        for a,b in ((.6,.9),(-.6,.9),(.1,.2),(2.,-2.)):
            f=p.wheel_targets(a,b,1.,.2);r=p.wheel_targets(-a,-b,1.,.2)
            self.assertAlmostEqual(f[0],-r[0]);self.assertAlmostEqual(f[1],-r[1])
            self.assertLessEqual(max(map(abs,f)),1.)


if __name__=='__main__':unittest.main()
