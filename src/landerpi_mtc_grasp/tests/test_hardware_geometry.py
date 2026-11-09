import sys
from pathlib import Path
import unittest
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from hardware_geometry import to_angle,to_pulse,validate,interpolate
from depth_registration import register

class HardwareGeometryTest(unittest.TestCase):
    def test_calibrated_mapping_and_reverse(self):
        for config in [dict(init=500,min=0,max=1000),dict(init=220,min=1000,max=0)]:
            self.assertAlmostEqual(to_angle(to_pulse(-.5,config),config),-.5,delta=.003)
            with self.assertRaises(ValueError):to_pulse(10.,config)
            with self.assertRaises(ValueError):to_angle(-1,config)
    def trajectory(self):
        return NS(joint_names=['joint1'],points=[NS(positions=[0.],time_from_start=NS(sec=0,nanosec=0)),
            NS(positions=[.3],time_from_start=NS(sec=1,nanosec=0))])
    def test_validated_interpolation(self):
        trajectory=self.trajectory();config={'joint1':dict(init=500,min=0,max=1000)}
        validate(trajectory,['joint1'],config,{'joint1':0.})
        self.assertAlmostEqual(interpolate(trajectory.points,[0.],.5)[0],.15)
        self.assertEqual(interpolate(trajectory.points,[0.],2.),[.3])
        trajectory.points[-1].time_from_start.sec=0
        with self.assertRaises(ValueError):validate(trajectory,['joint1'],config,{'joint1':0.})
    def test_bad_joint_and_jump_rejected(self):
        t=self.trajectory();c={'joint1':dict(init=500,min=0,max=1000)}
        with self.assertRaises(ValueError):validate(t,['joint2'],c,{'joint1':0.})
        t.points[0].positions=[.2]
        with self.assertRaises(ValueError):validate(t,['joint1'],c,{'joint1':0.})
        t.points[0].positions=[0.];t.points[-1].positions=[1.5]
        with self.assertRaises(ValueError):validate(t,['joint1'],c,{'joint1':0.})
    def test_registration_identity_and_missing_depth(self):
        depth=np.array([[1.,0.],[float('nan'),2.]],dtype=np.float32);k=np.array([100.,0,0,0,100.,0,0,0,1])
        result=register(depth,k,k,np.eye(3),[0,0,0],2,2)
        np.testing.assert_allclose(result,[[1.,0],[0,2.]])
    def test_registration_shift_and_occlusion(self):
        k=np.array([10.,0,0,0,10.,0,0,0,1])
        result=register(np.array([[1.,2.]],dtype=np.float32),k,k,np.eye(3),[.1,0,0],1,4)
        self.assertEqual(result[0,1],1.)
        self.assertEqual(result[0,2],2.)
if __name__=='__main__':unittest.main()
