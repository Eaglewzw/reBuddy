"""Identical feedback traces and recovery math on native C and Python; no I/O."""
import ctypes
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from rs_gravity_play import Controller, KD, CAPS
from rs_recovery_policy import damping_gain


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder=tempfile.TemporaryDirectory();cls.addClassCleanup(cls.folder.cleanup)
        path=Path(cls.folder.name)
        (path/'policy.c').write_text('#include "rb_recovery_policy.h"\n'
            'float damping(float g,float v,float d,float c){return rb_recovery_kd(g,v,d,c);}\n'
            'float velocity(float p,float q,float dt){return rb_filtered_velocity(p,q,dt);}\n')
        subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror','-shared','-fPIC',
                        '-I'+str(ROOT/'components/rebuddy_arm/include'),str(path/'policy.c'),'-lm','-o',str(path/'policy.so')],check=True)
        cls.lib=ctypes.CDLL(str(path/'policy.so'))
        cls.lib.damping.argtypes=[ctypes.c_float]*4;cls.lib.damping.restype=ctypes.c_float
        cls.lib.velocity.argtypes=[ctypes.c_float]*3;cls.lib.velocity.restype=ctypes.c_float

    def test_damping_stays_in_original_budget_and_matches_c(self):
        rng=np.random.default_rng(20261102)
        for _ in range(1000):
            g=rng.uniform(-1,1,7)*CAPS;v=rng.uniform(-20,20,7)
            py=damping_gain(g,v,KD,CAPS)
            native=np.array([self.lib.damping(*map(float,x)) for x in zip(g,v,KD,CAPS)])
            np.testing.assert_allclose(native,py,atol=2e-6,rtol=2e-5)
            self.assertTrue(np.all(py>=0) and np.all(py<=KD))
            self.assertTrue(np.all(abs(g-py*v)<=CAPS+1e-8))

    def test_quantized_stationary_feedback_does_not_create_false_motion(self):
        ctrl=Controller(lambda q:np.zeros(6));q=np.ones(7)*.4
        stamp=1_000_000_000;ctrl.observe(q,stamp);native=np.zeros(7)
        quantum=4*np.pi/32767
        for i in range(300):
            measured=np.ones(7)*(.4+(quantum if i%2 else 0))
            dt=.02;stamp+=20_000_000
            native=np.array([self.lib.velocity(float(v),float(d),dt) for v,d in zip(native,measured-q)])
            ctrl.observe(measured,stamp);q=measured
            np.testing.assert_allclose(native,ctrl.velocity,atol=1e-7)
            self.assertLess(np.max(abs(ctrl.velocity)),np.radians(1))

    def test_recovery_preserves_gravity_and_fixed_hold_anchor(self):
        gravity=np.array([1.,2.,3.,.5,.3,.1])
        ctrl=Controller(lambda q:gravity);q=np.ones(7)*.4
        ctrl.velocity=np.array([4.,-4.,3.,2.,-2.,1.,-10.])
        command=ctrl.recovery_command(q,1_000_000_000)
        np.testing.assert_array_equal(command['kp'],np.zeros(7))
        np.testing.assert_array_equal(command['feedforward_nm'],np.r_[gravity,0.])
        self.assertTrue(np.all(abs(np.array(command['estimated_torque_nm']))<=CAPS))
        brake_kd=np.array(command['kd']);anchor=q.copy()
        for i in range(1,60):ctrl.observe(q,1_000_000_000+i*20_000_000)
        for i,gain in enumerate(np.linspace(0,1,26),60):
            command=ctrl.recovery_command(q+1e-5,1_000_000_000+i*20_000_000,anchor,gain,brake_kd)
            np.testing.assert_array_equal(command['target_rad'],anchor)
            np.testing.assert_array_equal(command['feedforward_nm'],np.r_[gravity,0.])
            self.assertTrue(np.all(abs(np.array(command['estimated_torque_nm']))<=CAPS))

    def test_burst_sampling_accumulates_and_real_slow_motion_is_retained(self):
        ctrl=Controller(lambda q:np.zeros(6));start=np.ones(7)*.4
        ctrl.observe(start,1_000_000_000)
        ctrl.observe(start+1e-5,1_001_000_000)
        self.assertEqual(ctrl.previous_ns,1_000_000_000)
        for i in range(1,100):ctrl.observe(start+i*.02*.01,1_000_000_000+i*20_000_000)
        np.testing.assert_allclose(ctrl.velocity,np.ones(7)*.01,atol=1e-6)


if __name__=='__main__':unittest.main()
