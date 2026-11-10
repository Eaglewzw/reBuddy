"""Compare compiled C5 polynomial evaluation to the actual PC plans, no CAN."""
import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools'))
import export_daily_c5 as exporter
from rs_daily_prepare import load_bundle
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF

class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.addClassCleanup(cls.tmp.cleanup)
        lib=Path(cls.tmp.name)/'daily.so';core=ROOT/'components/rebuddy_core'
        subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror',
                        '-shared','-fPIC','-O2',f'-I{core}/include',str(core/'src/rb_daily.c'),
                        str(core/'src/rb_daily_data.c'),'-lm','-o',str(lib)],check=True)
        cls.lib=ctypes.CDLL(str(lib));cls.sample=cls.lib.rb_daily_sample
        ptr=ctypes.POINTER(ctypes.c_float)
        cls.sample.argtypes=[ctypes.c_uint,ctypes.c_float,ptr,ptr,ptr];cls.sample.restype=ctypes.c_bool
        cls.manifest,cls.plans,_,_=load_bundle(DEFAULT_CALIBRATION,DEFAULT_URDF)

    def test_export_is_current_and_identity_bound(self):
        code,report=exporter.render()
        self.assertEqual(exporter.OUTPUT.read_text(),code)
        self.assertEqual(exporter.REPORT.read_text(),report)
        r=json.loads(report);self.assertEqual(len(r['actions']),8)
        self.assertFalse(r['motor_output_enabled']);self.assertFalse(r['hardware_accepted'])

    def test_c_samples_match_pc_position_velocity_and_continuity(self):
        peak_q=peak_v=0.
        for name,plan in self.plans.items():
            points=list(np.linspace(0,plan.duration,1501));offset=0.
            for _,curve in plan.stages:
                for a,b,_ in exporter.pieces(curve):points.extend([offset+a,offset+b])
                offset+=curve.duration
            for t in points:
                t=float(np.float32(t));q=(ctypes.c_float*7)();v=(ctypes.c_float*7)();a=(ctypes.c_float*7)()
                self.assertTrue(self.sample(exporter.IDS[name],t,q,v,a))
                expected_q,expected_v=plan.at(t)
                peak_q=max(peak_q,float(np.max(abs(np.array(q)-expected_q))))
                peak_v=max(peak_v,float(np.max(abs(np.array(v)-expected_v))))
                self.assertTrue(np.all(np.isfinite(a)))
        self.assertLess(peak_q,2e-5)
        self.assertLess(peak_v,2e-4)

    def test_idle_and_sleep_are_static_seven_axis_anchors(self):
        for name,key in [('IDLE','idle_rad'),('SLEEP','zero_rad')]:
            q=(ctypes.c_float*7)();v=(ctypes.c_float*7)();a=(ctypes.c_float*7)()
            self.assertTrue(self.sample(exporter.IDS[name],123,q,v,a))
            np.testing.assert_allclose(q,self.manifest[key],atol=1e-7)
            np.testing.assert_array_equal(v,np.zeros(7))

    def test_retimed_tilt_c_acceleration_matches_exact_pc_clock(self):
        curve=self.plans['TILT'].stages[1][1]
        offset=self.plans['TILT'].stages[0][1].duration
        peak=0.
        for local in np.linspace(0,curve.duration,5001):
            t=float(np.float32(offset+local))
            q=(ctypes.c_float*7)();v=(ctypes.c_float*7)();a=(ctypes.c_float*7)()
            self.assertTrue(self.sample(exporter.IDS['TILT'],t,q,v,a))
            expected=curve.acceleration_at(np.clip(t-offset,0,curve.duration))
            peak=max(peak,float(np.max(abs(np.array(a)-expected))))
        self.assertLess(peak,.004)

    def test_nuzzle_c_acceleration_matches_pc_across_sway_boundaries(self):
        curve=self.plans['NUZZLE'].stages[1][1]
        offset=self.plans['NUZZLE'].stages[0][1].duration
        for local in np.r_[np.linspace(0,curve.duration,2001),curve.breakpoints]:
            t=float(np.float32(offset+local))
            q=(ctypes.c_float*7)();v=(ctypes.c_float*7)();a=(ctypes.c_float*7)()
            self.assertTrue(self.sample(exporter.IDS['NUZZLE'],t,q,v,a))
            expected=curve.spline(np.clip(t-offset,0,curve.duration),nu=2)
            np.testing.assert_allclose(a,expected,atol=.0001,rtol=0)

if __name__=='__main__':unittest.main()
