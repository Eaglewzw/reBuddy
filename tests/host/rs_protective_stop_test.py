"""Synthetic fault-stop tests, without CAN or physical motion."""
import io
from pathlib import Path
import queue
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import rs_protective_stop as stop_module
import rs_gravity_record as recorder


class Gravity:
    lower = np.full(6, -3.)
    upper = np.full(6, 3.)
    def __call__(self, q):
        return np.array([0., 2., 3., 0., 0., 0.])


class Tests(unittest.TestCase):
    def test_hold_anchor_latches_and_storage_failure_does_not_drop_torque(self):
        clock = [1.]; commands = queue.Queue(); events = []; sent = []
        class Journal:
            def put(self, row):
                events.append(row)
                raise OSError('disk full')
        class Backend:
            def heartbeat(self): pass
            def read(self):
                q = np.zeros(7)
                if clock[0] > 2.2: q[0] = .001
                n = int(clock[0]*1e9)
                return q, n, n
            def send(self, command):
                sent.append(command)
                if clock[0] > 2.7: commands.put('quit')
        with patch.object(stop_module.time, 'monotonic', side_effect=lambda:clock[0]), \
                patch.object(stop_module.time, 'monotonic_ns', side_effect=lambda:int(clock[0]*1e9)), \
                patch.object(stop_module.time, 'sleep', side_effect=lambda d:clock.__setitem__(0,clock[0]+d)), \
                patch('sys.stdout', io.StringIO()):
            result = stop_module.maintain_hold(Backend(), Gravity(), commands, [None], Journal(), lambda:None)
        self.assertEqual(result, 'home')
        self.assertTrue(any(e['event']=='protective_hold_entered' for e in events))
        self.assertGreater(sent[-1]['kp'][0], 0)
        self.assertEqual(sent[-1]['target_rad'][0], 0.)  # Does not follow subsequent drift.
        self.assertEqual(sent[-1]['feedforward_nm'][1], 2.)

    def test_bad_feedback_never_generates_hold_command(self):
        backend = types.SimpleNamespace(heartbeat=lambda:None,
            read=lambda:(_ for _ in ()).throw(RuntimeError('stale feedback')),
            send=lambda command:self.fail('must not command invalid feedback'))
        with patch('sys.stdout', io.StringIO()), self.assertRaisesRegex(RuntimeError,'stale feedback'):
            stop_module.maintain_hold(backend,Gravity(),queue.Queue(),[None],
                types.SimpleNamespace(put=lambda row:None),lambda:None)

    def test_recorder_estop_holds_until_explicit_home(self):
        commands=queue.Queue();events=[];sent=[]
        class Backend(recorder.MockBackend):
            ticks=0
            def read(self):
                n=recorder.time.monotonic_ns()
                return np.zeros(7),n,n
            def send(self,command):
                super().send(command);self.ticks+=1;sent.append(command)
                if self.ticks==140:commands.put('estop')
                if self.ticks==220:commands.put('quit')
            def disable(self):
                events.append(('disable',self.ticks));return super().disable()
        backend=Backend()
        def zero_factory(*_):
            return types.SimpleNamespace(run=lambda cancel,**kw: events.append(('home',backend.ticks)) or True)
        with tempfile.TemporaryDirectory() as tmp,patch('sys.stdout',io.StringIO()):
            folder=Path(tmp);cal=folder/'cal.json';cal.write_text('{}')
            args=types.SimpleNamespace(mode='record',supported=True,seconds=0.,output=folder,
                                      calibration=cal,manual_start=True)
            result=recorder.run(args,Gravity(),dict(calibration_sha256=recorder.digest(cal)),
                backend_factory=lambda:backend,commands=commands,zero_return_factory=zero_factory)
        self.assertTrue(result['protective_hold_requested'],result)
        self.assertEqual(result['protective_hold_exit'],'home')
        self.assertGreaterEqual(events[0][1],220)
        self.assertEqual([e[0] for e in events],['home','disable'])
        self.assertGreater(sent[-1]['kp'][0],0)


if __name__=='__main__':unittest.main()
