"""Simulated feedback and control clock; never opens CAN."""
import io,json,queue,types
from pathlib import Path
from unittest.mock import patch
import numpy as np
from rs_gravity_record import DEFAULT_CALIBRATION,DEFAULT_URDF
ROOT=Path(__file__).resolve().parents[2]
class MockPlayback:
    def simulate_console(self,fail=False,trial_speed=.3,bundle=None,action='10'):
        import rs_dance_console as console
        clock=[10.];events=[];state=dict(phase='',injected=False,held=False,hold_ticks=0,reads=0)
        commands=queue.Queue();initial=self.idle.copy()
        class Backend:
            q=initial.copy()
            def heartbeat(self):pass
            def read(self):
                if fail and state['phase'].startswith(action+':') and not state['injected']:
                    state['injected']=True;raise RuntimeError('synthetic pivot feedback loss')
                begin=int(clock[0]*1e9);clock[0]+=[.012,.018,.014,.016][state['reads']%4];state['reads']+=1
                return self.q.copy(),begin,int(clock[0]*1e9)
            def enable(self,q):events.append('enable')
            def send(self,command):
                self.q=np.asarray(command['target_rad'])
                if state['held']:
                    state['hold_ticks']+=1
                    if state['hold_ticks']==20:events.append('operator_exit');commands.put('quit')
            def disable(self):events.append('disable');return True
            def close(self):events.append('close')
        def zero_factory(backend,gravity,identity,args,journal):
            put=journal.put
            def observe(row):
                if row['event']=='segment_start':state['phase']=row['phase']
                if row['event']=='action_completed':commands.put('quit')
                if row['event']=='protective_hold_entered':events.append('held');state['held']=True
                put(row)
            journal.put=observe
            def home(cancel,**kw):events.append('home');return not cancel()
            return types.SimpleNamespace(run=home,args=types.SimpleNamespace(exit_zero_tolerance_deg=3.))
        args=types.SimpleNamespace(supported=True,action=action,bundle=bundle or ROOT/'record/dance_library',
            calibration=DEFAULT_CALIBRATION,urdf=DEFAULT_URDF,port='can0',pivot_speed=trial_speed)
        with patch.object(console,'ROOT',self.root), \
                patch.object(console.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(console.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                patch.object(console.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)), \
                patch('sys.stdout',io.StringIO()):
            result=console.run(args,self.manifest,self.plans,self.gravity,self.identity,self.reports,
                commands=commands,backend_factory=Backend,zero_factory=zero_factory)
        rows=[json.loads(line) for line in (Path(result['folder'])/'control.jsonl').read_text().splitlines()]
        return result,events,rows

