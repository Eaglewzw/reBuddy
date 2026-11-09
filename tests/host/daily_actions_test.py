"""Named actions must never silently dispatch unvalidated recordings to motors."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('daily_actions',ROOT/'tools/daily_actions.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class Tests(unittest.TestCase):
    def test_names_and_file_numbers(self):
        _,actions=m.catalog()
        for number,(id,names) in enumerate(m.NAMES.items(),1):
            for alias in (*names,id,id.lower(),str(number),f'{number:02d}'):
                self.assertEqual(m.resolve_action(alias,actions)['id'],id)
        self.assertEqual(m.resolve_action('熟睡呼吸与Z',actions)['id'],'SLEEP')
        self.assertEqual(m.resolve_action('05',actions)['id'],'HAPPY')
        with self.assertRaises(ValueError):m.resolve_action('害羞缩回',actions)

    def test_catalog_is_bound_to_real_sources(self):
        for name in m.NAMES:
            data=m.inspect_action(name)
            self.assertTrue(data['motion_ready'])
            self.assertFalse(data['hardware_accepted'])
            self.assertFalse(data['hardware_opened'])
            self.assertEqual(sum(p['screen_target_ms'] for p in data['phases']),data['screen_duration_s']*1000)
        self.assertIn('零位失能',m.inspect_action('SLEEP')['transition_note'])
        self.assertIn('使能后慢慢回待机',m.inspect_action('WAKE')['transition_note'])

    def test_every_play_is_rejected_without_hardware(self):
        backend=types.SimpleNamespace(Hardware=lambda *a: self.fail('opened hardware during play'))
        with patch.dict('sys.modules',rs_record_playback=backend),contextlib.redirect_stdout(io.StringIO()) as out:
            for name in m.NAMES:
                self.assertEqual(m.main(['play',name,'--json']),3)
        self.assertIn('未打开 CAN',out.getvalue())

    def test_design_inspection_without_bundle_or_legacy_recordings(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'record').mkdir()
            (root/'record/daily_design.json').write_bytes((ROOT/'record/daily_design.json').read_bytes())
            shutil.copytree(ROOT/'assets/gifs/daily',root/'assets/gifs/daily')
            with patch.object(m,'ROOT',root):
                for name in m.NAMES:
                    data=m.inspect_action(name)
                    self.assertFalse(data['motion_ready'])
                    self.assertIsNone(data['recording_reference'])

    def test_named_trial_routes_only_with_explicit_support(self):
        with patch.object(m,'trial',return_value=0) as trial,contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(m.main(['02','--supported']),0)
            trial.assert_called_with('can0',True,'PROBE')
            self.assertEqual(m.main(['05','--supported']),0)
            trial.assert_called_with('can0',True,'HAPPY')

    def test_command_errors_and_default_play(self):
        with contextlib.redirect_stdout(io.StringIO()) as out,contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(m.main(['轻蹭','--json']),3)
            self.assertEqual(json.loads(out.getvalue())['action'],'NUZZLE')
            for argv in (['play'],['check','不存在'],['probe','轻蹭'],['--json']):
                self.assertEqual(m.main(argv),2)

    def test_interactive_survives_error_and_blocked_play(self):
        commands=['不存在','play 开心回应','check 眨眼','quit']
        with patch('builtins.input',side_effect=commands),contextlib.redirect_stdout(io.StringIO()) as out,contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(m.main([]),0)
        self.assertIn('动作已整理',out.getvalue())
        self.assertIn('保持已验收',str(m.inspect_action('IDLE')['phases']))

    def test_probe_only_reads_and_closes_on_fault(self):
        calls=[]
        class Positions:
            def __mul__(self,x):return self
            def __truediv__(self,x):return self
            def tolist(self):return [0]*7
        class HW:
            driver_version='0.5.3'
            states={i:(0,0,0) for i in range(1,8)}
            def __init__(self,port):calls.append(('open',port))
            def positions(self):calls.append('positions');return Positions()
            def reports(self):calls.append('reports')
            def drain(self):calls.append('drain')
            def check_status(self,mode):calls.append(('status',mode))
            def close(self):calls.append('close')
        with tempfile.TemporaryDirectory() as temp,patch.object(m,'ROOT',Path(temp)),patch.dict('sys.modules',rs_record_playback=types.SimpleNamespace(Hardware=HW)):
            report=m.probe('can0')
            self.assertTrue(report['success']);self.assertFalse(report['enable_requested'])
            self.assertEqual(calls[-1],'close')
            self.assertEqual(json.loads(Path(report['log']).read_text()),report)
            with patch.object(HW,'check_status',side_effect=RuntimeError('motor already enabled')):
                report=m.probe('can0')
            self.assertFalse(report['success']);self.assertEqual(calls[-1],'close')
            self.assertIn('already enabled',report['error'])


if __name__=='__main__':unittest.main()
