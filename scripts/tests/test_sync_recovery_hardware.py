"""Celowane sprawdzenia procedury sprzętowej bez SocketCAN i bez ruchu."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
spec = importlib.util.spec_from_file_location('sync_bench', Path(__file__).parents[1] / 'test_sync_recovery_hardware.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MemoryBench(module.RecoveryBench):
    def __init__(self, reject=False):
        self.actions = []
        self.reject = reject
        self.stop_gap = .006
        self.guard = {}
        self.enable_settle_min = 1.61
        self.enable_settle_timeout = 4.49
        self.control_detection_only = False

    def send(self, axis, code, payload=b''):
        self.actions.append(('send', axis, code, payload))

    def event(self, name, **values):
        self.actions.append(('event', name))

    def receive(self, timeout):
        module.time.sleep(max(0, timeout))

    def stationary(self, baseline, seconds=.4, enabled=()):
        self.actions.append(('stationary', tuple(enabled)))

    def settle_enable(self, baseline, enabled=(), transition_axes=(), reason='enable'):
        self.actions.append(('settle', tuple(enabled), tuple(transition_axes)))
        return baseline.copy()

    def set_parameter(self, axis, code, value):
        self.actions.append(('parameter', axis, code, value))

    def read(self, axis, code):
        return b'\x01'

    def query(self, axis, code, payload=b'', expected=None):
        self.actions.append(('query', axis, code, payload, expected))
        if self.reject and code == 0xf5:
            raise RuntimeError('odrzucone F5 STOP')
        return expected


class SyncRecoveryProcedureTest(unittest.TestCase):
    def test_safety_stop_attempts_all_axes_and_leaves_enable_last(self):
        bench = MemoryBench();bench.safety_stop()
        frames = [a for a in bench.actions if a[0] == 'send']
        self.assertEqual(len(frames), 18)
        self.assertEqual([f[2] for f in frames], [0xf7] * 6 + [0xf5] * 6 + [0xf3] * 6)
        self.assertTrue(all(f[3] == bytes(6) for f in frames[6:12]))

    def test_recovery_does_not_send_sync_or_enable_and_verifies_disabled_before_overwrite(self):
        bench = MemoryBench();bench.recover(dict.fromkeys(module.AXES, 0))
        self.assertEqual(bench.actions[19], ('settle', (), module.AXES))
        self.assertFalse(any(a[0] == 'send' and a[2] == 0x4b for a in bench.actions))
        overwrites = [a for a in bench.actions if a[0] == 'query' and a[2] == 0xf5]
        self.assertEqual(len(overwrites), 6)
        self.assertTrue(all(a[3:] == (bytes(6), b'\x05') for a in overwrites))
        restores = [a for a in bench.actions if a[0] == 'parameter' and a[2] == 0x4a]
        self.assertEqual(len(restores), 6)

    def test_rejection_never_changes_sync_to_zero(self):
        bench = MemoryBench(reject=True)
        with self.assertRaisesRegex(RuntimeError, 'odrzucone'):
            bench.recover(dict.fromkeys(module.AXES, 0))
        self.assertFalse(any(a[0] == 'parameter' and a[2] == 0x4a for a in bench.actions))

    def test_physical_speed_never_exceeds_one_rpm(self):
        for microstep in (16, 32, 64, 128, 256):
            raw, speed = module.speed_code(microstep)
            self.assertGreaterEqual(raw, 1)
            self.assertLessEqual(speed, 1)
        for microstep in (0, 1, 2, 4, 8, 3):
            with self.assertRaises(RuntimeError):
                module.speed_code(microstep)

    def test_failed_trial_restores_visible_sync_marker_without_trigger(self):
        bench = MemoryBench();bench.latch_failed_recovery(dict.fromkeys(module.AXES, 0))
        sync = [a for a in bench.actions if a[0] == 'parameter' and a[2] == 0x4a]
        self.assertEqual(len(sync), 6)
        self.assertTrue(all(a[3] == b'\x01' for a in sync))
        self.assertFalse(any(a[0] == 'send' and a[2] == 0x4b for a in bench.actions))

    def test_enable_transient_is_recorded_and_new_baseline_requires_settling(self):
        bench = MemoryBench();clock = [0];samples = [0]
        def snapshot():
            samples[0] += 1
            return {a:dict(position=171 if a==1 else 0, rpm=0, motor_state=1, enabled=a==1) for a in module.AXES}
        def now():
            clock[0] += 60_000_000
            return clock[0]
        bench.snapshot = snapshot
        with patch.object(module.time,'monotonic_ns',side_effect=now), patch.object(module.time,'sleep'):
            baseline = module.RecoveryBench.settle_enable(bench,dict.fromkeys(module.AXES,0), enabled=(1,), transition_axes=(1,))
        self.assertEqual(baseline[1],171)
        self.assertEqual(bench.guard[1],171)
        self.assertGreaterEqual(samples[0],2)
        self.assertIn(('event','enable_transition_settled'),bench.actions)

    def test_settling_does_not_ignore_motion_of_other_axes(self):
        bench = MemoryBench()
        bench.snapshot=lambda:{a:dict(position=30 if a==2 else 0,rpm=0,motor_state=1,enabled=a==1) for a in module.AXES}
        with self.assertRaisesRegex(RuntimeError,'bez przejścia ENABLE'):
            module.RecoveryBench.settle_enable(bench,dict.fromkeys(module.AXES,0),enabled=(1,),transition_axes=(1,))

    def test_speed_noise_defers_confirmation_without_masking_other_axis_motion(self):
        for displaced in (False, True):
            bench = MemoryBench();bench.move_timeout=6.;bench.motion_stable_window=.6;bench.speeds={1:1}
            clock=[0.];samples=[0]
            def snapshot():
                samples[0]+=1;clock[0]+=.04
                return {a:dict(position=module.STEP if a==1 else 30 if a==2 and displaced else 0,
                    rpm=-1 if a==2 and samples[0]<=3 else 0,
                    motor_state=1,enabled=a==1) for a in module.AXES}
            bench.snapshot=snapshot
            with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
                 patch.object(module.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                 patch.object(module.time,'sleep'):
                if displaced:
                    with self.assertRaisesRegex(RuntimeError,'nieuczestniczącej osi'):
                        bench.move(1,module.STEP,dict.fromkeys(module.AXES,0),'test')
                else:
                    bench.move(1,module.STEP,dict.fromkeys(module.AXES,0),'test')
                    self.assertGreaterEqual(samples[0],19)
                    self.assertIn(('event','positive_control_pass'),bench.actions)

    def test_detection_control_keeps_target_error_separate_and_requires_real_displacement(self):
        for detection,position,passed in ((False,module.STEP-module.REST-5,False),
                (True,module.STEP-module.REST-5,True),(True,0,False)):
            bench=MemoryBench();bench.move_timeout=6.;bench.motion_stable_window=.6
            bench.speeds={1:1};bench.control_detection_only=detection;clock=[0.]
            def snapshot():
                clock[0]+=.04
                return {a:dict(position=position if a==1 else 0,rpm=0,motor_state=1,enabled=a==1) for a in module.AXES}
            bench.snapshot=snapshot
            with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]), \
                 patch.object(module.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)), \
                 patch.object(module.time,'sleep'):
                if passed:
                    self.assertEqual(bench.move(1,module.STEP,dict.fromkeys(module.AXES,0),'test')[1],position)
                else:
                    with self.assertRaisesRegex(RuntimeError,'timeout dojścia'):
                        bench.move(1,module.STEP,dict.fromkeys(module.AXES,0),'test')


if __name__ == '__main__':
    unittest.main()
