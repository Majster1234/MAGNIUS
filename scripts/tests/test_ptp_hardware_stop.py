"""Procedura narzędzia stanowiskowego na atrapach, bez otwierania SocketCAN."""
from collections import deque
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('ptp_hardware', Path(__file__).parents[1] / 'test_ptp_hardware.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class MemoryBench(module.Bench):
    def __init__(self, reject_active=False, failed_send=False):
        self.saved = dict.fromkeys((1, 2, 3, 5))
        self.sent, self.events, self.replies = [], [], deque()
        self.reject_active, self.failed_send = reject_active, failed_send

    def send(self, axis, code, payload=b''):
        self.sent.append((axis, code, payload))
        if self.failed_send and axis == 1 and code == 0xf7:
            raise RuntimeError('symulowana utrata TX')
        if axis in self.saved:
            accepted = not (code == 0xf6 and (axis != 1 or self.reject_active))
            self.replies.append((axis, bytes((code, int(accepted), 0))))

    def receive(self, _timeout):
        return self.replies.popleft() if self.replies else None

    def event(self, name, **values):
        self.events.append((name, values))


class StopSequenceTest(unittest.TestCase):
    def test_disabled_drives_do_not_abort_cleanup_or_get_enabled(self):
        bench = MemoryBench()
        bench.stop_all_zero(required=(1,))
        self.assertEqual(bench.sent[:6], [(axis, 0xf7, b'') for axis in range(1, 7)])
        self.assertEqual(bench.sent[6:], [(axis, 0xf6, b'\0\0\1') for axis in range(1, 7)])
        self.assertEqual(bench.events[-1][1]['replies']['f6']['2'], '00')

    def test_active_zero_rejection_fails_only_after_both_full_phases(self):
        bench = MemoryBench(reject_active=True)
        with self.assertRaisesRegex(RuntimeError, 'niepotwierdzone f6'):
            bench.stop_all_zero(required=(1,))
        self.assertEqual(len(bench.sent), 12)

    def test_failed_f7_still_attempts_every_zero_speed_command(self):
        bench = MemoryBench(failed_send=True)
        with self.assertRaisesRegex(RuntimeError, 'utrata TX'):
            bench.stop_all_zero(required=(1,))
        self.assertEqual([axis for axis, code, _ in bench.sent if code == 0xf6], list(range(1, 7)))


if __name__ == '__main__':
    unittest.main()
