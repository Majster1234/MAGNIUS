import contextlib
import copy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from benchmarks import harness
from scripts import benchmark_ros as bench


def run_record():
    return {
        'schema_version': 1, 'kind': 'run', 'status': 'complete', 'context': 'offline',
        'hardware': {'cpu_model': 'test', 'kernel': 'test', 'affinity': [0], 'logical_cpus': 1},
        'case': {'id': 'test', 'language': 'python', 'sources': [{'path': 'code.py', 'symbols': ['cycle']}],
                 'workload': 'Stałe wejście'},
        'settings': bench.SETTINGS.copy(), 'runtime': {'python': 'test'},
        'harness_sha256': {'adapter.py': 'a' * 64}, 'iterations': 10,
        'samples': [{'wall_ns': index * 100, 'cpu_ns': index * 50} for index in range(1, 8)],
        'code': {'revision': 'before'},
    }


def watch_record():
    result = run_record()
    result.update(kind='watch', duration_seconds=2.0, interval_seconds=1.0,
                  processes={'bridge': {'pid': 123, 'executable': 'python3', 'affinity': [0],
                                        'samples': [
                                            {'elapsed_seconds': 1., 'cpu_seconds': .1, 'cpu_percent': 10.},
                                            {'elapsed_seconds': 1., 'cpu_seconds': .3, 'cpu_percent': 30.}]}})
    return result


class StatisticsAndComparisonTests(unittest.TestCase):
    def test_statistics_are_normalized_per_cycle(self):
        record = run_record()
        summary = bench.summarize(record['samples'], record['iterations'])
        self.assertEqual(summary['wall_ns_per_cycle'], {'median': 40, 'min': 10, 'max': 70})
        self.assertEqual(summary['cpu_ns_per_cycle']['median'], 20)
        self.assertEqual(summary['cycles_per_second']['median'], 25_000_000)

    def test_improvement_and_regression_and_changed_code(self):
        before = run_record()
        for factor, expected in ((.5, 50), (1.5, -50)):
            with self.subTest(factor=factor):
                after = copy.deepcopy(before)
                after['code']['revision'] = 'after'
                for sample in after['samples']:
                    sample['wall_ns'] *= factor
                    sample['cpu_ns'] *= factor
                result = bench.compare(before, after)
                self.assertEqual(result['status'], 'comparable')
                self.assertEqual(result['metrics']['cpu_ns_per_cycle']['cost_reduction_percent'], expected)

    def test_mismatched_conditions_have_no_metrics(self):
        before = run_record()
        for key, value in (('context', 'other'), ('iterations', 20), ('runtime', {'python': 'other'}),
                           ('harness_sha256', {'adapter': 'b' * 64}),
                           ('case', {**before['case'], 'workload': 'Inne dane'}),
                           ('hardware', {**before['hardware'], 'cpu_model': 'other'})):
            with self.subTest(key=key):
                after = copy.deepcopy(before)
                after[key] = value
                result = bench.compare(before, after)
                self.assertEqual(result['status'], 'not_comparable')
                self.assertEqual(result['metrics'], {})

    def test_invalid_and_incomplete_measurements(self):
        before = run_record()
        changes = [('samples', None), ('samples', []), ('samples', [{}] * 7),
                   ('samples', [{'wall_ns': float('nan'), 'cpu_ns': 2}] * 7),
                   ('samples', [{'wall_ns': -1, 'cpu_ns': 2}] * 7),
                   ('samples', [{'wall_ns': 2, 'cpu_ns': 0}] * 7),
                   ('iterations', True), ('iterations', 0), ('runtime', {}),
                   ('hardware', {}), ('case', {}), ('status', 'incomplete'), ('schema_version', 2)]
        for key, value in changes:
            with self.subTest(key=key, value=value):
                after = copy.deepcopy(before)
                after[key] = value
                self.assertEqual(bench.compare(before, after)['status'], 'not_comparable')

    def test_missing_baseline_and_invalid_after(self):
        self.assertEqual(bench.compare(None, run_record())['status'], 'no_baseline')
        self.assertEqual(bench.compare(None, {})['status'], 'not_comparable')

    def test_watch_summary_and_new_pid(self):
        before = watch_record()
        after = copy.deepcopy(before)
        after['processes']['bridge']['pid'] = 456
        for sample in after['processes']['bridge']['samples']:
            sample['cpu_seconds'] *= .5
            sample['cpu_percent'] *= .5
        result = bench.compare(before, after)
        self.assertEqual(result['status'], 'comparable')
        metrics = result['metrics']['bridge']
        self.assertEqual(metrics['before']['cpu_percent_mean'], 20)
        self.assertEqual(metrics['before']['cpu_percent_max'], 30)
        self.assertEqual(metrics['cpu_mean_reduction_percent'], 50)

    def test_zero_cpu_baseline_is_not_divided(self):
        self.assertIsNone(bench.cost_reduction(0, 1))
        record = watch_record()
        for sample in record['processes']['bridge']['samples']:
            sample.update(cpu_seconds=0, cpu_percent=0)
        result = bench.compare(record, record)
        self.assertIsNone(result['metrics']['bridge']['cpu_mean_reduction_percent'])

    def test_watch_compatibility(self):
        before = watch_record()
        for change in ('duration', 'interval', 'labels', 'executable', 'affinity', 'partial', 'bad_sample'):
            with self.subTest(change=change):
                after = copy.deepcopy(before)
                if change == 'duration':
                    after['duration_seconds'] = 3
                elif change == 'interval':
                    after['interval_seconds'] = .5
                elif change == 'labels':
                    after['processes']['other'] = after['processes'].pop('bridge')
                elif change in ('affinity', 'executable'):
                    after['processes']['bridge'][change] = [1] if change == 'affinity' else 'other'
                elif change == 'partial':
                    after['status'] = 'incomplete'
                else:
                    after['processes']['bridge']['samples'] = [None]
                self.assertEqual(bench.compare(before, after)['status'], 'not_comparable')

    def test_inconsistent_watch_sample_is_rejected(self):
        after = watch_record()
        after['processes']['bridge']['samples'][0]['cpu_percent'] = 99
        self.assertEqual(bench.compare(watch_record(), after)['status'], 'not_comparable')


class PythonHarnessTests(unittest.TestCase):
    def test_preparation_and_verification_are_outside_measurement(self):
        clocks = {'cpu': 0, 'wall': 0}

        class State:
            def cycle(self, index):
                clocks['cpu'] += 2
                clocks['wall'] += 3

            def verify(self):
                clocks['cpu'] += 2000
                clocks['wall'] += 3000
                return True

        def prepare():
            clocks['cpu'] += 200
            clocks['wall'] += 300
            return State()

        with patch.object(harness.time, 'process_time_ns', lambda: clocks['cpu']), \
                patch.object(harness.time, 'perf_counter_ns', lambda: clocks['wall']):
            self.assertEqual(harness.measure(prepare, 5), {'cpu_ns': 10, 'wall_ns': 15})

    def test_invalid_output_fails_even_without_assertions(self):
        class Invalid:
            def cycle(self, index):
                pass

            def verify(self):
                return False

        with self.assertRaises(ValueError):
            harness.measure(Invalid, 1)

    def test_calibration_and_reuse_of_iterations(self):
        calls = []

        def measure(prepare, iterations):
            calls.append(iterations)
            return {'wall_ns': iterations * 1_000_000, 'cpu_ns': iterations * 500_000}

        with patch.object(harness, 'measure', measure):
            first = harness.execute(None, warmup_seconds=0)
            self.assertGreaterEqual(first['iterations'] * 1_000_000, 200_000_000)
            self.assertEqual(len(first['samples']), 7)
            calls.clear()
            second = harness.execute(None, iterations=first['iterations'], warmup_seconds=0)
            self.assertEqual(second['iterations'], first['iterations'])
            self.assertEqual(calls, [first['iterations']] * 8)


class RunnerTests(unittest.TestCase):
    def mock_run(self, raw=None, baseline=None, failure=None):
        base = {key: value for key, value in run_record().items()
                if key in ('schema_version', 'kind', 'status', 'context', 'hardware', 'code')}
        if raw is None:
            raw = {'iterations': 10, 'samples': run_record()['samples']}
        with patch.object(bench, 'record_base', return_value=base), \
                patch.object(bench, 'command', side_effect=failure, return_value=json.dumps(raw)):
            return bench.run_case('technical.python', bench.ROOT / 'benchmarks/cases.json', baseline=baseline)

    def test_baseline_is_reused_and_adapter_cannot_override_it(self):
        baseline = self.mock_run()
        self.assertEqual(baseline['status'], 'complete')
        self.assertEqual(self.mock_run(baseline=baseline)['iterations'], 10)
        changed = {'iterations': 20, 'samples': run_record()['samples']}
        result = self.mock_run(raw=changed, baseline=baseline)
        self.assertEqual(result['status'], 'error')
        self.assertIn('liczbę cykli', result['error'])

    def test_incompatible_baseline_is_rejected_before_running(self):
        baseline = self.mock_run()
        baseline['case']['workload'] = 'other'
        with self.assertRaisesRegex(ValueError, 'case'):
            self.mock_run(baseline=baseline)

    def test_timeout_and_malformed_adapter_response_are_recorded(self):
        result = self.mock_run(failure=subprocess.TimeoutExpired('adapter', 30))
        self.assertEqual(result['status'], 'error')
        self.assertIn('30 s', result['error'])
        self.assertEqual(self.mock_run(raw={'iterations': 10})['status'], 'error')
        self.assertEqual(self.mock_run(raw=[])['status'], 'error')
        result = self.mock_run(failure=subprocess.CalledProcessError(1, ['adapter']))
        self.assertEqual(result['status'], 'error')

    def test_nonfinite_adapter_data_still_produces_valid_error_json(self):
        raw = {'iterations': 10, 'samples': [{'wall_ns': float('nan'), 'cpu_ns': 1}] * 7}
        result = self.mock_run(raw=raw)
        self.assertEqual(result['status'], 'error')
        with tempfile.TemporaryDirectory() as directory:
            path = bench.save_record(result, Path(directory) / 'error.json')
            self.assertEqual(bench.load_json(path)['status'], 'error')

    def test_unknown_case_does_not_run_other_cases(self):
        with patch.object(bench, 'command') as command:
            with self.assertRaisesRegex(ValueError, 'Nieznany scenariusz'):
                bench.run_case('unknown', bench.ROOT / 'benchmarks/cases.json')
            command.assert_not_called()

    def test_output_cannot_overwrite_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'result.json'
            bench.save_record(run_record(), path)
            original = path.read_bytes()
            with self.assertRaises(FileExistsError):
                bench.save_record(watch_record(), path)
            self.assertEqual(path.read_bytes(), original)

    def test_cli_missing_baseline_saves_explicit_status(self):
        with tempfile.TemporaryDirectory() as directory:
            after = bench.save_record(run_record(), Path(directory) / 'after.json')
            output = Path(directory) / 'compare.json'
            with contextlib.redirect_stdout(io.StringIO()):
                code = bench.main(['compare', '--after', str(after), '--output', str(output)])
            self.assertEqual(code, 2)
            self.assertEqual(bench.load_json(output)['status'], 'no_baseline')


class ProcessTests(unittest.TestCase):
    def test_proc_stat_name_with_parentheses_and_cpu_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            process = Path(directory) / '123'
            process.mkdir()
            fields = ['S'] + ['0'] * 19
            fields[11], fields[12], fields[19] = '20', '3', '100'
            (process / 'stat').write_text('123 (name ) with spaces) ' + ' '.join(fields))
            (process / 'exe').symlink_to('/usr/bin/test')
            snapshot = bench.read_process(123, Path(directory))
            self.assertEqual(snapshot['ticks'], 23)
            self.assertEqual(snapshot['start_ticks'], 100)
            self.assertEqual(snapshot['executable'], '/usr/bin/test')
            fields[0] = 'Z'
            (process / 'stat').write_text('123 (test) ' + ' '.join(fields))
            with self.assertRaises(ValueError):
                bench.read_process(123, Path(directory))

    def test_cpu_can_exceed_one_core(self):
        before = {'ticks': 100, 'timestamp': 10, 'start_ticks': 1, 'executable': '/test'}
        after = {**before, 'ticks': 400, 'timestamp': 12}
        result = bench.cpu_interval(before, after, 100)
        self.assertEqual(result, {'elapsed_seconds': 2, 'cpu_seconds': 3, 'cpu_percent': 150})

    def test_pid_reuse_exec_and_invalid_deltas(self):
        before = {'ticks': 100, 'timestamp': 10, 'start_ticks': 1, 'executable': '/test'}
        for update in ({'start_ticks': 2}, {'executable': '/new'}, {'ticks': 99}, {'timestamp': 9}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                bench.cpu_interval(before, {**before, 'timestamp': 11, **update}, 100)

    def test_watch_records_partial_on_reuse_and_permission_error(self):
        initial = {'ticks': 100, 'timestamp': 1, 'start_ticks': 1, 'executable': '/test'}
        for failure in ({**initial, 'start_ticks': 2, 'timestamp': 2}, PermissionError('denied')):
            sequence = iter([initial, failure])

            def reader(pid):
                value = next(sequence)
                if isinstance(value, Exception):
                    raise value
                return value

            now = [0.]
            with patch.object(bench.os, 'sched_getaffinity', return_value={0}):
                result = bench.watch_processes(
                    {'test': os.getpid()}, 'test', duration=1, interval=1, reader=reader,
                    clock=lambda: now[0], sleep=lambda delta: now.__setitem__(0, now[0] + delta))
            self.assertEqual(result['status'], 'incomplete')
            self.assertTrue(result['errors'])

    def test_target_validation(self):
        self.assertEqual(bench.parse_targets(['bridge=123', 'diag=456']), {'bridge': 123, 'diag': 456})
        for values in (['123'], ['a=0'], ['a=-1'], ['a=1', 'a=2']):
            with self.subTest(values=values), self.assertRaises(ValueError):
                bench.parse_targets(values)
        for duration, interval in ((0, 1), (1, 0), (float('nan'), 1), (1, float('inf'))):
            with self.subTest(duration=duration, interval=interval), self.assertRaises(ValueError):
                bench.watch_processes({'self': os.getpid()}, 'test', duration, interval)

    def test_real_controlled_process_and_exit(self):
        child = subprocess.Popen([sys.executable, '-c',
                                  'import time; print("ready", flush=True); time.sleep(20)'],
                                 stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'ready')
            result = bench.watch_processes({'test': child.pid}, 'controlled sleeper', .08, .02)
            self.assertEqual(result['status'], 'complete')
            bench.validate_record(result)
            self.assertGreater(len(result['processes']['test']['samples']), 0)
        finally:
            child.terminate()
            child.wait(timeout=5)
            child.stdout.close()
        result = bench.watch_processes({'test': child.pid}, 'terminated', .02, .01)
        self.assertEqual(result['status'], 'incomplete')


class CppHarnessTests(unittest.TestCase):
    def test_release_helper_and_result_verification(self):
        build = Path(os.environ.get('ROBOT_BENCH_BUILD_DIR', bench.ROOT / 'build/benchmarks')).resolve()
        configure = subprocess.run(['cmake', '-S', str(bench.ROOT / 'benchmarks'), '-B', str(build),
                                    '-DCMAKE_BUILD_TYPE=Release'], capture_output=True, text=True, timeout=120)
        self.assertEqual(configure.returncode, 0, configure.stdout + configure.stderr)
        compile_result = subprocess.run(['cmake', '--build', str(build), '--target', 'harness_check'],
                                         capture_output=True, text=True, timeout=120)
        self.assertEqual(compile_result.returncode, 0, compile_result.stdout + compile_result.stderr)
        checked = subprocess.run([str(build / 'harness_check')], capture_output=True, text=True, timeout=30)
        self.assertEqual(checked.returncode, 0, checked.stdout + checked.stderr)


if __name__ == '__main__':
    unittest.main()
