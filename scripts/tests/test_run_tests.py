"""Runner regressions, independent of the RTL simulator."""
import importlib.util
import io
import os
import subprocess
import time
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('run_tests', Path(__file__).parents[1] / 'run_tests.py')
runner = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runner
SPEC.loader.exec_module(runner)


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.report = self.root / 'results.xml'
        self.log = self.root / 'suite.log'
        self.stream = io.StringIO()
        self.reporter = runner.Reporter(self.stream)

    def run_child(self, body):
        return runner.run_suite('fixture', [sys.executable, '-c', body],
                                self.report, self.log, self.reporter, 1, 1)

    def write_report(self, xml, exit_code=0):
        return self.run_child(f'from pathlib import Path; '
                              f'Path({str(self.report)!r}).write_text({xml!r}); '
                              f'print("simulator diagnostics"); raise SystemExit({exit_code})')

    def fixture_suite(self, name, index, barrier=False, fail=False):
        report = self.root / f'{name}.xml'
        code = f"""
from pathlib import Path
import time
root = Path({str(self.root)!r})
started = time.monotonic()
(root / {name!r}).with_suffix('.started').write_text(str(started))
if {barrier!r}:
    deadline = time.monotonic() + 5
    while len(list(root.glob('*.started'))) < 2:
        assert time.monotonic() < deadline, 'workers did not overlap'
        time.sleep(0.01)
time.sleep(0.05)
Path({str(report)!r}).write_text('<testsuite><testcase>{'<failure/>' if fail else ''}</testcase></testsuite>')
print('started=' + str(started))
print('ended=' + str(time.monotonic()))
"""
        return runner.Suite(name, [sys.executable, '-c', code], report,
                            self.root / f'{name}.log', index, self.root)

    def test_parallel_limit_overlap_order_and_failure_continuation(self):
        names = ['spi', 'integration', 'step_dir', 'encoder', 'pi']
        suites = [self.fixture_suite(name, i, barrier=True, fail=name == 'integration')
                  for i, name in enumerate(names)]
        results = runner.run_suites(suites, 2, self.reporter)
        self.assertEqual([r.name for r in results], names)
        self.assertEqual([r.status for r in results], ['PASS', 'FAIL', 'PASS', 'PASS', 'PASS'])
        intervals = {}
        for suite in suites:
            fields = dict(line.split('=') for line in suite.log.read_text().splitlines())
            intervals[suite.name] = (float(fields['started']), float(fields['ended']))
        first_two = sorted(names, key=lambda name: intervals[name][0])[:2]
        self.assertEqual(set(first_two), {'integration', 'step_dir'})
        events = sorted([(start, 1) for start, _ in intervals.values()] +
                        [(end, -1) for _, end in intervals.values()])
        active = maximum = 0
        for _, change in events:
            active += change
            maximum = max(maximum, active)
        self.assertEqual(maximum, 2)
        self.assertIn('5/5', self.stream.getvalue())

    def test_terminal_progress_tracks_and_clears_multiple_workers(self):
        class Terminal(io.StringIO):
            def isatty(self):
                return True
        stream = Terminal()
        suites = [self.fixture_suite(name, i, barrier=True)
                  for i, name in enumerate(['spi', 'encoder'])]
        with patch.dict(runner.os.environ, {}, clear=True):
            reporter = runner.Reporter(stream)
            results = runner.run_suites(suites, 2, reporter)
        self.assertEqual(len(results), 2)
        self.assertIn('\033[2A\033[J', stream.getvalue())
        self.assertIn('[0/2 done] spi', stream.getvalue())
        self.assertIn('[0/2 done] encoder', stream.getvalue())
        self.assertEqual(reporter.active_lines, 0)

    def test_single_worker_preserves_requested_launch_order(self):
        suites = [self.fixture_suite(name, i) for i, name in enumerate(['spi', 'integration'])]
        runner.run_suites(suites, 1, self.reporter)
        self.assertLess(self.stream.getvalue().index('RUN   spi'),
                        self.stream.getvalue().index('RUN   integration'))

    def test_cli_rejects_invalid_jobs_and_duplicate_suites(self):
        for args in (['--jobs', '0'], ['--jobs', '-1'], ['spi', 'spi']):
            with self.subTest(args=args):
                result = subprocess.run([sys.executable, str(SPEC.origin), *args],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)

    @unittest.skipUnless(os.name == 'posix', 'process-group signals require POSIX')
    def test_interrupt_cleans_up_all_workers_and_descendants(self):
        worker = self.root / 'worker.py'
        worker.write_text("""import os, subprocess, sys, time
from pathlib import Path
root = Path(sys.argv[1])
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
(root / (sys.argv[2] + '.pid')).write_text(str(os.getpid()) + ' ' + str(child.pid))
try:
    time.sleep(60)
finally:
    child.wait()
""")
        suites = [runner.Suite(name, [sys.executable, str(worker), str(self.root), name],
                               self.root / f'{name}.xml', self.root / f'{name}.log', i)
                  for i, name in enumerate(['spi', 'encoder'])]
        real_sleep = time.sleep
        interrupted = False
        def interrupt_when_started(delay):
            nonlocal interrupted
            if interrupted:
                return real_sleep(delay)
            deadline = time.monotonic() + 5
            while len(list(self.root.glob('*.pid'))) < 2:
                self.assertLess(time.monotonic(), deadline)
                real_sleep(0.01)
            interrupted = True
            raise KeyboardInterrupt
        try:
            with patch.object(runner.time, 'sleep', side_effect=interrupt_when_started):
                with self.assertRaises(KeyboardInterrupt):
                    runner.run_suites(suites, 2, self.reporter)
        finally:
            # Check that both simulator groups stopped, including subprocesses.
            for path in self.root.glob('*.pid'):
                for pid in map(int, path.read_text().split()):
                    proc = Path(f'/proc/{pid}/stat')
                    if proc.exists():
                        # An exited descendant may briefly remain as a zombie.
                        self.assertEqual(proc.read_text().split()[2], 'Z')

    def test_terminal_colors_and_no_color(self):
        class Terminal(io.StringIO):
            def isatty(self):
                return True
        stream = Terminal()
        with patch.dict(runner.os.environ, {}, clear=True):
            reporter = runner.Reporter(stream)
            reporter.line('RUN', 'spi', live=True)
            reporter.line('PASS', 'spi')
            self.assertIn('\033[1;32m', stream.getvalue())
            self.assertIn('\r\033[2K', stream.getvalue())
        stream = Terminal()
        with patch.dict(runner.os.environ, {'NO_COLOR': '1'}, clear=True):
            runner.Reporter(stream).line('PASS', 'spi')
            self.assertNotIn('\033[1;', stream.getvalue())
        stream = Terminal()
        with patch.dict(runner.os.environ, {'CI': 'true'}, clear=True):
            runner.Reporter(stream).line('PASS', 'spi')
            self.assertNotIn('\033', stream.getvalue())

    def test_pass_with_skips_and_plain_output(self):
        result = self.write_report('<testsuites><testsuite><testcase/><testcase><skipped/></testcase></testsuite></testsuites>')
        self.assertEqual((result.status, result.passed, result.skipped), ('PASS', 1, 1))
        self.assertNotIn('\033', self.stream.getvalue())
        self.assertNotIn('simulator diagnostics', self.stream.getvalue())
        self.assertEqual(runner.print_summary([result], 1, self.reporter), 0)

    def test_cocotb_failure_even_when_make_succeeds(self):
        result = self.write_report('<testsuite><testcase><failure message="bad RTL"/></testcase></testsuite>')
        self.assertEqual((result.status, result.failed), ('FAIL', 1))
        self.assertEqual(runner.print_summary([result], 1, self.reporter), 1)
        self.assertNotIn('simulator diagnostics', self.stream.getvalue())
        self.assertIn('Regression results', self.stream.getvalue())
        self.assertIn('1 failed', self.stream.getvalue())
        self.assertIn(str(self.log), self.stream.getvalue())

    def test_nonzero_exit_overrides_passing_xml(self):
        result = self.write_report('<testsuite><testcase/></testsuite>', 2)
        self.assertEqual(result.status, 'FAIL')
        self.assertIn('status 2', result.detail)

    def test_stale_report_removed(self):
        self.report.write_text('<testsuite><testcase/></testsuite>')
        result = self.run_child('print("compiler failed")')
        self.assertEqual(result.status, 'FAIL')
        self.assertFalse(self.report.exists())

    def test_invalid_and_empty_reports(self):
        for xml in ('bad XML', '<testsuite/>'):
            with self.subTest(xml=xml):
                self.assertEqual(self.write_report(xml).status, 'FAIL')

    def test_missing_executable(self):
        result = runner.run_suite('missing', [str(self.root / 'missing')],
                                  self.report, self.log, self.reporter, 1, 1)
        self.assertEqual(result.status, 'FAIL')
        self.assertIn('Could not start', result.detail)

    def test_errors_and_all_skipped(self):
        result = self.write_report('<testsuite><testcase><error/></testcase></testsuite>')
        self.assertEqual((result.status, result.failed), ('FAIL', 1))
        result = self.write_report('<testsuite><testcase><skipped/></testcase></testsuite>')
        self.assertEqual(result.status, 'SKIP')

    def test_failure_log_tail_bounded(self):
        result = self.run_child('print("\\n".join(str(i) for i in range(100)))')
        runner.print_summary([result], 1, self.reporter, verbose=True)
        self.assertIn('last 60 log lines', self.stream.getvalue())
        self.assertNotIn('\n39\n', self.stream.getvalue())
        self.assertIn('\n99\n', self.stream.getvalue())
        self.assertLess(self.stream.getvalue().index('\n99\n'),
                        self.stream.getvalue().index('Regression results'))


if __name__ == '__main__':
    unittest.main()
