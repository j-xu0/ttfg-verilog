"""Runner regressions, independent of the RTL simulator."""
import importlib.util
import io
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
        self.assertIn('simulator diagnostics', self.stream.getvalue())
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
        runner.print_summary([result], 1, self.reporter)
        self.assertIn('last 60 log lines', self.stream.getvalue())
        self.assertNotIn('\n39\n', self.stream.getvalue())
        self.assertIn('\n99\n', self.stream.getvalue())


if __name__ == '__main__':
    unittest.main()
