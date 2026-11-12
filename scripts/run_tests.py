#!/usr/bin/env python3
"""Compact progress and a final report for the RTL regression suites."""

import argparse
from dataclasses import dataclass
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODULES = ('spi', 'encoder', 'timebase', 'planner', 'compact_planner', 'pi',
           'step_dir', 'shared_controller')


@dataclass
class Result:
    name: str
    status: str
    seconds: float
    log: Path
    passed: int = 0
    failed: int = 0
    skipped: int = 0
    detail: str = ''


def read_results(path):
    """Read testcase nodes, shared by pytest JUnit and cocotb xUnit reports."""
    cases = list(ET.parse(path).getroot().iter('testcase'))
    if not cases:
        raise ValueError('report contains no test cases')
    failed = sum(any(case.find(tag) is not None for tag in ('failure', 'error'))
                 for case in cases)
    skipped = sum(case.find('skipped') is not None for case in cases)
    return len(cases) - failed - skipped, failed, skipped


class Reporter:
    def __init__(self, stream=sys.stdout):
        self.stream = stream
        self.interactive = stream.isatty() and not os.environ.get('CI')
        self.color = self.interactive and 'NO_COLOR' not in os.environ

    def line(self, status, message, live=False):
        colors = {'RUN': '33', 'PASS': '32', 'FAIL': '31', 'SKIP': '33'}
        badge = f' {status:<4} '
        if self.color and status in colors:
            badge = f'\033[1;{colors[status]}m{badge}\033[0m'
        prefix = '\r\033[2K' if self.interactive else ''
        print(prefix + badge + ' ' + message, file=self.stream,
              end='' if live else '\n', flush=True)


def run_suite(name, command, report, log, reporter, index, total, cwd=ROOT):
    # A stale report must never turn a failed build into a passing suite.
    report.unlink(missing_ok=True)
    start = time.monotonic()
    label = f'[{index}/{total}] {name}'
    reporter.line('RUN', label, live=reporter.interactive)
    detail = ''
    passed = failed = skipped = 0
    with log.open('w') as output:
        try:
            process = subprocess.Popen(command, cwd=cwd, stdout=output,
                                       stderr=subprocess.STDOUT,
                                       start_new_session=(os.name == "posix"))
            try:
                while True:
                    try:
                        code = process.wait(timeout=0.15)
                        break
                    except subprocess.TimeoutExpired:
                        if reporter.interactive:
                            elapsed = time.monotonic() - start
                            reporter.line('RUN', f'{label}  {elapsed:.1f}s', live=True)
            except KeyboardInterrupt:
                if os.name == "posix":
                    os.killpg(process.pid, signal.SIGTERM)
                else:
                    process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    if os.name == "posix":
                        os.killpg(process.pid, signal.SIGKILL)
                    else:
                        process.kill()
                    process.wait()
                raise
            try:
                passed, failed, skipped = read_results(report)
            except (OSError, ET.ParseError, ValueError) as error:
                detail = f'No valid test report: {error}'
            if code:
                detail = f'Command exited with status {code}. ' + detail
        except OSError as error:
            detail = f'Could not start command: {error}'
    status = 'FAIL' if detail or failed else ('SKIP' if not passed else 'PASS')
    result = Result(name, status, time.monotonic() - start, log,
                    passed, failed, skipped, detail.strip())
    reporter.line(status, f'{label}  ({result.seconds:.1f}s)')
    return result


def print_summary(results, seconds, reporter):
    for result in results:
        if result.status == 'FAIL':
            print(f'\n── {result.name} ──', file=reporter.stream)
            if result.detail:
                print(result.detail, file=reporter.stream)
            # Keep terminal output bounded; full compiler/simulator traces stay on disk.
            lines = result.log.read_text(errors='replace').splitlines()
            if len(lines) > 60:
                print('(last 60 log lines)', file=reporter.stream)
            print('\n'.join(lines[-60:]), file=reporter.stream)
            print(f'Full log: {result.log}', file=reporter.stream)
    failed_suites = sum(r.status == 'FAIL' for r in results)
    passed_suites = sum(r.status == 'PASS' for r in results)
    skipped_suites = sum(r.status == 'SKIP' for r in results)
    print(f'\nTest suites: {failed_suites} failed, {passed_suites} passed, '
          f'{skipped_suites} skipped, {len(results)} total', file=reporter.stream)
    print(f'Tests:       {sum(r.failed for r in results)} failed, '
          f'{sum(r.passed for r in results)} passed, '
          f'{sum(r.skipped for r in results)} skipped, '
          f'{sum(r.passed + r.failed + r.skipped for r in results)} total',
          file=reporter.stream)
    print(f'Time:        {seconds:.1f}s', file=reporter.stream)
    return 1 if failed_suites else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suites', nargs='*', metavar='SUITE',
                        help='suite names (default: all); use integration for the top-level tests')
    parser.add_argument('--gates', action='store_true', help='run gate-level integration only')
    parser.add_argument('--log-dir', type=Path, default=ROOT / 'test' / 'output' / 'runner')
    args = parser.parse_args()
    names = args.suites or (['integration'] if args.gates else [*MODULES, 'integration'])
    if any(name not in (*MODULES, 'integration') for name in names):
        parser.error('suites must be: ' + ', '.join((*MODULES, 'integration')))
    if args.gates and names != ['integration']:
        parser.error('--gates requires the integration suite only')
    logs = args.log_dir.resolve()
    logs.mkdir(parents=True, exist_ok=True)
    reporter = Reporter()
    print('\nRTL regression\n', flush=True)
    start = time.monotonic()
    results = []
    try:
        for index, name in enumerate(names, 1):
            report = logs / f'{name}.xml'
            if name == 'integration':
                report = ROOT / 'test' / 'results.xml'
                command = ['make', '-C', str(ROOT / 'test'), '-B',
                           f'COCOTB_RESULTS_FILE={report}']
                if args.gates:
                    command.append('GATES=yes')
            else:
                command = [sys.executable, '-m', 'pytest',
                           str(ROOT / 'test' / f'test_{name}.py'),
                           '-q', '--color=no', f'--junitxml={report}']
            results.append(run_suite(name, command, report, logs / f'{name}.log',
                                     reporter, index, len(names)))
    except KeyboardInterrupt:
        print('\nInterrupted; the active suite did not finish.', flush=True)
        print_summary(results, time.monotonic() - start, reporter)
        return 130
    code = print_summary(results, time.monotonic() - start, reporter)
    print(f'Logs:        {logs}\n')
    return code


if __name__ == '__main__':
    sys.exit(main())
