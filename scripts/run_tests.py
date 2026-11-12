#!/usr/bin/env python3
"""Compact progress and a final report for the RTL regression suites."""

import argparse
from collections import deque
from dataclasses import dataclass
import os
import signal
import shutil
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
        self.active_lines = 0
        self.color = self.interactive and 'NO_COLOR' not in os.environ

    def style(self, text, status):
        colors = {'RUN': '33', 'PASS': '32', 'FAIL': '31', 'SKIP': '33'}
        if self.color and status in colors:
            return f'\033[1;{colors[status]}m{text}\033[0m'
        return text

    def line(self, status, message, live=False):
        badge = self.style(f' {status:<4} ', status)
        prefix = '\r\033[2K' if self.interactive else ''
        print(prefix + badge + ' ' + message, file=self.stream,
              end='' if live else '\n', flush=True)

    def clear_active(self):
        if self.active_lines:
            print(f'\033[{self.active_lines}A\033[J', end='', file=self.stream, flush=True)
            self.active_lines = 0

    def progress(self, active, total, completed):
        if not self.interactive:
            return
        self.clear_active()
        width = shutil.get_terminal_size(fallback=(100, 24)).columns
        now = time.monotonic()
        for running in active:
            message = (f'[{completed}/{total} done] {running.suite.name}'
                       f'  {now - running.started:.1f}s')
            self.line('RUN', message[:max(1, width - 8)])
        self.active_lines = len(active)


@dataclass
class Suite:
    name: str
    command: list
    report: Path
    log: Path
    index: int
    cwd: Path = ROOT


@dataclass
class Running:
    suite: Suite
    process: subprocess.Popen
    output: object
    started: float


def finish_suite(running):
    running.output.close()
    suite = running.suite
    detail = ''
    passed = failed = skipped = 0
    try:
        passed, failed, skipped = read_results(suite.report)
    except (OSError, ET.ParseError, ValueError) as error:
        detail = f'No valid test report: {error}'
    if running.process.returncode:
        detail = f'Command exited with status {running.process.returncode}. ' + detail
    status = 'FAIL' if detail or failed else ('SKIP' if not passed else 'PASS')
    return Result(suite.name, status, time.monotonic() - running.started,
                  suite.log, passed, failed, skipped, detail.strip())


def stop_suites(active):
    # Signal every simulator group before waiting, so Ctrl-C stops all workers.
    def send(running, force=False):
        try:
            if os.name == 'posix':
                os.killpg(running.process.pid, signal.SIGKILL if force else signal.SIGTERM)
            elif force:
                running.process.kill()
            else:
                running.process.terminate()
        except ProcessLookupError:
            pass
    for running in active:
        send(running)
    deadline = time.monotonic() + 5
    for running in active:
        try:
            running.process.wait(timeout=max(0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            send(running, force=True)
            running.process.wait()
        finally:
            running.output.close()


def run_suites(suites, jobs, reporter, results=None):
    if jobs < 1:
        raise ValueError('jobs must be at least 1')
    results = [] if results is None else results
    # Put the longest suites on the critical path immediately in parallel mode.
    priority = ('integration', 'step_dir', 'compact_planner', 'shared_controller')
    queue = deque(sorted(suites, key=lambda s: priority.index(s.name)
                         if s.name in priority else len(priority)) if jobs > 1 else suites)
    active = []
    total = len(suites)
    try:
        while queue or active:
            while queue and len(active) < jobs:
                suite = queue.popleft()
                suite.report.unlink(missing_ok=True)
                started = time.monotonic()
                if not reporter.interactive:
                    reporter.line('RUN', suite.name)
                output = suite.log.open('w')
                try:
                    process = subprocess.Popen(suite.command, cwd=suite.cwd,
                                               stdout=output, stderr=subprocess.STDOUT,
                                               start_new_session=(os.name == 'posix'))
                except OSError as error:
                    output.close()
                    result = Result(suite.name, 'FAIL', time.monotonic() - started,
                                    suite.log, detail=f'Could not start command: {error}')
                    reporter.clear_active()
                    reporter.line(result.status,
                                  f'[{len(results) + 1}/{total}] {suite.name}  ({result.seconds:.1f}s)')
                    results.append(result)
                else:
                    active.append(Running(suite, process, output, started))
            for running in active[:]:
                if running.process.poll() is not None:
                    active.remove(running)
                    result = finish_suite(running)
                    reporter.clear_active()
                    reporter.line(result.status,
                                  f'[{len(results) + 1}/{total}] {result.name}'
                                  f'  ({result.seconds:.1f}s)')
                    results.append(result)
            reporter.progress(active, total, len(results))
            if active:
                time.sleep(0.1)
    finally:
        reporter.clear_active()
        stop_suites(active)
    order = {suite.name: suite.index for suite in suites}
    results.sort(key=lambda result: order[result.name])
    return results


def run_suite(name, command, report, log, reporter, index, total, cwd=ROOT):
    """Single-suite entry point, also used by the reporting regression tests."""
    return run_suites([Suite(name, command, report, log, 0, cwd)], 1, reporter)[0]


def print_summary(results, seconds, reporter, verbose=False):
    if verbose:
        for result in results:
            if result.status == 'FAIL':
                print(f'\n── {result.name} diagnostics ──', file=reporter.stream)
                lines = result.log.read_text(errors='replace').splitlines()
                if len(lines) > 60:
                    print('(last 60 log lines)', file=reporter.stream)
                print('\n'.join(lines[-60:]), file=reporter.stream)

    # Always finish with the compact report, even after verbose diagnostics.
    print('\nRegression results\n' + '─' * 68, file=reporter.stream)
    for result in results:
        counts = ', '.join(f'{count} {label}' for count, label in
                           ((result.passed, 'passed'), (result.failed, 'failed'),
                            (result.skipped, 'skipped')) if count)
        reporter.line(result.status,
                      f'{result.name:<21} {counts or "no results":<28} '
                      f'{result.seconds:>6.1f}s')
        if result.status == 'FAIL':
            if result.detail and not result.failed:
                print(f'       {result.detail}', file=reporter.stream)
            print(f'       Log: {result.log}', file=reporter.stream)

    def counts_line(label, failed, passed, skipped, total):
        values = [reporter.style(f'{failed} failed', 'FAIL') if failed else '0 failed',
                  reporter.style(f'{passed} passed', 'PASS') if passed else '0 passed',
                  reporter.style(f'{skipped} skipped', 'SKIP') if skipped else '0 skipped']
        print(f'{label:<13}' + ', '.join(values) + f', {total} total',
              file=reporter.stream)

    failed_suites = sum(r.status == 'FAIL' for r in results)
    print('─' * 68, file=reporter.stream)
    counts_line('Test suites:', failed_suites,
                sum(r.status == 'PASS' for r in results),
                sum(r.status == 'SKIP' for r in results), len(results))
    passed, failed, skipped = (sum(getattr(r, field) for r in results)
                               for field in ('passed', 'failed', 'skipped'))
    counts_line('Tests:', failed, passed, skipped, passed + failed + skipped)
    minutes, remainder = divmod(round(seconds), 60)
    duration = f'{minutes}m {remainder:02d}s' if minutes else f'{seconds:.1f}s'
    print(f'Time:        {duration}', file=reporter.stream)
    return 1 if failed_suites else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suites', nargs='*', metavar='SUITE',
                        help='suite names (default: all); use integration for the top-level tests')
    parser.add_argument('-j', '--jobs', type=int, default=1,
                        help='maximum concurrent suites (default: 1; try 4)')
    parser.add_argument('--verbose', action='store_true',
                        help='show failed log tails before the final report')
    parser.add_argument('--gates', action='store_true', help='run gate-level integration only')
    parser.add_argument('--log-dir', type=Path, default=ROOT / 'test' / 'output' / 'runner')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be at least 1')
    names = args.suites or (['integration'] if args.gates else [*MODULES, 'integration'])
    if any(name not in (*MODULES, 'integration') for name in names):
        parser.error('suites must be: ' + ', '.join((*MODULES, 'integration')))
    if len(set(names)) != len(names):
        parser.error('each suite may be selected only once')
    if args.gates and names != ['integration']:
        parser.error('--gates requires the integration suite only')
    logs = args.log_dir.resolve()
    logs.mkdir(parents=True, exist_ok=True)
    reporter = Reporter()
    print(f'\nRTL regression · {args.jobs} worker(s)\n', flush=True)
    start = time.monotonic()
    results = []
    suites = []
    for index, name in enumerate(names):
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
                       '-q', '--color=no', f'--junitxml={report}',
                       '-o', f'cache_dir={logs / name}']
        suites.append(Suite(name, command, report, logs / f'{name}.log', index))
    try:
        run_suites(suites, args.jobs, reporter, results)
    except KeyboardInterrupt:
        order = {name: index for index, name in enumerate(names)}
        results.sort(key=lambda result: order[result.name])
        print('\nInterrupted; active suites did not finish.', flush=True)
        print_summary(results, time.monotonic() - start, reporter, args.verbose)
        return 130
    code = print_summary(results, time.monotonic() - start, reporter, args.verbose)
    print(f'Logs:        {logs}\n')
    return code


if __name__ == '__main__':
    sys.exit(main())
