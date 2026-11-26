#!/usr/bin/env python3
"""Measure standard-cell area with an explicitly supplied GF180 Liberty file.

This is synthesis-only evidence. Placement, routing and STA still require the
matching Tiny Tapeout GF180 hardening flow.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def quoted(path):
    return '"' + str(path).replace('\\', '\\\\').replace('"', '\\"') + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lib', type=Path, required=True)
    parser.add_argument('--yosys', default='yosys')
    parser.add_argument('--abc', default='yosys-abc')
    parser.add_argument('--top', default='tt_um_j_xu0_stepper')
    parser.add_argument('--parameter', action='append', default=[],
                        help='Top parameter NAME=integer (repeatable)')
    parser.add_argument('--source-dir', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'src')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    lib = args.lib.resolve(strict=True)
    args.output.mkdir(parents=True, exist_ok=True)
    output = args.output.resolve()
    sources = sorted(args.source_dir.resolve().glob('*.v'))
    if not sources:
        parser.error('No Verilog sources found')
    commands = [
        f'read_liberty -lib {quoted(lib)}',
        'read_verilog -I ' + quoted(args.source_dir.resolve()) + ' ' +
        ' '.join(map(quoted, sources)),
    ]
    for parameter in args.parameter:
        name, value = parameter.split('=', 1)
        if not name.isidentifier():
            parser.error('Invalid parameter name')
        commands.append(f'chparam -set {name} {int(value, 0)} {args.top}')
    commands += [
        f'synth -top {args.top} -flatten -noabc',
        f'dfflibmap -liberty {quoted(lib)}',
        f'abc -exe {quoted(args.abc)} -liberty {quoted(lib)}',
        'clean',
        'check -assert',
        f'tee -o {quoted(output / "area.json")} stat -json -liberty {quoted(lib)}',
        f'write_json {quoted(output / "netlist.json")}',
    ]
    script = output / 'synth.ys'
    script.write_text('\n'.join(commands) + '\n')
    with (output / 'synth.log').open('w') as log:
        subprocess.run([args.yosys, '-s', str(script)], cwd=output,
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    report = json.loads((output / 'area.json').read_text())
    top = report['modules']['\\' + args.top]
    summary = {key: top[key] for key in
               ('area', 'sequential_area', 'num_cells')}
    summary['liberty'] = str(lib)
    summary['liberty_sha256'] = hashlib.sha256(lib.read_bytes()).hexdigest()
    summary['sources_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in sources + sorted(args.source_dir.resolve().glob('*.vh'))}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in
                      ('area', 'sequential_area', 'num_cells', 'liberty_sha256')}, indent=2))


if __name__ == '__main__':
    main()
