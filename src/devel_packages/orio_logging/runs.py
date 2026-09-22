#!/usr/bin/env python3
"""Run folder CLI for logging/rerun/<run_id>/.

    runs.py list
    runs.py open [run_id|latest]
    runs.py summary [run_id|latest]
    runs.py compare RUN_A RUN_B
    runs.py du
"""
import argparse
import json
import os
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
ROOT = Path(os.environ.get('ORIO_RUNS_ROOT', REPO / 'logging/rerun'))


def runs():
    return sorted((p for p in ROOT.iterdir() if p.is_dir()), key=lambda p: p.name) if ROOT.exists() else []


def resolve(name):
    if name in (None, 'latest'):
        if not runs():
            sys.exit('no runs')
        return runs()[-1]
    p = ROOT / name
    if not p.is_dir():
        sys.exit(f'no run {name}')
    return p


def size_mb(p):
    return sum(f.stat().st_size for f in p.rglob('*') if f.is_file()) / 1e6


def manifest(p):
    try:
        return json.loads((p / 'run.json').read_text())
    except (OSError, ValueError):
        return {}


def cmd_list(_):
    for p in runs():
        m = manifest(p)
        picks = m.get('summary', {}).get('picks', '-')
        ended = 'open' if 'ended_at' not in m else m['ended_at'][11:19]
        print(f'{p.name}  {size_mb(p):8.1f} MB  picks={picks:<3} {ended}  {m.get("git", {}).get("sha", "")[:8]}')


def cmd_open(a):
    p = resolve(a.run)
    files = sorted(str(f) for f in p.glob('*.rrd'))
    if not files:
        sys.exit(f'no .rrd in {p}')
    os.execvp('rerun', ['rerun', *files])


def cmd_summary(a):
    print(json.dumps(manifest(resolve(a.run)).get('summary', {}), indent=2))


def cmd_compare(a):
    ma, mb = (manifest(resolve(r)).get('summary', {}) for r in (a.a, a.b))
    for section in ('state_durations', 'service_latency', 'goto_joints'):
        keys = sorted(set(ma.get(section, {})) | set(mb.get(section, {})))
        if not keys:
            continue
        print(f'\n{section}  (median s: {a.a} | {a.b} | delta %)')
        for k in keys:
            x = ma.get(section, {}).get(k, {}).get('median_s')
            y = mb.get(section, {}).get(k, {}).get('median_s')
            d = f'{(y - x) / x * 100:+.1f}%' if x and y else '-'
            fmt = lambda v: f'{v:.3f}' if v is not None else '  -  '
            print(f'  {k:28s} {fmt(x)} | {fmt(y)} | {d}')


def cmd_du(_):
    total = 0.0
    for p in runs():
        s = size_mb(p)
        total += s
        print(f'{s:8.1f} MB  {p.name}')
    free = shutil.disk_usage(ROOT).free / 1e9 if ROOT.exists() else 0
    print(f'{total:8.1f} MB  total   ({free:.1f} GB free)')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('list').set_defaults(fn=cmd_list)
    p = sub.add_parser('open'); p.add_argument('run', nargs='?'); p.set_defaults(fn=cmd_open)
    p = sub.add_parser('summary'); p.add_argument('run', nargs='?'); p.set_defaults(fn=cmd_summary)
    p = sub.add_parser('compare'); p.add_argument('a'); p.add_argument('b'); p.set_defaults(fn=cmd_compare)
    sub.add_parser('du').set_defaults(fn=cmd_du)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
