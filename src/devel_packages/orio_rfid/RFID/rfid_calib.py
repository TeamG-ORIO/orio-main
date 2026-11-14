#!/usr/bin/env python3
"""
rfid_calib.py - Stage 0 phase calibration from STATIC logs made with rfid_log.py (antenna and tag not moving).

    python3 rfid_calib.py ambiguity <log.csv> [--tag "Ref S1 BL"]
        Per channel: is the phase of a static tag one cluster, or two clusters pi apart? Two clusters mean the
        reader's phase has a sign ambiguity and the SAR code must double the phase (2*phi mod 2pi).

    python3 rfid_calib.py offsets <log.csv> --tag "Ref S1 BL" --distance-m 1.20 [--double] [--out offsets.json]
        Per antenna and channel: measured phase minus the phase expected for a tag at that distance
        (round trip 4*pi*d*f/c). Save the offsets; a second static log with --compare offsets.json reports how
        repeatable they are (the plan wants a few tenths of a radian).

    python3 rfid_calib.py offsets <log2.csv> --tag ... --distance-m ... --compare offsets.json
"""
import argparse
import csv
import json
import math
import os
import sys

C = 299792458.0


def load(path, tag=None):
    rows = [r for r in csv.DictReader(l for l in open(path) if not l.startswith('#')) if r.get('phase_rad')]
    if tag:
        rows = [r for r in rows if r['name'] == tag or r['epc'].endswith(tag)]
    side = path.replace('.csv', '_reader.json')
    hop = json.load(open(side)).get('hop_table_mhz', []) if os.path.exists(side) else []
    return rows, hop


def circ(vals):   # circular mean and resultant length (1 = all identical, 0 = uniform)
    if not vals:
        return None, 0.0
    sx = sum(math.cos(v) for v in vals); sy = sum(math.sin(v) for v in vals)
    return math.atan2(sy, sx), math.hypot(sx, sy) / len(vals)


def by_channel(rows):
    d = {}
    for r in rows:
        d.setdefault((r['antenna'], int(r['channel'])), []).append(float(r['phase_rad']))
    return d


def ambiguity(args):
    rows, _ = load(args.log, args.tag)
    if not rows:
        sys.exit('no phase rows for that tag')
    print('%d reads of %s' % (len(rows), args.tag or 'all tags'))
    print('%-4s %-4s %6s %8s %8s   %s' % ('ant', 'ch', 'reads', 'R(phi)', 'R(2phi)', 'verdict'))
    votes = {'single': 0, 'double': 0}
    for (a, ch), ph in sorted(by_channel(rows).items()):
        if len(ph) < 6:
            continue
        _, r1 = circ(ph); _, r2 = circ([2 * p for p in ph])
        v = 'two clusters pi apart -> double the phase' if r2 > r1 + 0.25 and r2 > 0.7 else ('one cluster' if r1 > 0.7 else 'spread (multipath / moving?)')
        votes['double' if v.startswith('two') else 'single'] += 1
        print('%-4s %-4d %6d %8.2f %8.2f   %s' % (a, ch, len(ph), r1, r2, v))
    print('channels voting: single %d, double %d  ->  %s' % (votes['single'], votes['double'], 'DOUBLE the phase in the SAR code' if votes['double'] > votes['single'] else 'use the phase as reported'))


def offsets(args):
    rows, hop = load(args.log, args.tag)
    if not rows:
        sys.exit('no phase rows for that tag')
    if not hop:
        sys.exit('no hop table next to the log (rfid_log.py writes <log>_reader.json); cannot convert channel index to frequency')
    out = {}
    print('%d reads of %s at %.3f m; %d channels in the hop table' % (len(rows), args.tag, args.distance_m, len(hop)))
    print('%-4s %-4s %9s %6s %9s %7s' % ('ant', 'ch', 'MHz', 'reads', 'offset', 'R'))
    for (a, ch), ph in sorted(by_channel(rows).items()):
        if ch < 1 or ch > len(hop):
            continue
        f = hop[ch - 1] * 1e6
        exp = (4 * math.pi * args.distance_m * f / C) % (2 * math.pi)
        meas = [((2 * p) % (2 * math.pi)) if args.double else p for p in ph]
        if args.double:
            exp = (2 * exp) % (2 * math.pi)
        off, R = circ([(m - exp) for m in meas])
        out['%s/%d' % (a, ch)] = {'antenna': a, 'channel': ch, 'mhz': hop[ch - 1], 'offset_rad': off, 'R': R, 'reads': len(ph)}
        print('%-4s %-4d %9.2f %6d %9.3f %7.2f' % (a, ch, hop[ch - 1], len(ph), off, R))
    if args.out:
        json.dump({'tag': args.tag, 'distance_m': args.distance_m, 'double': args.double, 'log': os.path.basename(args.log), 'offsets': out}, open(args.out, 'w'), indent=1)
        print('saved', args.out)
    if args.compare:
        ref = json.load(open(args.compare))['offsets']; diffs = []
        for k, v in out.items():
            if k in ref:
                d = (v['offset_rad'] - ref[k]['offset_rad'] + math.pi) % (2 * math.pi) - math.pi; diffs.append(abs(d))
        if diffs:
            diffs.sort(); print('repeatability vs %s: %d channels, median |diff| %.2f rad, 90%% %.2f rad (want a few tenths)' % (args.compare, len(diffs), diffs[len(diffs) // 2], diffs[int(0.9 * (len(diffs) - 1))]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('ambiguity'); a.add_argument('log'); a.add_argument('--tag', default=None); a.set_defaults(fn=ambiguity)
    o = sub.add_parser('offsets'); o.add_argument('log'); o.add_argument('--tag', required=True); o.add_argument('--distance-m', type=float, required=True)
    o.add_argument('--double', action='store_true'); o.add_argument('--out', default=None); o.add_argument('--compare', default=None); o.set_defaults(fn=offsets)
    args = ap.parse_args(); args.fn(args)


if __name__ == '__main__':
    main()
