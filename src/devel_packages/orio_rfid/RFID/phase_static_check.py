#!/usr/bin/env python3
"""
phase_static_check.py - the static-base calibration of the c2 session, over SEVERAL reference tags at once.

    phase_static_check.py <static.csv> [--tag "Ref S1 BL=1.234" ...] [--compare other_static.csv] [--backface backface.csv]
                          [--out channel_offsets.json]

Per tag: is the phase one cluster per channel (no pi flip), how stable is it (R), RSSI and reads/s.
Per channel: the phase of every tag is split into
        phase[tag, ch] = expected(distance, f_ch) + tag_constant[tag] + channel_offset[ch] + residual
so the reader's channel offsets are separated from each tag's own constant (tag chip, mounting, multipath at that
spot). Distances are optional: a tag without one gets a fitted effective range (its phase slope across the band), and
only what is common to all tags across channels is kept; tape errors of a few cm change the expected phase by only ~0.01 rad per cm ACROSS the 902-928 MHz
band, so the channel offsets hardly depend on them (the tag constants absorb the rest).
  residual small (< ~0.3 rad)  -> one channel table describes the reader; usable as a fixed calibration
  residual large               -> channel offsets differ per tag (multipath); calibrate inside each pass from the
                                  reference tags instead (anchored-phase model)
--compare: the same split on a second static log (e.g. c2_static_end vs c2_static_start, or after a reader
power-cycle); prints how much the channel offsets moved after removing one global constant.
--backface: a static log with the panel turned 180 deg at the same spot; per tag, how many reads and how much RSSI
survive from behind (back lobe + reflections off the metal shelves). Tags still read well from behind are the
ones that can produce a mirror (front/back) solution in the localiser.
"""
import argparse
import csv
import json
import math
import os
import statistics
import sys

C = 299792458.0


def load(path):
    rows = [r for r in csv.DictReader(l for l in open(path) if not l.startswith('#')) if r.get('phase_rad')]
    side = path.replace('.csv', '_reader.json')
    hop = json.load(open(side)).get('hop_table_mhz', []) if os.path.exists(side) else []
    return rows, hop


def circ(vals):
    if not vals:
        return 0.0, 0.0
    sx = sum(math.cos(v) for v in vals); sy = sum(math.sin(v) for v in vals)
    return math.atan2(sy, sx), math.hypot(sx, sy) / len(vals)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def table(rows, hop, dist, min_reads, tags=None):
    """{tag: {ch: (circular mean phase - expected, R, n)}} for tags with enough reads."""
    by = {}
    for r in rows:
        name = r['name'] or r['epc'][-6:]
        if tags and name not in tags:
            continue
        by.setdefault(name, {}).setdefault(int(r['channel']), []).append(float(r['phase_rad']))
    out = {}
    for name, chans in by.items():
        t = {}
        for ch, ph in chans.items():
            if len(ph) < min_reads or not (1 <= ch <= len(hop)):
                continue
            m, R = circ(ph)
            if name in dist and hop:
                m = wrap(m - 4 * math.pi * dist[name] * hop[ch - 1] * 1e6 / C)
            t[ch] = (m, R, len(ph))
        if len(t) >= 10:
            out[name] = t
    return out


def decompose(tab, hop, fit_slope=(), iters=25):
    """Alternating fits: phase = tag_const + 4 pi d_tag f / c (tags in fit_slope only) + chan_off + residual."""
    tags = list(tab); chans = sorted({c for t in tab.values() for c in t})
    tc = {t: 0.0 for t in tags}; co = {c: 0.0 for c in chans}; dd = {t: 0.0 for t in tags}
    k = {c: 4 * math.pi * hop[c - 1] * 1e6 / C for c in chans}
    grid = [i * 0.005 for i in range(0, 1001)]          # 0 .. 5 m
    for _ in range(iters):
        for c in chans:
            v = [wrap(tab[t][c][0] - tc[t] - k[c] * dd[t]) for t in tags if c in tab[t]]
            co[c] = circ(v)[0] if v else 0.0
        for t in tags:
            if t in fit_slope:
                best = max(grid, key=lambda d: circ([tab[t][c][0] - co[c] - k[c] * d for c in tab[t]])[1])
                dd[t] = best
            tc[t] = circ([wrap(tab[t][c][0] - co[c] - k[c] * dd[t]) for c in tab[t]])[0]
    g = circ(list(co.values()))[0]
    co = {c: wrap(v - g) for c, v in co.items()}; tc = {t: wrap(v + g) for t, v in tc.items()}
    res = {t: [abs(wrap(tab[t][c][0] - tc[t] - co[c] - k[c] * dd[t])) for c in tab[t]] for t in tags}
    return tc, co, res, dd


def summary(path, dist, min_reads, top):
    rows, hop = load(path)
    if not hop:
        sys.exit('%s: no hop table (_reader.json) next to the log' % path)
    span = max(float(r['host_time_s']) for r in rows) - min(float(r['host_time_s']) for r in rows)
    tab = table(rows, hop, dist, min_reads)
    if dist:
        tab = {k: v for k, v in tab.items() if k in dist} or tab
    ranked = sorted(tab, key=lambda t: -sum(n for _, _, n in tab[t].values()))[:top]
    tab = {t: tab[t] for t in ranked}
    return rows, hop, span, tab


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log'); ap.add_argument('--tag', action='append', default=[], metavar='NAME=DIST_M',
                    help='reference tag and its distance from the antenna phase centre (panel back face) in m; repeatable')
    ap.add_argument('--compare', default=None); ap.add_argument('--backface', default=None); ap.add_argument('--out', default=None)
    ap.add_argument('--min-reads', type=int, default=6); ap.add_argument('--top', type=int, default=12, help='tags used when no --tag is given')
    a = ap.parse_args()
    dist = {}
    for s in a.tag:
        n, _, d = s.rpartition('=')
        dist[n] = float(d) if n else None
    dist = {k: v for k, v in dist.items() if v is not None}

    rows, hop, span, tab = summary(a.log, dist, a.min_reads, a.top)
    print('STATIC %s: %d reads in %.0f s, %d channels in the hop table, %d tags with >= 10 usable channels' % (
        os.path.basename(a.log), len(rows), span, len(hop), len(tab)))
    if len(tab) < 2:
        sys.exit('need at least 2 tags with enough reads per channel (longer log, or panel closer to the shelf)')
    print('\n%-14s %6s %6s %8s %7s %9s %10s' % ('tag', 'reads', 'chans', 'R med', 'pi-flip', 'RSSI dBm', 'dist m'))
    for t, chans in tab.items():
        trs = [r for r in rows if (r['name'] or r['epc'][-6:]) == t]
        flips = 0
        for ch in chans:
            ph = [float(r['phase_rad']) for r in trs if int(r['channel']) == ch]
            r1 = circ(ph)[1]; r2 = circ([2 * p for p in ph])[1]
            flips += (r2 > r1 + 0.25 and r2 > 0.7)
        rs = statistics.median(float(r['rssi_dbm']) for r in trs if r['rssi_dbm'])
        print('%-14s %6d %6d %8.2f %7d %9.1f %10s' % (t, len(trs), len(chans), statistics.median(R for _, R, _ in chans.values()), flips, rs,
                                                    '%.3f' % dist[t] if t in dist else '-'))
    fit = [t for t in tab if t not in dist]
    tc, co, res, dd = decompose(tab, hop, fit)
    allres = sorted(x for v in res.values() for x in v)
    print('\nsplit into tag constants + channel offsets: residual median %.2f rad, 90%% %.2f rad over %d tag-channels' % (
        statistics.median(allres), allres[int(0.9 * (len(allres) - 1))], len(allres)))
    for t in tab:
        print('  %-14s tag constant %+6.2f rad, residual median %.2f%s' % (t, tc[t], statistics.median(res[t]),
              '' if t in dist else ', fitted range %.2f m%s' % (dd[t], ' (grid edge: not a real range)' if dd[t] in (0.0, 5.0) else '')))
    spread = statistics.pstdev([v for v in co.values()])
    print('channel offsets: %d channels, spread (sd) %.2f rad%s' % (len(co), spread,
          '  -> negligible next to the residual: a channel table adds little' if spread < statistics.median(allres) else ''))
    verdict = ('ONE channel table fits all tags (residual small): fixed per-channel calibration is meaningful'
               if statistics.median(allres) < 0.3 else
               'channel offsets are NOT common to the tags (residual large): calibrate from reference tags inside each pass')
    if fit:
        verdict += ('  [%d of %d tags had no tape distance; their ranges were fitted, which makes the residual OPTIMISTIC - '
                    'give --tag "NAME=DIST" for the c2 static passes]' % (len(fit), len(tab)))
    print('VERDICT: ' + verdict)
    if a.out:
        json.dump({'log': os.path.basename(a.log), 'distances_m': dist, 'channel_offsets_rad': {str(c): v for c, v in co.items()},
                   'tag_constants_rad': tc, 'hop_table_mhz': hop, 'residual_median_rad': statistics.median(allres)},
                  open(a.out, 'w'), indent=1)
        print('saved', a.out)

    if a.compare:
        rows2, hop2, span2, tab2 = summary(a.compare, dist, a.min_reads, a.top)
        common = [t for t in tab if t in tab2]
        if len(common) < 2:
            print('\nCOMPARE: fewer than 2 tags in common with %s' % a.compare)
        else:
            fitc = [t for t in common if t not in dist]
            _, co2, _, _ = decompose({t: tab2[t] for t in common}, hop2, fitc); _, co1, _, _ = decompose({t: tab[t] for t in common}, hop, fitc)
            d = [wrap(co1[c] - co2[c]) for c in co1 if c in co2]
            g = circ(d)[0]; d = sorted(abs(wrap(x - g)) for x in d)
            print('\nCOMPARE with %s (%d common tags): global shift %+.2f rad; channel offsets moved by median %.2f rad, 90%% %.2f rad (%d channels)%s' % (
                os.path.basename(a.compare), len(common), g, d[len(d) // 2], d[int(0.9 * (len(d) - 1))], len(d),
                '  -> repeatable' if d[len(d) // 2] < 0.3 else '  -> NOT repeatable: per-session (per-pass) calibration needed'))

    if a.backface:
        rb, _ = load(a.backface)
        sb = max(float(r['host_time_s']) for r in rb) - min(float(r['host_time_s']) for r in rb)
        def per_tag(rr, s):
            d = {}
            for r in rr:
                d.setdefault(r['name'] or r['epc'][-6:], []).append(float(r['rssi_dbm']) if r['rssi_dbm'] else float('nan'))
            return {k: (len(v) / max(s, 1), statistics.median(v)) for k, v in d.items()}
        f, b = per_tag(rows, span), per_tag(rb, sb)
        print('\nFRONT vs BACK (panel turned 180 deg at the same spot): %.0f reads/s vs %.0f reads/s, %d vs %d tags' % (
            len(rows) / max(span, 1), len(rb) / max(sb, 1), len(f), len(b)))
        print('%-14s %10s %10s %9s %9s %8s' % ('tag', 'front r/s', 'back r/s', 'front dBm', 'back dBm', 'F/B dB'))
        shown = sorted(f, key=lambda k: -f[k][0])[:20]
        for k in shown:
            bb = b.get(k, (0.0, float('nan')))
            print('%-14s %10.1f %10.1f %9.1f %9.1f %8.1f' % (k, f[k][0], bb[0], f[k][1], bb[1], f[k][1] - bb[1] if bb[0] else float('nan')))
        only_back = [k for k in b if k not in f]
        if only_back:
            print('read ONLY with the panel facing away: %s' % ', '.join(sorted(only_back)[:20]))
        fb = [f[k][1] - b[k][1] for k in shown if k in b]
        if fb:
            print('median front/back RSSI difference over tags read both ways: %.1f dB (a panel alone is ~15-25 dB; much less = reflections)' % statistics.median(fb))


if __name__ == '__main__':
    main()
