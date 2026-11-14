"""
RFID_T2.py - Tracking Subsystem Hardware Test (T2): scan for a fixed list of tags, log power + phase
locally, then report detection rates per item category and plot power / phase over time.

Tag list (default t2_tags.csv next to this file), one item per line:
    epc,name,category,distance_m
    E28011B0A50500726AC8F408,gauze box,dry,1.2
category: dry (non liquid/metal) | liquid | metal.  Lines starting with # are ignored.

  1. scan      python3 RFID_T2.py scan scan1                 # until Ctrl-C; live table of the listed tags
               python3 RFID_T2.py scan scan2 --duration 120 --power 30 --antennas 4
     Every read of a LISTED tag goes to t2_logs/<label>_<time>.csv (other tags are only counted).
     At the end the report below is produced for that scan.

  2. report    python3 RFID_T2.py report t2_logs/scan1_*.csv [more logs ...]
     Per scan: detection table, pass/fail against T2 (dry >= 80 %, liquid/metal >= 70 %),
     noise figures, and per-item power + phase plots in t2_logs/<log name>_plots/.

Noise figures (per tag): RSSI std over the scan, and read-to-read noise = std of the difference
between consecutive reads / sqrt(2), which ignores slow changes from the robot moving. Phase noise
is computed the same way but only between consecutive reads on the SAME channel less than 0.5 s
apart (phase is not comparable across the reader's hop channels), wrapped to +/-90 deg because the
reader's phase has a 180 deg ambiguity.
"""
import argparse
import csv
import logging
import math
import os
import sys
import time
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, 't2_logs')
CATEGORIES = ('dry', 'liquid', 'metal')
CRITERIA = {'dry': 80.0, 'liquid': 70.0, 'metal': 70.0, 'liquid+metal': 70.0}
FIELDS = ['t', 'reader_utc_us', 'epc', 'name', 'category', 'distance_m', 'antenna', 'channel', 'rssi_dbm', 'phase_deg']


def norm_epc(s):
    return ''.join(str(s).split()).upper()


def load_tags(path):
    tags = {}
    with open(path) as f:
        for row in csv.reader(l for l in f if l.strip() and not l.lstrip().startswith('#')):
            if row[0].strip().lower() == 'epc':
                continue
            row = [c.strip() for c in row] + [''] * 4
            cat = row[2].lower() or 'dry'
            if cat not in CATEGORIES:
                sys.exit('%s: category of %s must be one of %s, not "%s"' % (path, row[0], '/'.join(CATEGORIES), row[2]))
            tags[norm_epc(row[0])] = {'name': row[1] or norm_epc(row[0])[-6:], 'category': cat, 'distance_m': row[3]}
    if not tags:
        sys.exit('%s has no tags yet - add lines "epc,name,category,distance_m"' % path)
    return tags


# ------------------------------------------------------------------ scan
def scan(args):
    from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT
    logging.getLogger('sllurp').setLevel(logging.WARNING)
    tags = load_tags(args.tags)
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, '%s_%s.csv' % (args.label, time.strftime('%Y%m%d_%H%M%S')))
    stats = {e: {'n': 0, 'rssi': None, 'phase': None, 'last': None} for e in tags}
    other = set()
    t0 = time.time()
    logf = open(log_path, 'w', newline='')
    out = csv.writer(logf)
    out.writerow(FIELDS)

    def on_tags(reader, reports):
        now = time.time() - t0
        for tag in reports:
            epc = tag.get('EPC-96', tag.get('EPCData', b''))
            epc = norm_epc(epc.decode('ascii', 'replace') if isinstance(epc, bytes) else epc)
            if epc not in tags:
                other.add(epc)
                continue
            raw = tag.get('ImpinjRFPhaseAngle')
            phase = raw * 360.0 / 4096.0 if raw is not None else None
            rssi = tag.get('ImpinjPeakRSSI')
            rssi = rssi / 100.0 if rssi is not None else tag.get('PeakRSSI')
            info = tags[epc]
            out.writerow(['%.4f' % now, tag.get('LastSeenTimestampUTC', ''), epc, info['name'], info['category'],
                          info['distance_m'], tag.get('AntennaID', ''), tag.get('ChannelIndex', ''),
                          '' if rssi is None else '%.2f' % rssi, '' if phase is None else '%.2f' % phase])
            s = stats[epc]
            s['n'] += 1
            s['rssi'], s['phase'], s['last'] = rssi, phase, now

    cfg = {
        'antennas': args.antennas, 'report_every_n_tags': 1,
        # session 0: a tag answers again on every inventory round, so each item gives a continuous
        # power / phase series (the sllurp default, session 2, reads a powered tag once and then it stays quiet)
        'session': args.session, 'tag_population': args.population,
        'tag_content_selector': {'EnableAntennaID': True, 'EnableChannelIndex': True, 'EnablePeakRSSI': True,
                                 'EnableTagSeenCount': True, 'EnableLastSeenTimestamp': True},
        'impinj_tag_content_selector': {'EnableRFPhaseAngle': True, 'EnablePeakRSSI': True,
                                        'EnableRFDopplerFrequency': False},
    }
    if args.power == 'max':
        cfg['tx_power'] = 0
    else:
        cfg['tx_power_dbm'] = float(args.power)
    reader = LLRPReaderClient(args.ip, LLRP_DEFAULT_PORT, LLRPReaderConfig(cfg))
    reader.add_tag_report_callback(on_tags)
    print('connecting to %s ...' % args.ip)
    reader.connect()
    t0 = time.time()

    def draw():
        now = time.time() - t0
        lines = ['T2 scan "%s"   %.0f s   power %s dBm   antennas %s   (Ctrl-C to finish)' % (
            args.label, now, args.power, args.antennas), '',
            '%-3s %-22s %-7s %6s %6s %9s %10s %8s' % ('', 'item', 'cat', 'dist', 'reads', 'RSSI dBm', 'phase deg', 'last seen')]
        for epc, info in tags.items():
            s = stats[epc]
            lines.append('%-3s %-22s %-7s %6s %6d %9s %10s %8s' % (
                'OK' if s['n'] else '--', info['name'][:22], info['category'], info['distance_m'] or '-', s['n'],
                '-' if s['rssi'] is None else '%.1f' % s['rssi'], '-' if s['phase'] is None else '%.1f' % s['phase'],
                '-' if s['last'] is None else '%.0f s ago' % (now - s['last'])))
        seen = sum(1 for s in stats.values() if s['n'])
        lines += ['', 'detected %d / %d listed tags   (%d other tags in range, ignored)' % (seen, len(tags), len(other))]
        sys.stdout.write('\033[H\033[J' + '\n'.join(lines) + '\n')
        sys.stdout.flush()

    try:
        while args.duration is None or time.time() - t0 < args.duration:
            if sys.stdout.isatty():
                draw()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    reader.disconnect()
    logf.close()
    print('\nlog: %s' % log_path)
    report_one(log_path, tags, args.plots)


# ------------------------------------------------------------------ report
def std(v):
    if len(v) < 2:
        return float('nan')
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def wrap90(d):
    return (d + 90.0) % 180.0 - 90.0


def report_one(log_path, tags, plots=True):
    rows = defaultdict(list)
    with open(log_path) as f:
        for r in csv.DictReader(f):
            rows[r['epc']].append(r)
    dur = max((float(r['t']) for v in rows.values() for r in v), default=0.0)
    print('\n================ %s   (%.0f s of reads) ================' % (os.path.basename(log_path), dur))
    print('%-3s %-22s %-7s %6s %6s %9s %8s %10s %10s' % (
        '', 'item', 'cat', 'dist', 'reads', 'RSSI avg', 'RSSI sd', 'RSSI noise', 'phase noise'))
    seen = defaultdict(int)
    total = defaultdict(int)
    series = {}
    for epc, info in tags.items():
        total[info['category']] += 1
        v = rows.get(epc, [])
        t = [float(r['t']) for r in v]
        rssi = [float(r['rssi_dbm']) for r in v if r['rssi_dbm']]
        d_rssi = [float(b['rssi_dbm']) - float(a['rssi_dbm']) for a, b in zip(v, v[1:])
                  if a['rssi_dbm'] and b['rssi_dbm'] and float(b['t']) - float(a['t']) < 0.5]
        d_ph = [wrap90(float(b['phase_deg']) - float(a['phase_deg'])) for a, b in zip(v, v[1:])
                if a['phase_deg'] and b['phase_deg'] and a['channel'] == b['channel'] and float(b['t']) - float(a['t']) < 0.5]
        if v:
            seen[info['category']] += 1
        series[epc] = (t, v)
        f1 = lambda x: '-' if x != x else '%.2f' % x
        print('%-3s %-22s %-7s %6s %6d %9s %8s %10s %10s' % (
            'OK' if v else '--', info['name'][:22], info['category'], info['distance_m'] or '-', len(v),
            '%.1f' % (sum(rssi) / len(rssi)) if rssi else '-', f1(std(rssi)),
            f1(std(d_rssi) / math.sqrt(2)) if len(d_rssi) > 1 else '-',
            f1(std(d_ph) / math.sqrt(2)) if len(d_ph) > 1 else '-'))
    print('(RSSI in dBm / dB, phase noise in deg; noise = read-to-read, see the top of this file)')
    print('\ndetection:')
    groups = [(c, seen[c], total[c]) for c in CATEGORIES if total[c]]
    if total['liquid'] and total['metal']:
        groups.append(('liquid+metal', seen['liquid'] + seen['metal'], total['liquid'] + total['metal']))
    for name, s, n in groups:
        pct = 100.0 * s / n
        print('  %-13s %2d / %2d = %5.1f %%   need >= %.0f %%  ->  %s' % (
            name, s, n, pct, CRITERIA[name], 'PASS' if pct >= CRITERIA[name] else 'FAIL'))

    if not plots:
        return
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed - no plots')
        return
    pdir = log_path[:-4] + '_plots'
    os.makedirs(pdir, exist_ok=True)
    n = 0
    for epc, (t, v) in series.items():
        if not v:
            continue
        info = tags[epc]
        fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 5.5))
        a1.plot([float(r['t']) for r in v if r['rssi_dbm']], [float(r['rssi_dbm']) for r in v if r['rssi_dbm']],
                '.', ms=4, color='#1f77b4')
        a1.set_ylabel('RSSI (dBm)')
        a1.set_title('%s  [%s, %s m]  %s  -  %d reads' % (info['name'], info['category'], info['distance_m'] or '?', epc, len(v)),
                     fontsize=10)
        ph = [r for r in v if r['phase_deg']]
        sc = a2.scatter([float(r['t']) for r in ph], [float(r['phase_deg']) for r in ph],
                        c=[int(r['channel'] or 0) for r in ph], cmap='viridis', s=8)
        fig.colorbar(sc, ax=[a1, a2], label='hop channel (phase is comparable only within one channel)')
        a2.set_ylabel('phase (deg)')
        a2.set_ylim(0, 360)
        a2.set_xlabel('time since scan start (s)')
        for a in (a1, a2):
            a.grid(alpha=0.3)
        fig.savefig(os.path.join(pdir, '%s_%s.png' % (info['category'], ''.join(c if c.isalnum() else '_' for c in info['name']))), dpi=110)
        plt.close(fig)
        n += 1
    print('\n%d power/phase plots in %s' % (n, pdir))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tags', default=os.path.join(HERE, 't2_tags.csv'), help='tag list CSV (epc,name,category,distance_m)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('scan', help='scan for the listed tags and log power + phase')
    s.add_argument('label', help='name of this scan, e.g. scan1, scan2_after_1h, scan3_liquid_metal')
    s.add_argument('--ip', default='192.168.0.20')
    s.add_argument('--antennas', type=int, nargs='+', default=[4])
    s.add_argument('--power', default='30', help="transmit power in dBm (reader range 10-33) or 'max'")
    s.add_argument('--session', type=int, default=0, choices=[0, 1, 2, 3],
                   help='Gen2 session: 0 = tags re-read continuously (time series); 2 = each tag read once, strong tags then stay quiet (helps weak tags get found)')
    s.add_argument('--population', type=int, default=32, help='expected number of tags in range (anti-collision hint)')
    s.add_argument('--duration', type=float, default=None, help='seconds; default = until Ctrl-C')
    s.add_argument('--no-plots', dest='plots', action='store_false')
    r = sub.add_parser('report', help='detection statistics, noise and plots from saved logs')
    r.add_argument('logs', nargs='+')
    r.add_argument('--no-plots', dest='plots', action='store_false')
    args = ap.parse_args()
    if args.cmd == 'scan':
        scan(args)
    else:
        tags = load_tags(args.tags)
        for p in args.logs:
            report_one(p, tags, args.plots)


if __name__ == '__main__':
    main()
