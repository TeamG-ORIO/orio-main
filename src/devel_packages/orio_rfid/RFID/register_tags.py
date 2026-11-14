#!/usr/bin/env python3
"""
register_tags.py - register already-tagged items one at a time with the Impinj R700.

The reader runs at its LOWEST transmit power so only the tag held right in front of
the antenna answers strongly. Press Enter, the tool collects reads for 3 s and shows
the top 3 tags by signal strength and read count, then says whether one tag is
CLEARLY the one in front (big RSSI margin over the runner-up, not a known background
tag). Only a clear winner can be registered; an ambiguous result is never saved.

    python3 register_tags.py                      # antenna 4, lowest power, 3 s windows
    python3 register_tags.py --antennas 1 --duration 5 --margin 8

Keys at the prompt (as few as possible):
    Enter   scan 3 s; a CLEAR tag is registered automatically as "Item N", category dry
    l       the previous item is a liquid object    m   a metal object
    s       the previous item sits on the SAME object as the one before it (for comparing
            regular / on-metal / liquid tags on one object; tag_type comes from the EPC prefix)
    r       reject (delete) the previous item
    b       background scan: hold NOTHING in front, remember what the reader still sees
    q       quit

Registered tags go to tag_registry.csv next to this file
(epc,name,category=object material,tag_type,object=group id,...).
"""
import argparse
import csv
import logging
import os
import re
import socket
import sys
import threading
import time
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
REGISTRY = os.path.join(HERE, 'tag_registry.csv')
REGISTRY_FIELDS = ['epc', 'name', 'category', 'tag_type', 'object', 'registered_at', 'rssi_dbm', 'reads', 'antenna', 'power_dbm']
# tag_type is read off the EPC prefix: the three tag families in use here
TAG_FAMILIES = [('E28011700000021ACF3C', 'metal'), ('E2806995', 'liquid'), ('E28011B0A505', 'regular')]


def tag_type(epc):
    for prefix, kind in TAG_FAMILIES:
        if epc.startswith(prefix):
            return kind
    return 'unknown'
CATEGORIES = ('dry', 'liquid', 'metal')


def norm_epc(s):
    return ''.join(str(s).split()).upper()


# ------------------------------------------------------------------ registry (CSV)
def load_registry(path):
    reg = {}
    if not os.path.exists(path):
        return reg
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            if row.get('epc'):
                row.setdefault('tag_type', tag_type(norm_epc(row['epc'])))
                row.setdefault('object', '')
                reg[norm_epc(row['epc'])] = row
    return reg


def append_registry(path, row):
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=REGISTRY_FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


# ------------------------------------------------------------------ analysis (pure, testable)
def summarize(reads):
    """reads: list of (epc, rssi_dbm or None, antenna).  -> list of dicts sorted best first.

    Ranking: mean RSSI first (the tag in front of the antenna is the loudest one), read
    count as the tie-breaker. Both are shown so the operator can judge for themselves.
    """
    by = defaultdict(list)
    for epc, rssi, ant in reads:
        by[epc].append(rssi)
    total = sum(len(v) for v in by.values()) or 1
    rows = []
    for epc, vals in by.items():
        r = [v for v in vals if v is not None]
        rows.append({
            'epc': epc, 'reads': len(vals), 'share': 100.0 * len(vals) / total,
            'rssi_mean': sum(r) / len(r) if r else None,
            'rssi_max': max(r) if r else None,
        })
    rows.sort(key=lambda x: (x['rssi_mean'] if x['rssi_mean'] is not None else -999, x['reads']), reverse=True)
    return rows


def decide(rows, margin_db, min_reads, background):
    """Return (verdict, reasons). verdict is 'CLEAR', 'AMBIGUOUS' or 'NONE'.

    CLEAR only if ALL hold:
      * at least one tag seen, with >= min_reads reads
      * its mean RSSI beats the runner-up by >= margin_db (or there is no runner-up)
      * it is not a background tag, unless it is now >= margin_db louder than it was
        in the background scan (i.e. it really was brought up to the antenna)
    """
    if not rows:
        return 'NONE', ['no tags read at all - hold the tag closer to the antenna face']
    top = rows[0]
    reasons = []
    ok = True
    if top['reads'] < min_reads:
        ok = False
        reasons.append('only %d reads of the top tag (need >= %d)' % (top['reads'], min_reads))
    if top['rssi_mean'] is None:
        return 'AMBIGUOUS', ['reader did not report RSSI']
    if len(rows) > 1 and rows[1]['rssi_mean'] is not None:
        gap = top['rssi_mean'] - rows[1]['rssi_mean']
        if gap < margin_db:
            ok = False
            reasons.append('top two tags only %.1f dB apart (need >= %.0f dB): move the item onto the '
                           'antenna and other tagged items away' % (gap, margin_db))
        else:
            reasons.append('%.1f dB louder than the runner-up' % gap)
        if rows[1]['reads'] > top['reads']:
            reasons.append('note: runner-up was read MORE often (%d vs %d) - RSSI still favours the top tag'
                           % (rows[1]['reads'], top['reads']))
    else:
        reasons.append('only tag in range')
    bg = background.get(top['epc'])
    if bg is not None:
        rise = top['rssi_mean'] - bg
        if rise < margin_db:
            ok = False
            reasons.append('BACKGROUND tag: was %.1f dBm with nothing in front, now %.1f dBm (+%.1f dB, need >= %.0f)'
                           % (bg, top['rssi_mean'], rise, margin_db))
        else:
            reasons.append('seen in background at %.1f dBm but now %.1f dB louder, so it was brought forward'
                           % (bg, rise))
    return ('CLEAR' if ok else 'AMBIGUOUS'), reasons


def fmt_table(rows, registry, background, n=3):
    lines = ['%-3s %-26s %6s %6s %9s %9s  %s' % ('#', 'EPC', 'reads', 'share', 'RSSI avg', 'RSSI max', 'flags')]
    for i, r in enumerate(rows[:n], 1):
        flags = []
        if r['epc'] in registry:
            flags.append('REGISTERED as "%s"' % registry[r['epc']].get('name', ''))
        if r['epc'] in background:
            flags.append('background %.1f dBm' % background[r['epc']])
        lines.append('%-3d %-26s %6d %5.0f%% %9s %9s  %s' % (
            i, r['epc'], r['reads'], r['share'],
            '-' if r['rssi_mean'] is None else '%.1f' % r['rssi_mean'],
            '-' if r['rssi_max'] is None else '%.1f' % r['rssi_max'], ', '.join(flags)))
    if len(rows) > n:
        lines.append('    ... %d more tag(s) seen, %d reads in total' % (len(rows) - n, sum(r['reads'] for r in rows)))
    return '\n'.join(lines)


# ------------------------------------------------------------------ reader
class Collector:
    """Buffers tag reports from sllurp's reader thread while a window is open."""

    def __init__(self):
        self.lock = threading.Lock()
        self.open = False
        self.reads = []
        self.total = 0

    def on_tags(self, reader, reports):
        with self.lock:
            self.total += len(reports)
            if not self.open:
                return
            for tag in reports:
                epc = tag.get('EPC-96', tag.get('EPCData', b''))
                epc = norm_epc(epc.decode('ascii', 'replace') if isinstance(epc, bytes) else epc)
                rssi = tag.get('ImpinjPeakRSSI')
                rssi = rssi / 100.0 if rssi is not None else tag.get('PeakRSSI')
                n = tag.get('TagSeenCount', 1) or 1
                self.reads.extend([(epc, rssi, tag.get('AntennaID'))] * int(n))

    def window(self, seconds):
        with self.lock:
            self.reads = []
            self.open = True
        time.sleep(seconds)
        with self.lock:
            self.open = False
            return list(self.reads)


def port_open(ip, port=5084, timeout=2.0):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def connect(args):
    from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT
    logging.getLogger('sllurp').setLevel(logging.WARNING)
    if not port_open(args.ip):
        sys.exit('cannot reach the reader at %s:5084.\n'
                 '  - is the Ethernet cable from the R700 plugged into the robot (eno1)?\n'
                 '  - NetworkManager profile "impinj-rfid" should give the robot 192.168.0.10\n'
                 '  - try: ping %s' % (args.ip, args.ip))
    cfg = {
        'antennas': args.antennas,
        'report_every_n_tags': 1,
        # session 0 + Dual Target: a tag answers on every inventory round, so the read count
        # really measures how often the reader saw it in the window
        'session': 0,
        'tag_population': 4,
        'tag_content_selector': {'EnableAntennaID': True, 'EnableChannelIndex': True, 'EnablePeakRSSI': True,
                                 'EnableTagSeenCount': True, 'EnableLastSeenTimestamp': True},
        'impinj_tag_content_selector': {'EnableRFPhaseAngle': False, 'EnablePeakRSSI': True,
                                        'EnableRFDopplerFrequency': False},
        # sllurp picks the table entry closest to this. Do NOT request 0 here: table index 0
        # is a placeholder that sllurp treats as "maximum power".
        'tx_power_dbm': float(args.power),
    }
    reader = LLRPReaderClient(args.ip, LLRP_DEFAULT_PORT, LLRPReaderConfig(cfg))
    col = Collector()
    reader.add_tag_report_callback(col.on_tags)
    print('connecting to %s ...' % args.ip)
    reader.connect()
    time.sleep(1.0)  # let capabilities + ROSpec settle so the power table is known
    table = [p for p in getattr(reader.llrp, 'tx_power_table', []) if p]
    used = {}
    if table:
        idx = getattr(reader.llrp.config, 'tx_power', {}) or {}
        used = {ant: reader.llrp.tx_power_table[i] for ant, i in idx.items()}
        lo, hi = min(table), max(table)
        print('reader power range %.2f .. %.2f dBm; using %s' % (
            lo, hi, ', '.join('ant %s = %.2f dBm' % kv for kv in sorted(used.items())) or '?'))
        if used and min(used.values()) > lo + 0.01:
            print('WARNING: not at the reader minimum (%.2f dBm). Re-run with --power %.2f' % (lo, lo))
    return reader, col, used


# ------------------------------------------------------------------ main loop
def save_registry(path, registry):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=REGISTRY_FIELDS)
        w.writeheader()
        for r in registry.values():
            w.writerow({k: r.get(k, '') for k in REGISTRY_FIELDS})


def next_item_number(registry):
    nums = [int(m.group(1)) for r in registry.values() for m in [re.match(r'Item (\d+)$', r.get('name', ''))] if m]
    return (max(nums) + 1) if nums else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ip', default='192.168.0.20')
    ap.add_argument('--antennas', type=int, nargs='+', default=[4], help='antenna port(s), default 4')
    ap.add_argument('--power', type=float, default=10.0,
                    help='dBm requested; 10 is the R700 minimum. sllurp uses the closest table entry')
    ap.add_argument('--duration', type=float, default=3.0, help='seconds per scan window')
    ap.add_argument('--bg-duration', type=float, default=10.0,
                    help='seconds for the b background scan; longer catches tags flickering at the edge of range')
    ap.add_argument('--margin', type=float, default=6.0,
                    help='dB the winner must beat the runner-up (and its own background level) by')
    ap.add_argument('--min-reads', type=int, default=5, help='minimum reads of the winner in the window')
    ap.add_argument('--registry', default=REGISTRY)
    args = ap.parse_args()

    registry = load_registry(args.registry)
    background = {}  # epc -> mean RSSI with nothing in front
    last = None      # epc of the most recent registration, for l / m / r / s
    prev = None      # the one before it, for s
    print('%d tag(s) already registered in %s' % (len(registry), args.registry))
    reader, col, used = connect(args)
    power_dbm = min(used.values()) if used else args.power
    print('\nEnter = scan %.0f s and register the tag in front as "Item N" (dry)\n'
          'l = previous item is a liquid object   m = metal object   r = reject previous item\n'
          's = previous item is on the SAME object as the one before it (tag type is read off the EPC)\n'
          'b = background scan (nothing in front)   q = quit\n' % args.duration)
    try:
        while True:
            try:
                key = input('> ').strip().lower()
            except EOFError:
                break
            if key == 'q':
                break
            if key in ('l', 'm', 'r', 's'):
                if last is None or last not in registry:
                    print('nothing to change: no item registered yet in this session')
                    continue
                row = registry[last]
                if key == 'r':
                    del registry[last]
                    save_registry(args.registry, registry)
                    print('rejected "%s" (%s) - %d registered' % (row['name'], last, len(registry)))
                    last = prev
                    prev = None
                elif key == 's':
                    if prev is None or prev not in registry:
                        print('nothing to group with: need two items registered in a row')
                        continue
                    obj = registry[prev]['object'] or registry[prev]['name']
                    registry[prev]['object'] = obj
                    row['object'] = obj
                    row['category'] = registry[prev]['category']
                    save_registry(args.registry, registry)
                    members = [r['name'] for r in registry.values() if r['object'] == obj]
                    print('"%s" is on the same object as %s -> object "%s" (%s tag)' % (
                        row['name'], ', '.join(m for m in members if m != row['name']), obj, row['tag_type']))
                else:
                    cat = 'liquid' if key == 'l' else 'metal'
                    changed = [row] + [r for r in registry.values() if row['object'] and r['object'] == row['object'] and r is not row]
                    for r in changed:
                        r['category'] = cat
                    save_registry(args.registry, registry)
                    print('"%s" (%s) is now a %s object%s' % (row['name'], last, cat,
                          '' if len(changed) == 1 else ' (and its %d grouped tags)' % (len(changed) - 1)))
                continue
            if key == 'b':
                print('background scan, %.0f s - keep tagged items (and yourself) away from the antenna ...' % args.bg_duration)
                rows = summarize(col.window(args.bg_duration))
                # keep earlier background tags too: a marginal tag can miss one scan and show in the next
                background.update({r['epc']: r['rssi_mean'] for r in rows if r['rssi_mean'] is not None})
                if rows:
                    print(fmt_table(rows, registry, {}, n=10))
                    print('-> %d background tag(s) remembered (accumulates over repeated b scans).' % len(background))
                else:
                    print('-> nothing seen: no background tags at this power. Good.')
                continue
            if key != '':
                print('unknown key %r (Enter, l, m, r, b, q)' % key)
                continue

            print('scanning %.0f s ...' % args.duration)
            rows = summarize(col.window(args.duration))
            print(fmt_table(rows, registry, background))
            verdict, reasons = decide(rows, args.margin, args.min_reads, background)
            if verdict != 'CLEAR':
                print('%s: %s  -> NOT registered, press Enter to retry' % (verdict, '; '.join(reasons)))
                continue
            top = rows[0]
            if top['epc'] in registry:
                if top['epc'] != last:
                    prev, last = last, top['epc']
                print('CLEAR but already registered as "%s" - nothing added. Next item.' % registry[top['epc']]['name'])
                continue
            name = 'Item %d' % next_item_number(registry)
            row = {'epc': top['epc'], 'name': name, 'category': 'dry', 'tag_type': tag_type(top['epc']), 'object': '',
                   'registered_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                   'rssi_dbm': '%.1f' % top['rssi_mean'], 'reads': top['reads'],
                   'antenna': ','.join(map(str, args.antennas)), 'power_dbm': '%.2f' % power_dbm}
            append_registry(args.registry, row)
            registry[top['epc']] = row
            prev, last = last, top['epc']
            print('CLEAR (%s)  ->  registered "%s" = %s  (%s tag)   [%d total]' % (
                '; '.join(reasons), name, top['epc'], row['tag_type'], len(registry)))
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndisconnecting ...')
        reader.disconnect()
        print('%d tag(s) in %s' % (len(registry), args.registry))


if __name__ == '__main__':
    main()
