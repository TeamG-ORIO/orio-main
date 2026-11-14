#!/usr/bin/env python3
"""
rfid_log.py - log every tag read with phase, channel, RSSI and timestamps for the corridor passes,
and report the per-tag read rate (which sets the robot speed).

    python3 rfid_log.py scan <label> [--power 30] [--mode 100] [--search-mode 2] [--session 0] [--duration 60]
    python3 rfid_log.py report <log.csv> [more logs ...]

Every read goes to <out-dir>/<label>_<time>.csv:
    host_time_s      robot clock when the report arrived (same clock as the ROS bag)
    reader_us        reader's FirstSeenTimestampUTC (microseconds, reader clock) - for the host<->reader offset
    epc, name, category, tag_type, object   from tag_registry.csv (unregistered tags are still logged)
    antenna, channel    reader antenna port, hop-table channel index
    phase_rad           ImpinjRFPhaseAngle * 2pi / 4096
    rssi_dbm            ImpinjPeakRSSI / 100
    power_dbm           transmit power requested

Settings that matter for SAR data, measured 2026-10-21 with ~55 tags in range: mode 0, Dual Target, session 0,
reports every 20 reads -> ~480 phase records/s, median 5-6 per tag (one report per read capped the reader at 37/s).
Mode 2 (dense reader) is 5x slower but read 76 tags instead of 53: use it for a slow coverage/presence pass. The report gives per-tag reads/s and
the speed at which the robot still gets a read every 3.5 cm.
"""
import argparse
import csv
import logging
import math
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
import register_tags as rt  # noqa: E402

FIELDS = ['host_time_s', 'reader_us', 'epc', 'name', 'category', 'tag_type', 'object', 'antenna', 'channel', 'phase_rad', 'rssi_dbm', 'power_dbm', 'seen']


def scan(args):
    from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT
    logging.getLogger('sllurp').setLevel(logging.WARNING)
    registry = rt.load_registry(args.registry)
    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, '%s_%s.csv' % (args.label, time.strftime('%Y%m%d_%H%M%S')))
    f = open(path, 'w', newline='', buffering=1); w = csv.writer(f); w.writerow(FIELDS)
    f.write('# ' + ' '.join(sys.argv) + '\n')
    counts = {}; recent = []; t_start = time.time(); closed = [False]; reports = [0]

    def on_tags(reader, tag_reports):
        if closed[0]:
            return
        now = time.time(); reports[0] += 1
        for tag in tag_reports:
            epc = tag.get('EPC-96', tag.get('EPCData', b''))
            epc = rt.norm_epc(epc.decode('ascii', 'replace') if isinstance(epc, bytes) else epc)
            r = registry.get(epc, {})
            raw = tag.get('ImpinjRFPhaseAngle'); phase = raw * 2 * math.pi / 4096.0 if raw is not None else None
            rssi = tag.get('ImpinjPeakRSSI'); rssi = rssi / 100.0 if rssi is not None else tag.get('PeakRSSI')
            n = tag.get('TagSeenCount', 1) or 1   # reads merged into this entry by the reader (phase/channel belong to one of them)
            w.writerow(['%.6f' % now, tag.get('FirstSeenTimestampUTC', ''), epc, r.get('name', ''), r.get('category', ''), r.get('tag_type', ''), r.get('object', ''),
                        tag.get('AntennaID', ''), tag.get('ChannelIndex', ''), '' if phase is None else '%.4f' % phase, '' if rssi is None else '%.2f' % rssi, args.power, n])
            counts[epc] = counts.get(epc, 0) + 1; recent.append((now, epc))

    cfg = {
        'antennas': args.antennas, 'report_every_n_tags': args.report_every, 'session': args.session, 'tag_population': args.population,
        'tx_power_dbm': float(args.power),
        'tag_content_selector': {'EnableAntennaID': True, 'EnableChannelIndex': True, 'EnablePeakRSSI': True, 'EnableTagSeenCount': True,
                                 'EnableFirstSeenTimestamp': True, 'EnableLastSeenTimestamp': False},
        'impinj_tag_content_selector': {'EnableRFPhaseAngle': True, 'EnablePeakRSSI': True, 'EnableRFDopplerFrequency': False},
    }
    if args.mode is not None:
        cfg['mode_identifier'] = args.mode
    if args.search_mode is not None:
        cfg['impinj_search_mode'] = args.search_mode
    reader = LLRPReaderClient(args.ip, LLRP_DEFAULT_PORT, LLRPReaderConfig(cfg))
    reader.add_tag_report_callback(on_tags)
    print('connecting to %s (power %s dBm, mode %s, search mode %s, session %d) ...' % (args.ip, args.power, args.mode, args.search_mode, args.session))
    reader.connect()
    print('logging to %s   (Ctrl-C to stop)' % path)
    def dump_caps():
        # reader capabilities once per log: the FCC hop table (channel index -> MHz, needed for wavelength-aware SAR),
        # the power table and the settings used
        try:
            import json
            caps = reader.llrp.capabilities
            hop = []
            try:
                fi = caps['RegulatoryCapabilities']['UHFBandCapabilities']['FrequencyInformation']
                fht = fi.get('FrequencyHopTable', fi.get('FrequencyHopTable0', {}))
                if isinstance(fht, dict):
                    fht = [fht]
                for t in fht:
                    hop = [f / 1000.0 for f in (t.get('Frequency') if isinstance(t.get('Frequency'), list) else [v for k, v in sorted(t.items()) if k.startswith('Frequency')])]
            except Exception as e:
                print('hop table not parsed:', e)
            side = {'settings': {'power_dbm': args.power, 'mode': args.mode, 'search_mode': args.search_mode, 'session': args.session, 'report_every': args.report_every, 'antennas': args.antennas},
                    'hop_table_mhz': hop, 'tx_power_table_dbm': list(getattr(reader.llrp, 'tx_power_table', [])), 'capabilities': caps}
            json.dump(side, open(path.replace('.csv', '_reader.json'), 'w'), indent=1, default=str)
            print('hop table: %d channels (%s ... %s MHz); power table max %s dBm -> %s' % (len(hop), hop[0] if hop else '?', hop[-1] if hop else '?',
                  max(side['tx_power_table_dbm']) if side['tx_power_table_dbm'] else '?', path.replace('.csv', '_reader.json')))
        except Exception as e:
            print('capabilities not saved:', e)
        return bool(hop)
    import signal

    def _term(signum, frame):     # SIGTERM (scripts/rfid_pass.sh stops the logger this way) = Ctrl-C: close cleanly
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    try:
        last = time.time(); dumped = False
        while args.duration is None or time.time() - t_start < args.duration:
            time.sleep(0.5)
            if not dumped and time.time() - t_start > 1.5:
                dumped = dump_caps() or (time.time() - t_start > 10)
            if time.time() - last >= 2.0:
                now = time.time(); cutoff = now - 2.0
                while recent and recent[0][0] < cutoff:
                    recent.pop(0)
                tags2 = {}
                for _, e in recent:
                    tags2[e] = tags2.get(e, 0) + 1
                per_tag = sorted(tags2.values())
                med = per_tag[len(per_tag) // 2] / 2.0 if per_tag else 0
                print('%6.0f s  %5.1f reads/s  %3d tags in the last 2 s  median %.1f reads/s per tag  (%d reads total)' % (
                    now - t_start, len(recent) / 2.0, len(tags2), med, sum(counts.values())))
                last = now
    except KeyboardInterrupt:
        pass
    finally:
        closed[0] = True; print('disconnecting ...'); reader.disconnect(); time.sleep(0.5); f.close()
        print('%d LLRP reports received' % reports[0])
        print('%d reads of %d tags in %.0f s -> %s' % (sum(counts.values()), len(counts), time.time() - t_start, path))
        report(argparse.Namespace(logs=[path], top=15, step_cm=3.5))


def report(args):
    for path in args.logs:
        rows = [r for r in csv.DictReader(l for l in open(path) if not l.startswith('#'))]
        if not rows:
            print(path, ': empty'); continue
        t = [float(r['host_time_s']) for r in rows]; T = max(t) - min(t) or 1e-9
        by = {}
        for r in rows:
            by.setdefault(r['epc'], []).append(r)
        seen = sum(int(r.get('seen') or 1) for r in rows)
        print('\n%s: %d phase records (%d air reads), %d tags, %.1f s, %.1f records/s' % (os.path.basename(path), len(rows), seen, len(by), T, len(rows) / T))
        ch = [int(r['channel']) for r in rows if r['channel']]
        if ch:
            hops = sum(1 for a, b in zip(ch, ch[1:]) if a != b); print('channels used: %d distinct, %d hops (%.2f s per channel)' % (len(set(ch)), hops, T / max(hops, 1)))
        rates = []
        print('%-12s %-9s %6s %8s %8s %8s %9s' % ('tag', 'type', 'reads', 'reads/s', 'gap p50', 'gap p90', 'rssi med'))
        for epc, rr in sorted(by.items(), key=lambda kv: -len(kv[1])):
            tt = sorted(float(r['host_time_s']) for r in rr); span = (tt[-1] - tt[0]) or 1e-9
            gaps = [b - a for a, b in zip(tt, tt[1:])]
            rate = (len(tt) - 1) / span if len(tt) > 1 else 0; rates.append(rate)
            rssi = [float(r['rssi_dbm']) for r in rr if r['rssi_dbm']]
            print('%-12s %-9s %6d %8.2f %8s %8s %9s' % ((rr[0]['name'] or epc[-8:])[:12], rr[0]['tag_type'][:9], len(rr), rate,
                  ('%.0f ms' % (1000 * statistics.median(gaps))) if gaps else '-', ('%.0f ms' % (1000 * sorted(gaps)[int(0.9 * (len(gaps) - 1))])) if gaps else '-',
                  ('%.1f' % statistics.median(rssi)) if rssi else '-'))
        rates.sort()
        if rates:
            p25 = rates[len(rates) // 4]; med = rates[len(rates) // 2]
            print('per-tag read rate: median %.2f reads/s, lower quartile %.2f reads/s' % (med, p25))
            print('robot speed for a read every %.1f cm: median tag %.2f m/s, weak quartile %.2f m/s' % (args.step_cm, med * args.step_cm / 100, p25 * args.step_cm / 100))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('scan'); s.add_argument('label')
    s.add_argument('--ip', default='192.168.0.20'); s.add_argument('--antennas', type=int, nargs='+', default=[4])
    s.add_argument('--power', default='33', help='dBm, 10-33'); s.add_argument('--session', type=int, default=0, choices=[0, 1, 2, 3])
    s.add_argument('--mode', type=int, default=0, help='Impinj RF mode identifier: 0 max throughput (default, best for SAR passes), 100/1003 similar, 2 = dense-reader M4 (5x slower, reads the weakest tags: use for a coverage pass); valid on this R700: 0-4,100,120-122,140-142,180-185,1002-1006,1110+')
    s.add_argument('--search-mode', type=int, default=2, choices=[1, 2, 3, 5, 6], help='Impinj search mode: 1 single target, 2 dual target (default), 3 single+suppression (TagFocus)')
    s.add_argument('--population', type=int, default=64); s.add_argument('--report-every', type=int, default=20, help='reader reports every N tag reads. Measured 2026-10-21: N=1 caps at ~37 records/s (per-report overhead); N=20 gives ~480 records/s with 97%% single-read entries; N>=50 merges reads and loses phases'); s.add_argument('--duration', type=float, default=None, help='seconds; default until Ctrl-C')
    s.add_argument('--registry', default=rt.REGISTRY); s.add_argument('--out-dir', default=os.path.join(HERE, 'pass_logs'))
    s.set_defaults(fn=scan)
    r = sub.add_parser('report'); r.add_argument('logs', nargs='+'); r.add_argument('--top', type=int, default=15); r.add_argument('--step-cm', type=float, default=3.5)
    r.set_defaults(fn=report)
    args = ap.parse_args(); args.fn(args)


if __name__ == '__main__':
    main()
