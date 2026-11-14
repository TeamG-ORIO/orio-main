"""
impinj_phase_read.py - read tags with the Impinj reader and show RSSI + RF phase angle.

    python3 impinj_phase_read.py                      # 5 s, antennas 1-4, 15 dBm
    python3 impinj_phase_read.py --power 30           # stronger signal (dBm, snapped to the reader's power table)
    python3 impinj_phase_read.py --power max          # the reader's maximum
    python3 impinj_phase_read.py --antennas 4 --duration 10 --live

Phase comes from Impinj's LLRP extension (ImpinjRFPhaseAngle, 12 bit: 0..4095 = 0..360 deg).
It is only comparable between reads on the SAME channel (the reader hops frequency), so the
channel index is shown with it.
"""
import argparse
import logging
import time
from collections import defaultdict

from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT

reads = defaultdict(list)       # epc -> [(t, antenna, channel, rssi_dbm, phase_deg)]
LIVE = False
T0 = time.time()


def tag_report_cb(reader, tag_reports):
    for tag in tag_reports:
        epc = tag.get('EPC-96', tag.get('EPCData', b'?'))
        if isinstance(epc, bytes):
            epc = epc.decode('ascii', 'replace').upper()
        raw_phase = tag.get('ImpinjRFPhaseAngle')
        phase = raw_phase * 360.0 / 4096.0 if raw_phase is not None else None
        rssi = tag.get('ImpinjPeakRSSI')
        rssi = rssi / 100.0 if rssi is not None else tag.get('PeakRSSI')
        row = (time.time() - T0, tag.get('AntennaID'), tag.get('ChannelIndex'), rssi, phase)
        reads[epc].append(row)
        if LIVE:
            print('%6.2fs  %s  ant %s  ch %2s  RSSI %s dBm  phase %s deg' % (
                row[0], epc, row[1], row[2],
                '%6.1f' % rssi if rssi is not None else '   n/a',
                '%6.1f' % phase if phase is not None else '   n/a'))


def main():
    global LIVE, T0
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ip', default='192.168.0.20')
    ap.add_argument('--antennas', type=int, nargs='+', default=[1, 2, 3, 4])
    ap.add_argument('--power', default='15', help="transmit power in dBm, or 'max'")
    ap.add_argument('--duration', type=float, default=5.0, help='seconds to scan')
    ap.add_argument('--live', action='store_true', help='print every single read as it arrives')
    args = ap.parse_args()
    LIVE = args.live
    logging.getLogger('sllurp').setLevel(logging.WARNING)

    cfg = {
        'antennas': args.antennas,
        'report_every_n_tags': 1,
        'tag_content_selector': {
            'EnableAntennaID': True, 'EnableChannelIndex': True, 'EnablePeakRSSI': True,
            'EnableTagSeenCount': True, 'EnableLastSeenTimestamp': True,
        },
        # Impinj extension: per-read RF phase angle and high-resolution RSSI
        'impinj_tag_content_selector': {
            'EnableRFPhaseAngle': True, 'EnablePeakRSSI': True, 'EnableRFDopplerFrequency': False,
        },
    }
    if args.power == 'max':
        cfg['tx_power'] = 0                       # sllurp: index 0 = the reader's maximum
    else:
        cfg['tx_power_dbm'] = float(args.power)

    reader = LLRPReaderClient(args.ip, LLRP_DEFAULT_PORT, LLRPReaderConfig(cfg))
    reader.add_tag_report_callback(tag_report_cb)
    print('connecting to %s ...' % args.ip)
    reader.connect()
    T0 = time.time()
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        pass
    table = [p for p in getattr(reader.llrp, 'tx_power_table', []) if p]
    used = {ant: reader.llrp.tx_power_table[idx] for ant, idx in getattr(reader.llrp.config, 'tx_power', {}).items()} \
        if table else {}
    reader.disconnect()

    if table:
        print('reader power range: %.2f .. %.2f dBm; used: %s' % (
            min(table), max(table), ', '.join('ant %s = %.2f dBm' % kv for kv in sorted(used.items()))))
    print('\n%d unique tags, %d reads in %.1f s' % (len(reads), sum(len(v) for v in reads.values()), args.duration))
    print('%-26s %5s %4s %10s %10s   %s' % ('EPC', 'reads', 'ant', 'RSSI avg', 'RSSI max', 'last phase (deg @ channel)'))
    for epc, rows in sorted(reads.items(), key=lambda kv: -len(kv[1])):
        rssi = [r[3] for r in rows if r[3] is not None]
        ants = sorted(set(r[1] for r in rows if r[1] is not None))
        last = ['%.1f@ch%s' % (r[4], r[2]) for r in rows[-4:] if r[4] is not None]
        print('%-26s %5d %4s %10s %10s   %s' % (
            epc, len(rows), ','.join(map(str, ants)),
            '%.1f' % (sum(rssi) / len(rssi)) if rssi else 'n/a', '%.1f' % max(rssi) if rssi else 'n/a',
            '  '.join(last) if last else 'n/a'))


if __name__ == '__main__':
    main()
