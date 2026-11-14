#!/usr/bin/env python3
"""
robot_sync.py - runs on the ROBOT. Scans RFID tags and keeps the main PC's inventory up to date, live.

    python3 robot_sync.py --pc http://<pc-ip>:8000 --reader          # real Impinj reader
    python3 robot_sync.py --pc http://<pc-ip>:8000 --simulate 3      # no reader: 3 random known items per cycle
    python3 robot_sync.py --pc http://<pc-ip>:8000 --follow <run>/reads_posed_live.csv [--stream-reads --run LABEL]
                                                                     # during an autonomous run (see below)

Every launch starts a new scan: the PC marks every item "not on shelf", then each item flips to
"on shelf" as soon as it is read (sent once per --send-every seconds). Use --resume to carry on the
previous scan instead (e.g. after a crash) without resetting anything.

Also pulls the PC's item list (GET /items) so newly labelled items are known here.
If the PC is unreachable nothing is lost: unsent sightings stay in the local DB and go out on the next try,
and the "new scan" reset reaches the PC together with the first batch that gets through.

The reader settings are the ones measured for rfid_log.py (RFID/rfid_log.py); the default RF mode is 2, the
slow dense-reader mode that picked up the most tags, which is what a presence scan wants.
The reader accepts ONE connection at a time: do not run this together with rfid_log.py / record_pass.sh.

--follow is for a run that already has the reader (scripts/rfid_nav_run.sh in orio_stretch runs rfid_log.py): the
sightings are taken from that run's posed read log (RFID/rfid_pose_live.py writes it) instead of a second reader
connection. With --stream-reads every posed read is also sent to the PC (POST /reads), for the item localiser
when it runs there; reads wait in the file while the PC is unreachable and go out in order afterwards.

Standard library + sllurp (only for --reader). Python 3.8+, no ROS.
"""
import argparse
import json
import csv
import io
import logging
import os
import random
import socket
import sqlite3
import sys
import threading
import time
import urllib.request

SCHEMA = ['''CREATE TABLE IF NOT EXISTS items (
    rfid_uid TEXT PRIMARY KEY, item_name TEXT, expiration_date TEXT,
    last_seen REAL,            -- unix seconds (UTC) of the newest read on the robot
    last_sent REAL)            -- newest last_seen the PC has confirmed''',
          'CREATE TABLE IF NOT EXISTS sync_state (key TEXT PRIMARY KEY, value REAL)']   # scan_started, scan_acked


def connect(path):
    conn = sqlite3.connect(path, timeout=10)
    for s in SCHEMA:
        conn.execute(s)
    conn.commit()
    return conn


def get_state(conn, key):
    row = conn.execute('SELECT value FROM sync_state WHERE key = ?', (key,)).fetchone()
    return row[0] if row else None


def set_state(conn, key, value):
    conn.execute('INSERT OR REPLACE INTO sync_state (key, value) VALUES (?, ?)', (key, value))


def norm_epc(epc):
    if isinstance(epc, bytes):
        epc = epc.decode('ascii', 'replace')
    return ''.join(str(epc).split()).upper()


def record_sighting(conn, rfid_uid, seen_at):
    """One tag read. Tags not in the item list are kept too (name empty); the PC reports them as unknown."""
    uid = norm_epc(rfid_uid)
    conn.execute('INSERT OR IGNORE INTO items (rfid_uid) VALUES (?)', (uid,))
    conn.execute('UPDATE items SET last_seen = ? WHERE rfid_uid = ? AND (last_seen IS NULL OR last_seen < ?)', (seen_at, uid, seen_at))


# ------------------------------------------------------------------ Impinj reader (same setup as RFID/rfid_log.py)
class Reader:
    """Stays connected to the reader; reads land in self.latest {epc: time of newest read} from sllurp's thread.
    If the reader refuses or drops the connection, check() reconnects every few seconds."""

    def __init__(self, args):
        logging.getLogger('sllurp').setLevel(logging.CRITICAL)   # connection problems are reported by check() instead
        self.args = args
        self.latest = {}
        self.lock = threading.Lock()
        self.lost = False
        self.warned = False
        self.retry_at = 0.0
        self.client = None
        self.open()

    def open(self):
        from sllurp.llrp import LLRPReaderConfig, LLRPReaderClient, LLRP_DEFAULT_PORT
        a = self.args
        cfg = {
            'antennas': a.antennas, 'report_every_n_tags': a.report_every, 'session': a.session, 'tag_population': 64,
            'tx_power_dbm': float(a.power), 'mode_identifier': a.mode, 'impinj_search_mode': a.search_mode,
            'tag_content_selector': {'EnableAntennaID': True, 'EnablePeakRSSI': True, 'EnableTagSeenCount': True},
        }
        self.lost = False
        self.client = LLRPReaderClient(a.reader_ip, LLRP_DEFAULT_PORT, LLRPReaderConfig(cfg))
        self.client.add_tag_report_callback(self.on_tags)
        self.client.add_disconnected_callback(self.on_lost)
        try:
            self.client.connect()
        except Exception:
            self.lost = True

    def on_lost(self, *a):
        self.lost = True

    def on_tags(self, reader, tag_reports):
        now = time.time()
        with self.lock:
            for tag in tag_reports:
                epc = norm_epc(tag.get('EPC-96', tag.get('EPCData', b'')))
                if epc:
                    self.latest[epc] = now

    def check(self):
        """Call every cycle: reports a refused/dropped reader connection once and keeps retrying."""
        if not self.lost:
            if self.warned and self.latest:
                print('%s  reader connected again' % time.strftime('%H:%M:%S'), flush=True)
                self.warned = False
            return
        if not self.warned:
            print('%s  NOT SCANNING: reader %s refused or dropped the connection. It accepts one program at a time - '
                  'is rfid_log.py, register_tags.py or db_manager.py still running? Retrying every 5 s.' % (
                      time.strftime('%H:%M:%S'), self.args.reader_ip), flush=True)
            self.warned = True
            self.retry_at = time.time() + 5.0
        elif time.time() >= self.retry_at:
            self.retry_at = time.time() + 5.0
            self.open()

    def drain(self):
        with self.lock:
            out, self.latest = self.latest, {}
        return out

    def close(self):
        try:
            if not self.lost:
                self.client.disconnect()
                time.sleep(0.5)
        except Exception as e:
            print('reader disconnect:', e)


# ------------------------------------------------------------------ a run's posed read log (rfid_pose_live.py)
class Follower:
    """New rows of the posed read log: drain() gives {epc: newest read time} for the sightings, and the rows
    themselves wait in self.rows until send_reads() has delivered them."""
    BATCH = 3000

    def __init__(self, path, keep_rows, meta=None):
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'RFID'))
        from rfid_pose_live import Tail
        self.Tail, self.path, self.keep_rows, self.meta = Tail, path, keep_rows, meta or {}
        self.tail, self.header, self.rows, self.sent, self.read = Tail(path), None, [], 0, 0

    def drain(self):
        latest = {}
        for line in self.tail.lines():
            row = next(csv.reader(io.StringIO(line)))
            if self.header is None:
                self.header = row
                self.i_t, self.i_epc = row.index('host_time_s'), row.index('epc')
                continue
            self.read += 1
            if self.keep_rows:
                self.rows.append(row)
            try:
                epc = norm_epc(row[self.i_epc])
                if epc:
                    latest[epc] = max(latest.get(epc, 0.0), float(row[self.i_t]))
            except (ValueError, IndexError):
                pass
        return latest

    def rewind(self, have):
        """the PC has `have` rows: read the file again from there (a batch or the PC's copy was lost)"""
        self.tail, self.header, self.rows, self.read = self.Tail(self.path), None, [], 0
        keep, self.keep_rows = self.keep_rows, True
        self.drain()
        self.keep_rows = keep
        del self.rows[:have]
        self.sent = have

    def send_reads(self, pc, run):
        """deliver the waiting rows in order -> number sent. Raises when the PC is unreachable."""
        n = 0
        while self.rows:
            batch = self.rows[:self.BATCH]
            reply = http_json(pc + '/reads', dict(self.meta, run=run, start=self.sent, header=self.header, rows=batch), timeout=5.0)
            if not reply.get('ok'):
                self.rewind(int(reply.get('have', 0)))
                continue
            done = int(reply['have']) - self.sent
            del self.rows[:done]; self.sent += done; n += done
            if done < len(batch):
                break
        return n


# ------------------------------------------------------------------ talking to the PC
def http_json(url, payload=None, timeout=3.0):
    data = None if payload is None else json.dumps(payload).encode()
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def send_pending(conn, pc, robot):
    """Send unsent sightings plus the scan start time. Returns (number sent, whether the PC reset for a new scan)."""
    scan_started = get_state(conn, 'scan_started')
    rows = conn.execute('SELECT rfid_uid, last_seen FROM items WHERE last_seen IS NOT NULL '
                        'AND (last_sent IS NULL OR last_sent < last_seen)').fetchall()
    if not rows and get_state(conn, 'scan_acked') == scan_started:
        return 0, False
    reply = http_json(pc + '/sightings', {'robot': robot, 'scan_started': scan_started,
                                          'sightings': [{'rfid_uid': u, 'seen_at': t} for u, t in rows]})
    if not reply.get('ok'):
        raise RuntimeError(reply.get('error', 'PC refused the sightings'))
    conn.executemany('UPDATE items SET last_sent = ? WHERE rfid_uid = ?', [(t, u) for u, t in rows])   # only after the PC said OK
    set_state(conn, 'scan_acked', scan_started)
    conn.commit()
    return len(rows), bool(reply.get('reset'))


def pull_items(conn, pc):
    items = http_json(pc + '/items')['items']
    for it in items:
        conn.execute('INSERT OR IGNORE INTO items (rfid_uid) VALUES (?)', (it['rfid_uid'],))
        conn.execute('UPDATE items SET item_name = ?, expiration_date = ? WHERE rfid_uid = ?',
                     (it['item_name'], it['expiration_date'], it['rfid_uid']))
    conn.commit()
    return len(items)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pc', required=True, help='address of pc_receiver.py, e.g. http://192.168.50.20:8000')
    ap.add_argument('--db', default='robot_items.db', help='local SQLite file (created if missing)')
    ap.add_argument('--send-every', type=float, default=1.0, help='seconds between sends')
    ap.add_argument('--pull-every', type=float, default=15.0, help='seconds between item-list pulls')
    ap.add_argument('--resume', action='store_true', help='continue the previous scan instead of starting a new one')
    src = ap.add_mutually_exclusive_group()
    src.add_argument('--reader', action='store_true', help='read tags from the Impinj reader')
    src.add_argument('--simulate', type=int, default=0, metavar='N', help='pretend to read N random known items each cycle')
    src.add_argument('--follow', default=None, metavar='CSV', help='take the reads from a run\'s posed read log (reads_posed_live.csv)')
    ap.add_argument('--stream-reads', action='store_true', help='with --follow: also send every posed read to the PC (POST /reads)')
    ap.add_argument('--run', default=None, help='run label for --stream-reads (default: the folder of the --follow file)')
    ap.add_argument('--method', default=None, help='with --stream-reads: the localisation method wanted for this run (F1..F4), passed on to the PC')
    ap.add_argument('--reader-ip', default='192.168.0.20')
    ap.add_argument('--antennas', type=int, nargs='+', default=[4])
    ap.add_argument('--power', type=float, default=30.0, help='transmit power, dBm, 10-33')
    ap.add_argument('--mode', type=int, default=2, help='Impinj RF mode: 2 = dense reader, most tags (default); 0 = fastest')
    ap.add_argument('--search-mode', type=int, default=2, choices=[1, 2, 3], help='2 = dual target (default)')
    ap.add_argument('--session', type=int, default=0, choices=[0, 1, 2, 3])
    ap.add_argument('--report-every', type=int, default=20, help='reader reports every N tag reads')
    args = ap.parse_args()
    if not 10 <= args.power <= 33:
        ap.error('--power must be 10-33 dBm (sllurp treats 0 as maximum power)')
    pc = args.pc.rstrip('/')
    robot = socket.gethostname()
    conn = connect(args.db)

    scan_started = get_state(conn, 'scan_started')
    if args.resume and scan_started is not None:
        print('resuming the scan started %s' % time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(scan_started)), flush=True)
    else:
        scan_started = time.time()
        set_state(conn, 'scan_started', scan_started)
        conn.commit()
        print('new scan: every item starts as not on shelf', flush=True)
    print('local db %s, PC %s (Ctrl-C to stop)' % (args.db, pc), flush=True)

    if args.reader:
        print('connecting to reader %s (antennas %s, %s dBm, mode %d) ...' % (args.reader_ip, args.antennas, args.power, args.mode), flush=True)
    reader = Reader(args) if args.reader else None
    follower = Follower(args.follow, args.stream_reads, {'method': args.method, 'robot': robot}) if args.follow else None
    run = args.run or (os.path.basename(os.path.dirname(os.path.abspath(args.follow))) if args.follow else None)
    reads_sent = 0
    next_pull = 0.0
    next_status = time.time() + 5.0
    link_up = None
    try:
        while True:
            if reader:
                reader.check()
                for epc, t in reader.drain().items():
                    record_sighting(conn, epc, t)
                conn.commit()
            elif follower:
                for epc, t in follower.drain().items():
                    record_sighting(conn, epc, t)
                conn.commit()
            elif args.simulate:
                known = [r[0] for r in conn.execute('SELECT rfid_uid FROM items WHERE item_name IS NOT NULL')]
                for uid in random.sample(known, min(args.simulate, len(known))):
                    record_sighting(conn, uid, time.time())
                conn.commit()
            try:
                if time.time() >= next_pull:
                    n = pull_items(conn, pc)
                    next_pull = time.time() + args.pull_every
                    if link_up is not True:
                        print('%s  PC reachable, item list: %d items' % (time.strftime('%H:%M:%S'), n), flush=True)
                sent, reset = send_pending(conn, pc, robot)
                if follower and args.stream_reads:
                    reads_sent += follower.send_reads(pc, run)
                if reset:
                    print('%s  PC started the new scan (all items set to not on shelf)' % time.strftime('%H:%M:%S'), flush=True)
                link_up = True
            except Exception as e:   # PC off, Wi-Fi down, ...: keep the data and retry next cycle
                if link_up is not False:
                    print('%s  PC unreachable (%s) - holding sightings until it is back' % (time.strftime('%H:%M:%S'), e), flush=True)
                link_up = False
            if time.time() >= next_status:
                known, other, unsent = conn.execute(
                    'SELECT SUM(item_name IS NOT NULL), SUM(item_name IS NULL), SUM(last_sent IS NULL OR last_sent < last_seen) '
                    'FROM items WHERE last_seen >= ?', (scan_started,)).fetchone()
                print('%s  this scan: %d of %d listed items seen, %d other tags, %d not yet sent' % (
                    time.strftime('%H:%M:%S'), known or 0, conn.execute('SELECT COUNT(*) FROM items WHERE item_name IS NOT NULL').fetchone()[0],
                    other or 0, unsent or 0) + (', reads sent to the PC: %d of %d' % (reads_sent, follower.read)
                                                 if follower and args.stream_reads else ''), flush=True)
                next_status = time.time() + 5.0
            time.sleep(args.send_every)
    except KeyboardInterrupt:
        pass
    finally:
        if reader:
            print('disconnecting from the reader ...', flush=True)
            reader.close()
        conn.close()


if __name__ == '__main__':
    main()
