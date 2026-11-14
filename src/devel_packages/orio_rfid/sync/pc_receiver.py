#!/usr/bin/env python3
"""
pc_receiver.py - runs on the MAIN PC. Receives RFID sightings from the robot and updates the master
SQLite inventory (the same medical_supplies table db_manager.py and inventory_dashboard.py use).

    python3 pc_receiver.py --db /path/to/medical_inventory.db [--port 8000] [--runs-dir runs]

Endpoints (JSON over plain HTTP, standard library only):
    GET  /health      -> {"ok": true, "items": <count>}
    GET  /items       -> {"items": [{"rfid_uid", "item_name", "expiration_date"}, ...]}   the robot pulls this
    POST /sightings   <- {"robot": "...", "sightings": [{"rfid_uid": "...", "seen_at": <unix seconds, UTC>}, ...]}
                         optional "scan_started": <unix seconds> - the time the robot launched its current scan
                      -> {"ok": true, "updated": n, "unknown": [rfid_uid, ...], "reset": true/false}
    POST /reads       <- {"run": "<label>", "start": <rows sent before this batch>, "header": [...], "rows": [[...], ...],
                          "method": "F4", "robot": "..."}   (method and robot are kept in <runs-dir>/<label>/run.json)
                         the robot's posed reads (RFID/rfid_pose_live.py), appended to <runs-dir>/<label>/reads_posed_live.csv
                         for the item localiser when it runs on this PC
                      -> {"ok": true, "have": <rows stored>}   ("ok": false + "have" when a batch does not line up: resend from there)
    POST /locations   <- {"run": "...", "method": "F4", "source": "<host>", "items": [{"rfid_uid", "item_name", "shelf",
                          "shelf_prob", "x_m", "y_m", "z_m", "region90_m", "n_reads", "updated_s"}, ...]}
                         where each item is (from the item localiser, loc/live); stored in the item_locations table
                      -> {"ok": true, "stored": n}
    POST /engine      <- {"note": "...", "error": false, "source": "<host>"}   the item localiser's heartbeat, every round
    GET  /state       -> everything the demo GUI shows: items with presence and location, the scan, stream counters,
                         and when the localiser was last heard from

A sighting moves last_seen forward (PC local time, the format the dashboard already reads) and, if it was
read during the current scan, sets is_on_shelf = 1. Older or repeated sightings are harmless.
When a newer scan_started arrives, every item is first set to is_on_shelf = 0 (a new scan began on the robot).
Items go missing only that way: nothing times out.

item_locations is a separate table in the same file (one row per tag, the newest estimate); medical_supplies is not
changed by it. A location stays when a new scan starts: for an item that is now missing it is where it was last.
"""
import argparse
import csv
import json
import os
import re
import sqlite3
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

TIME_FMT = '%Y-%m-%d %H:%M:%S'
SCHEMA = '''CREATE TABLE IF NOT EXISTS medical_supplies (
    rfid_uid TEXT PRIMARY KEY, item_name TEXT, expiration_date TEXT,
    date_labeled TEXT, last_seen TEXT, is_on_shelf INTEGER DEFAULT 0)'''
STATE_SCHEMA = 'CREATE TABLE IF NOT EXISTS sync_state (key TEXT PRIMARY KEY, value REAL)'   # scan_started
LOC_SCHEMA = '''CREATE TABLE IF NOT EXISTS item_locations (
    rfid_uid TEXT PRIMARY KEY, item_name TEXT,
    shelf INTEGER,             -- most likely shelf, NULL when the method uses no shelf map (F1, F2)
    shelf_prob REAL,           -- probability of that shelf
    x_m REAL, y_m REAL, z_m REAL,   -- room frame, metres; z from the floor
    region90_m REAL,           -- radius of a sphere with the volume of the 90 % region (smaller = more certain)
    method TEXT, n_reads INTEGER, run TEXT,
    updated TEXT,              -- PC local time, same format as last_seen
    updated_s REAL)            -- unix seconds
'''
LOC_COLS = ['rfid_uid', 'item_name', 'shelf', 'shelf_prob', 'x_m', 'y_m', 'z_m', 'region90_m', 'method', 'n_reads', 'run', 'updated', 'updated_s']
WRITE_LOCK = threading.Lock()
LIVE = {'robot': None, 'sightings_at': None, 'sightings_n': 0, 'unknown': set(),           # filled as posts arrive
        'reads_run': None, 'reads_rows': 0, 'reads_at': None, 'reads_rate': 0.0,
        'loc_at': None, 'loc_method': None, 'loc_source': None, 'loc_run': None, 'started': time.time(),
        'engine_at': None, 'engine_note': None, 'engine_error': False, 'engine_source': None}


def connect(path):
    conn = sqlite3.connect(path, timeout=10)   # timeout: db_manager.py / the dashboard may hold the file briefly
    conn.execute(SCHEMA)
    conn.execute(STATE_SCHEMA)
    conn.execute(LOC_SCHEMA)
    return conn


def store_reads(runs_dir, data):
    """append a batch of posed reads to <runs_dir>/<run>/reads_posed_live.csv -> (ok, rows stored)"""
    run = re.sub(r'[^A-Za-z0-9_.-]', '_', str(data['run']))[:120] or 'run'
    d = os.path.join(runs_dir, run); os.makedirs(d, exist_ok=True)
    path = os.path.join(d, 'reads_posed_live.csv')
    if LIVE['reads_run'] != run:                      # first batch of this run since the receiver started: count what is on disk
        have = max(0, sum(1 for _ in open(path)) - 1) if os.path.exists(path) else 0
        LIVE.update(reads_run=run, reads_rows=have, reads_rate=0.0)
    have, start, rows = LIVE['reads_rows'], int(data.get('start', 0)), data.get('rows') or []
    if start > have:
        return False, have                            # a batch was lost: the robot resends from `have`
    rows = rows[have - start:]                        # drop what is already stored (a resend after a lost reply)
    new = not os.path.exists(path)
    if new:        # what the item localiser needs to know about this run
        json.dump({'run': run, 'method': data.get('method'), 'robot': data.get('robot'), 'started': time.time()}, open(os.path.join(d, 'run.json'), 'w'))
    with open(path, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(data['header'])
        w.writerows(rows)
    now = time.time()
    if rows:        # when each batch arrived and the robot-clock time of its newest read: how long the reads take to get here
        try:
            with open(os.path.join(d, 'arrivals.csv'), 'a') as f:
                f.write('%.3f,%s,%d\n' % (now, rows[-1][data['header'].index('host_time_s')], len(rows)))
        except (ValueError, OSError):
            pass
    if rows and LIVE['reads_at']:
        LIVE['reads_rate'] = 0.7 * LIVE['reads_rate'] + 0.3 * len(rows) / max(now - LIVE['reads_at'], 0.2)
    LIVE.update(reads_rows=have + len(rows), reads_at=now if rows else LIVE['reads_at'])
    return True, LIVE['reads_rows']


def store_locations(conn, data):
    n = 0
    for it in data.get('items') or []:
        uid = ''.join(str(it['rfid_uid']).split()).upper()
        ts = float(it.get('updated_s') or time.time())
        row = dict(it, rfid_uid=uid, method=data.get('method'), run=data.get('run'), updated_s=ts,
                   updated=time.strftime(TIME_FMT, time.localtime(ts)))
        conn.execute('INSERT OR REPLACE INTO item_locations (%s) VALUES (%s)' % (', '.join(LOC_COLS), ', '.join('?' * len(LOC_COLS))),
                     [row.get(c) for c in LOC_COLS])
        n += 1
    conn.commit()
    LIVE.update(loc_at=time.time(), loc_method=data.get('method'), loc_source=data.get('source'), loc_run=data.get('run'))
    return n


def state(conn):
    items = []
    q = ('SELECT m.rfid_uid, m.item_name, m.expiration_date, m.last_seen, m.is_on_shelf, l.shelf, l.shelf_prob, l.x_m, l.y_m, l.z_m, '
         'l.region90_m, l.method, l.n_reads, l.updated_s FROM medical_supplies m LEFT JOIN item_locations l ON l.rfid_uid = m.rfid_uid '
         'ORDER BY m.item_name, m.rfid_uid')
    keys = ['rfid_uid', 'item_name', 'expiration_date', 'last_seen', 'is_on_shelf', 'shelf', 'shelf_prob', 'x_m', 'y_m', 'z_m',
            'region90_m', 'method', 'n_reads', 'loc_updated_s']
    for r in conn.execute(q):
        items.append(dict(zip(keys, r)))
    # tags the localiser placed that are not in the inventory table (e.g. registry tags the master database does not list)
    other = [dict(zip(LOC_COLS, r)) for r in conn.execute(
        'SELECT %s FROM item_locations l WHERE NOT EXISTS (SELECT 1 FROM medical_supplies m WHERE m.rfid_uid = l.rfid_uid)' % ', '.join('l.' + c for c in LOC_COLS))]
    row = conn.execute("SELECT value FROM sync_state WHERE key = 'scan_started'").fetchone()
    live = dict(LIVE, unknown=sorted(LIVE['unknown']))
    return {'ok': True, 'now': time.time(), 'scan_started': row[0] if row else None, 'items': items, 'located_other': other,
            'n_items': len(items), 'n_on_shelf': sum(1 for i in items if i['is_on_shelf']),
            'n_located': sum(1 for i in items if i['is_on_shelf'] and i['x_m'] is not None), 'live': live}


def apply_sightings(conn, sightings, scan_started=None):
    row = conn.execute("SELECT value FROM sync_state WHERE key = 'scan_started'").fetchone()
    current = row[0] if row else 0.0
    reset = False
    if scan_started is not None and float(scan_started) > current:   # the robot launched a new scan
        current = float(scan_started)
        conn.execute('UPDATE medical_supplies SET is_on_shelf = 0')
        conn.execute("INSERT OR REPLACE INTO sync_state (key, value) VALUES ('scan_started', ?)", (current,))
        reset = True
    updated, unknown = 0, []
    for s in sightings:
        uid = ''.join(str(s['rfid_uid']).split()).upper()
        seen_at = float(s['seen_at'])
        stamp = time.strftime(TIME_FMT, time.localtime(seen_at))
        row = conn.execute('SELECT last_seen, is_on_shelf FROM medical_supplies WHERE rfid_uid = ?', (uid,)).fetchone()
        if row is None:
            unknown.append(uid)
            continue
        changed = False
        if row[0] is None or row[0] < stamp:   # same fixed-width format, so string order = time order
            conn.execute('UPDATE medical_supplies SET last_seen = ? WHERE rfid_uid = ?', (stamp, uid))
            changed = True
        if seen_at >= current and row[1] != 1:   # reads from before this scan do not count as present
            conn.execute('UPDATE medical_supplies SET is_on_shelf = 1 WHERE rfid_uid = ?', (uid,))
            changed = True
        updated += changed
    conn.commit()
    return updated, unknown, reset


class Handler(BaseHTTPRequestHandler):
    db_path = None
    runs_dir = 'runs'

    def reply(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        conn = connect(self.db_path)
        try:
            if self.path == '/health':
                n = conn.execute('SELECT COUNT(*) FROM medical_supplies').fetchone()[0]
                self.reply(200, {'ok': True, 'items': n})
            elif self.path == '/items':
                rows = conn.execute('SELECT rfid_uid, item_name, expiration_date FROM medical_supplies').fetchall()
                self.reply(200, {'items': [{'rfid_uid': r[0], 'item_name': r[1], 'expiration_date': r[2]} for r in rows]})
            elif self.path == '/state':
                self.reply(200, state(conn))
            else:
                self.reply(404, {'ok': False, 'error': 'unknown path'})
        finally:
            conn.close()

    def do_POST(self):
        if self.path not in ('/sightings', '/reads', '/locations', '/engine'):
            return self.reply(404, {'ok': False, 'error': 'unknown path'})
        try:
            data = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
            if self.path == '/engine':
                LIVE.update(engine_at=time.time(), engine_note=str(data.get('note', ''))[:200], engine_error=bool(data.get('error')), engine_source=data.get('source'))
                return self.reply(200, {'ok': True})
            if self.path == '/reads':
                with WRITE_LOCK:
                    ok, have = store_reads(self.runs_dir, data)
                return self.reply(200, {'ok': ok, 'have': have})
            if self.path == '/locations':
                conn = connect(self.db_path)
                try:
                    with WRITE_LOCK:
                        n = store_locations(conn, data)
                finally:
                    conn.close()
                return self.reply(200, {'ok': True, 'stored': n})
            conn = connect(self.db_path)
            try:
                with WRITE_LOCK:
                    updated, unknown, reset = apply_sightings(conn, data['sightings'], data.get('scan_started'))
            finally:
                conn.close()
        except Exception as e:
            return self.reply(400, {'ok': False, 'error': str(e)})
        if reset:
            LIVE['unknown'] = set()
        LIVE.update(robot=data.get('robot'), sightings_at=time.time(), sightings_n=LIVE['sightings_n'] + len(data['sightings']))
        LIVE['unknown'].update(unknown)
        if reset:
            print('%s  %s started a new scan: all items set to not on shelf' % (time.strftime('%H:%M:%S'), data.get('robot', '?')), flush=True)
        if data['sightings']:
            print('%s  %s sent %d sightings: %d updated, %d unknown tags' % (
                time.strftime('%H:%M:%S'), data.get('robot', '?'), len(data['sightings']), updated, len(unknown)), flush=True)
        self.reply(200, {'ok': True, 'updated': updated, 'unknown': unknown, 'reset': reset})

    def log_message(self, *a):   # silence the default per-request log line
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--db', required=True, help='master SQLite file (created with an empty table if missing)')
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--runs-dir', default=None, help='where the robot\'s live reads are stored (default: runs/ next to the database)')
    args = ap.parse_args()
    Handler.db_path = args.db
    Handler.runs_dir = args.runs_dir or os.path.join(os.path.dirname(os.path.abspath(args.db)), 'runs')
    connect(args.db).close()
    print('master db %s, listening on port %d (Ctrl-C to stop)' % (args.db, args.port), flush=True)
    try:
        ThreadingHTTPServer(('0.0.0.0', args.port), Handler).serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
