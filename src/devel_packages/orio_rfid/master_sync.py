#!/usr/bin/env python3
"""Labels applied on this PC, and this PC's copy of abuela's master inventory.

The RFID node (db_manager.py) saves each new label here only, in master_sync/labels.db,
under the next item number from 1001 up. The inventory window lists the labels not
pushed yet; the ones you tick are pushed to abuela, each as
  * a row in master.db              (rfid_uid = the tag, item_name = "Item N", expiry)
  * a line in item_names.csv        (N, name, expiry: what the scan window shows)
  * a line in RFID/tag_registry.csv (the tag as "Item N", category dry, type unknown)
A label that is not pushed stays in the list for later; one can also be discarded.

pull() refreshes master_sync/copy/ from abuela: master.db (a consistent snapshot),
item_names.csv, tag_registry.csv and item_photos/. The item setup window takes its
item names and photos from there, the inventory window its stock.

    python3 master_sync.py pull      # refresh the copy now (run_full_demo.sh does this at start)
    python3 master_sync.py pending   # list the labels not pushed yet

abuela is reached with ssh (no password prompts): ORIO_MASTER_HOST (default
iam-abuela-local) and ORIO_MASTER_DIR (default /home/student/orio_scan). pull() and
push() run on the host, where the ssh key is; the RFID node in the container only
writes labels.db. Standard library only.
"""

import csv
import json
import os
import re
import shlex
import sqlite3
import subprocess
import sys
import tempfile
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
SYNC_DIR = os.path.join(HERE, 'master_sync')
LABELS_DB = os.path.join(SYNC_DIR, 'labels.db')
COPY_DIR = os.path.join(SYNC_DIR, 'copy')
COPY_DB = os.path.join(COPY_DIR, 'master.db')
COPY_NAMES = os.path.join(COPY_DIR, 'item_names.csv')
COPY_REGISTRY = os.path.join(COPY_DIR, 'tag_registry.csv')
COPY_PHOTOS = os.path.join(COPY_DIR, 'item_photos')
PULLED_STAMP = os.path.join(COPY_DIR, '.pulled')   # its mtime = the last complete pull

HOST = os.environ.get('ORIO_MASTER_HOST', 'iam-abuela-local')
REMOTE_DIR = os.environ.get('ORIO_MASTER_DIR', '/home/student/orio_scan')
REMOTE_DB = REMOTE_DIR + '/master.db'
REMOTE_NAMES = REMOTE_DIR + '/item_names.csv'
REMOTE_REGISTRY = REMOTE_DIR + '/orio_rfid/RFID/tag_registry.csv'
REMOTE_PHOTOS = REMOTE_DIR + '/item_photos/'

FIRST_NO = 1001                    # robot labels are Item 1001, 1002, ... (the registry's own tags are below)
CATEGORY, TAG_TYPE = 'dry', 'unknown'
REG_ANTENNA, REG_POWER_DBM = '1', '10.00'   # the Mercury labelling reader (db_manager.py: antenna 1, 1000 cdBm)
TS = '%Y-%m-%d %H:%M:%S'

SSH_OPTS = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
            '-o', 'ControlMaster=auto', '-o', 'ControlPersist=60',
            '-o', 'ControlPath=%s' % os.path.join(tempfile.gettempdir(), 'orio-master-%r@%h:%p')]

NUMBERED = re.compile(r'\s*item\s*#?\s*(\d+)\s*$', re.I)   # "Item 30": a number, not a name


# ── The names file and the registry (same rules as abuela's demo/inventory_model.py) ──

def parse_numbers(text):
    """'30', '30-35', '#30 31 40-42', 'Item 30' -> the list of numbers; None when it is not that"""
    out = []
    for tok in re.split(r'[\s;+&]+', re.sub(r'\s*-\s*', '-', re.sub(r'(?i)\bitems?\b|#', ' ', text or '')).strip()):
        m = re.fullmatch(r'(\d+)(?:-(\d+))?', tok)
        if not m:
            return None
        a, b = int(m.group(1)), int(m.group(2) or m.group(1))
        if not a <= b <= a + 2000:
            return None
        out.extend(range(a, b + 1))
    return out


def load_names(path=None):
    """item_names.csv -> {item number: (name, expiry or '')}; a number given twice: the later line wins"""
    out = {}
    try:
        with open(path or COPY_NAMES, newline='', encoding='utf-8-sig') as f:
            for row in csv.reader(f):
                first = (row[0] if row else '').strip()
                if not first or first.startswith('#'):
                    continue
                nos = parse_numbers(first)
                name = ' '.join((row[1] if len(row) > 1 else '').split())
                expiry = (row[2] if len(row) > 2 else '').strip()
                for no in nos or ():
                    out[no] = (name, expiry)
    except (OSError, csv.Error, UnicodeDecodeError):
        pass
    return {no: v for no, v in out.items() if v[0]}


def name_list():
    """the item names in the copy of item_names.csv, each once, sorted"""
    names = {}
    for name, _ in load_names().values():
        names.setdefault(name.lower(), name)
    return sorted(names.values(), key=str.lower)


def load_registry_numbers(path=None):
    """tag_registry.csv -> {rfid_uid: item number} for the tags it calls "Item N\""""
    out = {}
    try:
        with open(path or COPY_REGISTRY, newline='') as f:
            for r in csv.DictReader(f):
                m = NUMBERED.match(r.get('name') or '')
                if m and r.get('epc'):
                    out[r['epc'].strip().upper()] = int(m.group(1))
    except (OSError, csv.Error):
        pass
    return out


def slug(text):
    return re.sub(r'[^a-z0-9]+', '', str(text or '').lower())


_photo_index = {'mtime': None, 'index': {}}


def photo_for(name):
    """the photo of an item name in the copy of item_photos/ (matched as abuela's scan window does), or None"""
    try:
        mtime = os.stat(COPY_PHOTOS).st_mtime_ns
    except OSError:
        return None
    if mtime != _photo_index['mtime']:
        index = {}
        for n in sorted(os.listdir(COPY_PHOTOS)):
            stem, ext = os.path.splitext(n)
            if ext.lower() in ('.jpg', '.jpeg', '.png', '.webp', '.gif', '.bmp') and slug(stem):
                index.setdefault(slug(stem), os.path.join(COPY_PHOTOS, n))
        _photo_index.update(mtime=mtime, index=index)
    return _photo_index['index'].get(slug(name))


def copy_rows():
    """the copy of master.db's medical_supplies: [(rfid_uid, item_name, expiration_date, last_seen, is_on_shelf)]"""
    if not os.path.exists(COPY_DB):
        return []
    conn = sqlite3.connect('file:%s?mode=ro' % COPY_DB, uri=True, timeout=2.0)
    try:
        return conn.execute('SELECT rfid_uid, item_name, expiration_date, last_seen, is_on_shelf '
                            'FROM medical_supplies').fetchall()
    finally:
        conn.close()


def copy_items():
    """the copy's inventory as abuela's scan window sees it: the tags with an item number (from the
    registry, else an "Item N" item_name), each a dict with no, name ('' = no name yet), expiry,
    last_seen and on_shelf"""
    numbers, names = load_registry_numbers(), load_names()
    out = []
    for uid, item_name, expiry, last_seen, on_shelf in copy_rows():
        no = numbers.get(str(uid).upper())
        m = NUMBERED.match(item_name or '')
        if no is None and m:
            no = int(m.group(1))
        if no is None:
            continue
        name, name_expiry = names.get(no, ('', ''))
        out.append({'rfid_uid': uid, 'no': no, 'name': name, 'expiry': name_expiry or expiry or '',
                    'last_seen': last_seen, 'on_shelf': bool(on_shelf)})
    return out


def pulled_at():
    """time of the last complete pull, or None"""
    try:
        return os.path.getmtime(PULLED_STAMP)
    except OSError:
        return None


# ── The labels applied here ──────────────────────────────────────────────────

SCHEMA = '''CREATE TABLE IF NOT EXISTS labels (
    rfid_uid TEXT PRIMARY KEY,
    item_no INTEGER,
    item_name TEXT,
    expiration_date TEXT,
    date_labeled TEXT,
    rssi_dbm REAL,
    reads INTEGER,
    status TEXT DEFAULT 'pending',   -- pending (not pushed yet), pushed, discarded
    pushed_at TEXT,
    note TEXT                        -- why the last push of it failed
)'''


def _share(path):
    """Run as root (the RFID node in the container): give the file to the owner of this folder,
    so the inventory window on the host can write it too."""
    if hasattr(os, 'geteuid') and os.geteuid() == 0:
        st = os.stat(HERE)
        try:
            os.chown(path, st.st_uid, st.st_gid)
        except OSError:
            pass


def connect_labels():
    os.makedirs(SYNC_DIR, exist_ok=True)
    _share(SYNC_DIR)
    conn = sqlite3.connect(LABELS_DB, timeout=5.0)
    conn.execute(SCHEMA)
    conn.commit()
    _share(LABELS_DB)
    return conn


def next_item_no(conn):
    """one more than the highest number from FIRST_NO up in use here or in the copy"""
    used = {r[0] for r in conn.execute('SELECT item_no FROM labels') if r[0] is not None}
    used |= set(load_registry_numbers().values()) | set(load_names())
    try:
        for _, item_name, _, _, _ in copy_rows():
            m = NUMBERED.match(item_name or '')
            if m:
                used.add(int(m.group(1)))
    except sqlite3.Error:
        pass
    return max([n for n in used if n >= FIRST_NO], default=FIRST_NO - 1) + 1


def add_label(epc, name, expiry, rssi=None, reads=None):
    """Save a label applied here. -> (ok, message, item number or None)"""
    epc = epc.strip().upper()
    conn = connect_labels()
    try:
        row = conn.execute('SELECT item_no, item_name, status FROM labels WHERE rfid_uid = ?', (epc,)).fetchone()
        if row and row[2] != 'discarded':
            return False, 'This tag is already labelled here as Item %d (%s).' % (row[0], row[1]), None
        if epc in load_registry_numbers():
            return False, "This tag is already in abuela's tag registry.", None
        try:
            if any(str(r[0]).upper() == epc for r in copy_rows()):
                return False, "This tag is already in abuela's master.db.", None
        except sqlite3.Error:
            pass
        no = next_item_no(conn)
        conn.execute('INSERT OR REPLACE INTO labels (rfid_uid, item_no, item_name, expiration_date, date_labeled, '
                     'rssi_dbm, reads, status, pushed_at, note) VALUES (?, ?, ?, ?, ?, ?, ?, \'pending\', NULL, NULL)',
                     (epc, no, name, expiry, datetime.now().strftime(TS), rssi, reads))
        conn.commit()
        return True, 'Saved here as Item %d; not pushed to abuela yet.' % no, no
    finally:
        conn.close()


COLS = ('rfid_uid', 'item_no', 'item_name', 'expiration_date', 'date_labeled', 'rssi_dbm', 'reads',
        'status', 'pushed_at', 'note')


def _labels(where, args=(), order='item_no'):
    if not os.path.exists(LABELS_DB):
        return []
    conn = sqlite3.connect(LABELS_DB, timeout=5.0)
    try:
        conn.execute(SCHEMA)
        rows = conn.execute('SELECT %s FROM labels WHERE %s ORDER BY %s' % (', '.join(COLS), where, order), args)
        return [dict(zip(COLS, r)) for r in rows]
    finally:
        conn.close()


def pending():
    """the labels not pushed yet, by item number"""
    return _labels("status = 'pending'")


def history():
    """the labels already pushed, newest push first"""
    return _labels("status = 'pushed'", order='pushed_at DESC, item_no DESC')


def labelled_since(since):
    """every label applied here (not discarded) since `since` ("YYYY-MM-DD HH:MM:SS"), newest first"""
    return _labels("status != 'discarded' AND date_labeled >= ?", (since,), order='date_labeled DESC, item_no DESC')


def discard(epcs):
    """drop labels that were not pushed from the list; returns how many"""
    conn = connect_labels()
    try:
        n = 0
        for epc in epcs:
            n += conn.execute("UPDATE labels SET status = 'discarded' WHERE rfid_uid = ? AND status = 'pending'",
                              (epc,)).rowcount
        conn.commit()
        return n
    finally:
        conn.close()


# ── abuela ───────────────────────────────────────────────────────────────────

def _ssh_python(script, args=(), timeout=60):
    """run a Python script on abuela (sent on stdin) -> CompletedProcess with bytes stdout"""
    cmd = 'python3 - ' + ' '.join(shlex.quote(a) for a in args)
    return subprocess.run(['ssh'] + SSH_OPTS + [HOST, cmd], input=script.encode(),
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)


def _err(p):
    return (p.stderr.decode(errors='replace').strip().splitlines() or ['exit %d' % p.returncode])[-1]


SNAPSHOT_PY = r'''
import os, sqlite3, sys, tempfile
src = sqlite3.connect('file:%s?mode=ro' % sys.argv[1], uri=True, timeout=10)
fd, tmp = tempfile.mkstemp(suffix='.db')
os.close(fd)
try:
    dst = sqlite3.connect(tmp)
    src.backup(dst)
    dst.close()
    with open(tmp, 'rb') as f:
        sys.stdout.buffer.write(f.read())
finally:
    os.unlink(tmp)
'''


def pull(timeout=60):
    """Refresh the copy from abuela. -> (ok, message)"""
    os.makedirs(COPY_DIR, exist_ok=True)
    try:
        p = _ssh_python(SNAPSHOT_PY, [REMOTE_DB], timeout=timeout)
        if p.returncode != 0 or not p.stdout.startswith(b'SQLite format 3\x00'):
            return False, 'master.db: %s' % _err(p)
        tmp = COPY_DB + '.part'
        with open(tmp, 'wb') as f:
            f.write(p.stdout)
        os.replace(tmp, COPY_DB)
        rsync = ['rsync', '-a', '--timeout=%d' % timeout, '-e', 'ssh ' + ' '.join(shlex.quote(o) for o in SSH_OPTS)]
        for src, dst in ((REMOTE_NAMES, COPY_NAMES), (REMOTE_REGISTRY, COPY_REGISTRY),
                         (REMOTE_PHOTOS, COPY_PHOTOS + '/')):
            extra = ['--delete'] if src.endswith('/') else []
            p = subprocess.run(rsync + extra + ['%s:%s' % (HOST, src), dst],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
            if p.returncode != 0:
                return False, '%s: %s' % (os.path.basename(src.rstrip('/')), _err(p))
    except subprocess.TimeoutExpired:
        return False, 'abuela did not answer in %d s' % timeout
    except OSError as e:
        return False, str(e)
    with open(PULLED_STAMP, 'w') as f:
        f.write(datetime.now().strftime(TS) + '\n')
    return True, 'copy updated'


PUSH_PY = r'''
import csv, json, os, re, sqlite3, sys
items = json.loads(PAYLOAD)
db_path, names_path, reg_path = PATHS
NUMBERED = re.compile(r'\s*item\s*#?\s*(\d+)\s*$', re.I)


def parse_numbers(text):
    out = []
    for tok in re.split(r'[\s;+&]+', re.sub(r'\s*-\s*', '-', re.sub(r'(?i)\bitems?\b|#', ' ', text or '')).strip()):
        m = re.fullmatch(r'(\d+)(?:-(\d+))?', tok)
        if not m:
            return None
        a, b = int(m.group(1)), int(m.group(2) or m.group(1))
        if not a <= b <= a + 2000:
            return None
        out.extend(range(a, b + 1))
    return out


def names_numbers():
    out = {}
    with open(names_path, newline='', encoding='utf-8-sig') as f:
        for row in csv.reader(f):
            first = (row[0] if row else '').strip()
            if not first or first.startswith('#'):
                continue
            for no in parse_numbers(first) or ():
                out[no] = ' '.join((row[1] if len(row) > 1 else '').split())
    return out


def append_row(path, fields):
    """one CSV row at the end of the file, after a newline if the file does not end with one"""
    missing_nl = False
    with open(path, 'rb') as f:
        f.seek(0, os.SEEK_END)
        if f.tell() > 0:
            f.seek(-1, os.SEEK_END)
            missing_nl = f.read(1) != b'\n'
    with open(path, 'a', newline='', encoding='utf-8') as f:
        if missing_nl:
            f.write('\n')
        csv.writer(f, lineterminator='\n').writerow(fields)
        f.flush()
        os.fsync(f.fileno())


class Clash(Exception):
    pass


conn = sqlite3.connect(db_path, timeout=10)
names = names_numbers()
with open(reg_path, newline='') as f:
    reader = csv.DictReader(f)
    fields = reader.fieldnames
    reg = {(r.get('epc') or '').strip().upper(): (r.get('name') or '').strip() for r in reader if r.get('epc')}
reg_nos = {}
for e, n in reg.items():
    m = NUMBERED.match(n)
    if m:
        reg_nos[int(m.group(1))] = e

results = []
for it in items:
    epc, no, name = it['epc'], it['no'], it['name']
    label = 'Item %d' % no
    try:
        row = conn.execute('SELECT item_name FROM medical_supplies WHERE rfid_uid = ?', (epc,)).fetchone()
        if row and row[0] != label:
            raise Clash('the tag is already in master.db as "%s"' % row[0])
        if conn.execute('SELECT 1 FROM medical_supplies WHERE item_name = ? AND rfid_uid != ?', (label, epc)).fetchone():
            raise Clash('%s is already another tag in master.db' % label)
        if epc in reg and reg[epc] != label:
            raise Clash('the tag is already in tag_registry.csv as "%s"' % reg[epc])
        if reg_nos.get(no, epc) != epc:
            raise Clash('%s is already another tag in tag_registry.csv' % label)
        if names.get(no, name) != name:
            raise Clash('%s already has the name "%s" in item_names.csv' % (label, names[no]))
        conn.execute('INSERT OR IGNORE INTO medical_supplies (rfid_uid, item_name, expiration_date, date_labeled, '
                     'last_seen, is_on_shelf) VALUES (?, ?, ?, ?, NULL, 0)', (epc, label, it['expiry'], it['labelled']))
        conn.commit()
        if no not in names:
            append_row(names_path, [str(no), name, it['expiry']])
            names[no] = name
        if epc not in reg:
            row = {'epc': epc, 'name': label, 'category': it['category'], 'tag_type': it['tag_type'], 'object': '',
                   'registered_at': it['labelled'], 'rssi_dbm': it['rssi'], 'reads': it['reads'],
                   'antenna': it['antenna'], 'power_dbm': it['power']}
            append_row(reg_path, [row.get(k, '') for k in fields])
            reg[epc], reg_nos[no] = label, epc
        results.append({'epc': epc, 'ok': True, 'msg': 'pushed'})
    except Clash as e:
        results.append({'epc': epc, 'ok': False, 'msg': str(e)})
    except Exception as e:
        results.append({'epc': epc, 'ok': False, 'msg': '%s: %s' % (type(e).__name__, e)})
print(json.dumps(results))
'''


def push(epcs, timeout=60):
    """Push these labels (not pushed yet) to abuela, then refresh the copy.
    -> (ok, message, {rfid_uid: (ok, message)}); ok is False when abuela could not be reached"""
    todo = [r for r in pending() if r['rfid_uid'] in set(epcs)]
    if not todo:
        return True, 'nothing to push', {}
    payload = [{'epc': r['rfid_uid'], 'no': r['item_no'], 'name': r['item_name'], 'expiry': r['expiration_date'] or '',
                'labelled': r['date_labeled'] or '', 'category': CATEGORY, 'tag_type': TAG_TYPE,
                'rssi': '' if r['rssi_dbm'] is None else '%.1f' % r['rssi_dbm'],
                'reads': '' if r['reads'] is None else str(r['reads']),
                'antenna': REG_ANTENNA, 'power': REG_POWER_DBM} for r in todo]
    script = 'PAYLOAD = %r\nPATHS = %r\n' % (json.dumps(payload), [REMOTE_DB, REMOTE_NAMES, REMOTE_REGISTRY]) + PUSH_PY
    try:
        p = _ssh_python(script, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, 'abuela did not answer in %d s' % timeout, {}
    except OSError as e:
        return False, str(e), {}
    try:
        answers = json.loads(p.stdout.decode().strip().splitlines()[-1])
    except (ValueError, IndexError):
        return False, 'push failed: %s' % _err(p), {}
    results = {a['epc']: (a['ok'], a['msg']) for a in answers}
    now = datetime.now().strftime(TS)
    conn = connect_labels()
    try:
        for epc, (ok, msg) in results.items():
            if ok:
                conn.execute("UPDATE labels SET status = 'pushed', pushed_at = ?, note = NULL WHERE rfid_uid = ?", (now, epc))
            else:
                conn.execute('UPDATE labels SET note = ? WHERE rfid_uid = ?', (msg, epc))
        conn.commit()
    finally:
        conn.close()
    pull(timeout)
    n_ok = sum(1 for ok, _ in results.values() if ok)
    return True, '%d of %d pushed' % (n_ok, len(results)), results


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else ''
    if what == 'pull':
        ok, msg = pull()
        print('[master_sync] %s (%s)' % (msg, HOST), flush=True)
        return 0 if ok else 1
    if what == 'pending':
        rows = pending()
        for r in rows:
            print('Item %-5d %-24s %-40s %s%s' % (r['item_no'], r['rfid_uid'], r['item_name'], r['expiration_date'],
                                               ('   last push: ' + r['note']) if r['note'] else ''))
        print('%d label%s not pushed' % (len(rows), '' if len(rows) == 1 else 's'))
        return 0
    print('usage: master_sync.py pull | pending')
    return 2


if __name__ == '__main__':
    sys.exit(main())
