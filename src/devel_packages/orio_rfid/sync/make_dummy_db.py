#!/usr/bin/env python3
"""
make_dummy_db.py - make a throwaway master database for testing pc_receiver.py.

    python3 make_dummy_db.py dummy_master.db [--registry ../RFID/tag_registry.csv]

Copies seed_medical_inventory.db and rebuilds its index (the seed's index is missing one row, so that
item cannot be found by rfid_uid until it is rebuilt). The seed itself is not touched.

--registry adds every item tag of the robot's tag registry that the seed does not have (named as in the registry,
e.g. "Item 12"; the shelf corner tags, category "reference", are left out), so a run in the corridor has a row
for every tagged item. Without it only the seed's 72 items exist and the other tags come back as unknown.
"""
import csv
import os
import shutil
import sqlite3
import sys

SEED = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'seed_medical_inventory.db')

REG = sys.argv[sys.argv.index('--registry') + 1] if '--registry' in sys.argv[:-1] else None
ARGS = [a for a in sys.argv[1:] if a not in ('--registry', REG)]
if len(ARGS) != 1:
    sys.exit(__doc__)
out = ARGS[0]
if os.path.exists(out):
    sys.exit('%s already exists - delete it first or pick another name' % out)
shutil.copy(SEED, out)
conn = sqlite3.connect(out)
conn.execute('REINDEX')
conn.execute('UPDATE medical_supplies SET last_seen = NULL, is_on_shelf = 0')
added = 0
if REG:
    for r in csv.DictReader(open(REG)):
        if r.get('category') == 'reference' or not r.get('epc'):
            continue
        added += conn.execute('INSERT OR IGNORE INTO medical_supplies (rfid_uid, item_name, is_on_shelf) VALUES (?, ?, 0)',
                              (r['epc'].strip().upper(), r.get('name') or r['epc'])).rowcount
    print('added %d item tags from %s' % (added, REG))
conn.commit()
print('%s: %d items, check: %s' % (out, conn.execute('SELECT COUNT(*) FROM medical_supplies').fetchone()[0],
                                 conn.execute('PRAGMA integrity_check').fetchone()[0]))
