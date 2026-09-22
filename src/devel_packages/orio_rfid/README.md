# orio_rfid

RFID inventory subsystem for ORIO (part of the SVD/Encore demo).

## Nodes
- `db_manager.py` — `/inventory/add_item` service; owns the SQLite inventory DB.
- `RFID/shelf_scanner_node.py` — Impinj shelf reader → `/inventory/shelf_scans`.
- `RFID/labelling_zone_node.py` — Mercury labelling-station reader (`tmr:///dev/ttyUSB0`).
- `inventory_dashboard.py` — Tkinter dashboard. `bulk_read_insert.py` — bulk import tool.

## Database (seed vs runtime)
- **`seed_medical_inventory.db`** — the canonical product catalog (tracked in git; 71 items).
- **`medical_inventory.db`** — the mutable runtime copy the demo writes to (git-ignored).

`db_manager.py` copies the seed → runtime on first run if the runtime DB is missing
(paths resolved relative to the file, so it works from any CWD). To reset the catalog
to the seed: `rm medical_inventory.db` and restart `db_manager.py`.

## Dependencies
- `sllurp` (Impinj LLRP) — `pip install sllurp`
- `mercury` — built from the `RFID/python-mercuryapi` **submodule**
  (`cd RFID/python-mercuryapi && make && pip install .`; the submodule pins Team G's
  fork with SDK 1.37.5.49).
- `custom_msgs` for the ROS services/messages.

Fresh checkout: `git submodule update --init RFID/python-mercuryapi`.

> Note: `labelling_zone_node.py` references a `TriggerLabeling` service not yet present
> in `custom_msgs` (only `AddLabeledItem.srv` exists) — pre-existing; add that .srv or
> fix the import before running that node.
