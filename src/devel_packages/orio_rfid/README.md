# orio_rfid

RFID inventory subsystem for ORIO (part of the SVD/Encore demo).

## Nodes
- `db_manager.py` — `/inventory/add_item` service: reads the new label's tag with the
  Mercury reader and saves it on this PC (`master_sync/labels.db`) as Item 1001, 1002, …
- `master_sync.py` — the labels saved here and this PC's copy of abuela's master
  inventory (`master_sync/copy/`: `master.db`, `item_names.csv`, `tag_registry.csv`,
  `item_photos/`). `python3 master_sync.py pull` refreshes the copy; `pending` lists the
  labels not pushed yet. abuela is reached over ssh (`ORIO_MASTER_HOST`, default
  `iam-abuela-local`; `ORIO_MASTER_DIR`, default `/home/student/orio_scan`).
- `inventory_dashboard.py` — inventory window: abuela's stock (refreshed every 15 s) and
  each label as it is saved. **PUSH TO MASTER** lists the labels not pushed yet: tick
  some to push them to abuela (a row in `master.db`, a line in `item_names.csv` and in
  `RFID/tag_registry.csv`) or to discard them; pushed labels are listed below.
  `run_full_demo.sh` opens it when RFID is on; by hand:
  `python3 inventory_dashboard.py` (runs on the host, from any folder).
- `RFID/shelf_scanner_node.py` — Impinj shelf reader → `/inventory/shelf_scans`.
- `RFID/labelling_zone_node.py` — Mercury labelling-station reader (`tmr:///dev/ttyUSB0`).
- `bulk_read_insert.py` — bulk import tool (writes the old local `medical_inventory.db`).

## Item names and pictures
The item setup window at the start of a run (`orio/item_setup_gui.py`) lists the names
in abuela's `item_names.csv` (from the copy) plus the names typed in through **Other…**
(`item_catalog/local_items.json`, git-ignored). Pictures are abuela's `item_photos`;
items without one use `item_catalog/images/default.png`.

## Dependencies
- `sllurp` (Impinj LLRP, for `RFID/shelf_scanner_node.py`) — `pip install sllurp`
- `mercury` — built from the `RFID/python-mercuryapi` **submodule**
  (`cd RFID/python-mercuryapi && make && pip install .`; the submodule pins Team G's
  fork with SDK 1.37.5.49).
- `custom_msgs` for the ROS services/messages.

Fresh checkout: `git submodule update --init RFID/python-mercuryapi`.

> Note: `labelling_zone_node.py` references a `TriggerLabeling` service not yet present
> in `custom_msgs` (only `AddLabeledItem.srv` exists) — pre-existing; add that .srv or
> fix the import before running that node.
