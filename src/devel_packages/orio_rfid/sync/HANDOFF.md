# RFID robot-to-PC sync: handoff notes

Written 2026-11-09 for an AI agent (or engineer) picking this work up with no other context.
Everything below was established in one working session on the robot `stretch-re1-1076`.

## Update 2026-11-13: what changed since these notes

The notes below describe the state on 2026-11-09 and are kept as written. Since then:

- **The scan demo exists**: `../demo/run_scan_demo.sh` on the PC starts the receiver, the robot's agent, the item
  localiser and a window. Read `../demo/README.md` first.
- **The one-connection problem is solved for a run**: `robot_sync.py --follow <run>/reads_posed_live.csv` takes the
  sightings from the run's own read log (written live by `../RFID/rfid_pose_live.py`), so presence sync and
  `rfid_log.py` run in the same pass. `--reader` is unchanged and still has never read a tag.
- **Item location is no longer out of scope**: `--stream-reads` sends every posed read to the PC (`POST /reads`),
  and `../loc/live/engine.py` places the items (`POST /locations`, table `item_locations`). `GET /state` gives the
  window everything it shows.
- **A machine was tried as the PC**: abuela, over Tailscale, with a test database
  (`make_dummy_db.py --registry` adds the registry's item tags to the seed). Nothing has been run against the real
  master database or on the team router yet; items 3, 4 and 6 of "Remaining work" are still open.

## Goal

Team G (ORIO, CMU capstone) has a Stretch RE1 robot carrying an Impinj R700 RFID reader. The robot
drives around an operating-room supply room and reads RFID tags on supplies. A separate "main PC"
(the one connected to the robot arms and the labelling station) holds the master SQLite inventory.

The work here: send item presence from the robot to the main PC over the team's own router, and
send the PC's item list back to the robot, so the two databases stay in step.

Item *location* is explicitly out of scope for now. It is computed off the robot from recorded
logs, because the team is unsure the robot has spare compute while running ORB-SLAM.

## Current state in one paragraph

Three standalone Python scripts exist in this folder and are **uncommitted**. The data path
(robot DB -> HTTP -> PC DB, item list back, reset on each launch, recovery after a dropout) is
tested with simulated reads, both ends on the robot. A manual test across the team router, with
the receiver on a team member's laptop, also worked, but with the earlier version of the scripts
(before the scan reset and `--reader` were added). The real-reader option (`--reader`) is written
but **has never read a tag**: during testing another process held the reader's only connection.
Nothing has been run on the main PC yet.

## Files in this folder

| File | Runs on | Purpose |
|---|---|---|
| `pc_receiver.py` | main PC (or a laptop for testing) | HTTP server; applies sightings to the master DB; serves the item list |
| `robot_sync.py` | robot | Scans tags (real reader or simulated), keeps a local DB, sends sightings, pulls the item list |
| `make_dummy_db.py` | wherever the receiver runs | Builds a throwaway master DB from `../seed_medical_inventory.db` |
| `HANDOFF.md` | - | This file |

Dependencies: Python 3.8+ standard library only. `robot_sync.py --reader` also needs `sllurp`
(2.0.1 is installed on the robot). No ROS. This was deliberate: the router has no internet, so
nothing can be installed once the machines are on it.

## How the system works

### Databases

**Master DB on the PC** uses the team's existing table, unchanged:

```sql
CREATE TABLE medical_supplies (
    rfid_uid TEXT PRIMARY KEY,   -- EPC, 24 uppercase hex characters
    item_name TEXT,
    expiration_date TEXT,        -- 'YYYY-MM-DD'
    date_labeled TEXT,           -- 'YYYY-MM-DD HH:MM:SS', PC local time
    last_seen TEXT,              -- 'YYYY-MM-DD HH:MM:SS', PC local time
    is_on_shelf INTEGER DEFAULT 0
);
```

`pc_receiver.py` adds one small table to the same file:
`sync_state(key TEXT PRIMARY KEY, value REAL)`, holding `scan_started` (unix seconds).

**Robot DB** (`robot_items.db` by default, created by `robot_sync.py`):

```sql
CREATE TABLE items (
    rfid_uid TEXT PRIMARY KEY, item_name TEXT, expiration_date TEXT,
    last_seen REAL,   -- unix seconds (UTC), newest read on the robot
    last_sent REAL    -- newest last_seen the PC has confirmed
);
CREATE TABLE sync_state (key TEXT PRIMARY KEY, value REAL);  -- scan_started, scan_acked
```

A row is "unsent" when `last_seen` is set and `last_sent` is NULL or older. Only the newest
sighting per tag is kept, so the table has one row per tag and never grows into a log.

### HTTP interface (served by `pc_receiver.py`, default port 8000)

| Request | Body / reply |
|---|---|
| `GET /health` | `{"ok": true, "items": <count>}` |
| `GET /items` | `{"items": [{"rfid_uid", "item_name", "expiration_date"}, ...]}` |
| `POST /sightings` | Body: `{"robot": "<hostname>", "scan_started": <unix s>, "sightings": [{"rfid_uid": "...", "seen_at": <unix s>}]}`. Reply: `{"ok": true, "updated": n, "unknown": [uids], "reset": bool}` |

### Presence rules

- **Each launch of `robot_sync.py` is a new scan.** It stores its launch time as `scan_started`
  and sends it with every batch. When the PC receives a `scan_started` newer than the one it has
  stored, it sets `is_on_shelf = 0` for every item, then stores the new value.
- **A sighting sets `is_on_shelf = 1`** only if its `seen_at` is at or after the current
  `scan_started`. It always moves `last_seen` forward if it is newer.
- **`last_seen` survives across scans.** Only `is_on_shelf` is reset.
- **Nothing times out.** The old "not seen for 5 s = missing" rule in `db_manager.py` was a
  demo shortcut for a static antenna and the user does not want it. An item is missing if it
  was not read in the current scan.
- **`--resume`** reuses the stored `scan_started`, so a restart mid-scan does not reset anything.
- **Unknown tags** (read by the robot but absent from the PC's item list) are kept on the robot
  with an empty name and reported back by the PC in `unknown`. The PC ignores them.

### Dropout behaviour

Rows are marked sent only after the PC replies `ok`. If the PC is unreachable, sightings
accumulate and go out in one batch on reconnect. The scan reset rides along with the first
batch that gets through, so a robot launched while the PC is down still resets it correctly.

### Reader settings (`--reader`)

Taken from `../RFID/rfid_log.py`, which holds settings measured on 2026-10-21:

| Setting | Default | Note |
|---|---|---|
| Reader address | `192.168.0.20` | via the robot's ethernet port `eno1` (NetworkManager profile `impinj-rfid`, robot side `192.168.0.10`) |
| Antenna | port 4 | |
| Power | 30 dBm | allowed 10-33. Never pass 0: sllurp treats 0 as maximum power |
| RF mode | 2 | slow dense-reader mode; read 76 tags vs 53 for mode 0, so better for presence |
| Search mode / session | 2 (dual target) / 0 | |
| Report every | 20 reads | |

The reader accepts **one LLRP connection at a time**. `robot_sync.py --reader` cannot run
alongside `rfid_log.py`, `record_pass.sh`, `register_tags.py` or `db_manager.py`. If refused, it
prints `NOT SCANNING ...` and retries every 5 s.

## How to run

```bash
# Receiver (main PC or laptop). For a test database:
python3 make_dummy_db.py dummy_master.db        # needs ../seed_medical_inventory.db
python3 pc_receiver.py --db dummy_master.db     # --port 8000 by default

# Robot, simulated reads (3 random known items per second):
python3 robot_sync.py --pc http://<receiver-ip>:8000 --simulate 3

# Robot, real reader:
python3 robot_sync.py --pc http://<receiver-ip>:8000 --reader
```

Single-machine test: run both on one machine with `--pc http://127.0.0.1:8000`.
On the robot itself port 8765 is taken by another service; 8000 and 8123 were free.

## What was tested, and what was not

Tested on the robot over loopback, with simulated reads and a dummy master DB:

- Sightings update `last_seen` and `is_on_shelf` on the PC side; values match the robot's.
- An item inserted into the master DB appears in the robot DB on the next pull.
- Receiver killed mid-run and restarted: the backlog arrives in one batch, nothing lost.
- A new launch resets every item to not on shelf; `--resume` does not.
- A launch while the receiver is down delivers the reset when it comes back.
- A busy reader produces the `NOT SCANNING` message and retries.

Tested by the user across the router (robot -> laptop), with the earlier script version: the
basic send worked.

Not tested:

- **`--reader` reading real tags.** Blocked by a `register_tags.py` process (PID 7691, started
  17:54 on 2026-11-09) holding the reader. It was left running on purpose.
- The current script versions across two machines.
- Anything on the main PC, or against the real master database.
- Running alongside `inventory_dashboard.py`.

## Network

- Router: TP-Link Archer A54, Wi-Fi name `TeamG_ORIO_5G`, LAN changed to `192.168.50.0/24`
  (router `192.168.50.1`). The default `192.168.0.x` was avoided because it collides with the
  Impinj reader's subnet on `eno1`.
- The router has **no internet**. While the robot is on it: no Tailscale, no git, no pip/apt, no
  cloud-based tools, and no clock sync from the internet.
- The robot has one Wi-Fi radio. It also has saved profiles for `CMU-SECURE` and others with
  autoconnect on; on 2026-11-09 it joined `TeamG_ORIO_5G` at 19:22 and was back on `CMU-SECURE`
  at 19:40.
- On the router the robot received `192.168.50.105`, so its address reservation is not in
  effect. The plan was PC `192.168.50.20`, robot `192.168.50.10`.
- The first cross-machine attempt failed (ping OK, curl not) because `pc_receiver.py` had not
  been copied to the receiving machine. Diagnosis rule: "connection refused" means nothing is
  listening; "timed out" means a firewall.

## Known problems in the existing code (outside this folder)

1. **`seed_medical_inventory.db` has a damaged index.** The table has 72 rows but the primary-key
   index has 71 entries; the row for `E28011B0A50500726AC88E39` (Safety Infusion Set) cannot be
   found by `rfid_uid`, so it can never be marked seen. `REINDEX` fixes it.
   `make_dummy_db.py` repairs its own copy. The tracked seed was **not** modified; the user has
   not yet said whether to fix it. Any real database copied from the seed has the same fault.
2. **`db_manager.py` on the PC will fight the receiver.** Its loop pings the Impinj reader (not
   reachable from the PC) and sets `is_on_shelf = 0` for anything not seen in 5 s. That half must
   be disabled on the PC; the labelling half (`/inventory/add_item`, Mercury reader on
   `/dev/ttyUSB0`) stays. The Mercury reader is on the main PC, not the robot, and
   `state_machine.py` is not run on the robot.
3. **`RFID/shelf_scanner_node.py` is dead code** (publishes `/inventory/shelf_scans`, nothing
   subscribes) and **`RFID/labelling_zone_node.py` cannot run** (imports a service that does not
   exist in `custom_msgs`).
4. **Uncommitted work.** This folder and many newer scripts under `../RFID/` (`register_tags.py`,
   `rfid_log.py`, `record_pass.sh`, ...) exist only on this robot. `origin/main` is about 20
   commits ahead of the local checkout, none touching `orio_rfid` or `custom_msgs`.

## Remaining work for the final demo

1. Confirm `--reader` reads tags (free the reader first), and check power/mode in the real room.
2. Decide what starts and ends a scan. Today a scan is "one launch of the script". If the
   location logging in `rfid_log.py` must run during the same pass, the two programs have to be
   merged, because the reader allows one connection.
3. Run `pc_receiver.py` on the main PC against the real database; disable the Impinj half of its
   `db_manager.py`; `REINDEX` the database.
4. Network: set the address reservations, open port 8000 on the PC (`sudo ufw allow 8000/tcp`),
   make the robot prefer `TeamG_ORIO_5G`, and sync the robot clock to the PC with `chrony`
   (sighting times and `scan_started` come from the robot's clock).
5. Start both scripts automatically and show "last update from robot" on the dashboard.
6. Rehearse: full room pass, walk out of Wi-Fi range and back, restart each machine mid-run,
   label a new item and confirm it reaches the robot.
7. Commit this folder.

## Constraints to respect on this robot

- Do not kill processes you did not start. A cron-started `~/teleop_server` holds the robot's
  serial ports and camera, and teammates leave RFID tools running.
- Do not modify `~/grocery_bot`.
- Do not transmit at power 0 through sllurp (it means maximum).
- The user is new to networking; explain steps plainly.
