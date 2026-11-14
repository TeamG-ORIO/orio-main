# Item localisation: methods F1 to F4

Where each tagged item is, from the RFID reads of a run and the antenna pose at every read.

| Folder | What |
| --- | --- |
| `live/engine.py` | the localiser used in the demo: one method at a time, while the run is going, GPU or CPU |
| `live/check_run2.py` | scores a result on run 2 against the tape-measured positions and against the offline result |
| `offline/` | the method study's code, **exactly as on abuela** (`~/rfid_db/loc`, commit `a2168f4`, copied 2026-11-13). `engine.py` imports `relmodel.py`, `eval_v1.py` and `shelfprior_lib.py` from here unchanged. The scripts (`final_apply.py`, `viz_beliefs_v2.py`, ...) still have abuela's paths in them and are kept as the reference, not to be run here |
| `models/` | the trained models of run 2: read model (`relmodel_r2cv_fold0-4.pt`), end-to-end net (`e2e_r2cvL_fold0-4.pt`), which fold each tag belongs to, region temperatures |
| `site/` | the corridor: shelf geometry, tape-measured tag positions (9 Nov), locked test items, `live_site_v2.json` (map-to-room tie, grid, channel table), and the demo window's map scene (`scene_v2.json` + point cloud, from `make_scene.py`) |

Run 2's own tables and abuela's stored results, for checking, are data and live outside the repo:
`~/orio/rfid_loc/run2/` on the robot (78 MB).

## The four methods

All four add the same evidence on a 5 x 5 x 10 cm grid of the room: the learned read / no-read / signal-strength
model, the phase inside each channel dwell, and a learned end-to-end network. Tags on one object share evidence
(F1 to F3).

| Method | Adds | Needs |
| --- | --- | --- |
| F1 | nothing | no map, no reference tags |
| F2 | pull toward the corner tags whose signal pattern over the run looks alike | the corner tags' positions |
| F3 | F2, with 99 % of the probability inside the shelf boxes | corner tags + shelf boxes |
| F4 | shelf boxes weighted by where the other measured items sit | shelf boxes + the other items' positions |

`engine.py --method` picks one (the demo window's F1..F4 buttons). Settings are fixed, as chosen on run 2's dev items
(`offline/PLAN_methods_v2.md`).

## How well the live version does

Run 2 (2 laps, 212 s, 126,673 reads), every read fed through `engine.py` in 2 s rounds as in a live run. Median 3D
error in cm against the tape measure, all 132 scored items, next to the offline result of the same method
(`final_apply.py` on abuela):

| Method | Live | Offline | Live vs offline, median distance | Right shelf |
| --- | --- | --- | --- | --- |
| F4 | 20 | 20 | 6 | 132 of 132 |
| F3 | 24 | 22 | 10 | 132 of 132 |
| F2 | 30 | 29 | 12 | (no shelf map) |
| F1 | 37 | 34 | 15 | (no shelf map) |

F4 on the 28 locked test items: 17 live, 18 offline. Reproduce:

```bash
python3 live/engine.py --once ~/orio/datasets/corridor_v2_rfid_2laps_run2/reads_posed.csv --method F4 --out /tmp/f4.csv
python3 live/check_run2.py /tmp/f4.csv F4
```

## Speed and delay

Decision 2026-11-14: the computing is done on the PC (abuela's GPU), on the whole grid, with no shortcut that changes a
position. The robot only sends its reads.

Measured with run 2 replayed at real speed from the robot to abuela over Tailscale, one localiser on the GPU, final
settings (`--every 1`, `--estimate-budget 0.7`, all cells). Two delays matter:

- **read to picture**: how old the newest read is that the published positions are based on;
- **per item**: from a read of one item to that item's own new position. Longer, because nearly every tag is read all
  the time (the reader sees 100+ tags from anywhere in the corridor), so all ~120 objects queue to be re-estimated and
  each round does as many as fit in 0.7 s, longest-waiting first.

| On abuela | Read to picture (typical; range) | Per item (typical; 90 % within; worst) |
| --- | --- | --- |
| F4, all cells | 4.0 s (2.9 to 5.0; worst round 6.6) | 10.6 s (11.8; 15.6) |
| F3, all cells | 4.4 s (3.4 to 5.5; 6.8) | 10.6 s (12.0; 15.1) |
| F2, all cells | 4.4 s (3.3 to 5.6; 6.7) | 9.9 s (11.7; 14.8) |
| F1, all cells | 4.4 s (3.4 to 5.5; 6.5) | 7.9 s (9.8; 12.9) |
| F4, `--cells shelf` | 3.4 s (2.5 to 4.2; 5.9) | 4.0 s (4.6; 6.0) |

Before the localiser: a read has its pose 0.15 s after it is logged (95 % within 0.25 s) and reaches abuela 0.13 s after
that (95 % within 0.40 s; 1.3 Mbit/s). After it: the window asks twice a second and redraws four times a second, about
0.4 s on average. With all cells the localiser is busy 97 % of the time: there is no headroom, and a second program on
the GPU made F4 slip to 5.9 s / 19.5 s. Reproduce with `live/latency.py <the run's folder under runs/>`.

`--cells shelf` (conf: `CELLS=shelf`) is the one shortcut on offer for F3 / F4. F4 positions against all cells: run 2
identical to the millimetre for 132 of 132 items, tonight's run 135 of 135, `corridor_v2_rfid_2laps` 129 of 130 (one item
2.3 cm away). F3: 131 of 132 and 129 of 130 (1.2 and 3.0 cm). `--rm-block` (read model on coarser blocks) scored the same
on two recordings but is an approximation of the model itself and stays off.

**On the robot** (i5-8260U, no GPU; nothing else running, so better than in a real run):

| On the robot's CPU, F4 | Read to picture | Per item | Keeps up? |
| --- | --- | --- | --- |
| shelf cells (all cells is out of reach on a CPU) | 11.5 s and growing | 24.5 s typical, 91 s worst | no: 1.07 x real time |
| shelf cells + read model on 20 cm blocks | 2.0 s | 12.9 s typical, 18 s worst | only just (0.93 x real time, all four threads) |

During a run ORB-SLAM3, the depth cloud and `move_base` need those cores, and F1 / F2 need the whole grid (16 times the
work). The robot is not a place to run this.

What made the live version fast enough at all: the read model once per dwell for each (fold model, tag type) instead of
once per tag, evidence added as reads arrive, one method, and the blur for the dominant mode only where the tag's
probability is. None of these changes a position (checked item by item against the code before each change).

## Which tags get a position

A tag is placed once one pass of it has 30 reads (a pass ends after 8 s without a read) and it was read in 5 channel
dwells. That is abuela's own rule (`offline/loc_v1.py`), and the scores above are for the tags that pass it: 132 on run 2,
the same 132 the replay page shows. Without it the live localiser also placed 14 tags with 9 to 54 reads in the whole run,
a median 3.9 m from their tape positions. A tag below the rule is still marked present; it has no position.

## What differs from the offline pipeline

1. **Poses.** Live uses the robot's map poses as logged. Offline, abuela rebuilds a smoothed trajectory after the run
   (`adapt_v2.py`: odometry-propagated median over +-1 s).
2. **Map-to-room tie.** Offline it is refitted for every run from that run's corner-tag estimates; live it is fixed
   (`site/live_site_v2.json`, from run 2). The three recordings gave ties 2 degrees and up to 0.35 m apart, so on
   another day the live estimates can sit a few tens of cm off as a whole. Run 2 itself cannot show this.
3. **New tags.** A tag that is not in `models/relmodel_r2cv_folds.csv` gets a fold model from its name. Offline every
   tag is scored by the model that never saw its location; for a new item that cannot be known.
4. **The shelf map is the 9 Nov measurement.** F3 uses the box depths, F4 also where the other items sat. Items added
   or moved since then are not in it (they are still placed; F4's weighting is then a little out of date).
5. With `--cells shelf` the probability of "not on any shelf" is not computed (`p_lost` empty).
6. Corner tags are inputs, not outputs: they get no estimate.

## Not done

- The uncertainty ellipsoids of the replay page: only `region90_m` (radius of a sphere with the 90 % region's volume), which the
  demo window's map draws as a circle. The positions themselves are on that map, in 2D and 3D (`../demo/scene_view.py`).
- A live refit of the map-to-room tie.
