# Ground truth: tag positions, corridor c1 (arrangement of 2026-10-21)

> **URGENT (2026-10-29, not yet fixed): the shelf 7 item positions are mirrored in depth.** Items 14, 23 and 38 have `Y_room_m = BL_Y − depth_m`, which puts them in the aisle. The true value is `BL_Y + depth_m`, about 0.97–1.03 m. The S7 heading of 180° with BL at X 154 in makes the depth convention point toward the robot. Check with `loc/gt_facing_check.py`. The full write-up and fix plan are in the URGENT section of `reports/RFID single antenna error reduction.md`. Audit the rest of this table for similar mistakes before trusting it.

**Use `ground_truth_20261026_full.csv`.** It has 110 rows: every registered tag, made up of 28 shelf-corner reference tags and 82 items on shelves 1–7.

## Columns
- `epc`, `name`, `category`, `tag_type`, `object`: from the tag registry (`handover_20261023/rfid_tools/tag_registry.csv`).
- `shelf`: 1–7.
- `x_shelf_m`, `z_shelf_m`, `depth_m`: position in the shelf's own frame.
  - `x`: along the shelf face from the BL corner tag.
  - `z`: up from the BL corner tag.
  - `depth`: into the shelf behind the face.
- `X_room_m`, `Y_room_m`, `Z_room_m`: room frame. The origin is on the floor below the S1 BL tag. X points out of the S1 face, Y runs along S1 toward the back, and Z is up from the floor.
  - `Z_room_m = bl_height_m + z_shelf_m`.
  - `room_ok = 1` means the room Z is known.
- `bl_height_m`: height of that shelf's BL corner tag above the floor.
- `shelf_heading_deg`: direction of the shelf's x axis in the room frame.
- `source`:
  - `corner`: from the tape-measured shelf geometry;
  - `photo`: clicked in the Shelf Tag Mapper on the shelf photo;
  - `corner_geometry`: a corner the generator had to rebuild from the geometry because the export lacked it. There are none in the current file.

## Measured inputs
- **Shelf geometry** in inches: width W, height H, BL position X and Y, and heading.

  | shelf | W | H | BL X | BL Y | heading |
  |---|---|---|---|---|---|
  | S1 | 22 | 26 | 0 | 0 | 90° |
  | S2 | 22 | 40 | 0 | 70 | 90° |
  | S3 | 22 | 25 | −1.25 | 148.5 | 0° |
  | S4 | 22 | 26 | 52.5 | 148.5 | 0° |
  | S5 | 28 | 35.5 | 116 | 168 | 270° |
  | S6 | 49 | 25 | 108.75 | 104.5 | 270° |
  | S7 | 14.5 | 25.5 | 154 | 31.25 | 180° |

- **BL corner tag height above the floor**, measured 2026-10-26, in inches: S1 4, S2 6, S3 5.5, S4 3.5, S5 15, S6 2, S7 3.25.

## Provenance
- `raw/ground_truth_points_20261026_1644.txt`: the Mapper export used. It holds one cumulative block per shelf, and the last block is complete.
- `raw/ground_truth_points_20261026_1638.txt`: an earlier export whose last block missed shelf 7. Don't use it.
- Rebuild with:
  `python -I make_ground_truth.py raw/<export>.txt ../handover_20261023/rfid_tools/tag_registry.csv ground_truth_20261026_full.csv`
- `../loc/ground_truth_20261026.csv` (S1–S4 only, no room Z) is superseded. Its S1–S4 rows are identical to this file.

## Use in localization
Estimators must not use shelf information. Use this table for scoring, and for training labels on held-out folds. Room coordinates reach a SLAM map through one global rigid transform fitted on the corner tags.

## 2026-11-10 corrections (user-confirmed)
- `ground_truth_20261026_full_s7fix.csv`: run-1 labels with the shelf 7 depth bug fixed. The shelf is placed correctly; only the three S7 items
  (Items 14, 23, 38) had their depth pointing out of the shelf: `Y_room_m` is now `0.7937 + depth_m` (was `0.7937 - depth_m`). The original
  file is unchanged and is still what the cached run-1 dwell tables, models and ledger were built from; use the fixed file when run 1 is retrained.
  The shelf-frame normal for S7 in `eval_v1.py` / `shelfprior_lib.py` is NOT yet corrected (needed for the S7 shelf box when retraining).
- `ground_truth_v2.csv`: object "Item 83" (tags Item 83, Item 84) relabelled dry -> liquid. The user's exported label file is untouched.
- v2 reference-tag reassignment (name shelf -> Mapper shelf) confirmed by the user in full.
