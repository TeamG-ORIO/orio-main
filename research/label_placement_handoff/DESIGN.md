# Label placement: avoid text and folds — design and test handoff

Self-contained: nothing here assumes access to the robot or its code repository. Measurements
are from the real cell on 2026-10-27.

Files in this folder:

| File | What it is |
| --- | --- |
| `DESIGN.md` | This document. |
| `label_placement.py` | Reference implementation of the revised algorithm (runs on saved captures). |
| `current_opt_label_location.py` | The placement code the robot runs today (the baseline), unchanged. |
| `requirements.txt` | Python dependencies. |

Capture data is not in this folder (about 20 MB per capture); see [Data](#2-data).

---

## 1. Context

A robot arm presses a self-adhesive label onto medical-supply items: boxes, paper/Tyvek
pouches, clear plastic pouches and bags, some with folded or crumpled edges. A ZED-M
stereo camera looks straight down from 0.98 m above the table. Items lie on a black cloth.

- Image: 1920 × 1080, focal length fx = fy = 1509.6 px, so **about 0.65 mm per pixel** on the table.
- Depth: from Fast-FoundationStereo (a learned stereo network) run on the ZED's left/right
  pair at **half resolution** (960 × 540), then resized back to 1920 × 1080 by nearest
  neighbour. Metric, in metres, aligned pixel-for-pixel with the left colour image.
- Label footprint: a **120 × 40 px rectangle** (about 78 × 26 mm), placed along the item's
  major or minor axis. It is assumed symmetric under a 180° turn.
- Segmentation: on the robot, an open-vocabulary detector (GroundingDINO) plus SAM give
  each item a binary mask. For testing at home, `label_placement.py` has a stand-in that
  thresholds brightness against the black cloth (`segment_items_on_dark_background`).

The placement must:
1. lie wholly on the item;
2. not cover printed text (or barcodes, symbols);
3. sit on a flat part of the item, not across a fold, step, crease or the edge of something
   inside a pouch (the label would tent and not stick).

## 2. Data

Each capture is a folder:

| File | Content |
| --- | --- |
| `color.png` | Left image, 1920 × 1080, BGR (OpenCV order). |
| `depth.npy` | float32, metres, 1080 × 1920, NaN where invalid. Same pixel grid as `color.png`. |
| `K.npy` | 3 × 3 intrinsics. |
| `right.png` | Right image of the same stereo pair (not used by the algorithm). |
| `color.npy`, `preview.png` | RGB array, colour beside depth (convenience). |
| `meta.json` | Capture time, camera exposure/gain, depth summary. |

Only `color.png` and `depth.npy` are needed. The 11 captures from 2026-10-27, each with two
items (folder names are capture times):

| Folder | Exposure | Items |
| --- | --- | --- |
| `20261027_174708` | 30 | IV tubing set in a clear pouch; glove box (yellow/white, large and small text) |
| `20261027_174810` | 50 | Same two items, moved |
| `20261027_174830` | 50 | White kit pouch with blue print; back of a yellow box (white-on-yellow small text) |
| `20261027_174904` | 50 | IV fluid bag in a clear wrapper (bulging); white/yellow kit pouch |
| `20261027_174918` | 30 | **Same scene, items not moved** (repeat of 174904) |
| `20261027_175016` | 30 | Clear pack with gauze; stack of paper syringe packets (grooves between packets) |
| `20261027_175103` | 30 | Catheter in a pouch (card on a coil inside); rigid white/purple box |
| `20261027_175123` | 50 | **Same scene, not moved** (repeat of 175103) |
| `20261027_175224` | 50 | Rigid white box with a dense multilingual text list; small white crinkled packet with folded edges |
| `20261027_175243` | 30 | **Same scene, not moved** (repeat of 175224) |
| `20261027_175452` | 30 | Large bag (paper card, clear film end, crumpled top); small clear packet with a folded flap |

The brightness segmentation also finds a bright object at the top-left of the frame
(bbox starting at (120, 150)); it is background clutter. Ignore it.

## 3. The current algorithm (baseline)

`current_opt_label_location.py`, function `opt_label_loc`:

1. Edge map: Sobel gradient magnitude on the grey image, thresholded at 100.
2. Candidate centres: the peaks of `0.2·EDT(mask) + 0.8·EDT(non-edge pixels in the mask)`.
   EDT is the Euclidean distance transform, so these are points far from the item boundary
   and from edges. They are thinned with adaptive non-maximal suppression, up to 50.
3. In that order, the first candidate where a 120 × 40 footprint, at the major- or
   minor-axis angle, lies wholly inside the mask and has ≤ 5% of its pixels on edges wins.
   If none passes, it falls back to the mask centroid.

Problems found:
- **Too conservative.** Any colour boundary (panel seams, box outlines, graphics) counts
  as text. On the captures, only **0–3 of the 17–50 candidates** pass the edge check.
- **Misses text at the same time.** White-on-yellow text has almost no contrast in grey,
  and fine print is 4–6 px tall, so both slip under the threshold.
- **No depth term**, so labels land on folds and bulges. Results are in section 6.

## 4. What the depth looks like (measurements)

These findings set the design of the depth term.

1. **No holes or spikes** on clear plastic: depth is valid on 100% of item pixels and smooth.
   The network fills in rather than failing.
2. **The shape wanders between captures** on glossy and clear film. Repeat captures of the
   same unmoved items:
   - rigid opaque boxes changed by ≤ 0.4 mm (95th percentile);
   - film changed by 2–3 mm (95th percentile), up to ~5 mm, even at a fixed exposure.

   The change comes in **broad, smooth patches**, not pixel noise, and concentrates near real
   structure (edges of a card inside a pouch, crumpled ends). A plane fitted over a
   label-sized area absorbs most of it.
3. **Overexposure makes it much worse.** A white packet had 48% of its pixels clipped at
   exposure 50 and 1% at exposure 30. Its depth under the same spot went from flat to
   4–5 mm uneven between the two. Capture at exposure ≤ 30.
4. **Real steps are blurred** over 10–20 px, and item outlines have a staircase edge from
   the half-resolution depth. Jumps between neighbouring pixels are not a usable
   discontinuity signal; evaluate at label scale and keep away from the mask edge.
5. **What depth cannot see:**
   - A fold pressed flat is about 0.2–0.5 mm, below the noise. Only folds that stand up
     are visible.
   - A thin film margin lying on the table reads as flat. The mask must stop a label hanging off it.
6. Unevenness of label footprints (definition in section 5.1), by surface:

   | Surface | Median | Change between repeat captures (95th pct) |
   | --- | --- | --- |
   | Rigid boxes | 0.4–0.7 mm | ≤ 0.5 mm |
   | Mostly flat pouches/bags | 0.5–1.4 mm | 2–3 mm |
   | Folded packets | 3–8.5 mm | 3–5 mm |
   | Fluid-filled IV bag | 7–9 mm | ~3.7 mm |

   The flattest footprint chosen in one capture stayed ≤ 2 mm in every repeat capture,
   except across the exposure-50/30 pair of the white packet (point 3).

## 5. Revised algorithm

### 5.1 Unevenness of a footprint

For a footprint at centre (x, y) and angle θ:

1. Grow the 120 × 40 rectangle by `MARGIN` = 8 px (~5 mm) on each side. This covers arm
   placement error and uncertainty in where the stereo net puts a step. On the captures it
   made the one borderline case stable (2.6/3.7/1.3 mm became 1.8/2.0/1.8 mm).
2. Take the depth pixels inside it, at half resolution (no information lost, 4× fewer
   pixels). Keep those on the item mask **eroded by 10 px**, to avoid the staircase edge.
   Require at least 60% of the bare footprint's area.
3. Fit a least-squares plane z = a + b·u + c·v (z in mm) and take the residuals.
4. **Unevenness = 98th − 2nd percentile of the residuals (mm).** For a step of height s
   across the label, this is about s. Tilt (from b, c) is also reported but not used.

### 5.2 Search and decision

Searching only the old edge-based candidates does not work: so few pass the edge check
that the depth term has nothing to choose from. A depth filter bolted on after the old
search picked the same spot as today on every item. So:

1. **Grid search** every 6 px over the crop, at both axis angles.
2. **Hard constraints** per footprint (dense maps via `cv2.erode` and `cv2.filter2D` with
   the rotated footprint as kernel):
   - footprint wholly inside the mask;
   - text: fraction of footprint on the dilated text mask ≤ `TEXT_MAX` (0);
   - old edge check (≤ 5% on Sobel edges), kept for now as a safety net (see 5.3).
3. **Tiered choice by unevenness (PV)** among the feasible footprints:
   - **PV ≤ 2 mm counts as flat.** Below that, depth differences are noise and must not
     move the label. Among these, keep the old preference: the highest
     `0.2·EDT(mask) + 0.8·EDT(mask minus edges/text)`.
   - Otherwise take the **least uneven**: status `uneven` if ≤ 5 mm, `flagged` if more.
     A flagged item has no flat spot; the robot should skip it or ask an operator, not place
     at the centroid.

| Parameter | Value | Why |
| --- | --- | --- |
| `PV_FLAT` | 2.0 mm | Flattest film spots stayed ≤ 2 mm across repeats; rigid surfaces ≤ 0.7 mm |
| `PV_MAX` | 5.0 mm | Folds and bulges measure 4–15 mm |
| `MARGIN` | 8 px | ~5 mm; stability across repeat captures |
| `MASK_ERODE` | 10 px | Clears the half-resolution staircase at item edges |
| `STRIDE` | 6 px | Grid spacing; 20–230 ms per item in plain Python |
| `EDGE_MAX` | 0.05 | Unchanged from the current code |
| `TEXT_MAX` | 0.0 | No overlap with (dilated) detected text |

### 5.3 Text term and recommended detector

Text detection only needs to find *where* text is, not read it. **Recommendation:
PaddleOCR's PP-OCRv5 *server* detection model**, run through the `rapidocr` package
(version 3.10.0, ONNX Runtime backend). It takes the item crop and returns text-box
polygons; fill them and dilate by 6 px (`text_mask_ppocr` in `label_placement.py`).

```python
from rapidocr import RapidOCR, OCRVersion, ModelType, EngineType, LangDet
ocr = RapidOCR(params={"Det.ocr_version": OCRVersion.PPOCRV5, "Det.model_type": ModelType.SERVER,
                       "Det.lang_type": LangDet.CH, "Det.engine_type": EngineType.ONNXRUNTIME})
boxes = ocr(crop_bgr, use_det=True, use_cls=False, use_rec=False).boxes   # N x 4 x 2, or None
```

(`LangDet.CH` is the model's name; it detects Latin text too.)

Comparison on five item crops (number of text boxes found; CPU time per crop):

| Detector | Glove box front | Clear pouch | Syringe packets | Visistat text list | Yellow box back | CPU time |
| --- | --- | --- | --- | --- | --- | --- |
| PP-OCRv4 mobile | 5 | 2 | 5 | 4 | 2 | 0.05 s |
| PP-OCRv4 server | 10 | 2 | 1 | 6 | 1 | 3.5–5 s |
| PP-OCRv5 mobile | 8 | 3 | 3 | 5 | 2 | 0.05–0.08 s |
| **PP-OCRv5 server** | 9 | 2 | **35** | **38** | 2 | 2.8–4 s |

Box counts are a rough proxy. Only v5 server picked up the fine print on the syringe
packets and the multilingual list. All of them find headline text and ignore panel seams
and box outlines, which fixes the over-conservative part. **None of them finds the
white-on-yellow paragraph or most 4–6 px print and pictograms**; at 0.65 mm/px that is near
the resolution limit for any detector. Lower thresholds and more upscaling helped little
with v4 mobile.

Consequences:
- Keep the old edge check on alongside the text mask (`use_edges=True`, the default) until
  testing shows the text mask alone is safe enough. Text-dense items can then have no
  feasible spot. Example: the Visistat box in `20261027_175224` returns "no feasible
  placement" with the text mask.
- Speed: v5 server is too slow on CPU for the robot (~3–4 s per item). Use
  `onnxruntime-gpu` with `"EngineConfig.onnxruntime.use_cuda": True`, or fall back to v5
  mobile and accept missing fine print.
- **Not tested yet:** PP-OCRv6 detectors (in `rapidocr` 3.10 as `OCRVersion.PPOCRV6`,
  `ModelType.TINY/SMALL/MEDIUM`; they need `Det.lang_type` set to a language such as
  `"en"`, not `"multi"`) and CRAFT (character-level detector, via the `easyocr` package).
  Both are worth comparing on fine print.

## 6. Results on the captures

`label_placement.py` with the edge check and depth term, no text detector (edges+depth),
compared with the prototype's baseline. "Unevenness" is measured under the chosen label, one
number per repeat capture.

| Item (capture, bbox origin) | Current | Revised |
| --- | --- | --- |
| White kit pouch (174830, 499,433) | 2.8 mm | 1.2–1.7 mm |
| IV bag (174904/174918, 1084,415) | 1.9 / 3.2 mm | 1.3 / 2.0 mm |
| Syringe packet stack (175016, 1114,374) | 2.4 mm | 1.2–1.5 mm |
| White folded packet (175224/175243, 1245,496) | 8.5 / 2.7 mm | 2.7 / 1.2 mm, status `uneven` |
| Clear packet with flap (175452, 1213,495) | no placement | 0.7–1.9 mm |
| Clear gauze pack (175016, 500,417) | 5.1 mm | 3.4–5.0 mm, `flagged` |
| IV set in clear pouch (174810, 382,528) | 13.4 mm | ~10–14 mm, `flagged` (no flat spot exists) |
| Boxes, catheter pouch, big bag | 0.3–1.5 mm | same spot (flat tier keeps the old preference) |

Ranges come from slightly different evaluation details between the prototype and this
reference file. Expected console output of
`python label_placement.py <all 11 capture folders>` (edges + depth, no text):

```
20261027_174708 item0@(1189,499) flat    unevenness 0.3 mm ...
20261027_174708 item1@(460,564)  no feasible placement
20261027_174810 item2@(382,528)  flagged unevenness 9.6 mm ...
20261027_174830 item1@(499,433)  flat    unevenness 1.7 mm ...
20261027_174904 item1@(1084,415) flat    unevenness 1.9 mm ...
20261027_174918 item0@(1085,416) uneven  unevenness 2.8 mm ...
20261027_175016 item0@(1114,374) flat    unevenness 1.5 mm ...
20261027_175016 item1@(500,417)  flagged unevenness 5.0 mm ...
20261027_175224 item2@(1245,496) uneven  unevenness 3.5 mm ...
20261027_175243 item1@(1245,497) uneven  unevenness 2.9 mm ...
20261027_175452 item0@(388,311)  flat    unevenness 1.9 mm ...
20261027_175452 item1@(1213,495) flat    unevenness 1.9 mm ...
```

(Unevenness printed here includes the 8 px margin, so it reads higher than the table above.)
Each capture also gets an overlay image in `--out`: green = flat, orange = uneven, red = flagged.

## 7. How to run

```bash
python -m venv venv && . venv/bin/activate
pip install -r requirements.txt
python label_placement.py /path/to/captures/2026*                     # edges + depth
python label_placement.py /path/to/captures/2026* --text ppocr        # + PP-OCRv5 text mask
python label_placement.py /path/to/captures/2026* --text ppocr --no-edges
python label_placement.py /path/to/captures/2026* --no-depth          # grid search, no depth term
```

Outputs go to `placement_out/`: one overlay JPEG per capture and `results.json` (per item:
centre, angle, unevenness, tilt, status, box corners in full-image pixels).

The baseline: `current_opt_label_location.py` exposes `opt_label_loc(im, im_mask,
label_zone_num)`. It works on the left (zone 1) or right (zone 2) half of the image it is
given, and returns `(x, y, angle + 90)`. For a per-item comparison, call
`get_candidate_locs(crop, mask)` and `is_safe_oriented_placement(edges, mask, x, y, angle)`
directly on an item crop, as `place_label` does for the revised version.

## 8. Suggested work at home

1. **Build an evaluation set.** On 30–50 item crops, draw polygons around *all* text,
   barcodes and symbols, including fine print, and mark visible folds/steps. Metrics per method:
   - % of chosen labels that overlap text (safety);
   - % of chosen labels that cross a marked fold/step;
   - % of items with any placement (conservativeness);
   - stability: on the three repeat pairs, distance between the chosen centres.
2. **Compare text detectors** on that set: PP-OCRv5 server/mobile, PP-OCRv6 tiny/small/medium,
   CRAFT. Measure recall on fine print and time on GPU.
3. **Decide whether the old edge check can go.** Compare `--text ppocr` against
   `--text ppocr --no-edges` on the text-overlap metric.
4. **Tune `PV_FLAT` / `PV_MAX`** on the fold annotations. Keep `PV_FLAT` above the repeat
   noise (≥ ~2 mm).
5. **Speed up if needed:** vectorise the depth check. Every footprint fully on the eroded
   mask shares one design matrix, so one pseudo-inverse fits them all at once.

## 9. Out of scope / known gaps

- **Glare:** specular highlights can hide text from both the camera and the detector.
  Lower exposure helps, and a polarizer would help more. A glare map (bright, low saturation)
  treated as "unknown" is a possible extra constraint.
- **Folds pressed flat** are invisible to depth; only the colour edge/text check can see them.
- **Clear film over a gap:** stereo matches the texture seen through the film, so the
  measured surface can sit below loose film. Not verified.
- The brightness segmentation here is a test stand-in. On the robot the mask comes from
  GroundingDINO + SAM, and the depth crop must be sliced exactly like the colour crop.
