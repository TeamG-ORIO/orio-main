# Dex-Net: item mask and plane-fit approach axis

Written for an engineer or agent picking this work up cold. It is self-contained: it
restates the project context that the work depends on, the two failure cases that
motivated it, what was built, where it lives, how it is configured, how to turn it off,
and what is still open. The parent handbook for the Dex-Net integration as a whole is
`docs/DEXNET.md`; `COMMANDS.md` has the launcher flags. Dates are from `git log`.
Work done 2026-10-05 to 2026-10-06 on branch `feature/dexnet`, landed in commit
`2ef1d12` ("feat: add item mask, plane fit and pick offsets to dexnet").

---

## 1. Project context

ORIO is a bin-picking system: a Franka Panda ("robot 1", control PC `iam-doc`) with a
pneumatic suction cup picks medical items (boxes, pouches, blister packs) off a table
covered in black cloth, under a fixed ASUS Xtion RGB-D camera looking straight down from
about 0.9 to 1.0 m. The host is Ubuntu 22.04 with ROS 2 only; every ROS 1 process runs in
a Docker container. Nothing here involves robot 2 or the labelling path.

The grasp planner under test is Dex-Net 4.0's fully convolutional suction network
(FC-GQCNN-4.0-SUCTION, Berkeley gqcnn, vendored as the submodule
`src/devel_packages/gqcnn`). The pipeline, all started by `run_dexnet_pnp_single.sh`:

```
Xtion (openni2, registered depth)
  -> perception node   orio_perception/scripts/perception_control_combined_pass_through.py
                       container orio_perception; /compute_grasps (Trigger)
                       crop 640x480 -> rows 40..405, cols 115..515; build segmask;
                       call the planner; camera pose -> world pose; tilt gate
  -> Dex-Net planner   orio_bringup/docker/dexnet_grasp_planner.py
                       container orio_dexnet (TensorFlow 1); service
                       /dexnet_grasp_planner/plan_grasp (custom_msgs/PlanDexnetGrasp)
                       rescale 0.5 -> inpaint -> FC-GQCNN -> argmax -> one grasp
  -> pick loop         orio/dexnet_pnp.py (container orio_docker_container)
                       IK, pre-pick, contact, vacuum, retract, drop
```

Facts this work relies on:

- **Dex-Net's inputs** are a depth image, camera intrinsics and a binary *segmask* of
  pixels that may be grasped. The network scores every pixel; `_mask_predictions` in
  gqcnn zeroes every score outside the segmask before the argmax. The segmask is the only
  lever on *where* a grasp may land.
- **Dex-Net's output** is a pixel, a depth, a q-value in 0..1 and an approach axis. The
  axis is the X column of the rotation gqcnn returns (`SuctionPoint2D.pose()`); the
  perception node turns it into a tool rotation whose Z is the approach direction.
- **Camera to world:** camera +x is world +x, camera +y is world −y, camera +z (depth) is
  world −z. Image up is world +Y, image right is world +X.
- **Depth:** `/camera/depth_registered/image_raw`, 32FC1 metres. The Xtion has no
  calibration file; the driver's generic intrinsics are used (fx = fy ≈ 570.3,
  cx = 319.5, cy = 239.5). Depth noise at 0.9 m is roughly 2 to 3 mm.
- **Gates, in order:** planner rejects q below `DEXNET_MIN_Q_VALUE` (0.30 by default);
  perception rejects tilt above `dexnet.max_tilt_deg` (45°); the pick loop rejects tilt
  above its own `--max-tilt-deg` and positions outside `PICK_BOUNDS`.
- **The classical path** (`grasp_backend: classical`, the full dual-arm demo) detects items
  with GroundingDINO + SAM and fits a RANSAC plane to each item's point cloud
  (`orio_perception/scripts/grasp_solver.py`). It is the source of both ideas reused here.

---

## 2. The two failure cases

Both were observed on hardware before 2026-10-05, on the single-arm Dex-Net test loop.

### 2.1 Approach angle wildly off while the 3D point is fine

Dex-Net's grasp *position* was usually sensible, but the approach axis it returned was
sometimes tilted far from the surface's real normal, enough to be rejected by the 45°
tilt gate or to make the cup land at an angle on a flat box.

**Root cause.** The approach axis is not a network output. The network only picks the
pixel; the axis is computed afterwards in
`gqcnn/grasping/policy/fc_policy.py` (`_get_actions`, ~line 444) by reading a per-pixel
normal of the depth image at that one pixel. That depth image is the half-resolution
(rescale 0.5), inpainted one, where a pixel is about 3 mm of surface at 0.9 m, and the
normal comes from roughly a 3x3 pixel neighbourhood, about 1 cm. One Xtion quantisation
step (2 to 3 mm) across that baseline is worth about 20° of tilt; inpainted holes and
item edges are worse. So the axis is a noisy finite-difference estimate from a handful
of pixels. (The exact neighbourhood size in `autolab_core.normal_cloud_im` was taken
from memory, not verified; the conclusion does not depend on it.)

**Evidence after the fix.** On a static scene, three consecutive frames gave Dex-Net
tilts of 7.7°, 1.3° and 1.3° at the same grasp point, while the plane fit read 7.5°,
7.6° and 7.8° (`logging/dexnet_compare/selftest_live`, 2026-10-05).

### 2.2 Grasping the table when only thin items are present

With a few thin items (pouches, flat packs) on the cloth, Dex-Net sometimes chose a
grasp on the bare table.

**Root cause.** The segmask was only a depth band (`bin_depth_min` 0.60 m to
`bin_depth_max` 1.20 m, `_build_bin_mask`). The table is inside that band, so every
table pixel was a permitted grasp, and a flat table is a near-ideal suction surface
that scores highly. Items a few millimetres thick are below the depth noise, so no depth
threshold can separate them from the table; colour is needed.

---

## 3. What was built

### 3.1 Plane-fit approach axis (fix for 2.1)

After the planner returns a grasp, the perception node fits a plane to the *raw*
full-resolution depth under the suction cup and uses that plane's normal as the
approach axis. The 3D grasp point and the q-value are left as Dex-Net returned them.

Region selection, which is the part that matters because there is no object mask:

1. **Cup-footprint disc** around the grasp pixel, radius `plane_fit.radius` (0.03 m,
   about 1.5x the 2 cm cup radius), converted to pixels with `fx * r / z` (about 19 px at
   0.9 m, roughly 1,100 pixels).
2. **Valid raw depth only:** zeros and NaN dropped, so inpainted values never enter.
3. **Depth gate:** keep pixels within 3 cm of the grasp depth. Removes the floor and
   anything across a height step.
4. **Connected component** containing the grasp pixel (seeded from the nearest valid pixel
   within 3 px, since the grasp pixel itself may be a depth hole). Removes a neighbouring
   object at similar height across a gap.

Then a RANSAC plane (threshold 4 mm, 100 iterations) refined by SVD on the inliers. The
fit is accepted only if there are at least 100 points, the inlier fraction is at least
`plane_fit.min_inlier_frac` (0.7), and Dex-Net's 3D grasp point lies within 8 mm of the
plane (so the fitted surface is the one being grasped, not a dominant neighbour).

Decision, `plane_fit.mode`:

| mode | behaviour |
| --- | --- |
| `off` | Dex-Net's axis, untouched (the previous behaviour) |
| `shadow` | fit and log `Plane fit [shadow]: plane tilt .. DexNet tilt .. disagreement ..`, still execute Dex-Net's axis |
| `override` | execute the plane's axis (current default) |

On a bad fit (ridge, edge, strong curvature, too few points) in `override` mode the axis
falls back to `plane_fit.fallback`: `vertical` (straight down, default) or `dexnet`.
The override always applies on a good fit rather than only above a disagreement
threshold, because a threshold would keep the noisy estimate exactly where it cannot be
judged. The fit runs *before* the tilt gate, so a noisy axis can no longer reject a good
grasp. The plane normal is oriented toward the camera, mapped to world through the
camera TF, and flipped to point down like the original axis.

### 3.2 Item mask (fix for 2.2)

The segmask sent to Dex-Net becomes

```
(GroundingDINO+SAM item pixels, eroded)  OR  (pixels clearly above the table)
    AND  depth band  AND  valid depth
```

- **Items by colour:** `_detect_and_segment` (the classical path's GroundingDINO + SAM
  helper) with its own prompt, `item_mask.prompt`, currently
  `"medical item on black table"`. The classical path keeps its old prompt
  (`TEXT_PROMPT`, "bright box on black background"). Masks are OR'ed and eroded by
  `item_mask.erode_px` (3) so grasps stay off item edges.
- **Items by height:** the table plane is fitted by RANSAC on the in-band depth of every
  frame (`fit_table_plane`), accepted only if it is within 15° of world-vertical and
  fewer than 5% of points lie more than 1.5 cm below it (a box top fails this). The last
  valid plane is cached, so a pile that hides the table reuses the plane seen earlier.
  Pixels more than `item_mask.above_table_min` (0.015 m) above the plane count as items,
  after a 5x5 morphological opening to remove depth speckle.
- **Why both:** Dex-Net was chosen for cluttered piles, which is where the detector is
  weakest. Thin items come in through colour, tall or piled items through height, and
  the bare table is never a candidate. If no table plane has ever been found the mask is
  the detected items alone, with a warning.
- **Not done:** the idea of artificially pushing non-item depth down (to make thin items
  look like raised slabs to the network) was evaluated and deliberately not implemented.
  The mask alone fixes the table grasps; the depth edit would turn every SAM mask error
  into fake geometry and was judged not worth it until shown necessary.

Consequence: with `item_mask.enabled: true` the perception node loads SAM and
GroundingDINO on the Dex-Net backend too (they were previously skipped to save GPU
memory, commit `8f25c78`). Startup is slower and every plan includes a detector pass.
Neither has been timed. `_inference_lock` became an `RLock` because the Dex-Net branch
holds it while the detector takes it again.

### 3.3 Offline comparison tool (for the report)

`orio_perception/test/compare_dexnet_methods.py`, wrapped by
`orio_bringup/docker/run_dexnet_compare.sh`, plans the *same* frame with both methods
(old = depth band + Dex-Net axis, new = item mask + plane fit) by calling the node's own
`_plan_grasp_dexnet` with the two config keys flipped in memory, and renders a 2x3 figure
(depth inside the mask, segmask by source, chosen grasp with approach arrow) plus the six
panels as separate PNGs and a `summary.json`. No arm is involved. See section 5.

---

## 4. Where it lives

| What | File |
| --- | --- |
| Geometry (pure numpy/scipy, no ROS): `fit_plane_ransac`, `fit_local_plane`, `fit_table_plane`, `height_above_plane`, `above_plane_mask` | `src/devel_packages/orio_perception/scripts/dexnet_geometry.py` |
| Node integration: `_build_grasp_mask`, `_above_table_mask`, `_refine_approach`, called from `_plan_grasp_dexnet`; `_detect_and_segment(color, caption=None)`; `_dexnet_debug` dict of intermediates for the comparison tool | `src/devel_packages/orio_perception/scripts/perception_control_combined_pass_through.py` |
| Config: `dexnet.item_mask.*`, `dexnet.plane_fit.*` | `src/devel_packages/orio_perception/config/grasp.yaml` |
| Synthetic tests for the geometry (tilted planes with noise, step edge, hole, ridge, table vs box top) | `src/devel_packages/orio_perception/test/test_grasp_geometry.py` (`python3 test/test_grasp_geometry.py`, host, no ROS) |
| Comparison tool and wrapper | `src/devel_packages/orio_perception/test/compare_dexnet_methods.py`, `src/devel_packages/orio_bringup/docker/run_dexnet_compare.sh` |
| Frame capture used by the comparison | `src/devel_packages/orio_perception/test/capture_xtion.py`, `snap_xtion.sh` |

The plane fit was written in numpy rather than reusing open3d's `segment_plane` (which
`grasp_solver.py` uses) so the tests run on the host, which has no open3d.

### Config as of this writing

```yaml
dexnet:
  item_mask:
    enabled: true
    prompt: "medical item on black table"
    erode_px: 3
    above_table_min: 0.015     # m
  plane_fit:
    mode: override             # off | shadow | override
    radius: 0.03               # m
    min_inlier_frac: 0.7
    fallback: vertical         # vertical | dexnet
```

### Reverting to the previous behaviour

Set `item_mask.enabled: false` and `plane_fit.mode: off` in `grasp.yaml` and restart
perception (the yaml is read only at node start). Missing keys default to the old
behaviour as well. The only change not behind a key is the `RLock`, which is harmless.

### Log lines to look for

- `DexNet item mask: on, plane fit: override` at startup.
- `DexNet mask: N detections, item px=.., above-table px=.., final px=..` per plan.
- `Plane fit [override]: plane tilt .. deg, DexNet tilt .. deg, disagreement .. deg (N pts, inliers ..%)`
  or `Plane fit [override]: no usable plane (<reason>); DexNet tilt ..`.
- `DexNet grasp q=.. tilt=.. deg (axis: plane|dexnet|vertical) at world (..)`.
- `Table plane not found in this frame; reusing the last one.`

---

## 5. Running things

Normal pick loop, unchanged (new code is picked up through the bind mount and
`perception.launch`, which loads `grasp.yaml`):

```bash
bash run_dexnet_pnp_single.sh                    # dry run, Enter per pick
bash run_dexnet_pnp_single.sh --vacuum --auto
```

Do not pass `--straight-down` when evaluating the plane fit; it discards the tilt.

Comparison figures, no arm:

```bash
# live: starts roscore, the planner (with no q threshold) and the Xtion if needed;
# each Enter captures 3 frames, both methods plan all of them and keep their best score
bash src/devel_packages/orio_bringup/docker/run_dexnet_compare.sh --live --out logging/dexnet_compare/<name>
# replay saved scenes (folders with color.npy, depth.npy, K.npy, optionally color_1.npy ...)
bash src/devel_packages/orio_bringup/docker/run_dexnet_compare.sh <scene_dir> [...]
```

The wrapper refuses to run while the perception container is up, because the script
creates the perception node itself. Type a scene name at the prompt before Enter to name
the figures. Outputs: `<scene>_compare.png`, `<scene>_<old|new>_<depth|segmask|grasp>.png`,
`summary.json` (q, tilt, Dex-Net's own tilt, axis source, mask pixel counts, per-attempt
q, best frame), and the captured frames under `<out>/scenes/<name>/`. The depth panels
crop the floor strip at the bottom of the crop (the crop reaches past the table edge and
the floor at ~1.9 m wrecked the depth colour scale); the crop is display only.

---

## 6. What has been observed

- Geometry tests pass on synthetic data (noisy planes at 0°, 20°, 40° recovered within
  3°; step edge, hole, ridge, table-vs-box-top cases behave as intended).
- Live on hardware (operator report 2026-10-05): "working very well" with both features
  on. No success-rate numbers were recorded.
- Comparison tool on two saved captures from 2026-09-20 and on live frames of pouches:
  masks as intended (new mask covers only the items, old mask the whole table); same grasp
  point for both methods on those scenes, with the plane fit changing the tilt (13.4° ->
  6.5° on one scene). Dex-Net's own tilt flickered between 7.7° and 1.3° across frames of
  a static scene while the plane fit stayed within 0.3° (section 2.1).
- On the pouch scene all q-values were 0.07 to 0.085 for both methods, far below the
  0.30 threshold. Not investigated.

---

## 7. Open items and caveats

1. **Table-edge speckle.** The Xtion's depth bows toward the image edges; small patches of
   bare table near the crop edges have exceeded `above_table_min` and appeared as
   "above table" in the mask. Raise `above_table_min` if Dex-Net ever picks one.
2. **Detector recall and the prompt.** Items the detector misses and that are thinner
   than `above_table_min` are invisible to Dex-Net; the loop then thinks the bin is empty.
   The prompt and `BOX_THRESHOLD` (0.25) have only been tried on a few scenes.
3. **Latency and GPU memory** with SAM + GroundingDINO loaded next to the TF1 planner are
   unmeasured (the GPU had ~24 GB free when checked).
4. **Low q on thin pouches** (section 6) is unexplained; it is independent of the mask.
5. **`CUP_RADIUS_M = 0.02`** (plane-fit patch and the figure's ring) is the classical
   two-cup constant; confirm it matches the single cup on robot 1.
6. **Mask resolution.** Dex-Net samples the segmask on a coarse grid (about every 8 crop
   pixels if the network stride is 4; the stride was not checked), so very small items
   yield only a few candidate pixels.
7. **A third backend, `suction`** (the orio-grasping height-map network), was added on
   2026-10-06 in commit `88b5611`, after this work. It shares `_plan_grasp_dexnet` but
   plans on the full frame with an all-ones segmask, so the item mask does not apply to it.
   Whether `_refine_approach` (the plane fit) applies to that backend was not checked by
   the author of this document; read the current `_plan_grasp_dexnet` before assuming.
8. `docs/DEXNET.md` and `COMMANDS.md` were last updated in the same commit; keep them
   in step if the keys here change.
