# Standalone labelling-arm test with Fast-FoundationStereo depth — status notes

Reference notes for the lab report. Self-contained: no knowledge of the codebase is
assumed. Built 2026-10-22, tested on the robot 2026-10-27; all numbers are from those
sessions.

## 1. Context and goal

The system is a two-arm cell: a pick-and-place arm (Franka, suction cup) moves items from
an input bin into one of two labelling zones, and a second Franka ("labelling arm") takes a
self-adhesive label from a dispenser with its own suction cup and presses it onto the item.
A ZED-M stereo camera looks down on the labelling zones from about 0.98 m; a label-placement
routine finds a flat, clutter-free spot on the item (open-vocabulary detector + segmentation
model, then a distance-transform search over the mask) and returns a pixel and a label
angle.

**The limitation.** The placement routine gives the label's pixel position but not its
height. The ZED's own depth was found too noisy, so the demo instead reuses the height at
which the first arm grasped the item in the input bin, and converts the pixel to XY with a
hard-coded camera height. That means the labelling arm cannot operate without the first
arm, and it assumes the item sits at the same height in the zone as it did in the bin.

**The lead.** A teammate had evaluated FoundationStereo and Fast-FoundationStereo on the
ZED-M's rectified stereo pairs and found the depth clearly better than the ZED's built-in
neural modes (in particular on a white/green box the ZED leaves a hole in).
Fast-FoundationStereo (17.7 M parameters) ran at 36–142 ms per frame on the lab's RTX 4090,
versus ~1.2 s for FoundationStereo. That work lived outside the repository.

**Goal.** A self-contained test in which the labelling arm fetches a label and places it on
an item at a height *measured* by Fast-FoundationStereo from the ZED, runnable with or
without vacuum, without modifying the demo.

## 2. What was built

A separate test directory in the repository, sharing the demo's ROS master and robot
bring-up but with its own containers, topics and logs. Pipeline:

1. **ZED driver** (as in the demo): rectified left/right images and their camera
   calibration at 1920 × 1080.
2. **Depth node** (its own container; Fast-FoundationStereo plus a pure-Python ROS client):
   on request, runs the network on the newest stereo pair and publishes metric depth aso it may be worth adopting in the demo node too, but I'd test it on more of your inventory first — one word can swing GroundingDINO a lot either way.

￼
￼
You: Where is these parameters defined?
4:48 PM
￼
t
   full resolution together with the left frame and calibration it used, all stamped
   identically so a consumer can pair them.
3. **Labelling perception node** (the demo's perception image): runs the demo's own
   detection, segmentation and label-placement code — imported, not copied — on the left
   frame the depth was computed from; takes the median depth in a 7 × 7 pixel window at
   the chosen pixel; back-projects it with the camera calibration and transforms it into
   the robot frame with the existing camera-to-robot calibration; publishes a full pose
   (position with measured z, orientation from the label angle).
4. **Arm script** (the labelling robot's controller): the demo's two labelling states
   without the other arm — dispenser (approach, contact, cup on, retreat, safe pose) then
   placement (approach, contact, cup off, retreat, safe pose) — using the demo's inverse
   kinematics, collision model and workspace bounds. Options: vacuum on/off, confirm each
   placement or run automatically, choose the zone, skip the dispenser, run perception and
   IK only with no robot motion.
5. **One-terminal launcher/teardown** that brings the stack up, reuses what is already
   running, merges all logs, and saves a diagnostic image per perception call.

Design points:

- **Calibration comes from the camera driver, not constants.** Focal length from the left
  camera info, stereo baseline from the right one (its projection matrix encodes
  −fx·baseline). Live values: fx = fy = 1509.6 px, baseline 0.0632 m — these matched the
  constants hard-coded in the demo, so no calibration mismatch exists.
- **Half-resolution inference.** The authors recommend image widths under 1000 px;
  1920 × 1080 is halved to 960 × 540 and the depth is resized back with nearest-neighbour
  sampling so each colour pixel indexes the depth image directly.
- **Validation against the teammate's offline script**, same checkpoint, full resolution:
  median, 99th-percentile and maximum difference all 0.0000 m on the saved ZED pair. At half
  resolution the median difference from the full-resolution result was 3 mm, with the 99th
  percentile (0.93 m) at object edges, as expected from nearest-neighbour upsampling at
  depth discontinuities. Timing: 27 ms at half resolution, 90 ms at full (saved 1280 × 720
  pair); live at 1920 × 1080 halved, 137–185 ms per call including capture and transport,
  ~95 % of pixels valid.
- **Diagnostic image per call:** the colour frame with segmentation masks, detector boxes
  and scores, the chosen point, the label footprint actually tested by the placement
  routine (a rotated 120 × 40 px rectangle) and the measured depth; beside it the depth map
  with the same marks. Raw colour and depth are saved for offline reruns.

## 3. Issues found and how each was resolved

### 3.1 ZED camera unreachable (hardware)

The driver reported "camera not detected"; the USB bus showed only the ZED-M's HID
interface, not its video interface. This is a known ZED-M failure mode in which the USB3
video interface de-enumerates; only a physical unplug/replug recovers it. No software fix;
the launcher now checks for image data and points at this when there is none.

### 3.2 Flat item refused ("label z = 0.004 m outside the allowed band")

*Symptom.* In the first live run a flat pouch in zone 1 was refused on every call while a
5 cm box in zone 2 came out at z = 0.056 m.
*Cause.* A sanity bound I had set, not the measurement. I had assumed from a constant in
the demo that items sit on a raised zone surface ~5.5 cm up and refused anything below
4 cm. In fact items lie on the table, which the camera sees at z ≈ 0.004 m (0.976 m range
under a camera 0.98 m up), so a pouch a few millimetres thick legitimately measures near
zero. The depth was correct for both items; the box is clearly resolved in the depth map,
the pouch only faintly.
*Resolution.* Lower bound moved to −1 cm; automatic zone selection now moves on to the
other zone when one is refused (it had stopped at the first refusal even though zone 2 was
fine).

### 3.3 IK reported collisions for the flat item

*Symptom.* Every IK attempt for the pouch was "in collision" or converged poorly; the box
solved fine.
*Cause.* Collision-model margins. The scene model represents the table as boxes whose top is
at z = 0, and the robot's cup collision block extends about 1 cm past the physical cup tip.
Sweeping the final pose offline: colliding for z ≤ 7.5 mm, clear from 10 mm, and the only
colliding pair was (cup block, table). The demo never met this because its label heights
came from bin grasps, always ≥ 9 cm.
*Resolution.* For the final (contact) pose only, the cup block is allowed to meet
table-level obstacles — identified automatically as boxes whose top is at z ≤ 1 mm. Every
other block/obstacle pair, and the entire approach pose, is still checked. The pouch then
solved on the first attempt.

### 3.4 Arm swung round backwards (operator pressed the e-stop)

*Symptom.* After eight failed IK attempts the ninth was accepted and the arm swung its base
around through the dispenser side of the cell on its way to the approach pose.
*Cause.* The IK retry logic (inherited unchanged from the demo) re-seeds the solver
**uniformly over the whole joint range** after a failure. Attempt 9 converged with the base
joint at −2.90 rad (−166°, its limit) and the upper-arm joint at −2.88 rad: the
configuration that reaches the zone from behind. It was collision-free in the model and
within joint limits, so it was executed. The subsequent move toward the dispenser was the
next cycle starting after the e-stopped placement had "failed".
*Resolution.* (a) Retries re-seed within ±0.6 rad of the nominal seed — 50 of 50 trials
converged and none produced a wrapped solution. (b) Any solution with the base or upper-arm
joint beyond 2.0 rad is refused regardless of seed. (c) A placement the robot could not
complete (user stop, controller fault) now ends the run rather than starting the next
cycle, and the dispenser is visited only when the cup holds no label.
*Note.* The demo's own IK routine has the same exposure.

### 3.5 Robot "not ready" after the e-stop

*Symptom.* After releasing the user stop and unlocking the brakes, the pre-flight check
failed ("controller status not ready") although the robot-state topic was still being
published.
*Cause.* Two parts. (1) The user stop made the real-time controller process on the control
PC abort ("performing automatic error recovery … terminate called … core dumped"); the
terminal session and the ROS bridge beside it kept running, so the stack looked alive.
(2) Restarting the controller hung while acquiring a lock: the crash had left its boost
shared-memory segments with a locked mutex — the same wedge already documented for the
other arm.
*Resolution.* Stop the stack, clear the shared-memory segments only once no controller
process remains, start again. The launcher now detects "session present, controller
process absent" before bring-up and performs this itself.

### 3.6 Tool turned the long way / near-360° wrist turns

*Symptom.* With the label at ~91° the wrist turned ~120° one way where ~30° the other way
would do; historically, items oriented near 0° sometimes produced an almost full wrist
turn.
*Cause*, traced end to end:
1. The placement routine's orientation estimate takes a minimum-area-rectangle angle modulo
   180°. For a nearly axis-aligned item the raw angle jitters about 0, and modulo 180 maps
   −0.5° to 179.5°.
2. The pose builder turns the angle into a full quaternion, discarding the fact that a label
   is the same rectangle after a 180° turn.
3. The IK solver must honour that exact yaw, so the wrist joint is fully determined and
   nothing considers where the arm currently is.
   Measured for the zone-1 pose: angle 91.3° → wrist joint −81° (123° from the safe pose);
   the equivalent 271.3° → +74° (32° away). Angle −0.5° → +125°; the equivalent 179.5° →
   −16.5° with the wrist also re-solved — the "near-360" swing.
*Resolution.* (A) The arm solves IK for the published orientation and for it turned 180°
about the tool axis and takes the one with the smaller total joint travel from the current
joints (live: +30° vs −148° → the former chosen). (B) Perception normalises the angle into
(−90°, 90°] so the published pose is stable frame to frame.
*Assumption to state:* labels are treated as 180°-symmetric (centred on the cup, text
direction irrelevant). If text direction matters, only (B) applies.

### 3.7 Flipped packet not detected

*Symptom.* The pouch turned over (pale, translucent side up) gave "no candidate placement":
no detection box, hence no mask.
*Cause.* The open-vocabulary detector, not segmentation or depth. With the demo's prompt
("bright box on black background") the packet scored 0.19, below the 0.25 box threshold, so
it was dropped before segmentation ran; the depth map showed it faintly. Scores on the saved
frame (box / packet): demo prompt 0.79 / 0.19; "box" 0.60 / 0.27; "package" 0.84 / 0.34;
"packet" 0.85 / 0.44; "pouch" 0.69 / 0.33.
*Resolution.* Prompt and threshold made adjustable for the test, with the demo's values as
defaults. With "packet" both items were detected (0.80 / 0.86); the pouch got a placement in
its blank area at z = 0.004 m. The prompt should be evaluated on the whole inventory before
changing the demo.

## 4. Numbers for the report

| Quantity | Value |
| --- | --- |
| ZED-M resolution / calibration | 1920 × 1080; fx = fy = 1509.6 px, cx = 964.3, cy = 559.3; baseline 0.0632 m |
| Fast-FoundationStereo model | `c-fast-foundationstereo` release checkpoint, 17.7 M parameters |
| Inference time, RTX 4090 | 27 ms at 640 × 360; 90 ms at 1280 × 720; live 1920 × 1080 halved: 137–185 ms per call end to end |
| Agreement with the reference offline script | 0.0000 m (full resolution, same checkpoint) |
| Valid depth pixels (live) | ~95 % |
| Table height in robot frame (from camera) | z ≈ 0.004 m at 0.976 m range; camera 0.98 m above the table |
| Measured item heights | pouch 0.004–0.005 m; box 0.056–0.057 m, repeatable across calls |
| Collision-model margin at the table | contact pose flagged for z ≤ 7.5 mm, clear from 10 mm |
| Wrist-joint travel, published vs 180°-equivalent orientation | 123° vs 32° |
| Detector score on the flipped packet | 0.19 (demo prompt) vs 0.44 ("packet"); threshold 0.25 |

## 5. Status and open points

Working end to end on the real cell, without vacuum: dispenser → measured label pose →
placement, for a 5 cm box and a few-millimetre pouch, with confirmation before each
placement. Not yet done: placements with vacuum, and a quantitative comparison of
Fast-FoundationStereo against the ZED's own depth at the label point (the switch for it is
in place).

Suggested follow-ups for the demo: adopt the two IK changes (near-nominal re-seeding with
from-behind rejection; 180°-symmetric orientation choice); decide whether label text
direction matters; add hysteresis to the placement routine, which sometimes alternates
between the item's major and minor axis between calls (edge-ratio ties) and rotates the
label 90°; evaluate the detection prompt across the inventory; address the ZED-M USB wedge in
hardware (separate USB controller or cable).
