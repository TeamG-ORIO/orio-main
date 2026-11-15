#!/usr/bin/env python3
"""Offline check of arm 2's label IK over every label pose perception can hand it. No arm
moves. Needs ikpy and the collision model, so run it in the orio_docker image:

    docker exec -it orio_docker_container bash -c "source /home/ros_ws/devel/setup.bash && \\
        cd /home/ros_ws/src/devel_packages/orio && python3 label_ik_grid_test.py"

Poses: a grid of label points over both label zones (the ZED crop's halves at table height,
in arm 2's base frame, inside compute_label_joints' LABEL_POSE_BOUNDS), at two item heights;
at each, four label angles, pressed straight down and tilted 25 deg (perception's cap)
toward four directions. The rotation is built as perception builds it (the camera's axis
turned by the label angle, then turned toward the surface normal).

Each pose goes through RobotHardware._label_ik (the demo's IK) from LABELLING_SAFE, which
refuses any solution whose moves hit the arm-2 obstacles (label_path_collisions: now ->
pre-label -> label -> pre-label -> LABELLING_SAFE, every LABEL_PATH_STEP rad). For each
solution the moves are measured: joint motion to pre-label, the press's joint change and
how far the cup strays from the straight line, J3/J5. The same poses also go through the
label IK this replaced (ikpy from a fixed seed, pre-label straight up, only the end poses
collision-checked), with its whole path checked the same way, for comparison.

Writes logging/label_ik_grid/<time>/results.json and summary.txt.
"""

import argparse
import json
import os
import random
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", "..", "logging", "label_ik_grid"))
TF_YAML = os.path.normpath(os.path.join(HERE, "..", "manipulation", "config", "zed_to_label_tf.yaml"))

# The ZED (perception's LBL_* and label_zone_seg.LBL_CROP_XYXY): zone 1 the left half of the crop.
FX, FY, CX, CY = 1509.65, 1509.65, 964.33, 559.28
CROP = (160, 180, 1860, 1050)
STEP_M = 0.06                         # grid spacing over the whole zone
INSET_M = 0.04                        # kept off the zone's edges (labels lie on items, inside)
# Where labels have landed (the stack's placement on the 195 recorded demo zones, 20261107-15):
# x 0.42..0.61, zone 1 y 0.06..0.36, zone 2 y -0.34..-0.10; this box around it is gridded
# finer, and reported on its own.
REAL = {1: (0.38, 0.65, 0.02, 0.40), 2: (0.38, 0.65, -0.38, -0.06)}
REAL_STEP_M = 0.03
HEIGHTS = (0.005, 0.06)               # label height above the table (flat pouch .. tall box)
LABEL_ANGLES_DEG = (0, 45, 90, 135)   # 180-deg periodic: the IK tries the label turned 180 too
TILTS = [(0, 0)] + [(25, d) for d in (0, 90, 180, 270)]   # (deg, direction the surface leans)
# LABELLING_SAFE's joints (pose_to_joints('LABELLING_SAFE') in the 20261115_143734 run):
# where APPLY starts and ends.
SAFE = np.array([-0.065, -1.2718, 0.0356, -2.707, 0.0343, 1.4354, 0.7355])


def load_tf():
    import yaml
    from scipy.spatial.transform import Rotation as R
    with open(TF_YAML) as f:
        p = yaml.safe_load(f)["pose"]
    q, t = p["rotation"], p["translation"]
    T = np.eye(4)
    T[:3, :3] = R.from_quat([q["x"], q["y"], q["z"], q["w"]]).as_matrix()
    T[:3, 3] = [t["x"], t["y"], t["z"]]
    return T


def zone_rect(T, zone, z):
    """(x0, x1, y0, y1) of the zone at height z in arm 2's base frame."""
    x0, y0, x2, y2 = CROP
    mid = x0 + (x2 - x0) // 2
    us = (x0, mid) if zone == 1 else (mid, x2)
    pts = []
    for u in us:
        for v in (y0, y2):
            d = T[:3, :3] @ np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
            s = (z - T[2, 3]) / d[2]
            pts.append(T[:3, 3] + s * d)
    pts = np.array(pts)
    return pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max()


def press_rotation(T, label_deg, tilt_deg, tilt_dir_deg):
    """Perception's _press_orientation for a surface tilted tilt_deg toward tilt_dir_deg."""
    from scipy.spatial.transform import Rotation as R
    rot0 = T[:3, :3] @ R.from_euler("z", np.radians(label_deg)).as_matrix()
    if tilt_deg == 0:
        return rot0
    t, d = np.radians(tilt_deg), np.radians(tilt_dir_deg)
    n = np.array([np.sin(t) * np.cos(d), np.sin(t) * np.sin(d), np.cos(t)])
    z0, z1 = rot0[:, 2], -n
    axis = np.cross(z0, z1)
    ang = np.arctan2(np.linalg.norm(axis), z0 @ z1)
    return R.from_rotvec(axis / np.linalg.norm(axis) * ang).as_matrix() @ rot0


def grid_cases(bounds):
    T = load_tf()
    cases = []
    for zone in (1, 2):
        for z in HEIGHTS:
            x0, x1, y0, y1 = zone_rect(T, zone, z)
            x0, x1 = max(x0 + INSET_M, bounds["x_min"]), min(x1 - INSET_M, bounds["x_max"])
            y0, y1 = max(y0 + INSET_M, bounds["y_min"]), min(y1 - INSET_M, bounds["y_max"])
            rx0, rx1, ry0, ry1 = REAL[zone]
            pts = [(x, y, False) for x in np.arange(x0, x1 + 1e-9, STEP_M) for y in np.arange(y0, y1 + 1e-9, STEP_M)
                   if not (rx0 <= x <= rx1 and ry0 <= y <= ry1)]
            pts += [(x, y, True) for x in np.arange(rx0, rx1 + 1e-9, REAL_STEP_M) for y in np.arange(ry0, ry1 + 1e-9, REAL_STEP_M)]
            for x, y, real in pts:
                for a in LABEL_ANGLES_DEG:
                    for tilt, tdir in TILTS:
                        cases.append(dict(zone=zone, x=round(float(x), 3), y=round(float(y), 3), z=z, real=real,
                                          label_deg=a, tilt=tilt, tilt_dir=tdir,
                                          rot=press_rotation(T, a, tilt, tdir).tolist()))
    for i, c in enumerate(cases):
        c["id"] = i
    return cases


# ── per-worker robot model ────────────────────────────────────────────────────
fs = hw = None


def init_worker():
    global fs, hw
    import logging
    import warnings
    warnings.filterwarnings("ignore")
    sys.argv = [sys.argv[0]]          # full_sequence parses the command line on import
    import full_sequence
    import CollisionChecker
    import ikpy.chain
    fs = full_sequence
    logging.disable(logging.WARNING)
    fs.events.emit = lambda *a, **k: None
    hw = fs.RobotHardware.__new__(fs.RobotHardware)
    hw.ik_chain = ikpy.chain.Chain.from_urdf_file(fs.URDF_FILE, base_elements=["panda_link0"])
    hw.chain_length = len(hw.ik_chain.links)
    hw.ik_chain.active_links_mask = [False] + [True] * 7 + [False] * (hw.chain_length - 8)
    hw.la_collision_checker = CollisionChecker.CollisionChecker(arm_number=2)


def old_label_ik(task_pos, ori):
    """The label IK _label_ik replaced (full_sequence before this change): ikpy from a fixed
    seed (re-seeded with 0.6 rad of jitter), pre-label APPROACH_DISTANCE straight up, only
    the end poses collision-checked; of the two label orientations the less joint motion."""
    best = None
    for o in (ori, ori @ fs.LABEL_TOOL_FLIP):
        pre_pos = task_pos + np.array([0.0, 0.0, fs.APPROACH_DISTANCE])
        for attempt in range(10):
            seed = [0.0] * hw.chain_length
            seed[4] = -1.5
            if attempt:
                for i in range(1, 8):
                    lo, hi = hw.ik_chain.links[i].bounds
                    seed[i] = float(np.clip(seed[i] + random.uniform(-0.6, 0.6), lo, hi))
            pre = hw.ik_chain.inverse_kinematics(target_position=list(pre_pos), target_orientation=o,
                                                 orientation_mode="all", initial_position=seed)
            final = hw.ik_chain.inverse_kinematics(target_position=list(task_pos), target_orientation=o,
                                                   orientation_mode="all", initial_position=pre)
            if max(abs(pre[1]), abs(pre[3]), abs(final[1]), abs(final[3])) > fs.LABEL_MAX_BASE_TURN:
                continue
            if hw._label_collisions(pre[1:8]) or hw._label_collisions(final[1:8], allow_tool_on_table=True):
                continue
            if (np.linalg.norm(hw._fk7(pre[1:8])[:3, 3] - pre_pos) > 0.02
                    or np.linalg.norm(hw._fk7(final[1:8])[:3, 3] - task_pos) > 0.02):
                continue
            travel = float(np.abs(np.asarray(pre[1:8]) - SAFE).sum())
            if best is None or travel < best[0]:
                best = (travel, np.asarray(pre[1:8]), np.asarray(final[1:8]))
            break
    return None if best is None else (best[1], best[2])


def measure(pre, final):
    a, b = hw._fk7(pre)[:3, 3], hw._fk7(final)[:3, 3]
    ab = b - a
    off = 0.0
    for s in np.linspace(0.0, 1.0, 21):
        p = hw._fk7(pre + s * (final - pre))[:3, 3]
        t = np.clip((p - a) @ ab / max(ab @ ab, 1e-12), 0, 1)
        off = max(off, float(np.linalg.norm(p - (a + t * ab))))
    press_dir = ab / np.linalg.norm(ab)
    tool_z = hw._fk7(final)[:3, 2]
    hit = hw.label_path_collisions(SAFE, pre, final, SAFE)
    return {"travel_sum": round(float(np.abs(pre - SAFE).sum()), 3),
            "travel_max": round(float(np.abs(pre - SAFE).max()), 3),
            "press_change": round(float(np.abs(final - pre)[[0, 2, 4, 6]].max()), 3),
            "press_off_line_mm": round(1000 * off, 1),
            "press_vs_tool_deg": round(float(np.degrees(np.arccos(np.clip(press_dir @ tool_z, -1, 1)))), 1),
            "j3": round(float(final[2]), 3), "j5": round(float(final[4]), 3),
            "path_collision": None if hit is None else "%s at %.0f%%" % (hit[0], 100 * hit[1])}


def run_case(c):
    random.seed(c["id"])
    task_pos = np.array([c["x"], c["y"], c["z"]])
    ori = np.array(c["rot"])
    out = {k: c[k] for k in ("id", "zone", "x", "y", "z", "real", "label_deg", "tilt", "tilt_dir")}
    t0 = time.time()
    try:
        pre, final, tilted = hw.label_ik_or_straight(task_pos, ori, c["zone"], q_now=SAFE, q_safe=SAFE)
        out["new"] = measure(pre, final)
        out["new"]["tilted"] = bool(tilted)
    except RuntimeError as exc:
        out["new"] = {"fail": str(exc).split("(", 1)[-1].rstrip(")")}
    out["new_s"] = round(time.time() - t0, 2)
    old = old_label_ik(task_pos, ori)
    out["old"] = measure(*old) if old else {"fail": "no IK"}
    return out


def summarise(results, out_dir):
    L = []
    for name, sel in (("REAL LABEL AREA (x 0.38..0.65, 3 cm grid)", [r for r in results if r["real"]]),
                      ("REST OF THE ZONES (6 cm grid)", [r for r in results if not r["real"]])):
        L.append("== %s: %d poses" % (name, len(sel)))
        L += summarise_part(sel)
    text = "\n".join(L)
    print(text)
    with open(os.path.join(out_dir, "summary.txt"), "w") as f:
        f.write(text + "\n")


def summarise_part(results):
    L = []
    n = len(results)
    if not n:
        return L
    for key in ("new", "old"):
        ok = [r[key] for r in results if "fail" not in r[key]]
        coll = [m for m in ok if m["path_collision"]]
        L.append("%s IK: solved %d/%d, path collisions %d%s" % (key, len(ok), n, len(coll),
                 (", tilted presses kept tilted %d/%d (rest straight down)" % (
                     sum(o.get("tilted", False) for o in ok), sum(1 for r in results if r["tilt"] and "fail" not in r[key])))
                 if key == "new" else ""))
        if ok:
            for m, unit in (("travel_sum", "rad"), ("travel_max", "rad"), ("press_change", "rad"),
                            ("press_off_line_mm", "mm"), ("press_vs_tool_deg", "deg")):
                v = np.array([o[m] for o in ok])
                L.append("    %-18s median %6.2f  90th %6.2f  max %6.2f %s" % (m, np.median(v), np.percentile(v, 90), v.max(), unit))
            j35 = np.array([np.hypot(o["j3"], o["j5"]) for o in ok])
            L.append("    %-18s median %6.2f  90th %6.2f  max %6.2f rad" % ("|J3, J5|", np.median(j35), np.percentile(j35, 90), j35.max()))
    fails = {}
    for r in results:
        if "fail" in r["new"]:
            fails[r["new"]["fail"]] = fails.get(r["new"]["fail"], 0) + 1
    L.append("new IK failures by reason (starts tried: reasons):")
    for k, v in sorted(fails.items(), key=lambda kv: -kv[1])[:15]:
        L.append("    %4d  %s" % (v, k))
    # where the new IK fails
    L.append("new IK failures by zone / height / tilt:")
    for zone in (1, 2):
        for z in HEIGHTS:
            for tilt in (0, 25):
                sel = [r for r in results if r["zone"] == zone and r["z"] == z and r["tilt"] == tilt]
                bad = [r for r in sel if "fail" in r["new"]]
                L.append("    zone %d, z %.3f, tilt %2d: %3d/%3d failed%s" % (
                    zone, z, tilt, len(bad), len(sel),
                    ("  e.g. " + ", ".join("(%.2f, %.2f) %d deg" % (r["x"], r["y"], r["label_deg"]) for r in bad[:4])) if bad else ""))
    L.append("time per pose (new IK): median %.1f s, 90th %.1f s, max %.1f s" % (
        np.median([r["new_s"] for r in results]), np.percentile([r["new_s"] for r in results], 90),
        max(r["new_s"] for r in results)))
    return L


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--limit", type=int, help="only the first N poses (a quick try)")
    ap.add_argument("--every", type=int, default=1, help="only every Nth pose")
    ap.add_argument("--out", help="output folder (default logging/label_ik_grid/<time>)")
    args = ap.parse_args()
    sys.argv = [sys.argv[0]]
    import full_sequence
    cases = grid_cases(full_sequence.LABEL_POSE_BOUNDS)[::args.every]
    if args.limit:
        cases = cases[:args.limit]
    out_dir = args.out or os.path.join(OUT_ROOT, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print("%d poses, %d jobs -> %s" % (len(cases), args.jobs, out_dir), flush=True)
    t0 = time.time()
    with Pool(args.jobs, initializer=init_worker) as pool:
        results = []
        for k, r in enumerate(pool.imap_unordered(run_case, cases, chunksize=4)):
            results.append(r)
            if (k + 1) % 200 == 0:
                print("  %d/%d (%.0f s)" % (k + 1, len(cases), time.time() - t0), flush=True)
    results.sort(key=lambda r: r["id"])
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(results, f)
    summarise(results, out_dir)


if __name__ == "__main__":
    main()
