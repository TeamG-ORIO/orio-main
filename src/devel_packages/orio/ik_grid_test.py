#!/usr/bin/env python3
"""Offline check of the input-pick IK over every kind of grasp the planner can hand it.
No arm moves. Needs ikpy, so run it in the container:

    docker exec -it orio_docker_container bash -c "source /home/ros_ws/devel/setup.bash && \\
        cd /home/ros_ws/src/devel_packages/orio && python3 ik_grid_test.py"

Grasps: a grid of points in the world (robot base) frame over the planner's camera window
(the yellow dashed box on the grasp panels, x -0.18..0.53, y -0.72..-0.03 at the table)
inside FetchInput's PICK_BOUNDS, at a few item heights; at each, the cup axis straight down
and tilted 15/30/45 deg toward 8 directions, and the cup line along world x and world y.
The rotation is built as perception builds it from the planner's axis and cup line, and
the point is pushed PICK_PUSH_DISTANCE along the cup axis, as FetchInput does.

Each grasp goes through compute_flat_pick_joints (the demo's IK), then the moves FetchInput
makes are checked: home -> pre-pick (3 s) -> pick (2 s) -> 0.15 m retract (2 s). See CHECKS
for what is checked and the limits. The IK is also re-run from many more starts, to see
whether it missed a better arm pose (or declined a grasp it could have reached).

Writes logging/ik_grid/<time>/results.json and summary.txt; --report DIR re-scores a run
with the current limits without solving again.
"""

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", "..", "logging", "ik_grid"))

# ── the grid ──────────────────────────────────────────────────────────────────
GRID_X = np.round(np.linspace(-0.15, 0.50, 6), 3)
GRID_Y = np.round(np.linspace(-0.70, -0.10, 6), 3)
GRID_Z = (0.0, 0.07, 0.14)          # grasp point height (table top is z -0.013)
TILTS_DEG = (0, 15, 30, 45)          # planner limit is 35 (grasp.yaml suction.max_tilt_deg)
TILT_DIRS_DEG = tuple(range(0, 360, 45))   # direction the surface normal leans toward
CUP_LINES_DEG = (0, 90)              # cup line along world x, world y

TABLE_Z = -0.013
VEL_LIMIT = np.array([2.175] * 4 + [2.61] * 3)   # Franka joint speed limits, rad/s
CHECK_JOINTS = [0, 2, 4, 6]         # J1/J3/J5/J7
MOVES = (("to pre-pick", 3.0), ("descend", 2.0), ("retract", 2.0))

# ── checks: (name, what it measures, warn above, fail above) ──────────────────
# The fail limits on descent/retract change and line, and facing, are the PICK_MAX_*
# limits compute_flat_pick_joints enforces (kept as numbers here, so loosening those
# shows up as failures).
# Metrics are "bigger is worse"; clearances are stored negated so the same rule works.
CHECKS = [
    ("reach_mm", "pick pose position error (mm)", 1.0, 2.0),
    ("ori_deg", "pick orientation off the grasp's own or 180-deg-swapped one (deg)", 0.5, 1.0),
    ("descent_change", "largest J1/J3/J5/J7 change, pre-pick -> pick (rad)", 0.25, 0.5),
    ("descent_line_mm", "cup off the straight line on the descent (mm)", 5.0, 15.0),
    ("retract_change", "largest J1/J3/J5/J7 change on the 0.15 m retract (rad)", 0.35, 0.7),
    ("retract_line_mm", "cup off the straight line on the retract (mm)", 8.0, 20.0),
    ("no_retract", "no 0.15 m retract (falls back to pre-pick height)", 0.5, 2.0),
    ("facing", "J1 at pre-pick away from the item's direction from the base (rad)", 0.7, 1.3),
    ("cup_order", "wrist turn the other cup order saves (deg)", 1.0, 10.0),
    ("wrist_turn_deg", "J7 turn from home to pre-pick (deg)", 120.0, 170.0),
    ("speed", "peak joint speed / Franka limit, any move", 0.5, 0.8),
    ("elbow_low", "-(lowest elbow/forearm height above the table) (m)", -0.10, -0.04),
    ("near_base", "-(closest wrist/hand comes to the base axis below 0.5 m) (m)", -0.18, -0.12),
    ("dip_mm", "cup lower than pre-pick height on the way there (mm)", 30.0, 80.0),
    ("missed_cost", "IK cost above the best found from many more starts", 0.05, 0.3),
]
CHECK_NAMES = [c[0] for c in CHECKS]


def rotation_from_approach(approach_world, cup_line_world):
    """Perception's _rotation_from_approach (perception_control_combined_pass_through.py):
    tool Z along the approach, tool Y along the cup line, tool X kept toward world +X."""
    z_axis = approach_world / np.linalg.norm(approach_world)
    ref = np.cross(np.asarray(cup_line_world, dtype=float), z_axis)
    if ref[0] < 0:
        ref = -ref
    x_axis = ref - np.dot(ref, z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def grid_cases():
    cases = []
    for x in GRID_X:
        for y in GRID_Y:
            for z in GRID_Z:
                for tilt in TILTS_DEG:
                    for tdir in (TILT_DIRS_DEG if tilt else (0,)):
                        for cup in CUP_LINES_DEG:
                            cases.append(dict(x=float(x), y=float(y), z=float(z), tilt=tilt,
                                              tilt_dir=tdir, cup=cup))
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
    fs = full_sequence
    logging.disable(logging.WARNING)
    fs.events.emit = lambda *a, **k: None
    import ikpy.chain
    hw = fs.RobotHardware.__new__(fs.RobotHardware)
    hw.ik_chain = ikpy.chain.Chain.from_urdf_file(fs.URDF_FILE, base_elements=["panda_link0"])
    hw.chain_length = len(hw.ik_chain.links)
    hw.ik_chain.active_links_mask = [False] + [True] * 7 + [False] * (hw.chain_length - 8)


def frames(q):
    full = [0.0] * hw.chain_length
    full[1:8] = q
    return hw.ik_chain.forward_kinematics(full, full_kinematics=True)


def path(q0, q1, n=21):
    """Joint path of a goto_joints move (min-jerk in joint space: the same path as linear)."""
    return [q0 + (q1 - q0) * s for s in np.linspace(0.0, 1.0, n)]


def off_line_mm(points, a, b):
    ab = b - a
    t = np.clip([(p - a) @ ab / (ab @ ab) for p in points], 0, 1)
    return 1000.0 * max(np.linalg.norm(p - (a + ti * ab)) for p, ti in zip(points, t))


def rot_angle_deg(Ra, Rb):
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def assess(task_pos, ori, pre, final, gap, retract_ok):
    """Metrics (CHECKS names) for the FetchInput moves with these joints."""
    lims = np.array([hw.ik_chain.links[i].bounds for i in range(1, 8)], dtype=float)
    home = np.array(fs.HOME_JOINTS)
    retract = hw.input_retract_joints(final)   # what FetchInput runs (None: back to pre-pick)
    m = {}
    Tf = hw._fk7(final)
    m["reach_mm"] = 1000.0 * float(np.linalg.norm(Tf[:3, 3] - task_pos))
    m["ori_deg"] = min(rot_angle_deg(Tf[:3, :3], ori), rot_angle_deg(Tf[:3, :3], ori @ fs.CUP_FLIP))
    m["descent_change"] = float(np.abs(final - pre)[CHECK_JOINTS].max())
    tip = lambda q: frames(q)[-1][:3, 3]
    m["descent_line_mm"] = off_line_mm([tip(q) for q in path(pre, final)], tip(pre), tip(final))
    m["no_retract"] = 0.0 if retract is not None else 1.0
    m["raw_retract"] = 0.0 if hw.compute_retract_joints(final, fs.INPUT_RETRACT_DISTANCE,
                                                        quiet=True) is not None else 1.0
    back = retract if retract is not None else pre
    m["retract_change"] = float(np.abs(back - final)[CHECK_JOINTS].max())
    m["retract_line_mm"] = off_line_mm([tip(q) for q in path(final, back)], tip(final), tip(back))
    m["facing"] = abs(float(wrap(pre[0] - np.arctan2(task_pos[1], task_pos[0]))))
    # The other cup order: the same arm with J7 half a turn round, if J7 can get there.
    turn = abs(pre[6] - fs.PICK_J7_HOME)
    alts = [pre[6] + s * np.pi for s in (-1, 1)]
    alts = [a for a in alts if min(a, final[6] + a - pre[6]) - lims[6, 0] >= fs.FLAT_PICK_MIN_LIMIT_GAP
            and lims[6, 1] - max(a, final[6] + a - pre[6]) >= fs.FLAT_PICK_MIN_LIMIT_GAP]
    m["cup_order"] = max([0.0] + [np.degrees(turn - abs(a - fs.PICK_J7_HOME)) for a in alts])
    m["wrist_turn_deg"] = float(np.degrees(turn))
    peak = 0.0
    lowest_elbow, nearest_base, lowest_cup = np.inf, np.inf, np.inf
    for (name, dur), (q0, q1) in zip(MOVES, ((home, pre), (pre, final), (final, back))):
        peak = max(peak, float((1.875 * np.abs(q1 - q0) / dur / VEL_LIMIT).max()))
        for q in path(q0, q1):
            fr = frames(q)
            lowest_elbow = min(lowest_elbow, min(f[2, 3] for f in fr[3:7]) - TABLE_Z)
            for f in fr[6:]:
                if f[2, 3] < 0.5:
                    nearest_base = min(nearest_base, float(np.hypot(f[0, 3], f[1, 3])))
            if name == "to pre-pick":
                lowest_cup = min(lowest_cup, fr[-1][2, 3])
    m["speed"] = peak
    m["elbow_low"] = -float(lowest_elbow)
    m["near_base"] = -float(nearest_base if np.isfinite(nearest_base) else 1.0)
    m["dip_mm"] = max(0.0, 1000.0 * float(tip(pre)[2] - lowest_cup))
    m["gap"] = float(gap)
    return m


def score_joints(task_pos, ori, pre, final):
    """{pre, final, cost, metrics} for given pick joints, cost as compute_flat_pick_joints
    scores it (re-solved from the joints themselves, with the cup order they reach)."""
    pre, final = np.asarray(pre, dtype=float), np.asarray(final, dtype=float)
    Rf = hw._fk7(final)[:3, :3]
    o = min((ori, ori @ fs.CUP_FLIP), key=lambda o: rot_angle_deg(Rf, o))
    cand = hw.flat_pick_candidate(task_pos, o, list(final))
    cost = float(cand[0]) if cand is not None else float("nan")
    lims = np.array([hw.ik_chain.links[i].bounds for i in range(1, 8)], dtype=float)
    gap = float(min(np.min(np.minimum(q - lims[:, 0], lims[:, 1] - q)) for q in (pre, final)))
    rok = hw.input_retract_joints(final) is not None
    return dict(pre=pre.tolist(), final=final.tolist(), cost=cost,
                metrics=assess(task_pos, ori, pre, final, gap, rok))


# Picks seen on the arm: (name, logged pick joints, pre-pick joints, should it pass?).
# The grasp is rebuilt from the logged pick joints (FK), then those joints and the IK's
# answer for the same grasp are both scored.
KNOWN = [
    ("grasp_0054 (20261111) as run: arm reaching round from the other side",
     [0.0878, 0.4501, -1.1438, -2.7375, 0.1781, 3.1475, 1.2612],
     [0.9398, 0.5336, -1.8946, -2.8151, 0.2944, 2.8248, 1.1897], False),
]


def run_known():
    out = []
    for name, final, pre, should_pass in KNOWN:
        T = hw._fk7(np.array(final))
        task_pos, ori = T[:3, 3], T[:3, :3]
        logged = score_joints(task_pos, ori, pre, final)
        res = hw.compute_flat_pick_joints(task_pos, target_ori=ori)
        ik = score_joints(task_pos, ori, *res) if res is not None else None
        out.append(dict(name=name, should_pass=should_pass, logged=logged, ik=ik))
    return out


def dense_starts(task_pos, rng):
    """Many more _flat_ik starts than the demo's: its own, J1 turned either way, random."""
    yaw = np.arctan2(task_pos[1], task_pos[0])
    starts = list(hw.flat_pick_starts(task_pos))
    for d1 in (-1.5, -0.75, 0.75, 1.5):
        for j7 in (-2.0, 0.4, 2.6):
            starts.append([yaw + d1, 0.3, 0.0, -2.2, 0.0, 2.5, j7])
    lims = np.array([hw.ik_chain.links[i].bounds for i in range(1, 8)], dtype=float)
    for _ in range(8):
        starts.append(list(rng.uniform(lims[:, 0] + 0.1, lims[:, 1] - 0.1)))
    return starts


def run_case(c):
    t0 = time.time()
    n = np.array([np.sin(np.radians(c["tilt"])) * np.cos(np.radians(c["tilt_dir"])),
                  np.sin(np.radians(c["tilt"])) * np.sin(np.radians(c["tilt_dir"])),
                  np.cos(np.radians(c["tilt"]))])
    approach = -n
    cup_line = np.array([np.cos(np.radians(c["cup"])), np.sin(np.radians(c["cup"])), 0.0])
    ori = rotation_from_approach(approach, cup_line)
    task_pos = np.array([c["x"], c["y"], c["z"]]) + fs.PICK_PUSH_DISTANCE * approach
    out = dict(c)

    res = hw.compute_flat_pick_joints(task_pos, target_ori=ori)
    if res is not None:
        out["chosen"] = score_joints(task_pos, ori, *res)

    # Dense search: every distinct arm pose from many starts, with its cost and metrics.
    rng = np.random.default_rng(c["id"])
    found = []
    for o in (ori, ori @ fs.CUP_FLIP):
        for seed in dense_starts(task_pos, rng):
            cand = hw.flat_pick_candidate(task_pos, o, seed)
            if cand is None:
                continue
            cost, pre, final, gap, rok = cand
            twins = [cand]
            for turn in (-np.pi, np.pi):   # cups swapped, as compute_flat_pick_joints weighs
                tp, tf = pre.copy(), final.copy()
                tp[6] += turn
                tf[6] += turn
                t = hw.score_flat_pick(task_pos, tp, tf)
                if t is not None:
                    twins.append(t)
            for cost, pre, final, gap, rok in twins:
                if any(np.abs(final - f["final"]).max() < 0.02 for f in found):
                    continue
                found.append(dict(cost=float(cost), pre=pre, final=final, gap=gap, rok=rok))
    found.sort(key=lambda f: f["cost"])
    out["alternatives"] = [dict(cost=f["cost"], pre=f["pre"].tolist(), final=f["final"].tolist(),
                                metrics=assess(task_pos, ori, f["pre"], f["final"], f["gap"], f["rok"]))
                           for f in found]
    best = found[0]["cost"] if found else None
    if "chosen" in out:
        out["chosen"]["metrics"]["missed_cost"] = max(0.0, out["chosen"]["cost"] - best) if best is not None else 0.0
    out["seconds"] = time.time() - t0
    return out


# ── scoring ───────────────────────────────────────────────────────────────────
def grade(metrics):
    """('pass'|'warn'|'fail', [failed checks], [warned checks])"""
    fails = [n for n, _, w, f in CHECKS if metrics.get(n, 0.0) > f]
    warns = [n for n, _, w, f in CHECKS if w < metrics.get(n, 0.0) <= f]
    return ("fail" if fails else "warn" if warns else "pass"), fails, warns


def case_label(r):
    s = "(%+.2f, %+.2f, %.2f) tilt %2d" % (r["x"], r["y"], r["z"], r["tilt"])
    if r["tilt"]:
        s += " toward %3d" % r["tilt_dir"]
    return s + ", cups along %s" % ("x" if r["cup"] == 0 else "y")


def fmt_q(q):
    return " ".join("%.4f" % v for v in q)


def report(results, out_dir, known=()):
    lines = []
    p = lines.append
    if known:
        p("Known picks:")
        ok_all = True
        for k in known:
            st, fl, _ = grade(k["logged"]["metrics"])
            ok = (st != "fail") == k["should_pass"]
            ok_all &= ok
            p("  %s" % k["name"])
            p("    logged joints: %s %s -> %s" % (st, ",".join(fl), "as expected" if ok else "NOT AS EXPECTED"))
            if k["ik"] is None:
                p("    IK now: declines the grasp")
            else:
                st2, fl2, wr2 = grade(k["ik"]["metrics"])
                m = k["ik"]["metrics"]
                ok_all &= st2 != "fail"
                p("    IK now: %s %s (descent %.2f, facing %.2f, wrist turn %.0f deg)" % (
                    st2, ",".join(fl2 or wr2), m["descent_change"], m["facing"], m["wrist_turn_deg"]))
        p("  -> %s" % ("OK" if ok_all else "PROBLEM"))
        p("")
    by_status = {"pass": [], "warn": [], "fail": [], "declined": []}
    for r in results:
        if "chosen" not in r:
            r["status"] = "declined"
            r["missed"] = bool(r["alternatives"])
        else:
            r["status"], r["fails"], r["warns"] = grade(r["chosen"]["metrics"])
        by_status[r["status"]].append(r)

    p("IK grid test: %d grasps (%d points x tilts/directions x 2 cup lines)" % (
        len(results), len(GRID_X) * len(GRID_Y) * len(GRID_Z)))
    p("")
    p("%-10s %6s %6s %6s %9s" % ("tilt", "pass", "warn", "fail", "declined"))
    for t in TILTS_DEG:
        rs = [r for r in results if r["tilt"] == t]
        p("%-10s %6d %6d %6d %9d" % ("%d deg" % t, *[sum(r["status"] == s for r in rs)
                                                     for s in ("pass", "warn", "fail", "declined")]))
    p("%-10s %6d %6d %6d %9d" % ("all", *[len(by_status[s]) for s in ("pass", "warn", "fail", "declined")]))
    p("")
    p("Checks (warn above / fail above), on the IK's choice:")
    chosen = [r for r in results if "chosen" in r]
    for name, desc, w, f in CHECKS:
        vals = np.array([r["chosen"]["metrics"].get(name, 0.0) for r in chosen])
        sign = -1 if name in ("elbow_low", "near_base") else 1
        p("  %-16s %-68s warn %6s fail %6s | median %7.3f worst %7.3f | %4d warn %4d fail" % (
            name, desc, w, f, sign * np.median(vals), sign * (vals.max() if sign > 0 else vals.max()),
            int(((vals > w) & (vals <= f)).sum()), int((vals > f).sum())))
    p("")
    missed_decl = [r for r in by_status["declined"] if r["missed"]]
    p("Declined: %d (%d where more starts found an arm pose the IK did not)" % (
        len(by_status["declined"]), len(missed_decl)))
    for r in missed_decl[:10]:
        a = r["alternatives"][0]
        st, fl, _ = grade(a["metrics"])
        p("  %s: found one that would %s %s" % (case_label(r), st, ",".join(fl)))
    p("")
    fixable = [r for r in by_status["fail"]
               if any(grade(a["metrics"])[0] != "fail" for a in r["alternatives"])]
    p("Failed: %d. Where a passing/warning arm pose existed (IK chose badly): %d. "
      "Where none did (the grasp should be declined): %d" % (
          len(by_status["fail"]), len(fixable), len(by_status["fail"]) - len(fixable)))
    counts = {}
    for r in by_status["fail"]:
        for n in r["fails"]:
            counts[n] = counts.get(n, 0) + 1
    p("  failed checks: %s" % ", ".join("%s %d" % kv for kv in sorted(counts.items(), key=lambda kv: -kv[1])))
    worst = sorted(by_status["fail"], key=lambda r: -r["chosen"]["metrics"]["descent_change"])
    p("")
    p("Worst failures (replay with replay_pick.py --pre ... --final ... to see one on arm 1):")
    for r in worst[:15]:
        m = r["chosen"]["metrics"]
        alt = next((a for a in r["alternatives"] if grade(a["metrics"])[0] != "fail"), None)
        p("  #%d %s: %s | descent %.2f facing %.2f wrist %.0f deg%s" % (
            r["id"], case_label(r), ",".join(r["fails"]), m["descent_change"], m["facing"],
            m["wrist_turn_deg"], "  (a better pose exists)" if alt else ""))
        p("      --pre '%s' --final '%s'" % (fmt_q(r["chosen"]["pre"]), fmt_q(r["chosen"]["final"])))
    text = "\n".join(lines)
    with open(os.path.join(out_dir, "summary.txt"), "w") as fh:
        fh.write(text + "\n")
    print(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--limit", type=int, help="only the first N grasps (a quick try)")
    ap.add_argument("--every", type=int, default=1, help="only every Nth grasp")
    ap.add_argument("--report", metavar="DIR", help="re-score an earlier run's results.json")
    args = ap.parse_args()

    if args.report:
        with open(os.path.join(args.report, "results.json")) as fh:
            data = json.load(fh)
        report(data["grid"], args.report, data["known"])
        return
    cases = grid_cases()[::args.every][:args.limit]
    out_dir = os.path.join(OUT_ROOT, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print("%d grasps on %d processes -> %s" % (len(cases), args.jobs, out_dir), flush=True)
    t0 = time.time()
    results = []
    with Pool(args.jobs, initializer=init_worker) as pool:
        for i, r in enumerate(pool.imap_unordered(run_case, cases, chunksize=2)):
            results.append(r)
            if (i + 1) % 200 == 0 or i + 1 == len(cases):
                print("  %d/%d  (%.0f s)" % (i + 1, len(cases), time.time() - t0), flush=True)
    results.sort(key=lambda r: r["id"])
    init_worker()
    known = run_known()
    with open(os.path.join(out_dir, "results.json"), "w") as fh:
        json.dump(dict(grid=results, known=known), fh)
    report(results, out_dir, known)


if __name__ == "__main__":
    main()
