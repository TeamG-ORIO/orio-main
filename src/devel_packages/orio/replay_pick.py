#!/usr/bin/env python3
"""Replay one logged input pick on arm 1 (iam-doc), without the vacuum.

Finds the pick's IK joints in a demo log (the "[IK] flat-arm pick" line logged after the
grasp's "Grasp panel" line) and drives the same moves full_sequence.py made:

    home -> pre-pick (3 s) -> pick (2 s) -> back to pre-pick (2 s) -> home

The demo backed out 0.15 m along the cup axis after the pick; those joints are not
logged, so this backs out to the pre-pick joints instead (the same line, 0.10 m).

Without --go it only prints the plan. With --go it waits for Enter before each move
(--no-step to run straight through). The pick pose goes down to where the item was:
clear the bin under it, or the arm presses into whatever is there now.

    python3 replay_pick.py --grasp grasp_0046_145317                 # plan only
    python3 replay_pick.py --grasp grasp_0046_145317 --go --slow 2   # move, half speed
    python3 replay_pick.py --pre "j1 ... j7" --final "j1 ... j7" --go
"""

import argparse
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO_LOGS = os.path.normpath(os.path.join(HERE, "..", "..", "..", "logging", "demo"))
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
CHECK_JOINTS = [0, 2, 4, 6]   # J1/J3/J5/J7: should barely move on the straight descent
MAX_DESCENT_JOINT_CHANGE = 0.25

PICK_RE = re.compile(r"\[IK\] flat-arm pick: (.*?) — pre=\[([^\]]+)\] final=\[([^\]]+)\]")


def find_pick(grasp, log_path=None):
    """(log path, pre, final, summary) for the pick that used `grasp`."""
    grasp = os.path.basename(grasp).replace("_camera", "").replace(".png", "")
    logs = [log_path] if log_path else sorted(glob.glob(os.path.join(DEMO_LOGS, "*.log")))
    for path in logs:
        with open(path, errors="replace") as fh:
            lines = fh.readlines()
        for i, line in enumerate(lines):
            if "Grasp panel:" in line and grasp + ".png" in line:
                for later in lines[i + 1:]:
                    if "Grasp panel:" in later:
                        break   # the next plan started: this grasp was never picked
                    m = PICK_RE.search(later)
                    if m:
                        pre = np.array(m.group(2).split(), dtype=float)
                        final = np.array(m.group(3).split(), dtype=float)
                        return path, pre, final, m.group(1)
                sys.exit("%s is in %s, but no flat-arm pick follows it (declined, or "
                         "not a flat-arm pick)" % (grasp, path))
    sys.exit("no 'Grasp panel: .../%s.png' line in %s" % (grasp, ", ".join(logs)))


def parse_joints(text, name):
    q = np.array(text.replace(",", " ").replace("[", " ").replace("]", " ").split(), dtype=float)
    if q.shape != (7,):
        sys.exit("--%s needs 7 joint values, got %d" % (name, q.size))
    return q


def fmt(q):
    return "[" + " ".join("%+.3f" % v for v in q) + "]"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grasp", help="grasp record name, e.g. grasp_0046_145317 (the _camera.png name works too)")
    ap.add_argument("--log", help="demo log to search (default: every log in logging/demo)")
    ap.add_argument("--pre", help="pre-pick joints instead of --grasp (7 values)")
    ap.add_argument("--final", help="pick joints instead of --grasp (7 values)")
    ap.add_argument("--go", action="store_true", help="move the arm (default: print the plan only)")
    ap.add_argument("--slow", type=float, default=1.0,
                    help="multiply every move's duration (2 = half speed; default 1, as in the demo)")
    ap.add_argument("--no-step", action="store_true", help="don't wait for Enter before each move")
    ap.add_argument("--no-home", action="store_true", help="start from where the arm is, not home")
    args = ap.parse_args()

    if args.grasp:
        src, pre, final, summary = find_pick(args.grasp, args.log)
        print("Pick for %s, from %s" % (args.grasp, src))
        print("  logged: %s" % summary)
    elif args.pre and args.final:
        pre, final = parse_joints(args.pre, "pre"), parse_joints(args.final, "final")
    else:
        ap.error("give --grasp, or both --pre and --final")
    if args.slow < 1.0:
        ap.error("--slow below 1 would move faster than the demo did")

    change = np.abs(final - pre)
    print("  pre-pick %s" % fmt(pre))
    print("  pick     %s" % fmt(final))
    print("  change   %s" % fmt(final - pre))
    worst = max(CHECK_JOINTS, key=lambda j: change[j])
    print("  largest J1/J3/J5/J7 change: %s %.2f rad (%s the %.2f rad check)"
          % (JOINT_NAMES[worst], change[worst],
             "FAILS" if change[worst] > MAX_DESCENT_JOINT_CHANGE else "passes",
             MAX_DESCENT_JOINT_CHANGE))

    moves = [("pre-pick", pre, 3.0), ("pick (descend)", final, 2.0), ("back to pre-pick", pre, 2.0)]
    print("\nPlan (durations x%.1f):" % args.slow)
    if not args.no_home:
        print("  home")
    for name, q, d in moves:
        print("  %-17s %4.1f s  %s" % (name, d * args.slow, fmt(q)))
    if not args.no_home:
        print("  home")
    if not args.go:
        print("\nPlan only. Add --go to move arm 1 (clear the bin under the pick first).")
        return

    from frankapy import FrankaArm   # only needed to move
    fa = FrankaArm(robot_num=1, with_gripper=False, old_gripper=False)
    print("\nArm 1 now at %s" % fmt(fa.get_joints()))

    def step(name):
        if not args.no_step:
            input("Enter: %s  (Ctrl-C to stop) " % name)
        print("-> %s" % name)

    if not args.no_home:
        step("home")
        fa.reset_joints()
    for name, q, d in moves:
        now = np.asarray(fa.get_joints(), dtype=float)
        print("   largest joint travel %.2f rad (%s)" % (np.abs(q - now).max(),
                                                       JOINT_NAMES[int(np.argmax(np.abs(q - now)))]))
        step(name)
        fa.goto_joints(q, duration=d * args.slow)
        print("   reached %s" % fmt(fa.get_joints()))
    if not args.no_home:
        step("home")
        fa.reset_joints()
    print("Done.")


if __name__ == "__main__":
    main()
