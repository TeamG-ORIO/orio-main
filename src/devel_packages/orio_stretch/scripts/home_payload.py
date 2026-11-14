#!/usr/bin/env python3
"""
home_payload.py - re-home Stretch with a payload mounted on the gripper.

Same steps as stretch_robot_home.py (stretch_body Robot.home(): lift -> arm -> wrist yaw -> gripper),
with two changes for the payload:

  * Lift. Stock homing drives the lift UP until the motor effort passes contact_thresh_homing
    (+47.9 % on this robot, calibrated without a payload) and calls that spot "top of the mast".
    With the payload the effort passes that as soon as the lift moves, so the zero ends up wrong.
    Here the effort needed to raise the payload is measured first on a short move, and the homing
    threshold is set to that + a margin (in memory only, no YAML is changed).
  * The lift then parks at --lift (default 0.9 m) instead of 0.6 m, and the arm, wrist yaw (which
    swings all the way in to its hard stop) and optionally the gripper are homed UP THERE.

The gripper is not homed unless --gripper is given (things are mounted on it); its previous homing stays.

Nothing else may hold the robot (no stretch_driver / drive.sh). Runstop released.
The lift goes to the very top of the mast and the wrist swings fully in and then to forward: keep clear.

    python3 scripts/home_payload.py [--lift 0.9] [--gripper] [--skip-lift] [--margin 15]

Writes $ORIO_DATA/config/payload.env (LIFT_EFFORT=..), which every script that moves the lift up reads.
"""
import argparse
import os
import sys
import time

import stretch_body.robot

import orio_paths

MAX_THRESH = 95.0


def fail(robot, msg):
    print('\nHOMING FAILED: %s' % msg)
    robot.stop()
    sys.exit(1)


def measure_lift_up_effort(robot, dist=0.04):
    """Raise the lift a few cm with the contact stop wide open and return (peak effort %, distance moved)."""
    lift = robot.lift
    lift.motor.enable_guarded_mode()
    lift.pull_status()
    p0 = lift.status['pos']
    lift.move_by(x_m=dist, contact_thresh_pos=100.0, contact_thresh_neg=-100.0, req_calibration=False)
    robot.push_command()        # NOT lift.push_command(): the lift runs in sync mode, it only moves on the pimu sync trigger
    peak, t0, still = 0.0, time.time(), None
    while time.time() - t0 < 6.0:
        lift.pull_status()
        peak = max(peak, lift.status['motor']['effort_pct'])
        if time.time() - t0 > 0.5 and not lift.status['motor']['is_moving']:
            still = still or time.time()
            if time.time() - still > 0.3:
                break
        else:
            still = None
        time.sleep(0.02)
    lift.pull_status()
    return peak, lift.status['pos'] - p0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lift', type=float, default=0.9, help='lift height (m) to park at and home the arm / wrist at')
    ap.add_argument('--margin', type=float, default=15.0, help='effort %% added to the measured lifting effort (factory uses 10)')
    ap.add_argument('--skip-lift', action='store_true', help='do not re-home the lift, only raise it to --lift')
    ap.add_argument('--gripper', action='store_true', help='also home the gripper (opens/closes it fully)')
    args = ap.parse_args()

    robot = stretch_body.robot.Robot()
    if not robot.startup():
        print('Could not start the robot. Is stretch_driver / drive.sh / teleop_server still running?')
        sys.exit(1)
    time.sleep(0.5)
    if robot.pimu.status['runstop_event']:
        fail(robot, 'runstop is engaged - hold the head button ~2 s until it beeps, then run again')

    # ---------------- lift ----------------
    lift = robot.lift
    print('--------- Lift: measuring the effort needed to raise the payload ----')
    peak, moved = measure_lift_up_effort(robot)
    print('peak effort %.1f %% over %.3f m (factory homing threshold %.1f %%)' % (
        peak, moved, lift.params['contact_models']['effort_pct']['contact_thresh_homing'][1]))
    m = lift.motor.status
    print('lift motor: guarded_event=%s in_guarded_event=%s runstop_on=%s pos_calibrated=%s at_current_limit=%s' % (
        m['guarded_event'], m['in_guarded_event'], m['runstop_on'], m['pos_calibrated'], m['at_current_limit']))
    if moved < 0.015:
        fail(robot, 'the lift did not move up (%.3f m, effort %.0f %%): it is blocked, at the top, or the payload is too heavy' % (moved, peak))
    thresh = peak + args.margin
    if thresh > MAX_THRESH:
        fail(robot, 'raising the payload already needs %.0f %% effort - too heavy to home the lift safely' % peak)
    thresh = max(thresh, lift.params['contact_models']['effort_pct']['contact_thresh_homing'][1])
    print('using an upward contact threshold of %.1f %%' % thresh)

    if not args.skip_lift:
        print('--------- Homing Lift (up to the top of the mast, then down to %.2f m) ----' % args.lift)
        neg = lift.params['contact_models']['effort_pct']['contact_thresh_homing'][0]
        lift.params['contact_models']['effort_pct']['contact_thresh_homing'] = [neg, thresh]
        lift.home(end_pos=args.lift)
        lift.pull_status()
        if not lift.motor.status['pos_calibrated'] or abs(lift.status['pos'] - args.lift) > 0.03:
            fail(robot, 'lift homing did not finish (lift reads %.3f m, calibrated=%s)' % (
                lift.status['pos'], bool(lift.motor.status['pos_calibrated'])))
    else:
        print('--------- Raising Lift to %.2f m (not re-homed) ----' % args.lift)
        lift.move_to(x_m=args.lift, contact_thresh_pos=thresh, req_calibration=False)
        robot.push_command()
        time.sleep(1.0)
        lift.motor.wait_until_at_setpoint()
        lift.pull_status()
        if abs(lift.status['pos'] - args.lift) > 0.03:
            fail(robot, 'the lift stopped at %.3f m instead of %.2f m' % (lift.status['pos'], args.lift))
    print('lift at %.3f m' % lift.status['pos'])

    # ---------------- arm ----------------
    print('--------- Homing Arm ----')
    robot.arm.home()
    robot.arm.pull_status()
    if not robot.arm.motor.status['pos_calibrated']:
        fail(robot, 'arm homing did not finish')

    # ---------------- end of arm ----------------
    joints = ['wrist_yaw'] + (['stretch_gripper'] if args.gripper else [])
    for j in joints:
        if j not in robot.end_of_arm.joints:
            continue
        print('--------- Homing %s ----' % j)
        robot.end_of_arm.motors[j].home()
        if not robot.end_of_arm.motors[j].is_calibrated:
            fail(robot, '%s homing did not finish' % j)

    robot.pimu.trigger_beep()
    robot.push_command()
    time.sleep(0.5)
    robot.end_of_arm.pull_status()
    lift.pull_status()
    robot.arm.pull_status()
    print('\nHOMED. lift=%.3f m  arm=%.3f m  wrist_yaw=%.3f rad  is_homed=%s' % (
        lift.status['pos'], robot.arm.status['pos'], robot.end_of_arm.status['wrist_yaw']['pos'], robot.is_homed()))
    print('Check by eye: the arm should sit about %.0f cm below the top of its travel, the wrist pointing straight out.' % (
        100 * (lift.params['range_m'][1] - args.lift)))

    env = orio_paths.PAYLOAD_ENV
    os.makedirs(os.path.dirname(env), exist_ok=True)
    with open(env, 'w') as f:
        f.write('# written by scripts/home_payload.py: upward lift contact threshold (effort %%) with the payload mounted\n')
        f.write('LIFT_EFFORT=%.0f\n' % thresh)
    print('wrote %s (LIFT_EFFORT=%.0f)' % (env, thresh))
    robot.stop()


if __name__ == '__main__':
    main()
