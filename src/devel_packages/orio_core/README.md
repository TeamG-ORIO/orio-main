# orio_core

The **pure, unit-testable core** of ORIO: kinematics, geometry, perception math,
and planning helpers — no ROS, no hardware, no disk I/O, no logging, no global
state. Importable with no roscore running; tested with plain `pytest`.

## Design: honest core, dishonest edges

Functions here are *honest*: all inputs are arguments (including any rng or
clock), values are returned, no side effects. The *dishonest* work — `rospy`,
`FrankaArm` motion, service calls, Tkinter, file reads, seeding the rng — lives
in the ROS node scripts (e.g. `orio/state_machine.py`), which call these.

## Modules

- `robot_util.py` — homogeneous transforms, axis/angle, min-jerk interpolation,
  OBB/SAT collision primitives. Canonical home (`orio/RobotUtil.py` and
  `prm/RobotUtil.py` re-export it).
- `perception_geometry.py` — pinhole deprojection (pixel + depth + intrinsics →
  camera → world frame).
- `planning.py` — `make_ik_seed(...)` with an injected rng (deterministic IK
  random-restart).

## Tests

```bash
cd src/devel_packages/orio_core
python3 -m pytest -q
```
