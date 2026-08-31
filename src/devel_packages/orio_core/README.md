# orio_core

The **pure, unit-testable core** of the ORIO project. Kinematics, geometry,
perception math, and planning helpers — with **no ROS, no hardware, no disk I/O,
no logging, and no hidden global state**.

## The honest/dishonest contract

Every function here is *honest*: it takes all of its inputs as arguments
(including any random-number generator or clock it needs), returns values, and
has no side effects. That is exactly what lets this package be:

- imported with **no roscore running**, and
- tested with plain `pytest` (no ROS, no robot, no cameras).

The *dishonest* work — `rospy` pub/sub, `FrankaArm` motion, service calls,
Tkinter, pickle/JSON/YAML file reads, and **seeding the RNG** — stays in the ROS
node scripts (e.g. `orio/state_machine.py`), which fetch inputs from the outside
world and then call these pure functions.

## Modules

- `robot_util.py` — homogeneous transforms, axis/angle, min-jerk interpolation,
  OBB/SAT collision primitives. (Canonical home; `orio/RobotUtil.py` and
  `prm/RobotUtil.py` are thin compatibility shims that re-export this.)
- `perception_geometry.py` — pinhole deprojection (pixel + depth + intrinsics →
  camera frame → world frame). Honest extraction of the math in
  `state_machine.py::compute_label_joints`.
- `planning.py` — `make_ik_seed(...)` with an **injected** rng (deterministic
  IK random-restart). Honest extraction of the seed loop in
  `state_machine.py::compute_pick_joints`.

## Running the tests

```bash
cd src/devel_packages/orio_core
python3 -m pytest -q
```

## Status / next step (wiring)

`robot_util` is **fully wired in**: the live scripts import it through the shims,
so there is a single source of truth.

`perception_geometry` and `planning` are **extracted and tested but not yet
wired** into `state_machine.py` — the inline copies still run the demo. Rewiring
the node to call these (and deleting the inline copies) is a behaviour-sensitive
change that should be done one function at a time and **validated on hardware**;
the tests here pin the current behaviour so that rewire stays equivalent.
