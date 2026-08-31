# Control-PC environment (reference)

The Franka real-time control PCs (`iam-doc` = robot 1, `iamsleepy` = robot 2) run
**franka-interface** (C++ realtime controller), installed per-machine — not built
in this workspace. Reachable via the `ssh_doc` / `ssh_ep` aliases.

## franka-interface (reference mirror)

Pinned as a reference submodule so its exact code is browsable in-tree (outside
`src/`, so catkin never builds it):

| Fork | Pinned | Path |
|---|---|---|
| TeamG-ORIO/franka-interface (master) | `7a07921` | `reference/control_pcs/franka-interface` |

Team G edits vs upstream: termination handlers, cubic-hermite-spline joint
trajectory generator, `run_loop.cpp`, `cartesian_pose_skill.cpp`. Push control-PC
changes to the fork and bump the submodule here.

## frankapy (per-machine client)

frankapy is the Python client the demo imports — supplied per-machine at
`src/git_packages/frankapy` (git-ignored), not a reference submodule. See the
orio_bringup README.
