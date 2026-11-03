# ORIO

ORIO is a two-arm robot cell that picks items out of a bin, labels them and places them in
an output box. Arm 1 (Franka, control PC `iam-doc`) picks each item from the bin with a
suction cup, using a grasp planned by a learned suction network on an Xtion RGB-D camera,
and carries it through a labelling zone to the output box. Arm 2 (Franka, control PC
`iam-luisa`) takes a label from the dispenser, finds a flat spot on the item in the ZED
camera's view and applies the label. An RFID node registers each labelled item in the
inventory database.

Everything runs in Docker containers on the workstation and is started from one terminal.

## Running it

```bash
bash run_full_demo.sh            # the full two-arm demo: pick, label, place
bash run_pickup-only.sh    # arm 1 only: suction pick-and-place from the bin
bash stop_all.sh          # stop everything the two launchers started
```

Ctrl-C stops the current run only. The slow-to-start stack (roscore, containers, control
PCs, cameras, planner, perception) stays up and the next run reuses it. The stop script
tears it all down.

[COMMANDS.md](COMMANDS.md) lists every command with its options, including the helpers:
`grasp_test.sh` (plan and visit grasps without suction), `reset_robot.sh` (home an arm),
`snap_xtion.sh` (save one camera frame) and `lock_arms.py` (brakes via Desk).

## Where things are

| Path | What |
| --- | --- |
| `run_full_demo.sh`, `run_pickup-only.sh`, `stop_all.sh` | The launchers and the stop script. |
| `src/devel_packages/orio_bringup/stack.sh` | The bring-up library the launchers share: roscore, control PCs, containers, cameras, planner, perception, teardown. |
| `src/devel_packages/orio_bringup/docker/` | Dockerfiles and the `run_*.sh` scripts that start each container. |
| `src/devel_packages/orio/` | The arm code: `full_sequence.py` (both arms), `dexnet_pnp.py` (arm 1 only), IK and collision checking. |
| `src/devel_packages/orio_perception/` | The perception node: grasp and label-placement services, plus the Fast-FoundationStereo depth node. |
| `src/devel_packages/orio_grasping/` | The suction grasp network and its checkpoints. |
| `src/devel_packages/orio_rfid/` | The RFID inventory node and database. |
| `src/devel_packages/orio_logging/` | The rerun run recorder. |
| `src/devel_packages/manipulation/` | Camera launch files and the camera-to-robot transforms. |
| `research/` | Experiments not part of the demo: the one-arm labelling test and the label-placement handoff. |
| `docs/` | Setup, build, logging, DexNet, troubleshooting and open issues. |

## Setting up a machine

See [docs/SETUP.md](docs/SETUP.md) and [docs/BUILD.md](docs/BUILD.md). In short:

1. Install Docker with the NVIDIA container toolkit, then clone this repo with its
   submodules (`git submodule update --init --recursive`).
2. Build the images. The main one:
   ```bash
   bash src/devel_packages/orio_bringup/docker/build.sh   # -> orio_docker
   ```
   The planner and Fast-FoundationStereo images are built from
   `src/devel_packages/orio_bringup/docker/Dockerfile.suction` and `Dockerfile.ffs`; the
   commands are in [COMMANDS.md](COMMANDS.md).
3. Supply frankapy, a per-machine client that is git-ignored, at
   `src/git_packages/frankapy`:
   ```bash
   git clone -b akshitr/widen-workspace-walls --recursive \
     git@github.com:TeamG-ORIO/frankapy.git src/git_packages/frankapy
   ```
   The main container mounts it on its PYTHONPATH. Override the location with
   `FRANKAPY_DIR`.
4. Put the robot network in place (`setup_robot_network.sh`, once, with sudo) and make
   sure `iam-doc` and `iam-luisa` resolve. The control PCs are reached over ssh with keys.

The main container can be opened by hand for debugging:

```bash
bash src/devel_packages/orio_bringup/docker/run_orio.sh
```

## When something misbehaves

[docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) has the known machine-specific
problems and their fixes (camera start-up failures, control PC states, environment
issues). [docs/OPEN_ISSUES.md](docs/OPEN_ISSUES.md) lists what is still unresolved. Run
logs are under `logging/` (see [docs/LOGGING.md](docs/LOGGING.md)).
