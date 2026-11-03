# orio_bringup

Docker image, run helpers, and tmux bring-up for the ORIO demo.

## Docker image

`docker/Dockerfile` builds the single `orio_docker` image: ROS Noetic + MoveIt +
franka-ros, CUDA 11.8 + ZED SDK 4.1, OpenNI2, Azure Kinect, RealSense, ikpy,
sllurp, PyTorch, and the `python-mercuryapi` submodule.

```bash
git submodule update --init --recursive
bash src/devel_packages/orio_bringup/docker/build.sh   # -> orio_docker
```

Run: `bash orio_run_docker.sh` (state machine); the cameras run in the same
container via `roslaunch manipulation cameras.launch`.

`docker/Dockerfile.dexnet` builds a separate `orio/dexnet` image for DexNet 4.0 suction
grasping — see [docs/DEXNET.md](docs/DEXNET.md).

### frankapy in the container

frankapy is a **per-machine client** — not rebuilt here, not a tracked submodule.
Supply it at `src/git_packages/frankapy` (git-ignored); `orio_run_docker.sh` mounts
it at the image's PYTHONPATH (`/home/ros_ws/src/git_packages/frankapy`):

```bash
git clone -b akshitr/widen-workspace-walls --recursive \
  git@github.com:TeamG-ORIO/frankapy.git src/git_packages/frankapy
```

The `akshitr/widen-workspace-walls` branch carries the demo's widened
`WORKSPACE_WALLS` (a client-side check). Override the location with
`FRANKAPY_DIR=/path/to/frankapy` for development. franka-interface (the C++
controller that runs on the robots) stays a reference mirror under
`reference/control_pcs/`.

### ZED neural-depth weights

`orio_run_docker.sh` bind-mounts the host resources
(`-v /usr/local/zed/resources/:/usr/local/zed/resources/`). They can't be baked
at build time (the SDK needs a GPU to download them); on a fresh machine the SDK
downloads + optimizes them on the first neural-mode run, then reuses them.

## Bring-up

Both demos run from one terminal, no tmux. See `COMMANDS.md` for the options.

```bash
bash run_full_demo.sh            # full two-arm demo: pick, label, place
bash run_dexnet_pnp_single.sh    # single-arm suction pick-and-place
bash stop_dexnet_pnp.sh          # tear everything down
```

- `tmux/wait_for.sh` — readiness gates (roscore / container / topic / service) used by both launchers.

## Troubleshooting

Machine-specific gotchas (USB/driver quirks, robot states, env issues) and their fixes
are logged in `docs/TROUBLESHOOTING.md` — check there first when bring-up misbehaves in a
way the code doesn't explain.
