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

Run: `bash orio_run_docker.sh` (state machine) and `bash zed_run_docker.sh` (cameras).

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

Both run scripts bind-mount the host resources
(`-v /usr/local/zed/resources/:/usr/local/zed/resources/`). They can't be baked
at build time (the SDK needs a GPU to download them); on a fresh machine the SDK
downloads + optimizes them on the first neural-mode run, then reuses them.

## tmux bring-up

`launch_demo.sh` (repo root) runs the whole demo in one tmuxifier session: 8
readiness-gated panes (`tmux/wait_for.sh`) in a tiled window.

```bash
bash launch_demo.sh              # full demo
bash launch_demo.sh --no-vacuum  # dry-run, no pneumatics
```

- `tmux/layouts/orio.session.sh` — session layout.
- `tmux/wait_for.sh` — readiness gates (roscore / container / topic / service).

tmuxifier install (once per machine):
`git clone https://github.com/jimeh/tmuxifier.git ~/.tmuxifier`.
