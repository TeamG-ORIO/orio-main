# orio_logging

rerun recorder for ORIO runs. Design and measurements: [docs/LOGGING.md](../../../docs/LOGGING.md).

- `recorder.py` — sidecar node: ROS topics -> `<run_dir>/recorder.rrd` + `run.json`. Both arms in 3D from the URDF.
- `runs.py` — `list | open [run] | summary [run] | compare A B | du` over `logging/rerun/`.
- `config/cell.yaml` — arm 2 base pose in arm 1's frame (`placeholder: true` until measured).
- `franka_description/` — Panda meshes so rerun resolves the URDF's `package://` URIs on the host.
- `test/fake_publishers.py` — fake arms/state machine for a hardware-free recorder run.
- `test/bench_events.py` — per-call cost of the logging calls on the critical path.

```bash
source devel/setup.bash && source src/devel_packages/orio_perception/venv/bin/activate
python3 src/devel_packages/orio_logging/recorder.py            # standalone; launch_demo.sh starts it as a pane
python3 src/devel_packages/orio_logging/runs.py open latest    # rerun viewer
python3 -m pytest src/devel_packages/orio_logging/test
```
