# orio_perception tests

## test_grasp_geometry.py

Offline maths checks — no ROS, GPU or container needed.

```bash
python3 test/test_grasp_geometry.py
```

Covers `rotation_from_approach` (orthonormal, right-handed, Z == approach), quaternion
round-trip, waypoint parity with the original world-Z offsets, tilted-waypoint geometry,
retract, the tilt gate, and the bin-mask depth band.

## test_dexnet_service.py

Drives the real orchestrator code against a live planner. Needs a ROS master, the DexNet
container, and the perception venv.

```bash
bash src/devel_packages/orio_bringup/docker/run_dexnet.sh
source /opt/ros/noetic/setup.bash && source devel/setup.bash
src/devel_packages/orio_perception/venv/bin/python \
    src/devel_packages/orio_perception/test/test_dexnet_service.py \
    --json /tmp/results.json
```

Plans on Berkeley's five sample bin scenes (shipped with the gqcnn submodule, captured
with a PhoXi — a model-correctness check, not a proxy for the Xtion) and checks both
guard paths: an empty bin is declined rather than grasped, and the tilt gate rejects.

`sample_results.json` is the reference baseline; q-values should match to ~1e-3.

## visualize_grasps.py

Contact sheet from the JSON above: full scene, zoomed crop, and depth with the segmask
outlined.

```bash
src/devel_packages/orio_perception/venv/bin/python \
    src/devel_packages/orio_perception/test/visualize_grasps.py \
    --json /tmp/results.json --out /tmp/grasps.png
```

Note the sample colour images are twice the depth resolution; grasp pixels are in depth
coordinates and are scaled accordingly.
