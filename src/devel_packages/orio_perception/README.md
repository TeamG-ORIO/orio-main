# orio_perception

Grasp-pose and label-location perception for ORIO (ZED + Xtion cameras,
GroundingDINO + Segment-Anything + Open3D + grasp solver). ROS node, no frankapy
dependency.

## Scripts (`scripts/`)

- `perception_control_combined_pass_through.py` — the perception node (services
  `/compute_grasps`, `/compute_grasps_labelling`, `/get_depth_at_grasp`; publishes
  `/grasp_poses`, `/grasp_poses_labelling_z1/z2`, `/grasp_point_depth`).
- `grasp_solver.py`, `grasp_solver_extra.py`, `opt_label_location.py` — used by it.

## Runtime assets (per-machine, git-ignored)

Runs in a host venv (the docker image bakes only `torch`, not SAM/GroundingDINO/
Open3D). Large assets live under this package but are git-ignored:

| Path | What | ~Size |
|---|---|---|
| `venv/` | host venv (torch cu124, SAM, Open3D, editable GroundingDINO) | 7 GB |
| `weights/groundingdino_swint_ogc.pth` | GroundingDINO weights | 0.7 GB |
| `SAM_weights/sam_vit_b_01ec64.pth` | Segment-Anything ViT-B | 0.4 GB |
| `GroundingDINO/` | fork submodule (editable-installed) | 40 MB |
| `Xtion_imgs/ ZED_imgs/ Segmented_imgs/` | debug image dumps | — |

The node finds them via env vars (default to this package dir):
`ORIO_PERCEPTION_ASSETS` (weights + image dirs), `ORIO_GROUNDINGDINO_DIR`
(checkout), `ORIO_REPO` (manipulation TF yamls). `launch_demo.sh` sets these.

## Recreate the venv (fresh machine)

```bash
cd src/devel_packages/orio_perception
python3 -m venv venv && source venv/bin/activate
pip install -r requirements-perception.txt
pip install -e GroundingDINO/     # patched CUDA kernel
# download weights into weights/ and SAM_weights/
```

## GroundingDINO fork

`GroundingDINO/` = submodule of `git@github.com:TeamG-ORIO/GroundingDINO.git`
(branch `akshitr/orio-local-patches`, `9b89e28`), forked from upstream `856dde2`
with local patches: PyTorch 2.x CUDA-ext compat in `ms_deform_attn_cuda.cu`,
image-cache `or`→`and` in `groundingdino.py`, dropped torch pins. Full diff kept
in `groundingdino-local.patch`.
