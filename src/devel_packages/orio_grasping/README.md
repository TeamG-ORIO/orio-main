# orio_grasping (vendored)

The parts of the [orio-grasping](https://github.com/TeamG-ORIO/orio-grasping) repo the suction
grasp planner needs at run time, so `grasp_test.sh` does not depend on which branch
`~/orio-grasping` has checked out. Laid out like that repo, so `run_suction.sh` takes it as
`ORIO_GRASPING`.

Copied unchanged from branch `orioNet-segMasks`, commit `788a676`:

- `orio_grasping/`: `camera_model`, `config`, `cuda_env`, `dataset`, `geometry`, `heightmap`,
  `model`, `rgbd_seg`, `train_data`, `xtion` (and `__init__`)
- `scripts/predict_seg_masks.py`, `scripts/segment_captures.py` (the planner uses
  `instance_image` and `bin_region` from them)
- `config.toml`, `assets/calibration/xtion_pose.json` (camera pose)

Weights (git-ignored, copy them by hand onto a new machine):

- `runs/2026-10-14_010549_seg_v1/best.pt`: OrioNet seg_v1 (height, normals, unseen, seg)
- `runs/seg/rgbd_v2/checkpoint_best_total.pth`: RF-DETR-Seg Large RGB-D segmentation, the
  model seg_v1's mask channels come from
- `runs/2026-10-27_130723_seg_v2_deformables/epoch59.pt`: OrioNet seg_v2_deformables, the
  run's `last.pt` (epoch 59) without the optimiser state; its `best.pt` is epoch 23 (best
  validation top-1), and epoch 48 (lowest validation loss) was not saved
- `runs/seg/rgbd_v3_deformables/checkpoint_best_total.pth`: rgbd_v2 fine-tuned on deformable
  pouches, the model seg_v2_deformables' mask channels come from

To update, copy the same files from a newer orio-grasping commit and change the commit above.
