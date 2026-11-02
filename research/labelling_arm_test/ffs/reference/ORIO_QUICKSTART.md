# Fast-FoundationStereo — ORIO quickstart

Real-time zero-shot stereo (NVlabs, CVPR 2026). Set up 2026-09-10 in
`/home/orio/newest/zed/fast-foundationstereo`. Everything below is **verified
working** on this PC (RTX 4090, driver 580).

Sibling of the offline `../FoundationStereo` setup — same ZED-M rectified pairs,
same intrinsics, ~8–33× faster.

## TL;DR — run it

```bash
cd /home/orio/newest/zed/fast-foundationstereo

# headless inference on the current ZED capture (analysis/raw/stereo_{left,right}.png)
./run_ffs_orio.sh python fast-foundationstereo/scripts/ffs_orio_infer.py \
    --out_dir /workspace/fast-foundationstereo/output_orio

# runtime + VRAM benchmark across configs
./run_ffs_orio.sh python fast-foundationstereo/scripts/ffs_bench.py

# interactive shell in the container
./run_ffs_orio.sh
```

`run_ffs_orio.sh` mounts `/home/orio/newest/zed` at `/workspace` so the ZED
captures in `analysis/raw/` are visible inside the container.

## What's already done

- **Repo** cloned (`fast-foundationstereo/`).
- **Docker image `ffs`** built from `docker/dockerfile` and **fixed** (see gotcha
  below). torch 2.6.0+cu124 / torchvision 0.21.0 / xformers 0.0.29.post3 /
  numpy 2.5.3, TensorRT 10.11, conda env `my`.
- **Weights** in `weights/`:
  - `23-36-37/model_best_bp2_serialize.pth` — the working default (this is the
    HF **`nvidia/c-fast-foundationstereo`** release checkpoint, 17.7M params;
    also copied to `c-ffs/`).
  - ONNX exports for `20_26_39`, `20_30_48`, `23_36_37` at 320×736 and 576×960,
    iters 4/8 (downloaded fine from Drive) — for the TRT path.
  - `cfg.yaml` present for all checkpoints.
- **ORIO intrinsics**: `assets_orio_K.txt` (ZED-M rectified: fx=fy=732.559,
  cx=640.311, cy=368.349, baseline=0.063150 m).
- **Scripts** (added, headless-safe for SSH — no cv2.imshow / open3d window):
  - `scripts/ffs_orio_infer.py` — disparity → metric depth → cloud on a ZED pair.
  - `scripts/ffs_bench.py` — runtime / FPS / peak VRAM sweep.

## Measured performance (RTX 4090, C-Fast checkpoint, ORIO ZED pair)

| Config | Resolution | Time/frame | FPS | Peak VRAM |
|---|---|---|---|---|
| full-res, iters=8 | 720×1280 | 142 ms | 7.0 | 5.6 GB |
| full-res, iters=4 | 720×1280 | 129 ms | 7.8 | 5.0 GB |
| half-res, iters=8 | 360×640 | 40 ms | 24.7 | 1.4 GB |
| half-res, iters=4 | 360×640 | 36 ms | 27.9 | 1.4 GB |

For reference, `../FoundationStereo` (ViT-L, 375M) is ~1189 ms/frame, ~7 GB on
the same box. Fast-FoundationStereo is **~8× faster at full-res, ~33× at
half-res** (real-time), 17.7M params.

Tip from the authors: the model is best at image width < 1000 px, so `--scale
0.5` (half-res) is both faster **and** the intended operating point; upsample the
depth back with nearest-neighbor if you need full-res output.

## Two gotchas already handled (don't re-hit these)

1. **torch/torchvision mismatch in the stock Dockerfile.** The unpinned
   `xformers` in `docker/dockerfile` line 29 lets the resolver pull torch 2.14,
   which breaks torchvision 0.21 (`operator torchvision::nms does not exist`).
   The `ffs` image on disk is already corrected (torch pinned to 2.6.0, xformers
   to 0.0.29.post3, numpy kept at 2.x for scipy). If you ever rebuild from
   scratch, pin those in the Dockerfile first — see `docker/dockerfile` comment.

2. **HF checkpoint missing `args.normalize`.** The `c-fast-foundationstereo`
   pickle's `model.args` lacks the `normalize` key the code reads. Both ORIO
   scripts backfill it (`normalize=True`, the code default). The stock
   `scripts/run_demo.py` does **not** — if you run that with this checkpoint,
   add `OmegaConf.set_struct(model.args, False); model.args.normalize = True`
   after load, or use `ffs_orio_infer.py`.

## Getting the other numbered checkpoints (optional)

The Drive `.pth` files for the `23-36-37` / `20-30-48` speed/accuracy variants
were **quota-blocked** ("too many downloads", 24 h cooldown) on 2026-09-10. The
ONNX exports came through fine. To retry the raw checkpoints later:

```bash
cd weights
python3 -m gdown "1W1V1H64l9bAi97boEQQ2ueNzzGmSMz-E" -O 23-36-37/model_best_bp2_serialize.pth  # 23-36-37
python3 -m gdown "1ZsXw3WwBmtNdv1lqdEJ95kyvf7J4YusE" -O 20-30-48/model_best_bp2_serialize.pth  # 20-30-48 (fastest)
```

Or the full folder: `python3 -m gdown --folder https://drive.google.com/drive/folders/1HuTt7UIp7gQsMiDvJwVuWmKpvFzIIMap`.
The current default (`c-fast-foundationstereo`) is a fully working release
checkpoint, so this is only needed to A/B the family variants.

## TensorRT (later, for max speed)

We already have the ONNX exports. Inside the container, `trtexec` is aliased.
See the repo readme "ONNX/TRT" section — `scripts/run_demo_single_trt.py`
auto-detects `.onnx`/`.engine`. TRT roughly halves the PyTorch latency.

## Next steps (depth-quality comparison)

The natural follow-up is to score Fast-FoundationStereo on the **white/green box
void metric** the way `../FoundationStereo` was scored (`../score_fs.py`,
box masks in `../box_depth.py`) and drop it into the comparison report — i.e.
does a 17.7M real-time model still fill the white-box hole that ZED NEURAL
leaves. `output_orio/depth_meter.npy` is in the same format `score_fs.py` reads.
