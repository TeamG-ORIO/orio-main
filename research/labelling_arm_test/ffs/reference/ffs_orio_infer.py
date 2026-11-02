#!/usr/bin/env python3
"""Headless Fast-FoundationStereo inference on a ZED-M rectified stereo pair.

Mirrors scripts/run_demo.py but removes the GUI calls (cv2.imshow / open3d
window) so it runs over SSH, and defaults to the ORIO ZED capture + intrinsics.
Saves disparity, metric depth (.npy), a disparity visualization, and a cloud.ply.

Run inside the `ffs` docker (torch 2.6). Example:
  python scripts/ffs_orio_infer.py \
      --left_file  /workspace/analysis/raw/stereo_left.png \
      --right_file /workspace/analysis/raw/stereo_right.png \
      --out_dir    /workspace/fast-foundationstereo/output_orio \
      --valid_iters 8 --scale 1
"""
import os, sys, argparse, logging, time
code_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(f'{code_dir}/../')
import numpy as np, torch, imageio, yaml, cv2
from omegaconf import OmegaConf
from core.utils.utils import InputPadder
from Utils import (AMP_DTYPE, set_logging_format, set_seed, vis_disparity,
                   depth2xyzmap, toOpen3dCloud, o3d)

WS = os.environ.get("ORIO_WS", "/workspace")

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument('--model_dir', default=f'{code_dir}/../weights/23-36-37/model_best_bp2_serialize.pth')
    p.add_argument('--left_file',  default=f'{WS}/analysis/raw/stereo_left.png')
    p.add_argument('--right_file', default=f'{WS}/analysis/raw/stereo_right.png')
    p.add_argument('--intrinsic_file', default=f'{code_dir}/../assets_orio_K.txt')
    p.add_argument('--out_dir', default=f'{code_dir}/../output_orio')
    p.add_argument('--scale', default=1.0, type=float)
    p.add_argument('--valid_iters', default=8, type=int)
    p.add_argument('--max_disp', default=192, type=int)
    p.add_argument('--hiera', default=0, type=int)
    p.add_argument('--remove_invisible', default=1, type=int)
    p.add_argument('--zfar', default=3.0, type=float, help='max depth (m) kept in cloud')
    p.add_argument('--get_pc', default=1, type=int)
    args = p.parse_args()

    set_logging_format(); set_seed(0); torch.autograd.set_grad_enabled(False)
    os.makedirs(args.out_dir, exist_ok=True)

    with open(f'{os.path.dirname(args.model_dir)}/cfg.yaml') as ff:
        cfg = yaml.safe_load(ff)
    for k, v in vars(args).items():
        if v is not None:
            cfg[k] = v
    cfg = OmegaConf.create(cfg)

    t = time.time()
    model = torch.load(args.model_dir, map_location='cpu', weights_only=False)
    # Some released checkpoints (e.g. HF c-fast-foundationstereo) pickle an args
    # struct missing keys the code reads. Backfill with the code's defaults.
    OmegaConf.set_struct(model.args, False)
    if 'normalize' not in model.args:
        model.args.normalize = True
    model.args.valid_iters = args.valid_iters
    model.args.max_disp = args.max_disp
    model.cuda().eval()
    torch.cuda.synchronize()
    logging.info(f"model load + to-GPU: {time.time()-t:.1f}s  "
                 f"({sum(p.numel() for p in model.parameters())/1e6:.1f}M params)")

    img0 = imageio.v2.imread(args.left_file)[..., :3]
    img1 = imageio.v2.imread(args.right_file)[..., :3]
    if args.scale != 1.0:
        img0 = cv2.resize(img0, None, fx=args.scale, fy=args.scale)
        img1 = cv2.resize(img1, dsize=(img0.shape[1], img0.shape[0]))
    H, W = img0.shape[:2]
    img0_ori = img0.copy()
    t0 = torch.as_tensor(img0).cuda().float()[None].permute(0, 3, 1, 2)
    t1 = torch.as_tensor(img1).cuda().float()[None].permute(0, 3, 1, 2)
    padder = InputPadder(t0.shape, divis_by=32, force_square=False)
    t0, t1 = padder.pad(t0, t1)

    torch.cuda.reset_peak_memory_stats()
    t = time.time()
    with torch.amp.autocast('cuda', enabled=True, dtype=AMP_DTYPE):
        if args.hiera:
            disp = model.run_hierachical(t0, t1, iters=args.valid_iters, test_mode=True, small_ratio=0.5)
        else:
            disp = model.forward(t0, t1, iters=args.valid_iters, test_mode=True, optimize_build_volume='pytorch1')
    torch.cuda.synchronize()
    logging.info(f"forward (1st, incl. compile): {time.time()-t:.2f}s  "
                 f"peak VRAM {torch.cuda.max_memory_reserved()/1e9:.2f} GB")
    disp = padder.unpad(disp.float()).cpu().numpy().reshape(H, W).clip(0, None)

    vis = vis_disparity(disp)
    imageio.imwrite(f'{args.out_dir}/disp_vis.png', np.concatenate([img0_ori, vis], axis=1))
    np.save(f'{args.out_dir}/disp.npy', disp)

    if args.remove_invisible:
        yy, xx = np.meshgrid(np.arange(H), np.arange(W), indexing='ij')
        disp[(xx - disp) < 0] = np.inf

    with open(args.intrinsic_file) as f:
        lines = f.readlines()
        K = np.array(list(map(float, lines[0].split()))).reshape(3, 3).astype(np.float32)
        baseline = float(lines[1])
    K[:2] *= args.scale
    depth = K[0, 0] * baseline / disp
    np.save(f'{args.out_dir}/depth_meter.npy', depth)
    v = np.isfinite(depth) & (depth > 0) & (depth < args.zfar)
    logging.info(f"depth: {v.mean()*100:.1f}% valid in (0,{args.zfar}]m, "
                 f"median {np.median(depth[v]):.3f}m")

    if args.get_pc:
        xyz = depth2xyzmap(depth, K)
        pcd = toOpen3dCloud(xyz.reshape(-1, 3), img0_ori.reshape(-1, 3))
        pts = np.asarray(pcd.points)
        keep = np.where((pts[:, 2] > 0) & (pts[:, 2] <= args.zfar))[0]
        pcd = pcd.select_by_index(keep)
        o3d.io.write_point_cloud(f'{args.out_dir}/cloud.ply', pcd)
        logging.info(f"cloud.ply saved ({len(keep)} pts)")
    logging.info(f"outputs -> {args.out_dir}")
