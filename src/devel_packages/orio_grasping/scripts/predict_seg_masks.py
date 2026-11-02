"""Run the RGB-D segmentation model on generated scenes and save its instance masks next to them.

Run in the ``orio-grasping`` env::

    conda run -n orio-grasping python scripts/predict_seg_masks.py data/<run> \\
        --checkpoint /mnt/Shared/orio-grasping/runs/seg/rgbd_v2/checkpoint_best_total.pth

For OrioNet's mask input (``[train] channels`` with ``seg``): the grasp network should learn from
the masks the segmentation model really produces, mistakes included, so run this on scenes the
segmentation model was not trained on (on its own training scenes its masks are near perfect).

Each scene is fed as in ``train_seg_rgbd.py``'s validation: ``rgb.png`` and the height channels
of ``depth.png`` (the sensor-model depth), the whole image. Writes per scene:

* ``seg_pred.png``: uint16, 0 = no object, k = the k-th most confident mask (masks scoring at
  least ``--threshold``; where masks overlap, the more confident one wins);
* ``seg_pred.json``: checkpoint, threshold and each mask's score and pixel count.

Scenes that already have both files are skipped unless ``--force``, so an interrupted run resumes.
Then run ``make_heightmaps.py``, which projects ``seg_pred.png`` into the map (``noisy_seg``).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from orio_grasping.cuda_env import drop_isaac_libraries  # noqa: E402
from orio_grasping.dataset import list_scenes  # noqa: E402

PRED_PNG, PRED_JSON = "seg_pred.png", "seg_pred.json"


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("runs", type=Path, nargs="+", help="Run folders (or single scene folders).")
    p.add_argument("--checkpoint", type=Path, required=True, help="RGB-D weights (train_seg_rgbd.py's checkpoint_best_total.pth).")
    p.add_argument("--model", default="large", help="Size the checkpoint was trained as.")
    p.add_argument("--threshold", type=float, default=0.5, help="Lowest score of a mask kept.")
    p.add_argument("--batch-size", type=int, default=8)
    p.add_argument("--force", action="store_true", help="Redo scenes that already have predictions.")
    return p.parse_args()


def instance_image(masks: np.ndarray, scores: np.ndarray, shape) -> tuple[np.ndarray, list[dict]]:
    """uint16 image: 0 = none, k = the k-th most confident mask; overlaps go to the more confident."""
    out = np.zeros(shape, np.uint16)
    order = np.argsort(-scores)
    records = []
    for k in reversed(range(len(order))):  # least confident first, so the most confident paints last
        out[masks[order[k]]] = k + 1
    for k, i in enumerate(order):
        records.append({"id": k + 1, "score": round(float(scores[i]), 4), "pixels": int((out == k + 1).sum())})
    return out, records


def main():
    drop_isaac_libraries()
    args = parse_args()
    import rfdetr
    import torch

    from orio_grasping import rgbd_seg
    from segment_captures import MODELS

    scenes = []
    for r in args.runs:
        scenes += [r] if (r / "meta.json").is_file() else [s for s in list_scenes(r) if (s / "meta.json").is_file()]
    todo = [s for s in scenes if args.force or not ((s / PRED_PNG).is_file() and (s / PRED_JSON).is_file())]
    print(f"{len(scenes)} scenes, {len(todo)} to predict", flush=True)
    if not todo:
        return
    model = rgbd_seg.load_model(getattr(rfdetr, MODELS[args.model]), args.checkpoint)
    t0, done = time.perf_counter(), 0
    for b in range(0, len(todo), args.batch_size):
        batch = todo[b : b + args.batch_size]
        xs = []
        for s in batch:
            rgb = np.asarray(Image.open(s / "rgb.png").convert("RGB"))
            extra = rgbd_seg.encode_height(rgbd_seg.scene_height(s))
            xs.append(torch.cat([torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0, torch.from_numpy(extra)]))
        dets = model.predict(xs, threshold=args.threshold, include_source_image=False)
        dets = dets if isinstance(dets, list) else [dets]
        for s, x, det in zip(batch, xs, dets):
            shape = tuple(x.shape[1:])
            masks = det.mask if det.mask is not None else np.zeros((0, *shape), bool)
            img, records = instance_image(masks, np.asarray(det.confidence), shape)
            Image.fromarray(img).save(s / (PRED_PNG + ".tmp.png"))
            (s / (PRED_PNG + ".tmp.png")).replace(s / PRED_PNG)
            (s / PRED_JSON).write_text(json.dumps({"checkpoint": str(args.checkpoint), "threshold": args.threshold, "masks": records}))
        done += len(batch)
        if done % 400 < args.batch_size or done == len(todo):
            rate = done / (time.perf_counter() - t0)
            print(f"{done}/{len(todo)} scenes, {rate:.1f}/s, {(len(todo) - done) / rate / 60:.1f} min left", flush=True)


if __name__ == "__main__":
    main()
