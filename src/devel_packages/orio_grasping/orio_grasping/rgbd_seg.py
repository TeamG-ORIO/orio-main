"""RF-DETR segmentation with depth: colour plus a height-above-table channel and a valid-depth channel.

The model is RF-DETR-Seg unchanged except for its first layer. The DINOv2 patch embedding
(``patch_embeddings.projection``, a 12 x 12 stride-12 convolution over RGB) gets a second
convolution, ``patch_embeddings.depth_projection``, over the two depth channels, and the two
are summed into the same patch tokens. The depth convolution starts at zero, so the untrained
model is exactly the RGB model, and it is a separate parameter so it gets its own learning rate:
RF-DETR's layer-wise decay would give the patch embedding about 2 % of the encoder rate.
The RGB weights keep their names, so RGB checkpoints (COCO or ``train_seg.py``) load as they
are, with the depth branch left at zero.

Input channels, all in [0, 1] before normalisation:

0-2. RGB, as for the RGB model (ImageNet mean and std).
3.   Height above the table top: the depth back-projected with ``K`` and ``T_world_cam`` into the
     workcell frame, minus ``table_top_z``, mapped from ``HEIGHT_RANGE`` (metres) onto [0, 1]
     and clipped. Invalid depth reads as table height (0 m).
4.   Valid depth: 1 where the sensor returned a depth, 0 where not.

Height rather than raw depth: it does not change with camera height or tilt, the table is the
same value everywhere, and sim and real share it through the calibrated camera.

:func:`install` patches RF-DETR in this process so that training and evaluation build the depth
model and read depth next to every image; nothing in ``third_party/rf-detr`` or the RGB scripts
changes. Images of a COCO export (``scripts/make_coco.py``) are symlinks to ``rgb.png`` in a scene
folder; the depth is that scene's ``depth.png`` with its ``meta.json``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from .dataset import decode_depth, point_cloud, transform_points

HEIGHT_RANGE = (-0.05, 0.45)
"""Metres above the table mapped to 0 and 1. Sim object tops reach 0.31 m in 95 % of scenes."""
RGB_MEAN, RGB_STD = (0.485, 0.456, 0.406), (0.229, 0.224, 0.225)
DEPTH_MEAN, DEPTH_STD = (0.25, 0.5), (0.25, 0.5)
"""Height: the table (0.1) normalises to -0.6, 0.3 m to 1.8. Valid: 0 and 1 to -1 and 1."""
MEAN, STD = RGB_MEAN + DEPTH_MEAN, RGB_STD + DEPTH_STD
NUM_DEPTH_CHANNELS = len(DEPTH_MEAN)


# --- depth channels ------------------------------------------------------------------------------


def height_above_table(depth_m: np.ndarray, K: np.ndarray, T_world_cam: np.ndarray, table_z: float) -> np.ndarray:
    """(H, W) float32 height of each pixel's point above the table top, metres; NaN where the
    depth is invalid (0 or NaN)."""
    d = np.nan_to_num(np.asarray(depth_m, dtype=np.float32), nan=0.0)
    h = transform_points(point_cloud(d, np.asarray(K)), np.asarray(T_world_cam))[..., 2] - table_z
    return h.astype(np.float32)


def encode_height(h: np.ndarray, keep: np.ndarray | None = None) -> np.ndarray:
    """(2, H, W) float32 in [0, 1] from a height map (metres, NaN = invalid): scaled height
    and valid depth. Pixels where ``keep`` is False count as invalid; invalid pixels read as
    table height."""
    valid = np.isfinite(h)
    if keep is not None:
        valid &= keep
    lo, hi = HEIGHT_RANGE
    scaled = np.clip((np.where(valid, h, 0.0) - lo) / (hi - lo), 0.0, 1.0)
    return np.stack([scaled, valid]).astype(np.float32)


def depth_channels(depth_m: np.ndarray, K: np.ndarray, T_world_cam: np.ndarray, table_z: float, keep: np.ndarray | None = None) -> np.ndarray:
    """(2, H, W) float32 in [0, 1]: scaled height above the table and valid depth. Pixels where
    ``keep`` is False count as invalid."""
    h = height_above_table(depth_m, K, T_world_cam, table_z)
    h[~(np.nan_to_num(np.asarray(depth_m, dtype=np.float32)) > 0)] = np.nan
    return encode_height(h, keep)


def scene_height(scene: Path) -> np.ndarray:
    """Height above the table (metres, NaN = invalid) of a generated scene, from its
    ``depth.png`` (the sensor-model depth)."""
    meta = json.loads((scene / "meta.json").read_text())
    cam = meta["camera"]
    depth = decode_depth(np.asarray(Image.open(scene / cam.get("depth_file", "depth.png"))), cam["depth_scale"])
    h = height_above_table(depth, np.asarray(cam["K"]), np.asarray(cam["T_world_cam"]), meta["workcell"]["table_top_z"])
    h[~(depth > 0)] = np.nan
    return h


def scene_depth_channels(scene: Path) -> np.ndarray:
    """:func:`depth_channels` of a generated scene folder."""
    return encode_height(scene_height(scene))


# --- training augmentation of the height ---------------------------------------------------------
#
# What the real Xtion shows and the sim sensor model does not (measured on the labelled real
# captures in misc/real-piles against 60 set_v1 scenes, 2026-10-13):
# * holes inside object tops, one up to ~1500 px or 18-40 % of a top (20261006_155751, 162514);
#   the sim model only drops side faces, projector shadows and the frame border;
# * a slow tilt / warp of the whole surface, up to ~1 cm across the bin, so the table is not at
#   one height (20261006_*: 15-19 % of table pixels more than 4 mm above its median; sim 5 %);
# * fine 1-3 px speckle on every surface, where sim noise is smooth ~10 px blobs.


@dataclass(frozen=True)
class HeightAug:
    """Probabilities and sizes of the training-time height changes (each drawn per image)."""

    holes_p: float = 0.5
    """Chance of punching holes into the depth."""
    holes_count: tuple[int, int] = (1, 4)
    """Number of holes, inclusive range."""
    hole_area_px: tuple[float, float] = (50.0, 2000.0)
    """Hole area, log-uniform, in image pixels (the real holes reach ~1500 px)."""
    hole_on_object_p: float = 0.8
    """Chance a hole is centred on an object rather than anywhere."""
    tilt_p: float = 0.5
    tilt_mm: float = 8.0
    """Largest height change of a plane tilt across the image (uniform 0..this, random direction)."""
    warp_mm: float = 3.0
    """Largest std of a smooth warp (features ~100 px)."""
    speckle_p: float = 0.5
    speckle_mm: float = 1.5
    """Largest std of the fine (1-3 px) speckle."""


def augment_height(h: np.ndarray, objects: np.ndarray | None, rng: np.random.Generator, aug: HeightAug = HeightAug()) -> np.ndarray:
    """A copy of the height map ``h`` (metres, NaN = invalid) with random real-sensor effects:
    holes (set invalid), a tilt and warp of the whole surface, and fine speckle. ``objects`` is
    the union of the object masks (where holes are preferably placed), or None."""
    from scipy import ndimage

    h = np.array(h, dtype=np.float32, copy=True)
    H, W = h.shape
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    if rng.random() < aug.tilt_p:
        angle = rng.uniform(0, 2 * np.pi)
        ramp = np.cos(angle) * xx / W + np.sin(angle) * yy / H
        ramp -= ramp.mean()
        ramp /= max(np.ptp(ramp), 1e-6)
        h += rng.uniform(0, aug.tilt_mm) * 1e-3 * ramp
        coarse = rng.normal(size=(5, 5)).astype(np.float32)
        warp = ndimage.zoom(coarse, (H / 5, W / 5), order=3)[:H, :W]
        h += rng.uniform(0, aug.warp_mm) * 1e-3 * warp / max(warp.std(), 1e-6)
    if rng.random() < aug.speckle_p:
        grain = ndimage.gaussian_filter(rng.normal(size=(H, W)).astype(np.float32), rng.uniform(0.5, 1.2))
        h += rng.uniform(0, aug.speckle_mm) * 1e-3 * grain / max(grain.std(), 1e-6)
    if rng.random() < aug.holes_p:
        on_obj = np.flatnonzero(objects) if objects is not None else np.array([], int)
        for _ in range(rng.integers(aug.holes_count[0], aug.holes_count[1] + 1)):
            if len(on_obj) and rng.random() < aug.hole_on_object_p:
                k = rng.choice(on_obj)
                cy, cx = divmod(int(k), W)
            else:
                cy, cx = rng.integers(0, H), rng.integers(0, W)
            area = np.exp(rng.uniform(*np.log(aug.hole_area_px)))
            ratio = rng.uniform(0.4, 1.0)
            a = np.sqrt(area / (np.pi * ratio))
            b = a * ratio
            t = rng.uniform(0, np.pi)
            u = (xx - cx) * np.cos(t) + (yy - cy) * np.sin(t)
            v = -(xx - cx) * np.sin(t) + (yy - cy) * np.cos(t)
            ragged = ndimage.gaussian_filter(rng.normal(size=(H, W)).astype(np.float32), 2.0)
            hole = (u / a) ** 2 + (v / b) ** 2 < 1.0 + 0.6 * ragged / max(ragged.std(), 1e-6)
            h[hole] = np.nan
    return h


def scene_of_image(path: Path) -> Path:
    """The scene folder a COCO export image belongs to (the image is a symlink to its ``rgb.png``)."""
    target = path.resolve()
    if target == path.absolute() or not (target.parent / "meta.json").is_file():
        raise FileNotFoundError(f"{path} is not a symlink into a scene folder; export with make_coco.py without --copy")
    return target.parent


# --- model ---------------------------------------------------------------------------------------


def add_depth_branch(model) -> int:
    """Give every DINOv2 patch embedding in ``model`` a zero-initialised ``depth_projection``
    and make it take ``3 + NUM_DEPTH_CHANNELS`` channels. Returns how many were changed."""
    import torch
    from torch import nn

    from rfdetr.models.backbone.dinov2_with_windowed_attn import Dinov2WithRegistersPatchEmbeddings

    class RGBDPatchEmbeddings(Dinov2WithRegistersPatchEmbeddings):
        def forward(self, pixel_values: torch.Tensor) -> torch.Tensor:
            if pixel_values.shape[1] != self.num_channels:
                raise ValueError(f"expected {self.num_channels} channels (RGB + depth), got {pixel_values.shape[1]}")
            x = self.projection(pixel_values[:, :3]) + self.depth_projection(pixel_values[:, 3:])
            return x.flatten(2).transpose(1, 2)

    count = 0
    for module in list(model.modules()):
        if type(module) is Dinov2WithRegistersPatchEmbeddings:
            rgb = module.projection
            depth = nn.Conv2d(NUM_DEPTH_CHANNELS, rgb.out_channels, kernel_size=rgb.kernel_size, stride=rgb.stride, bias=False)
            nn.init.zeros_(depth.weight)
            module.depth_projection = depth.to(device=rgb.weight.device, dtype=rgb.weight.dtype)
            module.num_channels = 3 + NUM_DEPTH_CHANNELS
            module.__class__ = RGBDPatchEmbeddings
            count += 1
    if count == 0:
        raise RuntimeError("no DINOv2 patch embedding found to add the depth branch to")
    return count


def depth_parameters(model) -> list:
    return [p for n, p in model.named_parameters() if ".depth_projection." in n]


# --- RF-DETR patches -----------------------------------------------------------------------------


_SETTINGS = {"depth_lr": None, "neutral_depth": False, "depth_dropout": 0.0, "height_aug": None}


def install(depth_lr: float | None = None, neutral_depth: bool = False, depth_dropout: float = 0.0,
            height_aug: HeightAug | None = None) -> None:
    """Patch RF-DETR in this process: models built for training or evaluation get the depth
    branch, COCO datasets yield 5-channel images, and (with ``depth_lr``) the depth branch's
    parameters train at that learning rate. ``neutral_depth`` feeds every image's depth channels
    at their normalisation mean, so the depth branch adds nothing (colour only).
    ``depth_dropout`` does that to this fraction of training images (``train/`` split only), drawn
    afresh each time an image is loaded, so the model keeps working from colour alone and treats
    depth as an extra cue. ``height_aug`` applies :func:`augment_height` to the other training
    images (holes, tilt and warp, speckle as on the real sensor). Call before ``model.train`` / ``model.evaluate``; a later call changes
    the settings, the patches go in once."""
    _SETTINGS.update(depth_lr=depth_lr, neutral_depth=neutral_depth, depth_dropout=depth_dropout, height_aug=height_aug)
    import torch
    import torchvision
    from torchvision import tv_tensors
    from torchvision.transforms.v2 import functional as F

    import rfdetr.datasets.coco as coco
    import rfdetr.training.module_model as module_model
    from rfdetr.datasets.transforms import Normalize

    if getattr(coco, "_orio_rgbd", False):
        return

    class RGBDCocoDetection(coco.CocoDetection):
        """``CocoDetection`` whose images carry the two depth channels of their scene."""

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.is_train = Path(self.root).name == "train"
            steps = getattr(self._transforms, "transforms", [])
            swapped = 0
            for i, t in enumerate(steps):
                if isinstance(t, Normalize):
                    steps[i] = Normalize(MEAN, STD)
                    swapped += 1
            if swapped != 1:
                raise RuntimeError(f"expected one Normalize in the transform pipeline, found {swapped}: {steps}")

        def __getitem__(self, idx):
            img, anns = torchvision.datasets.CocoDetection.__getitem__(self, idx)
            img, target = self.prepare(img, {"image_id": self.ids[idx], "annotations": anns})
            path = Path(self.root) / self.coco.loadImgs(self.ids[idx])[0]["file_name"]
            dropped = self.is_train and _SETTINGS["depth_dropout"] > 0 and float(torch.rand(())) < _SETTINGS["depth_dropout"]
            h = scene_height(scene_of_image(path))
            if self.is_train and not dropped and _SETTINGS["height_aug"] is not None:
                objects = target["masks"].any(0).numpy() if "masks" in target and len(target["masks"]) else None
                rng = np.random.default_rng(int(torch.randint(2**31 - 1, ())))
                h = augment_height(h, objects, rng, _SETTINGS["height_aug"])
            extra = torch.from_numpy(encode_height(h))
            if _SETTINGS["neutral_depth"] or dropped:
                extra = torch.tensor(DEPTH_MEAN, dtype=torch.float32)[:, None, None].expand_as(extra).clone()
            rgb = F.to_dtype(F.pil_to_tensor(img), torch.float32, scale=True)
            if rgb.shape[1:] != extra.shape[1:]:
                raise ValueError(f"{path}: image {tuple(rgb.shape[1:])} and depth {tuple(extra.shape[1:])} differ")
            image = tv_tensors.Image(torch.cat([rgb, extra]))
            if self._transforms is not None:
                image, target = self._transforms(image, target)
            return image, target

    build_model = module_model.build_model_from_config
    get_param_dict = module_model.get_param_dict

    def build_rgbd_model(*args, **kwargs):
        model = build_model(*args, **kwargs)
        add_depth_branch(model)
        return model

    def rgbd_param_dict(args, model):
        groups = get_param_dict(args, model)
        depth_lr = _SETTINGS["depth_lr"]
        if depth_lr is not None:
            ids = {id(p) for p in depth_parameters(model)}
            hits = [g for g in groups if any(id(p) in ids for p in (g["params"] if isinstance(g["params"], list) else [g["params"]]))]
            if len(hits) != len(ids):
                raise RuntimeError(f"depth parameters spread over {len(hits)} optimiser groups, expected {len(ids)}")
            for g in hits:
                g["lr"] = depth_lr
            print(f"rgbd_seg: depth branch ({len(hits)} parameter tensors) trains at lr {depth_lr:g}", flush=True)
        return groups

    coco.CocoDetection = RGBDCocoDetection
    module_model.build_model_from_config = build_rgbd_model
    module_model.get_param_dict = rgbd_param_dict
    coco._orio_rgbd = True


def load_model(model_class, checkpoint: str | Path):
    """An RF-DETR model object (``rfdetr.RFDETRSegLarge`` etc.) with the depth branch and the
    weights of a checkpoint trained with :func:`install`, ready for ``predict`` on 5-channel
    (C, H, W) tensors in [0, 1] (see :func:`depth_channels`)."""
    import rfdetr.inference as inference

    build = inference.build_model

    def build_rgbd(*args, **kwargs):
        model = build(*args, **kwargs)
        add_depth_branch(model)
        return model

    inference.build_model = build_rgbd
    try:
        model = model_class(pretrain_weights=str(checkpoint))
    finally:
        inference.build_model = build
    if not any(p.abs().sum() > 0 for p in depth_parameters(model.model.model)):
        raise ValueError(f"{checkpoint} has no trained depth branch (an RGB checkpoint?)")
    model.model_config.num_channels = 3 + NUM_DEPTH_CHANNELS
    model.means, model.stds = list(MEAN), list(STD)
    return model
