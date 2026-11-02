"""Training samples from ``heightmap.npz`` and ``labels.npz`` (see docs/HEIGHTMAP_AND_LABELS.md).

One sample per scene: the noisy height map's channels as input, the labels as targets, and
per-target loss weights. Plain numpy apart from :class:`SceneDataset`.

Target layout, (2A + 1, H, W) for A cup angles: ``seal`` at angles 0..A-1, ``hold`` at angles
0..A-1, then ``access`` (the label is the same at every angle). Weights have the same layout:
every head is learned on every map cell (hold and access are 0 on the table: no object to hold
or reach); only padding weighs 0.

Augmentation is done online by default (``[train] online_augment``): :class:`SceneDataset`
applies a random one of the symmetries in ``[train] symmetries`` to every training sample as it
is loaded (:func:`d4_transform`, before padding), so each epoch sees every scene once in a fresh
orientation. Offline copies are still possible: :func:`augment_scene` (driven by
``scripts/augment_dataset.py``) writes rotated and flipped copies as their own scene folders,
which the loader reads like any other. The symmetries are the 8 of the square grid
(rotations by 90 deg and flips). The map's columns run along world -x and rows along world +y,
so ``np.rot90`` of the arrays is a +90 deg world rotation about z (normals turn with it, a cup
line at angle t becomes t + 90), and a left-right flip mirrors world x (nx changes sign, t
becomes 180 - t). Cup angles are taken modulo 180; the angle set must map onto itself (0 and 90,
or 0, 45, 90, 135). Every symmetry is a flip (or not) followed by k rotations.

The train / val / test split groups every folder by its original scene: an augmented copy's
``augment.json`` names its source, and without one a folder ``scene_<6 digits><suffix>`` goes
with ``scene_<6 digits>`` of the same run. Augmented copies never end up on both sides. Val and
test use the original scenes only unless ``[train] val_augmented`` is set (:func:`split_scenes`):
the copies move the fixed camera's shadows and unseen strip to sides the robot never sees.
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

import numpy as np

CHANNEL_SIZES = {"height": 1, "normals": 3, "unseen": 1, "seg": 3}
"""``seg``: on a predicted object, at an instance boundary, distance to the edge (``heightmap.seg_channels``)
from the segmentation model's masks projected into the map (``noisy_seg``, see ``scripts/predict_seg_masks.py``)."""


def input_channels(channels) -> int:
    return sum(CHANNEL_SIZES[c] for c in channels)


def split_of(name: str, val_fraction: float, test_fraction: float, seed: int = 0) -> str:
    """``train``, ``val`` or ``test`` from a stable hash of the scene's name (``run/scene``)."""
    h = int(hashlib.md5(f"{seed}:{name}".encode()).hexdigest()[:8], 16) / 0xFFFFFFFF
    if h < test_fraction:
        return "test"
    if h < test_fraction + val_fraction:
        return "val"
    return "train"


def find_scenes(runs) -> list[Path]:
    """Scene folders under the given run folders that have both ``heightmap.npz`` and ``labels.npz``."""
    out = []
    for run in runs:
        for folder in sorted(Path(run).glob("scene_*")):
            if (folder / "heightmap.npz").is_file() and (folder / "labels.npz").is_file():
                out.append(folder)
    return out


def scene_name(folder: Path) -> str:
    """The split key: ``run/scene_<index>`` of the original scene (augmented copies included)."""
    marker = Path(folder) / "augment.json"
    if marker.is_file():
        src = json.loads(marker.read_text())
        return f"{src['source_run']}/{src['source_scene']}"
    m = re.match(r"scene_\d{6}", folder.name)
    return f"{folder.parent.name}/{m.group(0) if m else folder.name}"


def is_augmented(folder: Path) -> bool:
    """True for an augmented copy written by ``augment_scene`` (it holds ``augment.json``)."""
    return (Path(folder) / "augment.json").is_file()


def split_scenes(folders, val_fraction: float, test_fraction: float, seed: int = 0, val_augmented: bool = False) -> dict:
    """``{"train", "val", "test"}`` lists of folders by :func:`split_of` on their original scene.
    Augmented copies are kept in train; in val and test only with ``val_augmented``, so by
    default those splits hold the original scenes alone (the fixed camera's view)."""
    out = {"train": [], "val": [], "test": []}
    for f in folders:
        s = split_of(scene_name(f), val_fraction, test_fraction, seed)
        if s == "train" or val_augmented or not is_augmented(f):
            out[s].append(f)
    return out


def load_sample(folder: Path, channels) -> dict:
    """Unpadded, unaugmented sample: ``input`` (C, H, W), ``target`` and ``weight`` (2A + 1, H, W),
    ``unseen`` (H, W) bool, ``angles_deg``."""
    with np.load(folder / "heightmap.npz") as hm:
        parts = []
        for c in channels:
            if c == "height":
                parts.append(hm["noisy_height"][None])
            elif c == "normals":
                parts.append(hm["noisy_normals"])
            elif c == "unseen":
                parts.append(hm["noisy_unseen"][None])
            elif c == "seg":
                if "noisy_seg" not in hm:
                    raise ValueError(f"{folder}: heightmap.npz has no noisy_seg; run predict_seg_masks.py, then make_heightmaps.py")
                from .heightmap import seg_channels

                cell = json.loads(str(hm["info"]))["grid"]["cell"]
                parts.append(seg_channels(hm["noisy_seg"], cell))
            else:
                raise ValueError(f"unknown input channel {c!r}")
        unseen = hm["noisy_unseen"].astype(bool)
        inp = np.concatenate([p.astype(np.float32) for p in parts])
    with np.load(folder / "labels.npz") as lb:
        labels = lb["labels"].astype(np.float32)  # (A, 3, H, W): seal, hold, access
        info = json.loads(str(lb["info"]))
    if info["heads"] != ["seal", "hold", "access"]:
        raise ValueError(f"{folder}: unexpected label heads {info['heads']}")
    a = labels.shape[0]
    target = np.concatenate([labels[:, 0], labels[:, 1], labels[:1, 2]])
    weight = np.ones_like(target)
    return {"input": inp, "target": target, "weight": weight, "unseen": unseen, "angles_deg": info["angles_deg"]}


def _angle_permutation(angles_deg, fn) -> list[int]:
    """perm[j] = the source angle index whose transformed angle is angles[j] (modulo 180)."""
    angles = [a % 180.0 for a in angles_deg]
    perm = []
    for j, aj in enumerate(angles):
        src = [i for i, ai in enumerate(angles) if abs(((fn(ai) - aj) + 90.0) % 180.0 - 90.0) < 1e-6]
        if len(src) != 1:
            raise ValueError(f"cup angles {angles_deg} do not map onto themselves under this symmetry")
        perm.append(src[0])
    return perm


def angle_permutation(angles_deg, k: int, flip: bool) -> list[int]:
    """Index map for per-angle arrays under the symmetry: ``new[j] = old[perm[j]]``."""
    perm = list(range(len(angles_deg)))
    if flip:
        p = _angle_permutation(angles_deg, lambda t: 180.0 - t)
        perm = [perm[i] for i in p]
    for _ in range(k % 4):
        p = _angle_permutation(angles_deg, lambda t: t + 90.0)
        perm = [perm[i] for i in p]
    return perm


def transform_grid(x: np.ndarray, k: int, flip: bool) -> np.ndarray:
    """Flip (if ``flip``, columns reversed) then rotate by ``k`` x 90 deg the last two axes."""
    if flip:
        x = np.flip(x, axis=-1)
    return np.ascontiguousarray(np.rot90(x, k % 4, axes=(-2, -1)))


def transform_normals(n: np.ndarray, k: int, flip: bool) -> np.ndarray:
    """(3, H, W) world-frame normals: moved with the grid and turned with the world."""
    out = transform_grid(n, k, flip).copy()
    if flip:
        out[0] = -out[0]
    for _ in range(k % 4):
        nx, ny = out[0].copy(), out[1].copy()
        out[0], out[1] = -ny, nx
    return out


def d4_transform(sample: dict, k: int, flip: bool, channels) -> dict:
    """Apply a left-right flip (if ``flip``) then ``k`` rotations by +90 deg to a sample from
    :func:`load_sample`, keeping the labels, angle channels and normals consistent."""
    a = len(sample["angles_deg"])
    inp = transform_grid(sample["input"], k, flip)
    if "normals" in channels:
        i = sum(CHANNEL_SIZES[c] for c in channels[: list(channels).index("normals")])
        inp[i : i + 3] = transform_normals(sample["input"][i : i + 3], k, flip)
    p = angle_permutation(sample["angles_deg"], k, flip)
    perm = p + [a + i for i in p] + [2 * a]
    out = dict(sample)
    out.update(
        input=inp,
        target=transform_grid(sample["target"], k, flip)[perm],
        weight=transform_grid(sample["weight"], k, flip)[perm],
        unseen=transform_grid(sample["unseen"], k, flip),
    )
    return out


SYMMETRIES = {
    "all": [(k, f) for f in (False, True) for k in range(4) if (k, f) != (0, False)],
    "rotations": [(1, False), (2, False), (3, False)],
}
"""Symmetries applied by ``augment_dataset.py``, as (k rotations by 90 deg, flip); the identity
is the original scene and is never written again."""


def online_symmetries(name: str) -> list[tuple[int, bool]]:
    """The identity plus ``SYMMETRIES[name]``: what online augmentation draws from uniformly
    (8 for ``all``, 4 for ``rotations``, only the identity for ``none``)."""
    if name == "none":
        return [(0, False)]
    if name not in SYMMETRIES:
        raise ValueError(f"unknown symmetries {name!r}: use one of {sorted(SYMMETRIES) + ['none']}")
    return [(0, False)] + SYMMETRIES[name]


def augment_name(source: Path, k: int, flip: bool) -> str:
    return f"{source.name}_r{90 * (k % 4):03d}{'f' if flip else ''}"


def augment_scene(src: Path, dst: Path, k: int, flip: bool, all_maps: bool = False) -> None:
    """Write the (flip, k) copy of scene folder ``src`` into ``dst``: ``heightmap.npz`` (the
    ``noisy_*`` maps the network reads, or every map with ``all_maps``), ``labels.npz`` and
    ``augment.json`` (source scene and symmetry). Grid and world-frame entries in the files'
    ``info`` describe the source scene; the copy is a virtual scene with no ``meta.json``."""
    dst.mkdir(parents=True, exist_ok=True)
    aug = {"source_run": src.parent.name, "source_scene": src.name, "k": k % 4, "rotation_deg": 90 * (k % 4), "flip": bool(flip)}
    with np.load(src / "heightmap.npz") as hm:
        prefixes = ("noisy", "clean", "exact") if all_maps else ("noisy",)
        maps = {}
        for name in hm.files:
            if name == "info" or not name.startswith(prefixes):
                continue
            x = hm[name]
            maps[name] = transform_normals(x, k, flip).astype(x.dtype) if name.endswith("_normals") else transform_grid(x, k, flip)
        info = json.loads(str(hm["info"]))
    info["augmentation"] = aug
    np.savez(dst / "heightmap.npz", info=json.dumps(info), **maps)
    with np.load(src / "labels.npz") as lb:
        info = json.loads(str(lb["info"]))
        perm = angle_permutation(info["angles_deg"], k, flip)
        labels = transform_grid(lb["labels"], k, flip)[perm]
        on_object = transform_grid(lb["on_object"], k, flip)
    info["augmentation"] = aug
    np.savez(dst / "labels.npz", labels=labels, on_object=on_object, info=json.dumps(info))
    (dst / "augment.json").write_text(json.dumps(aug, indent=1))


def pad_value(channels) -> np.ndarray:
    """Per input channel value for padding cells: flat table (height 0, normal up), unseen."""
    vals = []
    for c in channels:
        vals += {"height": [0.0], "normals": [0.0, 0.0, 1.0], "unseen": [1.0], "seg": [0.0, 0.0, 0.0]}[c]
    return np.asarray(vals, np.float32)


def pad_to(sample: dict, size: int, channels) -> dict:
    """Pad bottom and right to ``size`` x ``size``; padded cells weigh 0 and count as unseen.
    Adds ``valid`` (size, size) bool, true on real map cells."""
    c, h, w = sample["input"].shape
    if h > size or w > size:
        raise ValueError(f"map {h} x {w} does not fit in {size}")
    inp = np.empty((c, size, size), np.float32)
    inp[:] = pad_value(channels)[:, None, None]
    inp[:, :h, :w] = sample["input"]
    target = np.zeros((sample["target"].shape[0], size, size), np.float32)
    target[:, :h, :w] = sample["target"]
    weight = np.zeros_like(target)
    weight[:, :h, :w] = sample["weight"]
    unseen = np.ones((size, size), bool)
    unseen[:h, :w] = sample["unseen"]
    valid = np.zeros((size, size), bool)
    valid[:h, :w] = True
    out = dict(sample)
    out.update(input=inp, target=target, weight=weight, unseen=unseen, valid=valid)
    return out


def padded_size(h: int, w: int, multiple: int = 32) -> int:
    """Square side that fits the map in either orientation, rounded up to ``multiple``."""
    return int(np.ceil(max(h, w) / multiple) * multiple)


try:
    import torch
    from torch.utils.data import Dataset
except ImportError:  # the numpy helpers above work without torch
    torch = None
    Dataset = object


class SceneDataset(Dataset):
    """Padded samples as float32 tensors. With ``symmetries`` (a list of (k, flip), see
    :func:`online_symmetries`) every item is the scene under one of them drawn at random each
    time it is loaded (online augmentation, for training); without, the scene as rendered."""

    def __init__(self, folders, channels, symmetries=None):
        self.folders = list(folders)
        if not self.folders:
            raise ValueError("no scenes with heightmap.npz and labels.npz")
        self.channels = tuple(channels)
        self.symmetries = list(symmetries) if symmetries else None
        first = load_sample(self.folders[0], self.channels)
        self.angles_deg = first["angles_deg"]
        self.size = padded_size(*first["input"].shape[1:])

    def __len__(self) -> int:
        return len(self.folders)

    def __getitem__(self, i: int) -> dict:
        # torch's generator, not numpy's: DataLoader seeds it differently in every worker
        j = int(torch.randint(len(self.symmetries), ())) if self.symmetries else None
        return self.item(i, j)

    def item(self, i: int, symmetry: int | None = None) -> dict:
        """Item ``i`` under ``self.symmetries[symmetry]`` (None: as rendered)."""
        s = load_sample(self.folders[i], self.channels)
        if s["angles_deg"] != self.angles_deg:
            raise ValueError(f"{self.folders[i]}: cup angles {s['angles_deg']} differ from {self.angles_deg}")
        if symmetry is not None:
            k, flip = self.symmetries[symmetry]
            if (k, flip) != (0, False):
                s = d4_transform(s, k, flip, self.channels)
        s = pad_to(s, self.size, self.channels)
        return {
            "input": torch.from_numpy(s["input"]),
            "target": torch.from_numpy(s["target"]),
            "weight": torch.from_numpy(s["weight"]),
            "unseen": torch.from_numpy(s["unseen"]),
            "valid": torch.from_numpy(s["valid"]),
            "index": i,
            "symmetry": -1 if symmetry is None else symmetry,
        }

