"""The suction network: a segmentation_models_pytorch U-Net with a full-resolution refinement.

smp's U-Net decoder has no skip connection at the input resolution (its last block upsamples
from the encoder's stride-2 features), and a ResNet stem halves the resolution straight away.
Seal labels hinge on a cup rim being a few millimetres inside a face edge, which is one or two
2 mm cells, so the decoder output is concatenated with the raw input at full resolution and
passed through two 3x3 convolutions before the output layer.

Input: raw height-map channels as built by ``heightmap.network_input`` (heights in metres);
the fixed scaling lives in the model so the robot feeds the same arrays. The spatial size must
be a multiple of 32. Output: (B, 2A + 1, H, W) logits, ``seal`` at angles 0..A-1, ``hold`` at
angles 0..A-1, then ``access``; :func:`grasp_scores` turns them into seal x hold x access.
"""

from __future__ import annotations

import numpy as np
import torch
from torch import nn

from .config import TrainCfg
from .train_data import CHANNEL_SIZES, input_channels, pad_value, padded_size


class SuctionNet(nn.Module):
    def __init__(self, cfg: TrainCfg, n_angles: int, encoder_weights: str | None = "config"):
        super().__init__()
        import segmentation_models_pytorch as smp

        weights = cfg.encoder_weights if encoder_weights == "config" else encoder_weights
        self.channels = tuple(cfg.channels)
        self.n_angles = n_angles
        c_in = input_channels(self.channels)
        unet = smp.Unet(
            encoder_name=cfg.encoder,
            encoder_weights=weights,
            in_channels=c_in,
            classes=1,
            decoder_channels=tuple(cfg.decoder_channels),
        )
        self.encoder, self.decoder = unet.encoder, unet.decoder
        r = cfg.refine_channels
        self.refine = nn.Sequential(
            nn.Conv2d(cfg.decoder_channels[-1] + c_in, r, 3, padding=1, bias=False),
            nn.BatchNorm2d(r),
            nn.ReLU(inplace=True),
            nn.Conv2d(r, r, 3, padding=1, bias=False),
            nn.BatchNorm2d(r),
            nn.ReLU(inplace=True),
        )
        self.head = nn.Conv2d(r, 2 * n_angles + 1, 1)
        scale = []
        for c in self.channels:
            scale += [1.0 / cfg.height_scale] if c == "height" else [1.0] * CHANNEL_SIZES[c]
        self.register_buffer("input_scale", torch.tensor(scale).view(1, -1, 1, 1), persistent=False)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x * self.input_scale
        features = self.encoder(x)
        d = self.decoder(features)
        return self.head(self.refine(torch.cat([d, x], dim=1)))

    def encoder_parameters(self):
        return self.encoder.parameters()

    def other_parameters(self):
        enc = {id(p) for p in self.encoder.parameters()}
        return [p for p in self.parameters() if id(p) not in enc]


def split_logits(logits: torch.Tensor, n_angles: int) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """(B, 2A + 1, H, W) to seal (B, A, H, W), hold (B, A, H, W), access (B, 1, H, W)."""
    a = n_angles
    return logits[:, :a], logits[:, a : 2 * a], logits[:, 2 * a :]


def grasp_scores(logits: torch.Tensor, n_angles: int) -> torch.Tensor:
    """(B, A, H, W) seal x hold x access probabilities."""
    seal, hold, access = split_logits(logits, n_angles)
    return torch.sigmoid(seal) * torch.sigmoid(hold) * torch.sigmoid(access)


def target_scores(target: torch.Tensor, n_angles: int) -> torch.Tensor:
    """(B, A, H, W) seal x hold x access of the labels."""
    seal, hold, access = split_logits(target, n_angles)
    return seal * hold * access


@torch.no_grad()
def predict(model: SuctionNet, net_input: np.ndarray, device: str = "cpu") -> dict:
    """Run the model on one unpadded (C, H, W) input from ``heightmap.network_input``.

    Returns float32 numpy maps cropped back to (H, W): ``score`` (A, H, W) seal x hold x access,
    and ``seal``, ``hold`` (A, H, W) and ``access`` (H, W) probabilities."""
    model.eval()
    c, h, w = net_input.shape
    size = padded_size(h, w)
    x = np.empty((c, size, size), np.float32)
    x[:] = pad_value(model.channels)[:, None, None]
    x[:, :h, :w] = net_input
    logits = model(torch.from_numpy(x)[None].to(device)).float()
    seal, hold, access = (torch.sigmoid(t)[0, :, :h, :w].cpu().numpy() for t in split_logits(logits, model.n_angles))
    return {"score": seal * hold * access, "seal": seal, "hold": hold, "access": access[0]}


def load_checkpoint(path, device: str = "cpu") -> tuple[SuctionNet, dict]:
    """A model from ``scripts/train.py``'s ``best.pt`` or ``last.pt``, and the checkpoint dict."""
    ckpt = torch.load(path, map_location=device, weights_only=False)
    cfg = TrainCfg(**ckpt["train_cfg"])
    model = SuctionNet(cfg, ckpt["n_angles"], encoder_weights=None)
    model.load_state_dict(ckpt["model"])
    return model.to(device).eval(), ckpt

