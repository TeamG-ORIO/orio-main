"""Configuration for the Orio grasp-data simulation environment.

Every physical number that came out of the workcell description lives here so that
nothing is buried in code. Units are metres, kilograms, radians and degrees where named.

The defaults below can be overridden without editing code: ``config.toml`` at the repository
root (or any file passed to a script's ``--config``) holds ``[section] key = value`` entries
that map onto the dataclass tree, see :func:`load_config`.

World frame (matches the MuJoCo scene in orio-main): +x points from the left table
toward the right table, +z is up, the table tops are at z = 0.9.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path


IN = 0.0254
"""Inch to metre. The workcell was measured in inches (misc/setup-dimensions/)."""

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_FILE = REPO_ROOT / "config.toml"
"""Overrides applied by :func:`load_config` when no other file is given and this one exists."""


@dataclass
class TableCfg:
    """The arm's table, measured at 28 x 53 in (misc/setup-dimensions/table-dim.jpeg).

    The arm base defines the frame: its centre is at (x = ``RobotCfg.base_position[0]``, y = 0).
    The base's near face is 30 in from the near (-y) table edge and its side is 18 in from the
    +x edge; with a Franka base about 7.5 in across, the centre is 33.75 in from the near edge
    and 6.25 in from the -x edge.
    """

    x_min: float = -28 * IN
    x_max: float = 0.0
    y_min: float = -33.75 * IN
    y_max: float = (53 - 33.75) * IN
    top_z: float = 0.9
    """Height of the table top above the floor. The table is modelled as one solid block."""

    @property
    def size_xy(self) -> tuple[float, float]:
        return (self.x_max - self.x_min, self.y_max - self.y_min)

    @property
    def center_xy(self) -> tuple[float, float]:
        return ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)


@dataclass
class BinCfg:
    """Region of the table top where boxes are piled (misc/setup-dimensions/bin-dim.jpeg).

    The region spans the full table width and runs from the near (-y) table edge to 12 in in
    front of the arm base's near face. By default the region is only a boundary: boxes that
    leave it are culled. ``walls`` (on in config.toml) adds four walls just outside its edges.
    """

    x_min: float = -28 * IN
    x_max: float = 0.0
    y_min: float = -33.75 * IN
    y_max: float = -(3.75 + 12) * IN
    edge_buffer: float = 0.10
    """Boxes are released with their whole footprint this far inside the region's edges
    (:attr:`spawn_bounds`). Three of the region's edges are table edges; the +y edge faces the
    arm. Whether a box that slides into the buffer afterwards is kept depends on ``cull_rule``."""
    cull_rule: str = "table"
    """Which settled boxes are dropped again (``PileCfg.redrop_passes``) and then removed:

    * ``"table"``: only boxes that left the table top (centre outside the table or fallen);
      the buffer constrains release positions only, so boxes can settle near an edge;
    * ``"centre"``: also boxes whose centre is inside the buffer;
    * ``"footprint"``: also boxes any part of whose footprint is inside the buffer (strict;
      with large items and a wide buffer most of a pile gets culled)."""
    walls: bool = False
    wall_height: float = 0.15
    wall_thickness: float = 0.01
    overhang_floor_thickness: float = 0.02
    """Floor panel under any part of the region that extends past the table edge."""

    @property
    def length_x(self) -> float:
        return self.x_max - self.x_min

    @property
    def width_y(self) -> float:
        return self.y_max - self.y_min

    @property
    def center_xy(self) -> tuple[float, float]:
        return ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)

    @property
    def spawn_bounds(self) -> tuple[float, float, float, float]:
        """``(x_min, x_max, y_min, y_max)`` of the region boxes may occupy."""
        b = self.edge_buffer
        if self.x_max - self.x_min <= 2 * b or self.y_max - self.y_min <= 2 * b:
            raise ValueError(f"edge_buffer {b} m leaves no room in the {self.length_x:.2f} x {self.width_y:.2f} m bin region")
        return (self.x_min + b, self.x_max - b, self.y_min + b, self.y_max - b)


@dataclass
class CrossbarCfg:
    """``crossbar_2`` and the ``asus_camera`` box from the MuJoCo scene, in world coordinates."""

    enabled: bool = True
    """Include the bar and camera housing in the scene. They sit 0.96 m above the table and
    shadow the dome light; the real camera never sees them."""
    crossbar_center: tuple[float, float, float] = (-0.38, -0.56, 1.86)
    crossbar_size: tuple[float, float, float] = (1.44, 0.04, 0.04)
    camera_box_center: tuple[float, float, float] = (-0.30, -0.501, 1.86)
    camera_box_size: tuple[float, float, float] = (0.20, 0.055, 0.04)


@dataclass
class CameraCfg:
    """Fixed overhead camera modelled on an Asus Xtion.

    The defaults below are the MuJoCo placement (optical centre on the bottom face of the
    ``asus_camera`` box). When ``pose_calibration`` names an existing file, the pose and FOV
    solved from the real depth images (``scripts/calibrate_camera_from_depth.py``) replace them.
    """

    position: tuple[float, float, float] = (-0.30, -0.501, 1.84)
    look_at: tuple[float, float, float] | None = None
    """World point the optical axis passes through. ``None`` means the bin floor centre."""
    up_hint: tuple[float, float, float] = (0.0, -1.0, 0.0)
    """World direction that should point toward the top of the image."""
    pose_calibration: str | None = "assets/calibration/xtion_pose.json"
    """Measured ``position``, ``look_at``, ``up_hint`` and ``hfov_deg``, relative to the repository
    root. ``None`` or a missing file keeps the defaults."""
    width: int = 640
    height: int = 480
    hfov_deg: float = 58.0
    """Horizontal field of view. The vertical FOV follows from the aspect ratio (about 45 deg)."""
    depth_range: tuple[float, float] = (0.8, 3.5)
    """Nominal valid range of the real sensor, recorded in ``meta.json`` for post-processing."""
    clip_to_depth_range: bool = False
    """Write simulated depth outside ``depth_range`` as 0 (invalid), the way the sensor would.
    Off: every pixel carries its exact simulated depth and masking is left to the consumer."""
    color_calibration: str | None = "assets/calibration/xtion_color.json"
    """Camera response fitted by ``scripts/fit_color_calibration.py`` (blur, per-channel tone
    curve, noise) applied to every captured RGB image. ``None`` or a missing file disables it."""
    depth_noise: str | None = "assets/calibration/xtion_depth_noise.json"
    """Depth sensor model fitted by ``scripts/fit_depth_noise.py`` (disparity quantisation and
    noise, outline jitter, side-face and shadow dropout, invalid border) applied to the simulated
    depth; ``depth.png`` is then the noisy depth and ``depth_clean.png`` the exact one. ``None``
    or a missing file writes exact depth only."""
    depth_noise_dropout: bool = True
    """Include the invalid pixels of the sensor model (side-face and shadow dropout, border). Off
    keeps the value noise but leaves every pixel valid."""
    crop: str = "table"
    """What the written images cover. ``"none"``: the full sensor frame. ``"table"``: the
    largest rectangle that shows nothing but table top (``camera_model.table_inner_crop``),
    cut after the sensor models have been applied to the full frame; ``K`` in ``meta.json``
    is shifted accordingly and ``camera.crop`` records the box in full-frame pixels."""
    crop_inset_px: int = 0
    """Pull every side of the table crop this many pixels further inside the table polygon.
    The table edges are known to about 1 cm (6 px), see docs/TODO.md."""

    @property
    def focal_length_mm(self) -> float:
        aperture = 20.955  # Isaac default horizontal aperture, in mm
        return aperture / (2.0 * math.tan(math.radians(self.hfov_deg) / 2.0))

    def apply_pose_calibration(self) -> bool:
        """Load ``pose_calibration`` if it exists. Returns whether it was applied."""
        if not self.pose_calibration:
            return False
        path = Path(self.pose_calibration)
        if not path.is_absolute():
            path = REPO_ROOT / path
        if not path.is_file():
            return False
        d = json.loads(path.read_text())
        self.position = tuple(float(v) for v in d["position"])
        self.look_at = tuple(float(v) for v in d["look_at"])
        self.up_hint = tuple(float(v) for v in d["up_hint"])
        self.hfov_deg = float(d["hfov_deg"])
        return True


@dataclass
class SuctionEndEffectorCfg:
    """Cylinder end effector with two suction cups, mounted on the Franka flange.

    TODO(docs/TODO.md): cup radius, spacing and orientation are placeholders.
    """

    flange_offset: float = 0.107
    """Distance from the panda_link7 origin to the flange face along the link's z axis."""
    flange_yaw_deg: float = -45.0
    """Rotation about z between panda_link7 and the flange, as in the stock Panda hand."""
    cylinder_radius: float = 0.03
    cylinder_length: float = 0.13
    cylinder_mass: float = 0.73
    cup_radius: float = 0.015875
    cup_spacing: float = 0.03302
    """Centre-to-centre distance between the two cups."""
    cup_height: float = 0.02
    cup_axis_angle_deg: float = 0.0
    """Angle in the flange plane between the flange x axis and the line joining the cups."""


@dataclass
class RobotCfg:
    """The pick-and-place arm (``mj_left`` in the MuJoCo scene)."""

    base_position: tuple[float, float, float] = ((-28 + 6.25) * IN, 0.0, 0.9)
    """Base centre, 6.25 in from the -x table edge (see :class:`TableCfg`)."""
    base_yaw_deg: float = 90.0
    """In the photos the base's connector panel faces the bin (-y), so the arm's front (+x of
    its base frame) points to +y."""
    parked_joint_pos: tuple[float, ...] = (0.0, 0.0, 0.0, -2.9, 0.0, 1.5, 0.785)
    """Joint configuration held while scenes are captured: upright, elbow folded, reaching
    toward +y (away from the bin and the camera). Chosen by grid search for the fewest robot
    pixels in the camera image."""
    ee: SuctionEndEffectorCfg = field(default_factory=SuctionEndEffectorCfg)


@dataclass
class BoxCfg:
    """Random box objects. Dimensions are sampled uniformly within the ranges."""

    length_range: tuple[float, float] = (0.05, 0.30)
    width_range: tuple[float, float] = (0.05, 0.30)
    height_range: tuple[float, float] = (0.005, 0.05)
    density_range: tuple[float, float] = (100.0, 500.0)
    """kg/m^3, roughly packed cardboard boxes."""
    static_friction: float = 0.5
    dynamic_friction: float = 0.5
    friction_range: tuple[float, float] | None = None
    """When set, each body's static and dynamic friction is drawn uniformly from this range when
    the stage is built (one value for both), instead of ``static_friction`` / ``dynamic_friction``."""
    restitution: float = 0.0
    max_per_env: int = 20
    """Number of box bodies that exist in each environment. A scene uses a subset of them."""
    texture_tile_size: float = 0.3
    """Edge length in metres that one texture repeat covers on random boxes."""
    catalog_path: str = "assets/items/catalog.json"
    """Real-item catalog from ``scripts/extract_item_textures.py``, relative to the repo root.
    Missing file means random boxes only."""
    catalog_ratio: float = 0.5
    """Probability that a box slot is a catalog item rather than a random cardboard box."""
    pouch_catalog_path: str = "assets/pouches/catalog.json"
    """Real soft packages (front and back textures, estimated sizes, allowed shape kinds) from
    ``scripts/extract_item_textures.py``, relative to the repo root. Missing file means no pouches."""
    pouch_shapes_path: str = "assets/pouch_shapes/shapes.json"
    """Pouch shape library (``scripts/prepare_pouch_shapes.py``), relative to the repo root."""
    imported_catalog_path: str = "assets/imported/catalog.json"
    """Imported textured objects (``scripts/fetch_imported_objects.py``), relative to the repo
    root. Missing file means no imported objects."""
    object_mix: tuple[float, float, float] = (1.0, 0.0, 0.0)
    """Shares of the three body types in a scene: (boxes, pouches, imported objects). Normalised,
    so (1, 0, 0) is boxes only and (0.5, 0, 0.5) half boxes, half imported; a share of 0 turns that
    type off. Boxes are catalog items with probability ``catalog_ratio``, random cardboard boxes
    otherwise. A type with a share but nothing loaded is an error."""

    def object_shares(self, have_pouches: bool, have_imported: bool) -> tuple[float, float, float]:
        """``object_mix`` normalised. Raises if it is malformed or gives a share to a type with no
        items loaded (boxes are always available: random boxes need no assets)."""
        w = [float(v) for v in self.object_mix]
        if len(w) != 3 or min(w) < 0 or sum(w) <= 0:
            raise ValueError(f"boxes.object_mix must be three shares >= 0 for [boxes, pouches, imported], not all 0; got {self.object_mix}")
        for k, have, name, script in ((1, have_pouches, "pouches", "scripts/extract_item_textures.py and scripts/prepare_pouch_shapes.py"),
                                      (2, have_imported, "imported objects", "scripts/fetch_imported_objects.py")):
            if w[k] > 0 and not have:
                raise RuntimeError(f"boxes.object_mix gives {name} a share of {w[k]} but none are loaded; run {script}")
        total = sum(w)
        return tuple(v / total for v in w)
    homogeneous_fraction: float = 0.0
    """Probability that an environment is homogeneous when its stage is built: every body slot is
    a copy of one item (body type drawn with ``object_mix``), so its piles are many copies of one
    product, as in a real bin of one product type. The other environments draw every slot
    independently. 0 gives the same specs as before this option existed."""
    homogeneous_size_jitter: float = 0.02
    """Relative size jitter (each axis, uniform +-) between the copies in a homogeneous environment
    of random cardboard boxes. Catalog items, pouches and imported objects are exact copies (pouches
    still take a shape per copy)."""
    item_tint_min: float = 0.85
    """Catalog items get a random brightness tint in [item_tint_min, 1] per scene."""
    item_roughness: float = 0.15
    """Reflection roughness of the catalog items' printed faces: coated card is glossy enough to
    show the ceiling light as a soft glare patch (fitted against the 2026-10-07 captures together
    with ``LightingCfg``). Used as is when replicating the captures."""
    item_roughness_range: tuple[float, float] = (0.1, 0.3)
    """Per-scene random roughness of each catalog item in generated data."""


@dataclass
class PileCfg:
    count_range: tuple[int, int] = (5, 20)
    """Inclusive range of boxes dropped per scene."""
    drop_height: float = 0.15
    """Release height above the table top, the wall top if walls are enabled, or the pile."""
    drop_batch: int = 2
    """Boxes released together; they are stacked ``drop_spacing`` apart vertically."""
    drop_spacing: float = 0.35
    drop_interval_steps: int = 45
    """Simulation steps between releases. Boxes land before the next ones arrive, which keeps
    thin boxes from bouncing out of a 15 cm bin."""
    random_orientation: bool = True
    redrop_passes: int = 1
    """Boxes that end up outside the bin region are dropped again this many times before
    being removed from the scene. Matters without walls, where boxes can slide off."""
    settle_steps_max: int = 600
    """Steps allowed for the pile to come to rest after the last release."""
    settle_check_interval: int = 10
    """Pose change is measured every this many steps. Velocities are not used because PhysX
    reports stale values for sleeping bodies."""
    settle_pos_tol: float = 0.001
    """Max position change (m) of any pile box between checks for the pile to count as still."""
    settle_rot_tol: float = 0.005
    """Max rotation (rad) of any pile box between checks for the pile to count as still."""
    settle_quiet_checks: int = 2
    """Consecutive still checks required."""
    render_warmup_steps: int = 4
    """Simulation steps rendered after settling before the camera buffers are read."""
    arranged_fraction: float = 0.0
    """Share of scenes whose items are laid flat in rows (``orio_grasping.arrange``) instead of
    dropped into a pile. Drawn per scene, after everything else, so 0 keeps the piles of a seed."""
    arranged_gap_m: tuple[float, float] = (0.005, 0.06)
    """Range of an arranged scene's gap between neighbours (log-uniform per scene, each gap 0.5-1.5x)."""
    arranged_yaw_jitter_deg: float = 4.0
    """Std of an arranged item's turn away from square to the table."""
    arranged_margin_m: float = 0.015
    """Arranged items keep this far inside the bin region (its walls)."""


@dataclass
class LightingCfg:
    """Room lighting: an even dome light for the diffuse part and small ceiling lights for the
    highlights on glossy boxes.

    The ceiling light was located from the highlights in the 2026-10-07 captures: every
    highlight on a box top was mirrored back along its reflected ray to the ceiling (at least
    20 ft up, about 5.2 m above the table) and the spot that all eight captures agreed on kept.
    Its size, strength and the items' roughness were then fitted by rendering 16 variants at a
    time in parallel environments and comparing the glare with the real captures
    (``scripts/replicate_captures.py --variants``). Two weaker spots the trace also suggested
    made glare where the real captures have none and were dropped. The light is a disc facing
    down, limited to a cone so that it does not reach the neighbouring environments (which needs
    ``SimCfg.env_spacing`` of about 8 m or more).
    """

    dome_intensity: float = 1400.0
    dome_color: tuple[float, float, float] = (0.95, 0.95, 1.0)
    dome_specular: float = 1.0
    """How much the dome shows up in glossy reflections (USD ``inputs:specular``). The real lab
    around the boxes is dark apart from the ceiling light, so a bright, evenly reflected dome
    puts a white sheen on glossy faces that the real boxes do not have."""
    ceiling_lights_xy: tuple[float, ...] = (-1.6, -1.8)
    """World x, y of each ceiling light, flattened (x1, y1, x2, y2, ...)."""
    ceiling_light_intensity: tuple[float, ...] = (1.0,)
    """Relative strength of each light, multiplied by ``ceiling_intensity``."""
    ceiling_intensity: float = 4.94e4
    """Overall strength (USD radiance) of the ceiling lights; 0 leaves them out."""
    ceiling_height: float = 5.2
    """Metres above the table top."""
    ceiling_light_radius: float = 0.45
    ceiling_light_cone_deg: float = 30.0
    """Half-angle of each light's cone. The camera's view of the table lies within about 25 deg
    of the light; another environment's table must lie beyond the cone."""
    intensity_jitter: float = 0.3
    """Per-scene random scale of the ceiling lights in generated data, in [1 - j, 1 + j]."""
    position_jitter: float = 0.3
    """Per-scene random shift of each ceiling light in x and y, in metres, so the glare does not
    always fall on the same pixels."""


@dataclass
class RandomizationCfg:
    """Per-scene sim-to-real variation of what the calibration pins down only to a range
    (``orio_grasping/variation.py``). Every sampled value is written to ``meta.json`` under
    ``randomization``. Off by default; ``config.toml`` turns it on for generated data.

    Ranges are symmetric about the calibrated value unless they are given as ``(lo, hi)``. The
    defaults come from the calibration spread: the eight captures agree on the camera pose to
    4 mm and 0.4 deg but two calibrations differ by 15.7 mm; the driver's focal length is 3.7%
    off the fitted one; the depth scale is ambiguous by about 4% until the tape measurement.
    """

    enabled: bool = False
    nominal_fraction: float = 0.15
    """Share of capture rounds rendered with every value at its calibrated setting (all
    environments of a round together, since the dome light is shared), so that the most likely
    setup stays well covered."""

    # camera pose, rendered (true parallax)
    camera_position_radius: float = 0.008
    """Metres. The camera position is drawn uniformly from a ball of this radius about the
    calibrated one. Also covers the unmeasured table height, which looks the same in the image."""
    camera_rotation_deg: float = 0.5
    """Tilt about the camera's x and y axes (OpenCV: x right, y down), uniform in +-this."""
    camera_roll_deg: float = 1.5
    """Rotation about the optical axis, uniform in +-this. Wider than the tilts: the near table edge
    in the real captures lies about 1.2 deg off the calibrated one
    (``scripts/check_randomization_ranges.py``)."""

    # intrinsics, applied by resampling a render with a margin around the sensor frame
    focal_scale: float = 0.02
    """Relative focal length change, uniform in +-this."""
    principal_point_px: float = 6.0
    """Principal point shift in u and v, uniform in +-this."""
    distortion_k1: float = 0.02
    """First radial distortion coefficient, uniform in +-this (the simulated lens has none)."""
    render_margin_px: int = 20
    """Extra pixels rendered on every side of the sensor frame so that the resampling for the
    intrinsics above never reads outside the render."""

    # lighting
    dome_intensity_scale: float = 0.25
    """Relative change of the dome light, uniform in +-this. The dome is shared by every
    environment, so this is drawn once per capture round."""
    color_temperature_k: tuple[float, float] = (5000.0, 8000.0)
    """Colour temperature of every light, drawn once per round. 6500 K is the calibrated look;
    the range is about +-30 mired around it."""
    ceiling_scale_range: tuple[float, float] = (0.7, 1.8)
    """Strength of the ceiling lights relative to ``LightingCfg.ceiling_intensity`` (replaces
    ``LightingCfg.intensity_jitter`` when randomization is on). Skewed up: the real glare saturates
    (252 to 255) where the calibrated render reaches about 230."""
    ceiling_radius_range: tuple[float, float] = (0.3, 0.6)
    """Radius of each ceiling light (glare size); intensity is rescaled to keep its power."""
    extra_ceiling_lights: int = 2
    """Up to this many extra, weaker ceiling lights per scene (other lab lights on or off).
    Their prims exist in every environment, switched off when unused."""
    extra_light_intensity: tuple[float, float] = (0.1, 0.4)
    """Strength of an extra light relative to ``LightingCfg.ceiling_intensity``."""
    extra_light_spread: float = 2.5
    """Extra lights are placed within +-this many metres (x and y) of the bin centre."""
    occluder_probability: float = 0.25
    """Chance of a person-sized box standing beside the table, shading part of the bin from the
    dome light. Never placed where the camera crop could see it."""
    occluder_size: tuple[float, float, float] = (0.45, 0.30, 1.70)
    occluder_distance: tuple[float, float] = (0.25, 0.70)
    """Distance of the occluder's centre from the table edge, metres."""

    # table top
    table_texture_shift: bool = True
    """Shift the table texture by a random fraction of a tile in x and y. (Turning it by 180 deg
    would change nothing: the texture is a 2x2 mirrored tile.)"""
    table_tint_scale: float = 0.10
    """Relative brightness change of the table, uniform in +-this."""

    # depth sensor, applied by the writer
    depth_scale_rel: float = 0.02
    """Relative change of the sensor's depth scale, uniform in +-this, pivoting about the table
    depth so the table stays put and heights above it scale."""
    depth_offset_m: float = 0.005
    """Depth offset added on top, uniform in +-this."""
    depth_rgb_shift_px: int = 2
    """Registration error between depth and RGB: integer shift of depth.png in u and v."""
    disparity_noise_scale: tuple[float, float] = (0.7, 1.5)
    """Multiplier on the fitted disparity noise."""
    lateral_noise_scale: tuple[float, float] = (0.7, 1.5)
    """Multiplier on the fitted outline jitter."""
    steep_incidence_deg: float = 5.0
    """Change of the side-face dropout angle, uniform in +-this."""
    fill_px: float = 1.0
    """Change of the side-face and shadow fill depths, uniform in +-this."""

    # colour response, applied by the writer
    color_gain: float = 0.08
    """Relative per-channel gain change, uniform in +-this (white balance and exposure drift)."""
    color_gamma: float = 0.05
    """Relative change of the tone curve's gamma (all channels together), uniform in +-this."""
    rgb_noise_std: tuple[float, float] = (1.0, 3.0)
    """Gaussian sensor noise, 8-bit units (the fitted response has none)."""
    blur_sigma: tuple[float, float] = (0.0, 0.6)
    """Blur in pixels, replacing the fitted 0.5. The real items carry more fine detail than the
    calibrated render, so the range reaches down to no blur."""


@dataclass
class OutputCfg:
    rgb: bool = True
    rgb_raw: bool = False
    """Also write ``rgb_raw.png``, the render before colour calibration."""
    depth: bool = True
    """``depth.png``, uint16 in 0.1 mm steps (see ``dataset.DEPTH_SCALE``)."""
    point_cloud: bool = False
    """``points.npy``, the organised point cloud. Off by default: it is 3.7 MB per scene and
    ``dataset.point_cloud(depth, K)`` rebuilds it exactly."""
    segmentation: bool = True
    ground_truth: bool = True
    """``meta.json``."""
    preview: bool = True
    """``preview.png``: RGB and colourised depth side by side, for eyeballing scenes."""
    workers: int = 4
    """Background processes that apply the sensor models and encode the files (0.3 s of CPU per
    scene) while the simulator runs the next round. 0 does that work in the simulator process."""


@dataclass
class HeightMapCfg:
    """Top-down height map built from a depth image (``orio_grasping/heightmap.py``).

    The grid lies in the table plane with columns running along world -x and rows along world
    +y, so it reads in the same orientation as the camera image.
    """

    cell_size: float = 0.002
    """Edge length of one cell, metres. The Xtion sees about 1.7 mm per pixel on the table."""
    x_range: tuple[float, float] | None = None
    """World x extent of the map; ``None`` uses the scene's bin region (``workcell.bin_inner``)."""
    y_range: tuple[float, float] | None = None
    max_height: float | None = None
    """Heights are metres above the table top, clipped below at 0 and above at this value;
    ``None`` leaves them uncapped (piles can be taller than any fixed guess)."""
    gap_min_neighbours: int = 5
    """An empty cell with at least this many of its 8 neighbours measured takes their median
    and counts as measured (closes the gaps between pixel footprints)."""
    normal_radius: float = 0.016
    """Radius of the disc each cell's normal is fitted over; about one cup radius."""


@dataclass
class LabelCfg:
    """Per-cell, per-cup-angle grasp labels (``orio_grasping/labels.py``), judged on the exact
    surface from the object poses. Cup radius and spacing come from ``robot.ee``.

    Three heads per angle, each in [0, 1]: ``seal`` (both cups on one face of one object, with
    positioning slack), ``hold`` (box centre close to the grasp axis, distance along the cup line
    counting less than across it), ``access`` (nothing rests on the box, approach near vertical).
    """

    angles_deg: tuple[float, ...] = (0.0, 90.0)
    """Directions of the cup line, degrees from world +x toward +y. The cups are symmetric, so
    angles are taken modulo 180."""
    seal_margin: float = 0.006
    """Seal rises linearly from 0 when a cup's rim just touches the face's edge to 1 when every
    rim is at least this far inside it (tolerance to positioning error)."""
    min_seen_fraction: float = 0.5
    """Seal is scaled by how much of the two cup footprints the camera measured (noisy map's
    unseen mask): 0 at or below this fraction, rising linearly to 1 when all of it was seen."""
    hold_sigma: float = 0.03
    """Hold = exp(-d_eff^2 / (2 sigma^2)), metres. d_eff combines the box centre's offset from
    the grasp axis across the cup line with its offset along the line divided by
    ``hold_along_ratio``."""
    hold_along_ratio: float = 2.0
    """How much more offset along the cup line is tolerated than across it (two cups resist
    moments along their line much better than a single cup's peel across it)."""
    cover_tolerance: float = 0.02
    """A box counts as covered (access 0) when more than this fraction of its top-down outline
    has another object on top; small overlaps from boxes touching at an edge are forgiven."""
    max_tilt_deg: float = 35.0
    """Access falls as 1 - (tilt / max_tilt)^2 with the face's tilt from vertical, reaching 0 here
    (arm IK and workspace limits)."""


@dataclass
class TrainCfg:
    """Suction network and its training (``orio_grasping/model.py``, ``scripts/train.py``).

    Input: the noisy height map's channels; output per cell: seal and hold logits per cup angle
    and one access logit (access does not depend on the angle).
    """

    channels: tuple[str, ...] = ("height", "normals", "unseen")
    """Input channels, any of ``height`` (1), ``normals`` (3), ``unseen`` (1), in this order."""
    height_scale: float = 0.1
    """Heights are divided by this (metres) inside the model; fixed, never per image."""
    encoder: str = "resnet34"
    """Any segmentation_models_pytorch encoder (``resnet18``, ``resnet50``, ``tu-<timm name>``...)."""
    encoder_weights: str | None = "imagenet"
    """Pretrained encoder weights, or ``None`` to train from scratch."""
    decoder_channels: tuple[int, ...] = (256, 128, 64, 32, 16)
    refine_channels: int = 32
    """Width of the full-resolution block that sees the decoder output together with the raw input."""
    batch_size: int = 16
    epochs: int = 100
    lr_encoder: float = 3e-4
    lr_decoder: float = 1e-3
    weight_decay: float = 1e-4
    warmup_epochs: float = 1.0
    amp: str | None = "bf16"
    """Mixed precision on CUDA: ``bf16``, or ``None`` for float32."""
    num_workers: int = 8
    loss: str = "bce"
    """``bce`` (soft-target binary cross-entropy) or ``focal``."""
    focal_gamma: float = 2.0
    pos_weight_max: float = 3.0
    """Positive cells of each head are weighted by (negative / positive count) from the training
    set, capped here; 1 disables the weighting."""
    head_weights: tuple[float, float, float] = (1.0, 1.0, 1.0)
    """Loss weights of seal, hold, access."""
    success_threshold: float = 0.5
    """A chosen grasp counts as good when its label seal x hold x access is at least this."""
    val_fraction: float = 0.1
    test_fraction: float = 0.05
    """Scenes are split by a hash of run and scene name, so the split is stable as runs grow."""
    online_augment: bool = True
    """Rotate / flip every training sample at random as it is loaded (``SceneDataset``). Offline
    copies (folders with ``augment.json``) are then left out of training, so nothing is augmented
    twice. Val and test are never augmented online."""
    symmetries: str = "all"
    """What online augmentation draws from: ``all`` (the 8 symmetries of the square grid),
    ``rotations`` (4) or ``none``."""
    val_augmented: bool = False
    """Also validate on augmented copies of val scenes. Off: val holds original scenes only, as
    the fixed camera sees them, and ``augment_dataset.py`` writes copies of train scenes only."""
    log_images: int = 4
    """Validation scenes drawn to TensorBoard every epoch."""
    debug_plot_every: int = 2500
    """Optimiser steps between debug plots (``orio_grasping/train_plots.py``); 0 disables them."""
    debug_plot_scenes: int = 20
    """Number of fixed scenes in the debug plots, drawn once with ``seed``."""
    debug_plot_split: str = "test"
    """Split the debug scenes come from (original scenes only). Only drawn, never used to choose
    checkpoints; ``val`` keeps the test split unseen."""
    seed: int = 0


@dataclass
class SimCfg:
    dt: float = 1.0 / 120.0
    device: str = "cuda:0"
    num_envs: int = 4
    env_spacing: float = 4.0
    """Grid spacing between environments. Must keep every neighbour's workcell and parking
    area outside this environment's camera footprint, which at floor level is roughly
    x in [-1.3, 0.7], y in [-0.9, 0.7]."""
    parking_origin: tuple[float, float, float] = (1.0, 1.0, 0.3)
    """Where unused boxes are kept, relative to the environment origin. With a 5-column grid at
    ``parking_pitch`` this covers x in [1.0, 2.4], y in [1.0, 2.05], which no camera sees."""
    parking_pitch: float = 0.35
    antialiasing: str = "Off"
    """RTX anti-aliasing: "Off", "FXAA", "TAA", "DLSS" or "DLAA". Every mode smears the item
    print at 640x480: measured against a perfect camera on the 2026-10-07 replicas, rendered box
    tops keep 0.14 (DLSS, Isaac Lab's default) to 0.22 (DLAA) of the detail, 0.48 with "Off",
    and the real Xtion 0.62. The Xtion's own softness is then added back by the colour
    calibration's blur."""
    dl_denoiser: bool = False
    """RTX's learned denoiser (ray reconstruction), on in Isaac Lab's default "balanced" mode.
    It also smooths texture detail."""
    texture_streaming: bool = False
    """RTX texture streaming loads low-resolution mip levels first and refines them over many
    frames; a camera read a few frames after a texture change sees the blurry ones."""


@dataclass
class OrioCfg:
    table: TableCfg = field(default_factory=TableCfg)
    bin: BinCfg = field(default_factory=BinCfg)
    crossbar: CrossbarCfg = field(default_factory=CrossbarCfg)
    camera: CameraCfg = field(default_factory=CameraCfg)
    robot: RobotCfg = field(default_factory=RobotCfg)
    boxes: BoxCfg = field(default_factory=BoxCfg)
    pile: PileCfg = field(default_factory=PileCfg)
    outputs: OutputCfg = field(default_factory=OutputCfg)
    sim: SimCfg = field(default_factory=SimCfg)
    lighting: LightingCfg = field(default_factory=LightingCfg)
    randomization: RandomizationCfg = field(default_factory=RandomizationCfg)
    heightmap: HeightMapCfg = field(default_factory=HeightMapCfg)
    labels: LabelCfg = field(default_factory=LabelCfg)
    train: TrainCfg = field(default_factory=TrainCfg)
    texture_dir: str = "assets/textures"
    """Folder scanned for texture sets, relative to the repository root unless absolute."""
    table_texture: str | None = "assets/workcell/table_color.jpg"
    """Colour texture tiled over the (bare) table top, relative to the repository root. ``None``
    for a plain dark surface."""
    table_tile_size: float = 0.41
    """Real-world edge length in metres covered by one repeat of the table texture (the texture
    is a 2x2 mirrored tile of a patch about 20 cm across)."""
    table_tint: float = 0.5
    """Brightness multiplier on the table texture's (linear) albedo. The phone's HDR lifted the
    dark table in the photo the texture comes from; this brings it back down so that one tone
    curve (``scripts/fit_color_calibration.py``) fits both the boxes and the table."""

    def __post_init__(self):
        self.camera.apply_pose_calibration()

    def camera_look_at(self) -> tuple[float, float, float]:
        if self.camera.look_at is not None:
            return self.camera.look_at
        cx, cy = self.bin.center_xy
        return (cx, cy, self.table.top_z)


# --- overrides from a TOML file -----------------------------------------------------------------


def load_config(path: str | Path | None = None) -> OrioCfg:
    """The configuration with the overrides of a TOML file applied.

    ``None`` means ``config.toml`` at the repository root if it exists, otherwise the defaults.
    Sections in the file are the fields of :class:`OrioCfg` (``[boxes]``, ``[pile]``,
    ``[robot.ee]``...), keys are the fields of those dataclasses. Lists become tuples and an
    empty string means ``None``. Unknown keys raise, so typos do not pass silently.
    """
    cfg = OrioCfg()
    if path is None:
        path = DEFAULT_CONFIG_FILE if DEFAULT_CONFIG_FILE.is_file() else None
    if path is not None:
        import tomllib

        with open(path, "rb") as f:
            data = tomllib.load(f)
        apply_overrides(cfg, data, source=str(path))
    return cfg


def apply_overrides(obj, data: dict, source: str = "overrides", prefix: str = "") -> None:
    """Set the fields of dataclass ``obj`` (recursively) from a nested dict."""
    by_name = {f.name: f for f in fields(obj)}
    # a new camera pose calibration file is applied before any explicit camera values, so
    # that values written in the config win over the calibration
    if "pose_calibration" in data and hasattr(obj, "apply_pose_calibration"):
        obj.pose_calibration = _coerce(data["pose_calibration"], by_name["pose_calibration"], prefix + "pose_calibration", source)
        obj.apply_pose_calibration()
        data = {k: v for k, v in data.items() if k != "pose_calibration"}
    for key, value in data.items():
        if key not in by_name:
            raise KeyError(f"{source}: unknown setting '{prefix}{key}' (see orio_grasping/config.py)")
        current = getattr(obj, key)
        if is_dataclass(current):
            if not isinstance(value, dict):
                raise TypeError(f"{source}: '{prefix}{key}' is a section, give it as a table")
            apply_overrides(current, value, source, f"{prefix}{key}.")
        else:
            setattr(obj, key, _coerce(value, by_name[key], prefix + key, source))


def _coerce(value, f, name: str, source: str):
    """Check a TOML value against a field's annotation and convert it to the field's type."""
    kind = str(f.type)  # annotations are strings here (``from __future__ import annotations``)
    optional = "None" in kind
    if value == "" and optional:
        return None
    if kind.startswith("tuple"):
        if not isinstance(value, list):
            raise TypeError(f"{source}: '{name}' must be a list, got {value!r}")
        item = "str" if "str" in kind else "int" if "int" in kind else "float"
        return tuple(_coerce_scalar(v, item, f"{name}[{i}]", source) for i, v in enumerate(value))
    scalar = kind.replace(" | None", "")
    return _coerce_scalar(value, scalar, name, source)


def _coerce_scalar(value, kind: str, name: str, source: str):
    if kind == "bool":
        if not isinstance(value, bool):
            raise TypeError(f"{source}: '{name}' must be true or false, got {value!r}")
        return value
    if kind == "int":
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value != int(value):
            raise TypeError(f"{source}: '{name}' must be an integer, got {value!r}")
        return int(value)
    if kind == "float":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"{source}: '{name}' must be a number, got {value!r}")
        return float(value)
    if kind == "str":
        if not isinstance(value, str):
            raise TypeError(f"{source}: '{name}' must be a string, got {value!r}")
        return value
    raise TypeError(f"{source}: '{name}' has unsupported type {kind}")
