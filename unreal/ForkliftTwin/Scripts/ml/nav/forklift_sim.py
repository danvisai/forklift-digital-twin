"""Fast numpy twin of the UE warehouse for the learned lidar perception and navigation work (M2 of the plan).

World (UE units: cm, yaw deg from +X toward +Y), loaded from `world.json` that `-ForkliftLidarDump` writes next to the
dumps (static obstacle boxes = racks + outer walls, loose pallets, drop slots). Everything here is vectorised numpy:

  * `ray_cast`        - 3D slab ray casting of a multi-channel lidar against axis-aligned boxes + the floor plane
  * `clearance_gt`    - exact per-bearing free distance in 2D from the truck origin (same definition as the UE dump)
  * `Clutter`         - random floor-level objects (flat pallets, totes, cartons, posts) so close objects matter
  * `Footprint`       - truck outline (body + tines + optional carried pallet) and box collision tests
  * `step_vehicle`    - the kinematics identified from UE in vehicle_model.py, with per-env parameter scaling

The racks are solid boxes here but open frames in UE, so the sim is pessimistic around them; walls and clutter are exact.
"""
import json
import os

import numpy as np

from vehicle_model import PARAMS, step as _vehicle_step

# ---- sensor mounts (body frame, cm) and scan patterns, mirrored from AForkliftVehicle ------------------------------
ROOF = dict(pos=(-50.0, 0.0, 116.0), channels=16, steps=360, v_min=-25.0, v_max=10.0, h_fov=360.0, r_min=30.0, r_max=3000.0)
LOW = dict(pos=(115.0, 0.0, 32.0), channels=6, steps=240, v_min=-12.0, v_max=3.0, h_fov=240.0, r_min=30.0, r_max=1500.0)

BODY_Z0 = 0.0  # truck origin height above the floor in the sim (UE settles it at ~0-5 cm)


def load_world(path=None):
    """Static obstacle boxes as (N,6) [xmin,ymin,zmin,xmax,ymax,zmax] plus the raw json dict."""
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.normpath(os.path.join(here, "..", "..", "..", "Saved", "LidarDump", "world.json"))
    with open(path) as f:
        w = json.load(f)
    boxes = np.array(w["obstacles"], dtype=np.float32).reshape(-1, 6)
    return boxes, w


def lidar_dirs(cfg):
    """Unit ray directions in the sensor frame, shape (channels, steps, 3). UE convention: azimuth + toward +Y."""
    c, s = cfg["channels"], cfg["steps"]
    elev = np.linspace(cfg["v_min"], cfg["v_max"], c) if c > 1 else np.array([0.5 * (cfg["v_min"] + cfg["v_max"])])
    full = cfg["h_fov"] >= 359.99
    az = np.linspace(-cfg["h_fov"] / 2, cfg["h_fov"] / 2, s, endpoint=not full) if s > 1 else np.zeros(1)
    el, a = np.meshgrid(np.radians(elev), np.radians(az), indexing="ij")
    return np.stack([np.cos(el) * np.cos(a), np.cos(el) * np.sin(a), np.sin(el)], -1).astype(np.float32)


_DIRS = {id(ROOF): lidar_dirs(ROOF), id(LOW): lidar_dirs(LOW)}


def sensor_pose(x, y, yaw_deg, cfg):
    """World position of a mounted sensor for a truck pose."""
    yaw = np.radians(yaw_deg)
    mx, my, mz = cfg["pos"]
    return np.array([x + mx * np.cos(yaw) - my * np.sin(yaw), y + mx * np.sin(yaw) + my * np.cos(yaw), BODY_Z0 + mz], dtype=np.float32)


def ray_cast(boxes, origin, yaw_deg, cfg, noise_std=0.0, dropout=0.0, rng=None, tilt_deg=(0.0, 0.0)):
    """Ranges (channels, steps) in cm, -1 = no return. boxes (M,6); origin world xyz; the truck's own body is ignored.
    tilt_deg = (pitch, roll) of the body: the real truck is never perfectly level (UE floor rings vary ~+-3 cm around the circle)."""
    d = _DIRS.get(id(cfg))
    if d is None:
        d = _DIRS[id(cfg)] = lidar_dirs(cfg)
    pitch, roll = np.radians(tilt_deg[0]), np.radians(tilt_deg[1])
    if pitch != 0.0 or roll != 0.0:                        # body pitch (about Y) and roll (about X): the suspension tilts the sensors
        cp, sp, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
        Rm = np.array([[cp, sp * sr, sp * cr], [0.0, cr, -sr], [-sp, cp * sr, cp * cr]], np.float32)
        d = d @ Rm.T
    yaw = np.radians(yaw_deg)
    cy, sy = np.cos(yaw), np.sin(yaw)
    dw = np.empty_like(d)                                  # rotate sensor-frame dirs into the world by the truck yaw
    dw[..., 0] = d[..., 0] * cy - d[..., 1] * sy
    dw[..., 1] = d[..., 0] * sy + d[..., 1] * cy
    dw[..., 2] = d[..., 2]
    flat = dw.reshape(-1, 3)                               # (R,3)
    inv = 1.0 / np.where(np.abs(flat) < 1e-9, 1e-9, flat)  # (R,3)
    lo = (boxes[None, :, :3] - origin[None, None, :]) * inv[:, None, :]   # (R,M,3)
    hi = (boxes[None, :, 3:] - origin[None, None, :]) * inv[:, None, :]
    tmin = np.minimum(lo, hi).max(-1)
    tmax = np.maximum(lo, hi).min(-1)
    hit = (tmax >= np.maximum(tmin, 0.0))
    t = np.where(hit, np.maximum(tmin, 0.0), np.inf).min(-1)               # (R,)
    # floor plane z = 0
    dz = flat[:, 2]
    tf = np.where(dz < -1e-6, -origin[2] / np.where(dz < -1e-6, dz, -1.0), np.inf)
    t = np.minimum(t, tf)
    ok = (t >= cfg["r_min"]) & (t <= cfg["r_max"])
    r = np.where(ok, t, -1.0).astype(np.float32)
    if noise_std > 0 or dropout > 0:
        rng = rng or np.random.default_rng()
        if noise_std > 0:
            r = np.where(r > 0, np.clip(r + rng.normal(0, noise_std, r.shape), cfg["r_min"], cfg["r_max"]), r).astype(np.float32)
        if dropout > 0:
            r = np.where(rng.random(r.shape) < dropout, -1.0, r).astype(np.float32)
    return r.reshape(cfg["channels"], cfg["steps"])


def clearance_gt(boxes, x, y, yaw_deg, bins=72, max_cm=3000.0, z_floor=2.0):
    """Exact free distance per bearing from the truck origin, vectorised over poses.

    x, y, yaw_deg: (B,); boxes: (B,M,6) or (M,6). Returns (B, bins), bearing k = yaw + k*360/bins toward +Y.
    Boxes whose top is below z_floor are ignored (flat markings). Same definition as the UE dump's ground truth.
    """
    x, y, yaw_deg = np.atleast_1d(x), np.atleast_1d(y), np.atleast_1d(yaw_deg)
    if boxes.ndim == 2:
        boxes = np.broadcast_to(boxes, (len(x),) + boxes.shape)
    ang = np.radians(yaw_deg[:, None] + np.arange(bins)[None, :] * (360.0 / bins))                  # (B,K)
    dx, dy = np.cos(ang), np.sin(ang)
    inv_x = 1.0 / np.where(np.abs(dx) < 1e-9, 1e-9, dx)
    inv_y = 1.0 / np.where(np.abs(dy) < 1e-9, 1e-9, dy)
    bx0, by0, bx1, by1 = (boxes[:, None, :, i] for i in (0, 1, 3, 4))                              # (B,1,M)
    tx0, tx1 = (bx0 - x[:, None, None]) * inv_x[:, :, None], (bx1 - x[:, None, None]) * inv_x[:, :, None]
    ty0, ty1 = (by0 - y[:, None, None]) * inv_y[:, :, None], (by1 - y[:, None, None]) * inv_y[:, :, None]
    tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
    tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
    valid = (boxes[:, None, :, 5] > z_floor) & (tmax >= np.maximum(tmin, 0.0))
    t = np.where(valid, np.maximum(tmin, 0.0), np.inf).min(-1)
    return np.minimum(t, max_cm).astype(np.float32)


# ---- random floor-level clutter -----------------------------------------------------------------------------------
CLUTTER_KINDS = [  # (probability, size x, y, z in cm)
    (0.40, (120.0, 100.0, 14.0)),   # flat pallet
    (0.35, (60.0, 40.0, 28.0)),     # tote
    (0.15, (50.0, 40.0, 50.0)),     # carton
    (0.10, (30.0, 30.0, 180.0)),    # post / person-sized
]


def random_clutter(rng, n, center_xy, radius=900.0, bounds=((-1700.0, 1700.0), (-1100.0, 1100.0)), static=None, min_dist=150.0):
    """(n,6) boxes scattered around `center_xy`, kept inside the room and off the static boxes (like the UE dump)."""
    probs = np.array([k[0] for k in CLUTTER_KINDS])
    out = []
    for _ in range(n * 4):
        if len(out) >= n:
            break
        kind = CLUTTER_KINDS[rng.choice(len(CLUTTER_KINDS), p=probs / probs.sum())][1]
        ang, dist = rng.uniform(-np.pi, np.pi), rng.uniform(min_dist, radius)
        cx, cy = center_xy[0] + dist * np.cos(ang), center_xy[1] + dist * np.sin(ang)
        yaw = np.radians(rng.uniform(0, 180))
        hx = 0.5 * (kind[0] * abs(np.cos(yaw)) + kind[1] * abs(np.sin(yaw)))
        hy = 0.5 * (kind[0] * abs(np.sin(yaw)) + kind[1] * abs(np.cos(yaw)))
        if not (bounds[0][0] < cx < bounds[0][1] and bounds[1][0] < cy < bounds[1][1]):
            continue
        box = np.array([cx - hx, cy - hy, 0.0, cx + hx, cy + hy, kind[2]], dtype=np.float32)
        if static is not None and np.any((box[0] < static[:, 3]) & (box[3] > static[:, 0]) & (box[1] < static[:, 4]) & (box[4] > static[:, 1])):
            continue
        out.append(box)
    return np.array(out, dtype=np.float32).reshape(-1, 6)


# ---- truck footprint and collisions -------------------------------------------------------------------------------
class Footprint:
    """Truck outline in the body frame: x -85..+215 (tines) by y +-55; a carried pallet extends it to the pallet's far edge."""

    def __init__(self, carrying=False, margin=10.0):
        xs = np.linspace(-85.0, 215.0, 16)
        ys = np.linspace(-55.0, 55.0, 6)
        pts = np.stack(np.meshgrid(xs, ys, indexing="ij"), -1).reshape(-1, 2)
        if carrying:   # pallet 120 x 100 sitting on the tines, its far edge ~ 25 cm beyond the tips
            px, py = np.meshgrid(np.linspace(130.0, 240.0, 6), np.linspace(-50.0, 50.0, 6), indexing="ij")
            pts = np.concatenate([pts, np.stack([px, py], -1).reshape(-1, 2)])
        self.pts, self.margin = pts.astype(np.float32), margin

    def world_points(self, x, y, yaw_deg):
        yaw = np.radians(yaw_deg)[:, None]
        c, s = np.cos(yaw), np.sin(yaw)
        wx = x[:, None] + self.pts[None, :, 0] * c - self.pts[None, :, 1] * s
        wy = y[:, None] + self.pts[None, :, 0] * s + self.pts[None, :, 1] * c
        return wx, wy

    def collides(self, x, y, yaw_deg, boxes):
        """boxes (B,M,6) or (M,6) -> bool (B,). Footprint points within `margin` of any box with top > 2 cm."""
        if boxes.ndim == 2:
            boxes = np.broadcast_to(boxes, (len(x),) + boxes.shape)
        wx, wy = self.world_points(np.asarray(x), np.asarray(y), np.asarray(yaw_deg))
        m = self.margin
        inx = (wx[:, :, None] > boxes[:, None, :, 0] - m) & (wx[:, :, None] < boxes[:, None, :, 3] + m)
        iny = (wy[:, :, None] > boxes[:, None, :, 1] - m) & (wy[:, :, None] < boxes[:, None, :, 4] + m)
        tall = boxes[:, None, :, 5] > 2.0
        return (inx & iny & tall).any((1, 2))


def step_vehicle(state, throttle, steer, dt, speed_scale=1.0):
    """state (5,B) = x, y, yaw_deg, v_drive, delta_deg -> next state. speed_scale (B,) randomises top speed (+-12% in training)."""
    p = dict(PARAMS)
    p["vmax"] = PARAMS["vmax"] * np.asarray(speed_scale)
    return _vehicle_step(state, throttle, steer, dt, p)


if __name__ == "__main__":
    # Quick self-check: scan the UE dump poses that had no clutter and compare to the UE ranges.
    import sys
    boxes, world = load_world()
    here = os.path.dirname(os.path.abspath(__file__))
    dump = os.path.normpath(os.path.join(here, "..", "..", "..", "Saved", "LidarDump"))
    meta = np.genfromtxt(os.path.join(dump, "meta.csv"), delimiter=",", skip_header=1)
    shown = 0
    for r in meta:
        if r[4] != 0 or shown >= 8:
            continue
        s = int(r[0])
        ue = np.load(os.path.join(dump, f"scene{s:03d}_roof.npy"))
        sim = ray_cast(boxes, sensor_pose(r[1], r[2], r[3], ROOF), r[3], ROOF)
        both = (ue > 0) & (sim > 0)
        err = np.abs(ue - sim)[both]
        gt = clearance_gt(boxes, np.array([r[1]]), np.array([r[2]]), np.array([r[3]]))[0]
        print(f"scene {s:3d}: roof rays both-hit {both.mean():.2f}  median |err| {np.median(err):6.1f} cm  p90 {np.percentile(err, 90):6.1f} cm   "
              f"clearance sim-vs-ue median |err| {np.median(np.abs(gt - r[5:77])):.1f} cm")
        shown += 1
    sys.exit(0)
