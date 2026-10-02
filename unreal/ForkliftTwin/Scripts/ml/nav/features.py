"""Per-bearing features for the learned clearance network (single source of truth: training, UE scans and the nav server).

A scan pair (roof 16x360, low 6x240 ranges in cm, -1 = no return) becomes a (67, 72) array: 72 bearings of 5 deg from the
truck origin, circular, bearing 0 = straight ahead, + toward the right (UE convention). Per sensor channel and bearing the
nearest return contributes [valid, horizontal range / sensor max, height above the floor / 150 cm]; the last channel marks the
bearings the low lidar can see (its 240 deg forward fan). Returns inside the self/load box (tines + carried pallet) are dropped
identically everywhere, so a carried pallet never looks like an obstacle.
"""
import numpy as np

from forklift_sim import LOW, ROOF, _DIRS, lidar_dirs, BODY_Z0

BINS = 72
N_FEAT = 16 * 3 + 6 * 3 + 1
MAX_CM = 3000.0

# Self / load mask in the body frame (cm): tines x 35..215, carried pallet up to ~250, pallet + load up to 150 cm tall.
MASK_X = (35.0, 260.0)
MASK_Y = 75.0
MASK_Z = 150.0


def _dirs(cfg):
    d = _DIRS.get(id(cfg))
    if d is None:
        d = _DIRS[id(cfg)] = lidar_dirs(cfg)
    return d


def _sensor_features(r, cfg):
    """r (channels, steps) -> (channels, 3, BINS) features + (channels, BINS) min horizontal range (inf if none) + z-of-min."""
    C, S = r.shape
    d = _dirs(cfg)                                       # (C,S,3) sensor-frame unit dirs (sensor yaw == body yaw)
    mx, my, mz = cfg["pos"]
    valid = r > 0
    x = mx + r * d[..., 0]
    y = my + r * d[..., 1]
    z = BODY_Z0 + mz + r * d[..., 2]
    inmask = (x > MASK_X[0]) & (x < MASK_X[1]) & (np.abs(y) < MASK_Y) & (z < MASK_Z)
    valid &= ~inmask
    rho = np.hypot(x, y)
    bearing = np.degrees(np.arctan2(y, x)) % 360.0
    k = np.minimum((bearing / (360.0 / BINS)).astype(np.int32), BINS - 1)
    best = np.full((C, BINS), np.inf, dtype=np.float32)
    zbest = np.zeros((C, BINS), dtype=np.float32)
    ci = np.broadcast_to(np.arange(C)[:, None], (C, S))
    rr = np.where(valid, rho, np.inf).astype(np.float32)
    np.minimum.at(best, (ci.ravel(), k.ravel()), rr.ravel())
    # z of the nearest return per (channel, bin): second pass picks the ray that attains the minimum
    hit = valid & (rr == best[ci, k])
    zbest[ci[hit], k[hit]] = z[hit]
    feat = np.zeros((C, 3, BINS), dtype=np.float32)
    ok = np.isfinite(best)
    feat[:, 0] = ok
    feat[:, 1] = np.where(ok, np.minimum(best, cfg["r_max"]) / cfg["r_max"], 1.0)
    feat[:, 2] = np.clip(zbest / 150.0, -1.0, 2.0) * ok
    return feat, best, zbest


def build_features(roof, low):
    """roof (16,360), low (6,240) ranges in cm -> (N_FEAT, BINS) float32."""
    fr, _, _ = _sensor_features(roof, ROOF)
    fl, _, _ = _sensor_features(low, LOW)
    az = (np.arange(BINS) * (360.0 / BINS) + 2.5 + 180.0) % 360.0 - 180.0   # bin centre bearing in -180..180
    fov = (np.abs(az) <= LOW["h_fov"] / 2).astype(np.float32)
    return np.concatenate([fr.reshape(-1, BINS), fl.reshape(-1, BINS), fov[None]], 0)


def baseline_clearance(feat, min_height_cm=10.0):
    """Geometric height-filter baseline straight from the features: nearest return more than `min_height_cm` above the floor
    (the floor itself returns z ~ 0), 3000 where nothing qualifies. feat (..., N_FEAT, BINS) -> (..., BINS) in cm."""
    out = np.full(feat.shape[:-2] + (BINS,), MAX_CM, dtype=np.float32)
    for off, n, rmax in ((0, 16, ROOF["r_max"]), (48, 6, LOW["r_max"])):
        v = feat[..., off:off + 3 * n, :].reshape(feat.shape[:-2] + (n, 3, BINS))
        valid = (v[..., 0, :] > 0.5) & (v[..., 2, :] * 150.0 > min_height_cm)
        rho = np.where(valid, v[..., 1, :] * rmax, MAX_CM)
        out = np.minimum(out, rho.min(-2))
    return out
