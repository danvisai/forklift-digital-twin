"""Generate ClearanceNet training data from the Python twin (M3): random truck pose + floor clutter (+ a carried pallet) ->
roof/low lidar scans -> (N_FEAT, 72) features, with the exact 72-bin clearance ground truth (clutter + racks + walls, NOT the
carried load) and a second ground truth for floor-level objects only (top < 30 cm), used to score close-object recall.

    python gen_perception_data.py --train 60000 --val 6000 --test 6000 --workers 6
Writes Saved/NavData/perception_{train,val,test}.npz  (X float16 (N,67,72), Y (N,72), Ylow (N,72), pose (N,3), carry (N,))
"""
import argparse
import multiprocessing as mp
import os
import time

import numpy as np

import features as F
import forklift_sim as fs

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Saved", "NavData"))


def _free_pose(rng, static, fp):
    for _ in range(200):
        x, y, yaw = rng.uniform(-1650, 1650), rng.uniform(-1050, 1050), rng.uniform(-180, 180)
        if not fp.collides(np.array([x]), np.array([y]), np.array([yaw]), static)[0]:
            return x, y, yaw
    return 0.0, 675.0, 0.0


def gen_chunk(args):
    seed, n, world_path = args
    rng = np.random.default_rng(seed)
    static, _ = fs.load_world(world_path)
    fp = fs.Footprint(margin=30.0)
    X = np.zeros((n, F.N_FEAT, F.BINS), np.float16)
    Y = np.zeros((n, F.BINS), np.float32)
    Ylow = np.zeros((n, F.BINS), np.float32)
    pose = np.zeros((n, 3), np.float32)
    carry = np.zeros(n, np.int8)
    for i in range(n):
        x, y, yaw = _free_pose(rng, static, fp)
        # far clutter everywhere, plus a few objects dropped close to the truck so the close range is well covered
        boxes = [fs.random_clutter(rng, rng.integers(0, 6), (x, y), radius=900.0, static=static)]
        if rng.random() < 0.7:
            boxes.append(fs.random_clutter(rng, rng.integers(1, 4), (x, y), radius=420.0, static=static, min_dist=90.0))
        clutter = np.concatenate(boxes) if boxes else np.zeros((0, 6), np.float32)
        if len(clutter):   # nothing inside the truck's own footprint
            keep = ~np.array([fp.collides(np.array([x]), np.array([y]), np.array([yaw]), c[None])[0] for c in clutter])
            clutter = clutter[keep]
        world = np.concatenate([static, clutter])
        scan_boxes = world
        if rng.random() < 0.3:   # carried pallet (scanned by the low lidar, masked out of the features, absent from the labels)
            c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
            cx, cy = x + 190.0 * c, y + 190.0 * s
            hx = 0.5 * (120 * abs(c) + 100 * abs(s)); hy = 0.5 * (120 * abs(s) + 100 * abs(c))
            load = np.array([[cx - hx, cy - hy, 14.0, cx + hx, cy + hy, 40.0 + rng.uniform(0, 100)]], np.float32)
            scan_boxes = np.concatenate([world, load])
            carry[i] = 1
        noise = rng.uniform(0.0, 4.0)
        drop = rng.uniform(0.0, 0.04)
        # domain randomisation of the sensor pose: the UE truck rides on springs, so its lidars sit a little off nominal height and tilt
        dz = float(np.clip(rng.normal(0.0, 2.0), -6.0, 6.0))
        tilt = (float(np.clip(rng.normal(0.0, 0.7), -2.0, 2.0)), float(np.clip(rng.normal(0.0, 0.7), -2.0, 2.0)))
        po, pl = fs.sensor_pose(x, y, yaw, fs.ROOF), fs.sensor_pose(x, y, yaw, fs.LOW)
        po[2] += dz
        pl[2] += dz
        roof = fs.ray_cast(scan_boxes, po, yaw, fs.ROOF, noise_std=noise, dropout=drop, rng=rng, tilt_deg=tilt)
        low = fs.ray_cast(scan_boxes, pl, yaw, fs.LOW, noise_std=noise, dropout=drop, rng=rng, tilt_deg=tilt)
        X[i] = F.build_features(roof, low).astype(np.float16)
        xs, ys, ya = np.array([x]), np.array([y]), np.array([yaw])
        Y[i] = fs.clearance_gt(world, xs, ys, ya)[0]
        flat = clutter[clutter[:, 5] <= 30.0] if len(clutter) else clutter
        Ylow[i] = fs.clearance_gt(flat, xs, ys, ya)[0] if len(flat) else F.MAX_CM
        pose[i] = (x, y, yaw)
    return X, Y, Ylow, pose, carry


def generate(n, seed, workers, world_path):
    chunk = max(50, n // (workers * 6))
    jobs = [(seed * 100003 + k, min(chunk, n - k * chunk), world_path) for k in range((n + chunk - 1) // chunk)]
    with mp.Pool(workers) as pool:
        parts = pool.map(gen_chunk, jobs)
    return [np.concatenate([p[j] for p in parts]) for j in range(5)]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", type=int, default=60000)
    ap.add_argument("--val", type=int, default=6000)
    ap.add_argument("--test", type=int, default=6000)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--world", default=None)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for name, n, seed in (("train", a.train, 1), ("val", a.val, 2), ("test", a.test, 3)):
        t = time.time()
        X, Y, Ylow, pose, carry = generate(n, seed, a.workers, a.world)
        np.savez(os.path.join(OUT, f"perception_{name}.npz"), X=X, Y=Y, Ylow=Ylow, pose=pose, carry=carry)
        print(f"{name}: {len(X)} samples in {time.time() - t:.0f}s  ({X.nbytes / 1e6:.0f} MB)", flush=True)
