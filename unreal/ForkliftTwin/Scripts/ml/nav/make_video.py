"""Encode the frames written by `-ForkliftRecord` (Saved/Video/<camera>/fNNNNN.jpg + captions.csv) into mp4 files.

One file per camera, plus `overview.mp4`: a 3x2 grid of chase / top / side / tine (with the detector's boxes) / lidar radar / wide, with the
mission status burned in. Detector boxes and lidar come from `sensors.jsonl` (written by -ForkliftRecord).
Uses OpenCV only (no ffmpeg needed). Usage:  python make_video.py [--fps 12] [--out DIR]
"""
import argparse
import csv
import glob
import json
import os

import cv2
import numpy as np


def caption_rows(folder):
    path = os.path.join(folder, "captions.csv")
    rows = {}
    if os.path.exists(path):
        with open(path, newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                rows[int(r["frame"])] = r
    return rows


def load_sensors(folder):
    path = os.path.join(folder, "sensors.jsonl")
    out = {}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    r = json.loads(line)
                    out[r["f"]] = r
    return out


DET_COLORS = {"pallet": (80, 220, 80), "loaded_pallet": (60, 180, 255), "pallet_pocket": (80, 80, 255), "tote": (255, 200, 80), "rack": (200, 120, 255)}


def draw_detections(img, rec):
    """Boxes from the tine detector, scaled from the frame the detector saw to this image."""
    if not rec or not rec.get("det"):
        return img
    w0, h0 = rec.get("det_wh", [0, 0])
    if w0 <= 0 or h0 <= 0:
        return img
    sx, sy = img.shape[1] / w0, img.shape[0] / h0
    for name, conf, x1, y1, x2, y2 in rec["det"]:
        col = DET_COLORS.get(name, (255, 255, 255))
        p1, p2 = (int(x1 * sx), int(y1 * sy)), (int(x2 * sx), int(y2 * sy))
        cv2.rectangle(img, p1, p2, col, 2)
        cv2.putText(img, f"{name} {int(conf * 100)}%", (p1[0], max(14, p1[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    return img


def draw_radar(rec, size=360, range_cm=1500.0):
    """Top-down lidar view, truck forward = up. Roof returns: dark green = floor, bright = above the floor; low lidar in magenta
    (it is the one that sees objects close to the truck)."""
    img = np.full((size, size, 3), 28, np.uint8)
    c = size // 2
    scale = (size / 2 - 6) / range_cm
    for r in (300, 600, 1000, 1500):
        cv2.circle(img, (c, c), int(r * scale), (60, 60, 60), 1, cv2.LINE_AA)
        cv2.putText(img, f"{r // 100}m", (c + 3, c - int(r * scale) + 11), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (110, 110, 110), 1, cv2.LINE_AA)
    cv2.line(img, (c, c), (c, 4), (60, 60, 60), 1)

    def pts(flat, sx, sy):   # sensor-frame (x forward, y right) -> pixels, forward up; sensor offset along x
        a = np.array(flat, np.float32).reshape(-1, 3)
        if len(a) == 0:
            return a, np.zeros((0, 2), int)
        px = c + ((a[:, 1]) * scale).astype(int)
        py = c - ((a[:, 0] + sx) * scale).astype(int)
        return a, np.stack([px, py], 1)

    if rec:
        a, p = pts(rec["roof"], -50.0, 0.0)
        for (px, py), z in zip(p, a[:, 2] if len(a) else []):
            if 0 <= px < size and 0 <= py < size:
                col = (40, 120, 40) if z < -100 else (90, 255, 120)   # roof z is relative to the sensor (116 cm up): < -100 ~ floor
                img[py, px] = col
        a, p = pts(rec["low"], 115.0, 0.0)
        for (px, py) in p:
            if 0 <= px < size and 0 <= py < size:
                cv2.circle(img, (px, py), 1, (255, 60, 255), -1)
    # truck outline (body -85..215 x +-55 cm)
    x0, x1 = c + int(-55 * scale), c + int(55 * scale)
    y0, y1 = c - int(215 * scale), c - int(-85 * scale)
    cv2.rectangle(img, (x0, y0), (x1, y1), (230, 230, 230), 1)
    cv2.putText(img, "lidar  green=roof  magenta=low", (6, size - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
    return img


def burn(img, lines):
    h, w = img.shape[:2]
    cv2.rectangle(img, (0, h - 22 * len(lines) - 8), (w, h), (0, 0, 0), -1)
    for i, t in enumerate(lines):
        cv2.putText(img, t[:140], (8, h - 8 - 22 * (len(lines) - 1 - i)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, "..", "..", "..", "Saved", "Video"))
    ap.add_argument("--src", default=root)
    ap.add_argument("--out", default=None)
    ap.add_argument("--fps", type=float, default=12.0)
    a = ap.parse_args()
    out = a.out or a.src
    cams = [d for d in sorted(os.listdir(a.src)) if os.path.isdir(os.path.join(a.src, d))]
    caps = caption_rows(a.src)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    written = []
    frames = {c: sorted(glob.glob(os.path.join(a.src, c, "f*.jpg"))) for c in cams}

    for c in cams:
        if not frames[c]:
            continue
        first = cv2.imread(frames[c][0])
        h, w = first.shape[:2]
        path = os.path.join(out, f"{c}.mp4")
        vw = cv2.VideoWriter(path, fourcc, a.fps, (w, h))
        for i, f in enumerate(frames[c]):
            img = cv2.imread(f)
            r = caps.get(i)
            if r:
                burn(img, [f"t={float(r['sim_t']):6.1f}s  {r['mission']}"])
            vw.write(img)
        vw.release()
        written.append(path)

    grid = [c for c in ("chase", "top", "side", "tine", "wide") if frames.get(c)]
    sensors = load_sensors(a.src)
    if len(grid) == 5:
        n = min(len(frames[c]) for c in grid)
        W, H = 640, 360
        path = os.path.join(out, "overview.mp4")
        vw = cv2.VideoWriter(path, fourcc, a.fps, (W * 3, H * 2 + 52))
        for i in range(n):
            rec = sensors.get(i)
            tiles = {}
            for c in grid:
                img = cv2.resize(cv2.imread(frames[c][i]), (W, H))
                if c == "tine":
                    draw_detections(img, rec)
                    cv2.putText(img, "tine camera + detector", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
                else:
                    cv2.putText(img, c, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
                tiles[c] = img
            radar = np.full((H, W, 3), 28, np.uint8)
            if sensors:
                radar[:, (W - H) // 2:(W - H) // 2 + H] = draw_radar(rec, size=H)
            rows = [np.hstack([tiles["chase"], tiles["top"], tiles["side"]]), np.hstack([tiles["tine"], radar, tiles["wide"]])]
            big = np.vstack(rows)
            bar = np.zeros((52, big.shape[1], 3), np.uint8)
            r = caps.get(i)
            if r:
                burn(bar, [f"t={float(r['sim_t']):6.1f}s  {r['mission']}", r["nav"] or r["pickup"]])
            vw.write(np.vstack([big, bar]))
        vw.release()
        written.append(path)
    for p in written:
        print(p, round(os.path.getsize(p) / 1e6, 1), "MB")


if __name__ == "__main__":
    main()
