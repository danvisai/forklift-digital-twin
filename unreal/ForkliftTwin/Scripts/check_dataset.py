"""QA for a dataset written by the UE synthetic data generator (Saved/Dataset/<name>).

    python Scripts/check_dataset.py Saved/Dataset/pilot                # stats only
    python Scripts/check_dataset.py Saved/Dataset/pilot --draw 12      # + overlay images in <dataset>/qa/
    python Scripts/check_dataset.py Saved/Dataset/pilot --draw 12 --depth --split train --seed 3

Prints frames and boxes per split and class, box-size statistics (how many pockets are under 8 px on their
shorter side tells you whether the image resolution is enough), the share of frames with no pallet label
(hard negatives), depth-map sanity, and image/label consistency. Overlays draw every label: pallet (green),
loaded_pallet (orange), pallet_pocket (red), tote (cyan), rack (magenta); with --depth the depth map is shown
beside the RGB image.

Needs: numpy, pillow (matplotlib only for --depth's colour map).
"""
import argparse
import json
import pathlib
import random
import sys
from collections import Counter, defaultdict

import numpy as np
from PIL import Image, ImageDraw

NAMES = ["pallet", "loaded_pallet", "pallet_pocket", "tote", "rack"]
COLORS = [(60, 220, 60), (255, 150, 30), (255, 50, 50), (40, 220, 230), (230, 60, 230)]
SPLITS = ("train", "val", "test")


def read_labels(path):
    rows = []
    if path.exists():
        for line in path.read_text().splitlines():
            parts = line.split()
            if len(parts) == 5:
                rows.append((int(parts[0]), *map(float, parts[1:])))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", type=pathlib.Path)
    ap.add_argument("--draw", type=int, default=0, help="write this many overlay images to <dataset>/qa/")
    ap.add_argument("--depth", action="store_true", help="add a depth-map panel to the overlays")
    ap.add_argument("--split", choices=SPLITS, help="draw only from this split")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    root = args.dataset

    frames = defaultdict(list)  # split -> [stem]
    for split in SPLITS:
        for p in sorted((root / "images" / split).glob("*.png")):
            frames[split].append(p.stem)
    total = sum(len(v) for v in frames.values())
    if total == 0:
        print(f"no images under {root / 'images'}")
        return 1

    counts = defaultdict(Counter)              # split -> class -> boxes
    sizes = defaultdict(list)                  # class -> [(w_px, h_px)]
    frames_no_pallet = 0
    problems = []
    img_size = None
    for split, stems in frames.items():
        for stem in stems:
            rows = read_labels(root / "labels" / split / f"{stem}.txt")
            if not rows:
                problems.append(f"{split}/{stem}: empty label file")
            if img_size is None:
                img_size = Image.open(root / "images" / split / f"{stem}.png").size
            W, H = img_size
            if not any(r[0] in (0, 1) for r in rows):
                frames_no_pallet += 1
            for cls, cx, cy, w, h in rows:
                counts[split][cls] += 1
                sizes[cls].append((w * W, h * H))
                if not (0 <= cx <= 1 and 0 <= cy <= 1 and 0 < w <= 1 and 0 < h <= 1):
                    problems.append(f"{split}/{stem}: box out of range {cls} {cx:.3f} {cy:.3f} {w:.3f} {h:.3f}")
            for kind, ext in (("depth", ".npy"), ("meta", ".json")):
                if not (root / kind / split / f"{stem}{ext}").exists():
                    problems.append(f"{split}/{stem}: missing {kind}")

    print(f"dataset {root}  image size {img_size[0]}x{img_size[1]}  frames {total}: " + ", ".join(f"{s} {len(frames[s])}" for s in SPLITS))
    header = f"{'class':16s}" + "".join(f"{s:>8s}" for s in SPLITS) + f"{'all':>8s}   size px (w x h) min / median / max   under 8px"
    print(header)
    for cls, name in enumerate(NAMES):
        per = [counts[s][cls] for s in SPLITS]
        wh = np.array(sizes[cls]) if sizes[cls] else np.zeros((0, 2))
        if len(wh):
            mn, md, mx = wh.min(0), np.median(wh, 0), wh.max(0)
            tiny = int((wh.min(1) < 8).sum())
            stat = f"{mn[0]:4.0f}x{mn[1]:<4.0f} / {md[0]:4.0f}x{md[1]:<4.0f} / {mx[0]:4.0f}x{mx[1]:<4.0f}   {tiny} ({100 * tiny / len(wh):.0f}%)"
        else:
            stat = "-"
        print(f"{cls} {name:14s}" + "".join(f"{n:8d}" for n in per) + f"{sum(per):8d}   {stat}")
    print(f"frames with no pallet / loaded_pallet label (hard negatives): {frames_no_pallet} ({100 * frames_no_pallet / total:.0f}%)")

    # Depth sanity on a few frames.
    some = [(s, st) for s, v in frames.items() for st in v[:2]][:4]
    for split, stem in some:
        d = np.load(root / "depth" / split / f"{stem}.npy").astype(np.float32)
        print(f"depth {split}/{stem}: shape {d.shape} dtype-in-file half, min {d.min():.0f} median {np.median(d):.0f} max {d.max():.0f} cm, {100 * (d >= 59999).mean():.0f}% no-geometry")
    print("problems:", "none" if not problems else f"{len(problems)} e.g. {problems[:5]}")

    if args.draw:
        rng = random.Random(args.seed)
        pool = [(s, st) for s, v in frames.items() for st in v if not args.split or s == args.split]
        pick = rng.sample(pool, min(args.draw, len(pool)))
        out = root / "qa"
        out.mkdir(exist_ok=True)
        for split, stem in pick:
            im = Image.open(root / "images" / split / f"{stem}.png").convert("RGB")
            W, H = im.size
            dr = ImageDraw.Draw(im)
            for cls, cx, cy, w, h in read_labels(root / "labels" / split / f"{stem}.txt"):
                x0, y0, x1, y1 = (cx - w / 2) * W, (cy - h / 2) * H, (cx + w / 2) * W, (cy + h / 2) * H
                dr.rectangle([x0, y0, x1, y1], outline=COLORS[cls], width=2 if cls != 2 else 1)
                if cls != 2:
                    dr.text((x0 + 2, y0 + 1), NAMES[cls], fill=COLORS[cls])
            if args.depth:
                d = np.load(root / "depth" / split / f"{stem}.npy").astype(np.float32)
                v = np.clip(d, 0, 1200) / 1200.0
                v[d >= 59999] = 1.0
                import matplotlib.cm as cm
                dimg = Image.fromarray((cm.turbo(1.0 - v)[..., :3] * 255).astype(np.uint8))
                canvas = Image.new("RGB", (W * 2, H))
                canvas.paste(im, (0, 0))
                canvas.paste(dimg, (W, 0))
                im = canvas
            im.save(out / f"{split}_{stem}.png")
        print(f"wrote {len(pick)} overlays to {out}")
    return 0 if not problems else 2


if __name__ == "__main__":
    sys.exit(main())
