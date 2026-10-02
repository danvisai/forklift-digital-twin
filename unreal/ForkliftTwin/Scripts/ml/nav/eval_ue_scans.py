"""Score ClearanceNet on REAL UE lidar scans (the sim-to-UE check of M3).

Reads the dumps written by `UnrealEditor ... -ForkliftLidarDump=N` (Saved/LidarDump/sceneNNN_{roof,low}.npy + meta.csv, whose
last 72 columns are UE's exact clearance ground truth with the random floor clutter included), builds the same features the
net was trained on, and compares the net and the height-filter baseline with that ground truth.

    python eval_ue_scans.py [--weights ../../../weights/clearance_net_v2.pt]
"""
import argparse
import os

import numpy as np
import torch

import features as F
from train_perception import ClearanceNet

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=os.path.join(ROOT, "weights", "clearance_net.pt"))
    ap.add_argument("--dump", default=os.path.join(ROOT, "Saved", "LidarDump"))
    a = ap.parse_args()
    ck = torch.load(a.weights, map_location="cpu", weights_only=False)
    model = ClearanceNet(width=ck["width"])
    model.load_state_dict(ck["model"])
    model.eval()

    meta = np.genfromtxt(os.path.join(a.dump, "meta.csv"), delimiter=",", skip_header=1)
    X, Y, clutter = [], [], []
    for r in meta:
        s = int(r[0])
        roof = np.load(os.path.join(a.dump, f"scene{s:03d}_roof.npy"))
        low = np.load(os.path.join(a.dump, f"scene{s:03d}_low.npy"))
        X.append(F.build_features(roof, low))
        Y.append(r[5:77])
        clutter.append(r[4])
    X, Y, clutter = np.array(X, np.float32), np.array(Y, np.float32), np.array(clutter)
    with torch.no_grad():
        lc, _ = model(torch.from_numpy(X))
    P = ClearanceNet.to_cm(lc).numpy()
    base = F.baseline_clearance(X)

    m = Y < 1500
    print(f"{len(X)} UE scenes, {int(clutter.sum())} clutter objects")
    print(f"clearance MAE (bins < 15 m):  ClearanceNet {np.abs(P - Y)[m].mean():6.1f} cm   baseline {np.abs(base - Y)[m].mean():6.1f} cm")
    for lo, hi in ((0, 250), (250, 400), (400, 800), (800, 1500)):
        mm = (Y >= lo) & (Y < hi)
        if mm.any():
            print(f"  GT {lo:4d}-{hi:4d} cm: {mm.sum():5d} bins   net MAE {np.abs(P - Y)[mm].mean():6.1f}   baseline MAE {np.abs(base - Y)[mm].mean():6.1f}")
    near = (Y > 100) & (Y < 400)
    print(f"bins with something 1-4 m away: {near.sum()}   caught (pred < GT + 40 cm): ClearanceNet {100 * (P[near] < Y[near] + 40).mean():.1f}%   "
          f"baseline {100 * (base[near] < Y[near] + 40).mean():.1f}%")
    free = Y > 600
    print(f"false alarms (free bearing, pred < 3 m): ClearanceNet {100 * (P[free] < 300).mean():.2f}%   baseline {100 * (base[free] < 300).mean():.2f}%")


if __name__ == "__main__":
    main()
