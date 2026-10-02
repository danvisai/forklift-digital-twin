"""Train ClearanceNet (M3): per-bearing clearance + obstacle-within-3m from the multi-ring lidar features.

    python train_perception.py --epochs 30            # then watch it live:  python serve_dashboard.py  ->  http://localhost:8770
Writes Saved/NavRuns/clearance/{metrics.jsonl, curves.png, results.json} and weights/clearance_net.pt.

Model: circular 1D CNN over the 72 bearings (the bearing axis wraps), residual blocks with growing dilation, two heads:
  clearance  -> log1p(distance/100 cm), L1 loss         obstacle -> P(clearance < 300 cm), BCE loss
Scored against the geometric height-filter baseline on a held-out set: clearance error, close floor-object recall
(floor-level object is the nearest thing in the bin, 1-4 m away) and false alarms (free bearings reported blocked).
"""
import argparse
import json
import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as Fn

import features as F

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
DATA = os.path.join(ROOT, "Saved", "NavData")
RUN = os.path.join(ROOT, "Saved", "NavRuns", os.environ.get("NAV_RUN", "clearance"))
WEIGHTS = os.path.join(ROOT, "weights")


class CircConv(nn.Module):
    def __init__(self, cin, cout, k, dil=1):
        super().__init__()
        self.pad = dil * (k - 1) // 2
        self.conv = nn.Conv1d(cin, cout, k, dilation=dil)

    def forward(self, x):
        return self.conv(Fn.pad(x, (self.pad, self.pad), mode="circular"))


class Block(nn.Module):
    def __init__(self, c, dil):
        super().__init__()
        self.a, self.b = CircConv(c, c, 3, dil), CircConv(c, c, 3, 1)
        self.n1, self.n2 = nn.GroupNorm(8, c), nn.GroupNorm(8, c)

    def forward(self, x):
        h = Fn.gelu(self.n1(self.a(x)))
        return x + self.n2(self.b(h))


class ClearanceNet(nn.Module):
    def __init__(self, cin=F.N_FEAT, width=96, dilations=(1, 2, 4, 8, 1)):
        super().__init__()
        self.inp = CircConv(cin, width, 5)
        self.blocks = nn.Sequential(*[Block(width, d) for d in dilations])
        self.head = nn.Sequential(nn.GroupNorm(8, width), nn.GELU(), CircConv(width, 2, 3))

    def forward(self, x):
        o = self.head(self.blocks(Fn.gelu(self.inp(x))))
        return o[:, 0], o[:, 1]   # log-clearance, obstacle logit

    @staticmethod
    def to_cm(logc):
        return (torch.expm1(logc.clamp(min=0.0, max=np.log1p(30.0))) * 100.0).clamp(max=F.MAX_CM)


def load(split, device):
    d = np.load(os.path.join(DATA, f"perception_{split}.npz"))
    return (torch.from_numpy(d["X"]), torch.from_numpy(d["Y"]), torch.from_numpy(d["Ylow"]), d["carry"])


def load_ue(folder):
    """Real UE scans (-ForkliftLidarDump) -> features, UE's exact clearance, and a floor-object label that UE does not provide (3000)."""
    d = os.path.join(ROOT, "Saved", folder)
    meta = np.genfromtxt(os.path.join(d, "meta.csv"), delimiter=",", skip_header=1)
    X, Y = [], []
    for r in meta:
        s = int(r[0])
        X.append(F.build_features(np.load(os.path.join(d, f"scene{s:03d}_roof.npy")), np.load(os.path.join(d, f"scene{s:03d}_low.npy"))).astype(np.float16))
        Y.append(r[5:77])
    X, Y = torch.from_numpy(np.array(X)), torch.from_numpy(np.array(Y, np.float32))
    return X, Y, torch.full_like(Y, F.MAX_CM)


@torch.no_grad()
def evaluate_ue(model, X, Y, device):
    model.eval()
    P = ClearanceNet.to_cm(model(X.to(device).float())[0]).cpu().numpy()
    base = F.baseline_clearance(X.float().numpy())
    Yn = Y.numpy()
    m, near, free = Yn < 1500.0, (Yn > 100) & (Yn < 400), Yn > 600.0
    return dict(ue_mae=float(np.abs(P - Yn)[m].mean()), ue_base_mae=float(np.abs(base - Yn)[m].mean()),
                ue_recall=float((P[near] < Yn[near] + 40.0).mean()), ue_base_recall=float((base[near] < Yn[near] + 40.0).mean()),
                ue_fa=float((P[free] < 300.0).mean()), ue_base_fa=float((base[free] < 300.0).mean()))


@torch.no_grad()
def evaluate(model, X, Y, Ylow, device, batch=1024):
    model.eval()
    preds = []
    for i in range(0, len(X), batch):
        lc, _ = model(X[i:i + batch].to(device).float())
        preds.append(ClearanceNet.to_cm(lc).cpu())
    P = torch.cat(preds).numpy()
    base = F.baseline_clearance(X.float().numpy())
    Yn, Yl = Y.numpy(), Ylow.numpy()
    m = Yn < 1500.0
    near = (Yl > 100) & (Yl < 400) & (Yl <= Yn + 1.0)            # a floor-level object is the nearest thing, 1-4 m out
    free = Yn > 600.0
    front = np.zeros(F.BINS, bool); front[:int(F.BINS * 120 / 360) + 1] = True; front[-int(F.BINS * 120 / 360):] = True
    res = dict(
        mae=float(np.abs(P - Yn)[m].mean()), base_mae=float(np.abs(base - Yn)[m].mean()),
        mae_front=float(np.abs(P - Yn)[m & front[None]].mean()), mae_rear=float(np.abs(P - Yn)[m & ~front[None]].mean()),
        recall=float((P[near] < Yl[near] + 40.0).mean()), base_recall=float((base[near] < Yl[near] + 40.0).mean()),
        false_alarm=float((P[free] < 300.0).mean()), base_false_alarm=float((base[free] < 300.0).mean()),
        near_bins=int(near.sum()),
    )
    return res, P, base


def log(f, rec):
    f.write(json.dumps(rec) + "\n")
    f.flush()


def plot_curves(path, steps, losses, epochs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    ax[0, 0].plot(steps, losses, lw=0.8); ax[0, 0].set_title("train loss"); ax[0, 0].set_yscale("log")
    e = [r["epoch"] for r in epochs]
    ax[0, 1].plot(e, [r["mae"] for r in epochs], label="ClearanceNet"); ax[0, 1].plot(e, [r["base_mae"] for r in epochs], "--", label="height-filter baseline")
    ax[0, 1].set_title("clearance error (cm, bins < 15 m)"); ax[0, 1].legend()
    ax[1, 0].plot(e, [100 * r["recall"] for r in epochs], label="ClearanceNet"); ax[1, 0].plot(e, [100 * r["base_recall"] for r in epochs], "--", label="baseline")
    ax[1, 0].axhline(95, color="g", lw=0.8); ax[1, 0].set_title("close floor-object recall (%, 1-4 m)"); ax[1, 0].legend()
    ax[1, 1].plot(e, [100 * r["false_alarm"] for r in epochs], label="ClearanceNet"); ax[1, 1].plot(e, [100 * r["base_false_alarm"] for r in epochs], "--", label="baseline")
    ax[1, 1].set_title("false alarms (% of free bearings reported < 3 m)"); ax[1, 1].legend()
    for a in ax.ravel():
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--width", type=int, default=96)
    ap.add_argument("--floor_weight", type=float, default=0.0, help="extra loss weight on bins where a floor-level object is the nearest thing (< 4 m)")
    ap.add_argument("--tag", default="", help="suffix for the weights file")
    ap.add_argument("--ue_train", default="", help="Saved/<dir> of UE lidar dumps to mix into training")
    ap.add_argument("--ue_test", default="", help="Saved/<dir> of held-out UE lidar dumps to score each epoch")
    ap.add_argument("--ue_repeat", type=int, default=8, help="oversampling of the UE training scans")
    a = ap.parse_args()
    dev = torch.device(os.environ.get("NAV_DEVICE") or ("cuda" if torch.cuda.is_available() else "cpu"))
    torch.manual_seed(0)
    os.makedirs(RUN, exist_ok=True)
    os.makedirs(WEIGHTS, exist_ok=True)
    Xtr, Ytr, Yltr, _ = load("train", dev)
    Xva, Yva, Ylva, _ = load("val", dev)
    Xte, Yte, Ylte, cte = load("test", dev)
    if a.ue_train:
        Xu, Yu, Ylu = load_ue(a.ue_train)
        Xtr = torch.cat([Xtr] + [Xu] * a.ue_repeat); Ytr = torch.cat([Ytr] + [Yu] * a.ue_repeat); Yltr = torch.cat([Yltr] + [Ylu] * a.ue_repeat)
        print(f"mixed in {len(Xu)} UE scans x{a.ue_repeat}", flush=True)
    ue_eval = load_ue(a.ue_test) if a.ue_test else None
    model = ClearanceNet(width=a.width).to(dev)
    n_par = sum(p.numel() for p in model.parameters())
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    steps_per_epoch = len(Xtr) // a.batch
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=a.epochs * steps_per_epoch, pct_start=0.15)
    mfile = open(os.path.join(RUN, "metrics.jsonl"), "w")
    log(mfile, dict(type="start", params=n_par, device=str(dev), train=len(Xtr), val=len(Xva), epochs=a.epochs, t=time.time()))
    print(f"ClearanceNet {n_par / 1e3:.0f}k params on {dev}, {len(Xtr)} train / {len(Xva)} val samples", flush=True)
    Xg, Yg, Ylg = Xtr.to(dev), Ytr.to(dev), Yltr.to(dev)
    steps, losses, epochs, step = [], [], [], 0
    t0 = time.time()
    for ep in range(1, a.epochs + 1):
        model.train()
        perm = torch.randperm(len(Xg), device=dev)
        run = 0.0
        for i in range(steps_per_epoch):
            idx = perm[i * a.batch:(i + 1) * a.batch]
            x, y, yl = Xg[idx].float(), Yg[idx], Ylg[idx]
            lc, lo = model(x)
            target = torch.log1p(y / 100.0)
            w = 1.0 + 2.0 * (y < 500.0).float()                   # the close range matters most to the driver
            w = w + a.floor_weight * ((yl < 450.0) & (yl <= y + 1.0)).float()   # ...and floor-level objects most of all
            loss = (w * (lc - target).abs()).mean() + 0.3 * Fn.binary_cross_entropy_with_logits(lo, (y < 300.0).float())
            opt.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 2.0)
            opt.step()
            sched.step()
            step += 1
            run += loss.item()
            if step % 20 == 0:
                steps.append(step); losses.append(loss.item())
                log(mfile, dict(type="step", step=step, loss=loss.item(), lr=sched.get_last_lr()[0]))
        res, _, _ = evaluate(model, Xva, Yva, Ylva, dev)
        res.update(type="epoch", epoch=ep, train_loss=run / steps_per_epoch, elapsed=time.time() - t0)
        if ue_eval is not None:
            res.update(evaluate_ue(model, ue_eval[0], ue_eval[1], dev))
        epochs.append(res)
        log(mfile, res)
        print(f"ep {ep:2d}  loss {res['train_loss']:.4f}  val MAE {res['mae']:6.1f} cm (baseline {res['base_mae']:.1f})  "
              f"close-object recall {100 * res['recall']:5.1f}% (baseline {100 * res['base_recall']:.1f}%)  false alarms {100 * res['false_alarm']:.1f}%"
              + (f"  | UE scans: MAE {res['ue_mae']:.1f} (base {res['ue_base_mae']:.1f}) caught {100 * res['ue_recall']:.1f}% (base {100 * res['ue_base_recall']:.1f}%) FA {100 * res['ue_fa']:.1f}%" if ue_eval is not None else ""), flush=True)
        torch.save(dict(model=model.state_dict(), width=a.width, epoch=ep), os.path.join(WEIGHTS, f"clearance_net{a.tag}.pt"))
        plot_curves(os.path.join(RUN, "curves.png"), steps, losses, epochs)
    res, P, base = evaluate(model, Xte, Yte, Ylte, dev)
    if ue_eval is not None:
        res.update(evaluate_ue(model, ue_eval[0], ue_eval[1], dev))
    res["carry_mae"] = float(np.abs(P - Yte.numpy())[(Yte.numpy() < 1500) & (cte[:, None] == 1)].mean())
    res["epochs"] = a.epochs
    res["params"] = n_par
    json.dump(res, open(os.path.join(RUN, "results.json"), "w"), indent=1)
    log(mfile, dict(type="done", **res))
    print("TEST:", json.dumps({k: round(v, 3) if isinstance(v, float) else v for k, v in res.items()}), flush=True)


if __name__ == "__main__":
    main()
