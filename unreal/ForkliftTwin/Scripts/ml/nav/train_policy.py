"""PPO for the learned local navigation policy (M4), written directly on torch (no gym / SB3 dependency).

    python train_policy.py --iters 600            # live curves:  python serve_dashboard.py --run policy --kind policy
Writes Saved/NavRuns/policy/metrics.jsonl and weights/nav_policy.pt. 128 parallel numpy envs (forklift_env.BatchEnv), MLP 256x2,
curriculum over goal distance / clutter / heading requirement / carrying (forklift_env.LEVELS), advancing when the recent
success rate passes --advance. The classical controller's success on each level is logged for reference.
"""
import argparse
import json
import os
import time

import numpy as np
import torch
import torch.nn as nn

import forklift_env as E

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
RUN = os.path.join(ROOT, "Saved", "NavRuns", os.environ.get("NAV_RUN", "policy"))
WEIGHTS = os.path.join(ROOT, "weights")


class RunningNorm:
    def __init__(self, dim):
        self.mean, self.var, self.count = np.zeros(dim), np.ones(dim), 1e-4

    def update(self, x):
        bm, bv, bn = x.mean(0), x.var(0), len(x)
        d = bm - self.mean
        tot = self.count + bn
        self.mean = self.mean + d * bn / tot
        self.var = (self.var * self.count + bv * bn + d ** 2 * self.count * bn / tot) / tot
        self.count = tot

    def __call__(self, x):
        return np.clip((x - self.mean) / np.sqrt(self.var + 1e-6), -8, 8).astype(np.float32)


class CircConv1d(nn.Module):
    def __init__(self, cin, cout, k, dil=1):
        super().__init__()
        self.pad = dil * (k - 1) // 2
        self.conv = nn.Conv1d(cin, cout, k, dilation=dil)

    def forward(self, x):
        return self.conv(nn.functional.pad(x, (self.pad, self.pad), mode="circular"))


class Encoder(nn.Module):
    """Circular conv over the 72 clearance bins (weights shared across bearings, so 'something close ahead -> steer away' is
    learned once for every direction) concatenated with the goal / state features, then an MLP."""
    def __init__(self, out_dim, hidden=256):
        super().__init__()
        self.conv = nn.Sequential(CircConv1d(1, 16, 5), nn.Tanh(), CircConv1d(16, 32, 5, 2), nn.Tanh(), CircConv1d(32, 32, 3, 4), nn.Tanh())
        self.proj = nn.Sequential(nn.Linear(32 * 72, 128), nn.Tanh())
        self.mlp = nn.Sequential(nn.Linear(128 + E.OBS_DIM - 72, hidden), nn.Tanh(), nn.Linear(hidden, hidden), nn.Tanh(), nn.Linear(hidden, out_dim))

    def forward(self, o):
        c = self.proj(self.conv(o[:, None, :72]).flatten(1))
        return self.mlp(torch.cat([c, o[:, 72:]], 1))


class ActorCritic(nn.Module):
    def __init__(self, obs_dim=E.OBS_DIM, hidden=256):
        super().__init__()
        self.pi = Encoder(2, hidden)
        self.v = Encoder(1, hidden)
        self.log_std = nn.Parameter(torch.full((2,), -0.7))

    def dist(self, o):
        return torch.distributions.Normal(self.pi(o), self.log_std.exp())

    def value(self, o):
        return self.v(o).squeeze(-1)


def log(f, rec):
    f.write(json.dumps(rec) + "\n")
    f.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=600)
    ap.add_argument("--envs", type=int, default=128)
    ap.add_argument("--horizon", type=int, default=128)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--advance", type=float, default=0.85)
    ap.add_argument("--min_level_iters", type=int, default=25)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--threads", type=int, default=8)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    a = ap.parse_args()
    torch.set_num_threads(a.threads)
    torch.manual_seed(a.seed)
    os.makedirs(RUN, exist_ok=True)
    os.makedirs(WEIGHTS, exist_ok=True)
    env = E.BatchEnv(a.envs, seed=a.seed, level=0)
    dev = torch.device(a.device)
    net = ActorCritic().to(dev)
    opt = torch.optim.Adam(net.parameters(), lr=a.lr, eps=1e-5)
    norm = RunningNorm(E.OBS_DIM)
    mf = open(os.path.join(RUN, "metrics.jsonl"), "w")
    base = {lv: E.run_classical(lv, episodes=192, seed=100 + lv) for lv in range(len(E.LEVELS))}
    log(mf, dict(type="start", levels=len(E.LEVELS), baseline=base, envs=a.envs, t=time.time()))
    print("classical baseline per level:", {k: (round(v["success"], 2), round(v["collision"], 2)) for k, v in base.items()}, flush=True)

    obs = env.obs()
    B, T = a.envs, a.horizon
    steps_total, level, level_iters = 0, 0, 0
    recent = []       # recent episode outcomes at the current level: (success, collided, timeout, time_s)
    best = -1.0
    t0 = time.time()
    for it in range(1, a.iters + 1):
        O = np.zeros((T, B, E.OBS_DIM), np.float32); A = np.zeros((T, B, 2), np.float32); LP = np.zeros((T, B), np.float32)
        R = np.zeros((T, B), np.float32); D = np.zeros((T, B), np.float32); V = np.zeros((T + 1, B), np.float32)
        ep_r = []
        tr = time.time()
        for k in range(T):
            norm.update(obs)
            o = torch.from_numpy(norm(obs)).to(dev)
            with torch.no_grad():
                dist = net.dist(o)
                act = dist.sample()
                lp = dist.log_prob(act).sum(-1)
                V[k] = net.value(o).cpu().numpy()
            O[k], A[k], LP[k] = o.cpu().numpy(), act.cpu().numpy(), lp.cpu().numpy()
            obs, r, done, info = env.step(A[k])
            R[k], D[k] = r, done
            for j in np.flatnonzero(done):
                recent.append((bool(info["success"][j]), bool(info["collided"][j]), bool(info["timeout"][j]), info["t"][j] / 10.0))
        with torch.no_grad():
            V[T] = net.value(torch.from_numpy(norm(obs)).to(dev)).cpu().numpy()
        steps_total += T * B
        # GAE
        adv = np.zeros((T, B), np.float32)
        last = 0.0
        for k in reversed(range(T)):
            nonterm = 1.0 - D[k]
            delta = R[k] + 0.99 * V[k + 1] * nonterm - V[k]
            last = delta + 0.99 * 0.95 * nonterm * last
            adv[k] = last
        ret = adv + V[:T]
        fo, fa, flp, fadv, fret = (torch.from_numpy(x.reshape(T * B, *x.shape[2:])).to(dev) for x in (O, A, LP, adv, ret))
        fadv = (fadv - fadv.mean()) / (fadv.std() + 1e-8)
        pl = vl = ent = 0.0
        for ep in range(4):
            perm = torch.randperm(T * B, device=dev)
            for mb in perm.split(4096):
                d = net.dist(fo[mb])
                nlp = d.log_prob(fa[mb]).sum(-1)
                ratio = (nlp - flp[mb]).exp()
                s1, s2 = ratio * fadv[mb], ratio.clamp(0.8, 1.2) * fadv[mb]
                loss_pi = -torch.min(s1, s2).mean()
                loss_v = 0.5 * ((net.value(fo[mb]) - fret[mb]) ** 2).mean()
                e = d.entropy().sum(-1).mean()
                loss = loss_pi + 0.5 * loss_v - 0.003 * e
                opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(net.parameters(), 0.5); opt.step()
                pl, vl, ent = loss_pi.item(), loss_v.item(), e.item()
        recent = recent[-400:]
        n = len(recent)
        succ = float(np.mean([r[0] for r in recent])) if n else 0.0
        coll = float(np.mean([r[1] for r in recent])) if n else 0.0
        tout = float(np.mean([r[2] for r in recent])) if n else 0.0
        tm = float(np.mean([r[3] for r in recent if r[0]])) if any(r[0] for r in recent) else float("nan")
        level_iters += 1
        rec = dict(type="iter", iter=it, steps=steps_total, level=level, success=succ, collision=coll, timeout=tout, time=None if np.isnan(tm) else tm,
                   reward=float(R.mean()), policy_loss=pl, value_loss=vl, entropy=ent, std=float(net.log_std.exp().mean().item()), fps=int(T * B / (time.time() - tr)),
                   elapsed=time.time() - t0, base_success=base[level]["success"], base_collision=base[level]["collision"], base_time=base[level]["mean_time"])
        log(mf, rec)
        if it % 5 == 0 or it == 1:
            print(f"it {it:4d} L{level} steps {steps_total / 1e6:5.2f}M  success {succ:5.2f} (classical {base[level]['success']:.2f})  collide {coll:5.2f}  "
                  f"timeout {tout:5.2f}  time {tm:5.1f}s  R {R.mean():+.3f}  std {rec['std']:.2f}  {rec['fps']} sps", flush=True)
        score = level + succ
        if score > best and n >= 100:
            best = score
            torch.save(dict(net=net.state_dict(), norm=(norm.mean, norm.var, norm.count), level=level, success=succ, iter=it), os.path.join(WEIGHTS, "nav_policy.pt"))
        if succ >= a.advance and n >= 200 and level_iters >= a.min_level_iters and level < len(E.LEVELS) - 1:
            level += 1
            level_iters = 0
            env.level = level
            env.reset(np.ones(B, bool))
            obs = env.obs()
            recent = []
            log(mf, dict(type="level", iter=it, level=level))
            print(f"-> curriculum level {level}", flush=True)
    log(mf, dict(type="done", level=level, success=succ, iters=a.iters))
    torch.save(dict(net=net.state_dict(), norm=(norm.mean, norm.var, norm.count), level=level, success=succ, iter=a.iters), os.path.join(WEIGHTS, "nav_policy_last.pt"))
    print("done", flush=True)


if __name__ == "__main__":
    main()
