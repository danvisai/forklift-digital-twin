"""Learned policy vs the classical controller on fixed seeds, every curriculum level (M4 gate).

    python eval_nav.py [--weights ../../../weights/nav_policy.pt] [--episodes 256]
Both controllers drive the same episodes (same seed -> same starts, goals and clutter), policy acting deterministically.
"""
import argparse
import os

import numpy as np
import torch

import forklift_env as E
from train_policy import ActorCritic

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))


def run(env, act_fn, episodes):
    done_n, s, c, t, times = 0, 0, 0, 0, []
    obs = env.obs()
    while done_n < episodes:
        obs, _, done, info = env.step(act_fn(obs))
        for k in np.flatnonzero(done):
            done_n += 1
            s += int(info["success"][k]); c += int(info["collided"][k]); t += int(info["timeout"][k])
            if info["success"][k]:
                times.append(info["t"][k] / 10.0)
    return s / done_n, c / done_n, t / done_n, (float(np.mean(times)) if times else float("nan"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=os.path.join(ROOT, "weights", "nav_policy.pt"))
    ap.add_argument("--episodes", type=int, default=256)
    a = ap.parse_args()
    ck = torch.load(a.weights, map_location="cpu", weights_only=False)
    net = ActorCritic()
    net.load_state_dict(ck["net"])
    net.eval()
    mean, var = ck["norm"][0], ck["norm"][1]
    print(f"policy trained to level {ck['level']} (iter {ck['iter']}, recent success {ck['success']:.2f})")
    print(f"{'level':>5} | {'learned: success  collide  timeout  time':<44} | classical: success  collide  timeout  time")
    for lv in range(len(E.LEVELS)):
        env = E.BatchEnv(64, seed=1000 + lv, level=lv)
        def pol(obs):
            o = torch.from_numpy(np.clip((obs - mean) / np.sqrt(var + 1e-6), -8, 8).astype(np.float32))
            with torch.no_grad():
                return torch.tanh(net.pi(o)).numpy() if False else net.pi(o).numpy()
        L = run(env, pol, a.episodes)
        env = E.BatchEnv(64, seed=1000 + lv, level=lv)
        C = run(env, lambda obs: E.classical_action(env), a.episodes)
        print(f"{lv:>5} | {100*L[0]:5.1f}%   {100*L[1]:5.1f}%   {100*L[2]:5.1f}%   {L[3]:5.1f}s{'':<12} | {100*C[0]:5.1f}%   {100*C[1]:5.1f}%   {100*C[2]:5.1f}%   {C[3]:5.1f}s", flush=True)


if __name__ == "__main__":
    main()
