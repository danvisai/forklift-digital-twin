"""Kinematic model of the ForkliftB truck, identified from UE open-loop logs (-ForkliftIdentTest).

Units: cm, seconds, degrees at the interfaces (UE conventions: yaw is measured from +X toward +Y, steer +1 = left,
and a left turn DEcreases yaw).  Rear wheel steers and drives; the front rollers are fixed, so the reference point on
the fixed axle has no lateral velocity:  u = v*cos(delta),  yaw_rate = -u*tan(delta)/L  (sign from the UE yaw convention).

Mirrors AForkliftVehicle: drive speed is a first-order lag (DriveResponseSec) toward throttle*VMAX, the steer angle
integrates steer_cmd*SteerRate and re-centres at the same rate when the command is zero, |delta| <= MaxSteerDeg.

Run `python vehicle_model.py` to replay every Saved/Ident/case*.csv and print position/yaw error; `--fit` refits.
"""
import glob
import os
import sys

import numpy as np

# Fitted constants (see fit() below; re-run with --fit after changing the vehicle).
PARAMS = dict(vmax=156.0, tau=0.6, wheelbase=141.0, axle_offset=-88.0, steer_rate=60.0, max_steer=60.0)
# Replay error vs UE (M0): straight cases <9 cm; 4 s full-lock turns 10-90 cm, 5-12 deg (speed drops 0.88-1.02x under lock) -> randomise vmax +-12% in training.


def step(state, throttle, steer_cmd, dt, p=PARAMS):
    """state: dict/array of shape (5, ...) = x, y, yaw_deg, v_drive, delta_deg. Vectorised over trailing dims."""
    x, y, yaw, v, d = state
    # drive speed lag
    target = throttle * p["vmax"]
    v = v + (target - v) * (1.0 - np.exp(-dt / p["tau"]))
    # steering: integrate, or re-centre when the command is ~0
    sr = p["steer_rate"] * dt
    centre = np.where(np.abs(d) <= sr, 0.0, d - np.sign(d) * sr)
    d = np.where(np.abs(steer_cmd) > 1e-6, d + steer_cmd * p["steer_rate"] * dt, centre)
    d = np.clip(d, -p["max_steer"], p["max_steer"])
    dr = np.radians(d)
    u = v * np.cos(dr)                                   # speed of the fixed-axle point along the body
    w = -u * np.tan(dr) / p["wheelbase"]                 # rad/s, UE yaw convention
    yaw_rad = np.radians(yaw)
    # the actor origin sits axle_offset cm ahead(+)/behind(-) the fixed axle reference point
    vx_ref = u * np.cos(yaw_rad)
    vy_ref = u * np.sin(yaw_rad)
    vx = vx_ref - w * p["axle_offset"] * np.sin(yaw_rad)
    vy = vy_ref + w * p["axle_offset"] * np.cos(yaw_rad)
    x = x + vx * dt
    y = y + vy * dt
    yaw = yaw + np.degrees(w) * dt
    return np.array([x, y, yaw, v, d])


def load_cases(folder):
    cases = []
    for f in sorted(glob.glob(os.path.join(folder, "case*.csv"))):
        cases.append((os.path.basename(f), np.genfromtxt(f, delimiter=",", names=True)))
    return cases


def replay(d, p=PARAMS):
    t = d["t"]
    s = np.array([d["x"][0], d["y"][0], d["yaw"][0], 0.0, 0.0])
    out = [s]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        s = step(s, d["throttle_cmd"][i - 1], d["steer_cmd"][i - 1], dt, p)
        out.append(s)
    return np.array(out)


def errors(cases, p=PARAMS):
    res = []
    for name, d in cases:
        o = replay(d, p)
        pos = np.hypot(o[:, 0] - d["x"], o[:, 1] - d["y"])
        dyaw = (o[:, 2] - d["yaw"] + 180.0) % 360.0 - 180.0
        res.append((name, float(pos.max()), float(np.abs(dyaw).max()), float(pos[-1]), float(d["t"][-1])))
    return res


def fit(cases, names=("vmax", "tau", "wheelbase", "axle_offset"), iters=400, seed=0):
    """Plain random-restart coordinate search (no scipy)."""
    rng = np.random.default_rng(seed)
    p = dict(PARAMS)

    def cost(q):
        e = errors(cases, q)
        return sum(r[1] + 2.0 * r[2] for r in e)

    best = cost(p)
    scale = {k: 0.08 * max(abs(p[k]), 1.0) for k in names}
    for it in range(iters):
        q = dict(p)
        k = names[rng.integers(len(names))]
        q[k] = p[k] + rng.normal() * scale[k]
        c = cost(q)
        if c < best:
            best, p = c, q
        if it % 100 == 99:
            scale = {k: v * 0.6 for k, v in scale.items()}
    return p, best


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    folder = os.path.normpath(os.path.join(here, "..", "..", "..", "Saved", "Ident"))
    cases = load_cases(folder)
    if "--fit" in sys.argv:
        p, c = fit(cases)
        print("fitted:", {k: round(v, 3) for k, v in p.items()}, "cost", round(c, 2))
        PARAMS.update(p)
    print("%-28s %10s %10s %10s" % ("case", "max pos cm", "max yaw deg", "end pos cm"))
    for name, mp, my, ep, T in errors(cases):
        print("%-28s %10.1f %10.1f %10.1f   (%.1fs)" % (name, mp, my, ep, T))
