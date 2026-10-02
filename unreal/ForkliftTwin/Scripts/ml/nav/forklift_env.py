"""Batched navigation environment on the Python twin (M4) + the classical controller as a baseline.

Task: drive from a random pose to a goal pose (position within 35 cm, and - when required - heading within 12 deg) without
touching the racks, walls or random floor clutter, optionally carrying a pallet. The goal is the same kind of target the mission
hands to UForkliftNavComponent (a staging / pre-pose point with the heading the next skill needs).

Observation (83): 72 clearance bins (cm / 1500, clipped), goal in the truck frame (dx, dy in 5 m units, distance in 10 m units),
required-heading error (cos, sin) and a flag, speed and steer angle (normalised), carrying flag, previous action (2).
Actions (2): throttle and steer-rate in [-1, 1] - exactly the channels of AForkliftVehicle::SetDriveInput.
The policy runs at 10 Hz (6 vehicle steps of 1/60 s per env step). Clearance is the exact 2D free distance with a noise model
standing in for ClearanceNet's error (the net's measured error on real UE scans is ~27 cm median-abs).
"""
import numpy as np

import forklift_sim as fs

DT = 1.0 / 60.0
SUBSTEPS = 6
OBS_DIM = 72 + 3 + 3 + 2 + 1 + 2
MAX_STEPS = 600           # 60 s of sim
POS_TOL = 35.0
YAW_TOL = 12.0
K_CLUTTER = 8

LEVELS = [  # (dmin, dmax cm, max clutter, heading prob, carry prob)
    (300.0, 600.0, 0, 0.3, 0.0),
    (300.0, 800.0, 1, 0.4, 0.0),
    (300.0, 1000.0, 2, 0.5, 0.0),
    (300.0, 1500.0, 4, 0.7, 0.2),
    (300.0, 2000.0, 6, 0.8, 0.3),
]


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class BatchEnv:
    def __init__(self, n_envs, world_path=None, seed=0, level=0):
        self.B = n_envs
        self.rng = np.random.default_rng(seed)
        self.static, self.world = fs.load_world(world_path)
        self.n_static = len(self.static)
        self.level = level
        self.fp = {False: fs.Footprint(carrying=False, margin=10.0), True: fs.Footprint(carrying=True, margin=10.0)}
        B = n_envs
        self.state = np.zeros((5, B))
        self.boxes = np.zeros((B, self.n_static + K_CLUTTER, 6), np.float32)
        self.goal = np.zeros((B, 3))             # x, y, yaw
        self.need_heading = np.zeros(B, bool)
        self.carry = np.zeros(B, bool)
        self.speed_scale = np.ones(B)
        self.t = np.zeros(B, np.int32)
        self.prev_dist = np.zeros(B)
        self.prev_herr = np.zeros(B)
        self.prev_act = np.zeros((B, 2))
        self.reversing = np.zeros(B, bool)        # classical controller's mode memory
        self.reset(np.ones(B, bool))

    # ---- episode construction ---------------------------------------------------------------------------------------
    def _collides(self, idx, x, y, yaw):
        """collision of the env subset idx at poses (x,y,yaw)."""
        out = np.zeros(len(idx), bool)
        for carry in (False, True):
            sel = self.carry[idx] == carry
            if sel.any():
                out[sel] = self.fp[carry].collides(x[sel], y[sel], yaw[sel], self.boxes[idx[sel]])
        return out

    def reset(self, mask):
        idx = np.flatnonzero(mask)
        if len(idx) == 0:
            return
        lv = LEVELS[min(self.level, len(LEVELS) - 1)]
        dmin, dmax, nmax, hp, cp = lv
        rng = self.rng
        n = len(idx)
        self.boxes[idx, :self.n_static] = self.static
        self.boxes[idx, self.n_static:] = 0.0           # absent clutter: zero-height boxes (ignored by clearance and collisions)
        self.carry[idx] = rng.random(n) < cp
        self.need_heading[idx] = rng.random(n) < hp
        self.speed_scale[idx] = rng.uniform(0.88, 1.12, n)
        self.reversing[idx] = False
        # start pose: free floor (keeps clear of racks/walls), random heading
        sx, sy, syaw = np.zeros(n), np.zeros(n), np.zeros(n)
        todo = np.arange(n)
        for _ in range(60):
            if len(todo) == 0:
                break
            sx[todo] = rng.uniform(-1500, 1500, len(todo)); sy[todo] = rng.uniform(-950, 950, len(todo)); syaw[todo] = rng.uniform(-180, 180, len(todo))
            bad = self._collides(idx[todo], sx[todo], sy[todo], syaw[todo])
            todo = todo[bad]
        # goal: at a distance in [dmin, dmax] in a random direction, inside the room and clear of the statics
        gx, gy, gyaw = np.zeros(n), np.zeros(n), np.zeros(n)
        todo = np.arange(n)
        for _ in range(80):
            if len(todo) == 0:
                break
            d = rng.uniform(dmin, dmax, len(todo)); a = rng.uniform(-np.pi, np.pi, len(todo))
            gx[todo] = sx[todo] + d * np.cos(a); gy[todo] = sy[todo] + d * np.sin(a); gyaw[todo] = rng.uniform(-180, 180, len(todo))
            inside = (np.abs(gx[todo]) < 1600) & (np.abs(gy[todo]) < 1000)
            clear = ~fs.Footprint(margin=40.0).collides(gx[todo], gy[todo], gyaw[todo], self.static)
            todo = todo[~(inside & clear)]
        # clutter (never on the start, never on the goal)
        for j, e in enumerate(idx):
            k = rng.integers(0, nmax + 1) if nmax else 0
            if k:
                c = fs.random_clutter(rng, k, ((sx[j] + gx[j]) / 2, (sy[j] + gy[j]) / 2), radius=max(300.0, 0.5 * np.hypot(gx[j] - sx[j], gy[j] - sy[j]) + 200.0),
                                      static=self.static, min_dist=0.0)
                if len(c):
                    keep = []
                    for b in c:
                        near_s = (b[0] - 150 < sx[j] < b[3] + 150) and (b[1] - 150 < sy[j] < b[4] + 150)
                        near_g = (b[0] - 120 < gx[j] < b[3] + 120) and (b[1] - 120 < gy[j] < b[4] + 120)
                        if not (near_s or near_g):
                            keep.append(b)
                    if keep and rng.random() < 0.6:     # make at least one object matter: drop it near the straight start->goal line
                        t = rng.uniform(0.25, 0.75)
                        cx = sx[j] + t * (gx[j] - sx[j]) + rng.uniform(-80, 80)
                        cy = sy[j] + t * (gy[j] - sy[j]) + rng.uniform(-80, 80)
                        w, h = keep[0][3] - keep[0][0], keep[0][4] - keep[0][1]
                        keep[0] = np.array([cx - w / 2, cy - h / 2, 0.0, cx + w / 2, cy + h / 2, keep[0][5]], np.float32)
                    for q, b in enumerate(keep[:K_CLUTTER]):
                        self.boxes[e, self.n_static + q] = b
        self.state[:, idx] = np.stack([sx, sy, syaw, np.zeros(n), np.zeros(n)])
        self.goal[idx] = np.stack([gx, gy, gyaw], 1)
        self.t[idx] = 0
        self.prev_act[idx] = 0.0
        self.prev_dist[idx] = np.hypot(gx - sx, gy - sy)
        self.prev_herr[idx] = np.abs(wrap(gyaw - syaw))

    # ---- observation ------------------------------------------------------------------------------------------------
    def clearance(self, noise=True):
        s = self.state
        c = fs.clearance_gt(self.boxes, s[0], s[1], s[2])
        if noise:   # ClearanceNet-like error: ~25 cm jitter, rare gross misses (bin reads far) and rare false alarms (reads near)
            c = c * np.exp(self.rng.normal(0.0, 0.05, c.shape)) + self.rng.normal(0.0, 8.0, c.shape)
            r = self.rng.random(c.shape)
            c = np.where(r < 0.004, 3000.0, np.where(r > 0.996, self.rng.uniform(150.0, 600.0, c.shape), c))
        return np.clip(c, 0.0, 3000.0).astype(np.float32)

    def goal_in_frame(self):
        s = self.state
        yaw = np.radians(s[2])
        dx, dy = self.goal[:, 0] - s[0], self.goal[:, 1] - s[1]
        fwd = dx * np.cos(yaw) + dy * np.sin(yaw)
        rgt = -dx * np.sin(yaw) + dy * np.cos(yaw)
        return fwd, rgt, np.hypot(dx, dy), wrap(self.goal[:, 2] - s[2])

    def obs(self, c=None):
        s = self.state
        c = self.clearance() if c is None else c
        fwd, rgt, dist, herr = self.goal_in_frame()
        hr = np.radians(herr)
        o = np.concatenate([
            np.minimum(c, 1500.0) / 1500.0,
            np.stack([np.clip(fwd / 500.0, -4, 4), np.clip(rgt / 500.0, -4, 4), np.minimum(dist / 1000.0, 4)], 1),
            np.stack([np.cos(hr) * self.need_heading, np.sin(hr) * self.need_heading, self.need_heading.astype(float)], 1),
            np.stack([s[3] / 160.0, s[4] / 60.0], 1),
            self.carry[:, None].astype(float),
            self.prev_act,
        ], 1)
        return o.astype(np.float32)

    # ---- dynamics ---------------------------------------------------------------------------------------------------
    def step(self, act):
        """act (B,2) in [-1,1]. Returns obs, reward, done, info (arrays); finished envs are auto-reset (their final obs is dropped)."""
        act = np.clip(act, -1.0, 1.0)
        thr, st = act[:, 0], act[:, 1]
        for _ in range(SUBSTEPS):
            self.state = fs.step_vehicle(self.state, thr, st, DT, self.speed_scale)
        self.t += 1
        s = self.state
        fwd, rgt, dist, herr = self.goal_in_frame()
        aherr = np.abs(herr)
        collided = np.zeros(self.B, bool)
        for carry in (False, True):
            sel = self.carry == carry
            if sel.any():
                collided[sel] = self.fp[carry].collides(s[0][sel], s[1][sel], s[2][sel], self.boxes[sel])
        success = (dist < POS_TOL) & (~self.need_heading | (aherr < YAW_TOL))
        timeout = self.t >= MAX_STEPS
        r = (self.prev_dist - dist) / 100.0                                   # progress in metres
        near = dist < 500.0
        r += np.where(self.need_heading & near, 0.4 * (self.prev_herr - aherr) / 45.0, 0.0)
        r -= 0.004                                                           # time
        r -= 0.002 * np.abs(st) + 0.002 * (thr < 0)                          # effort / reversing
        # proximity shaping: slow down and steer away when something is within ~1.5 m of the end of the truck in the direction of travel
        gt = fs.clearance_gt(self.boxes, s[0], s[1], s[2])
        k = np.arange(-3, 4)
        ahead = np.take_along_axis(gt, (k[None, :] % 72).repeat(self.B, 0), 1).min(1) - 215.0
        behind = np.take_along_axis(gt, ((36 + k)[None, :] % 72).repeat(self.B, 0), 1).min(1) - 85.0
        gap = np.where(s[3] >= 0, ahead, behind)
        r -= 0.3 * np.square(np.clip((150.0 - gap) / 150.0, 0.0, 1.0)) * np.minimum(np.abs(s[3]) / 100.0, 1.5)
        r += np.where(success, 10.0, 0.0) + np.where(collided & ~success, -12.0, 0.0) + np.where(timeout & ~success & ~collided, -1.0, 0.0)
        self.prev_dist, self.prev_herr, self.prev_act = dist, aherr, act
        done = success | collided | timeout
        info = dict(success=success, collided=collided, timeout=timeout & ~success & ~collided, t=self.t.copy(), dist=dist, herr=aherr)
        self.reset(done)
        return self.obs(), r.astype(np.float32), done, info


# ---- the classical controller (UForkliftNavComponent's last leg, vectorised) -------------------------------------------
def classical_action(env, lineup=250.0, carrot=120.0, enter_deg=100.0, exit_deg=70.0):
    """Same logic as the UE follower: reverse when the target is more than 100 deg off the nose (back to forward below 70),
    steer toward a carrot on the goal heading axis when heading matters, pure pursuit of the point otherwise."""
    s = env.state
    yaw = np.radians(s[2])
    gx, gy, gyaw = env.goal[:, 0], env.goal[:, 1], env.goal[:, 2]
    ax, ay = np.cos(np.radians(gyaw)), np.sin(np.radians(gyaw))
    px, py = s[0] - gx, s[1] - gy
    along = px * ax + py * ay                                   # position along the goal axis (0 at the goal)
    dist = np.hypot(px, py)
    use_axis = env.need_heading & (dist < lineup * 1.6 + 150.0)
    cs = np.where(along <= 0, np.minimum(along + carrot, 0.0), np.maximum(along - carrot, 0.0))
    aim_x = np.where(use_axis, gx + ax * cs, gx)
    aim_y = np.where(use_axis, gy + ay * cs, gy)
    dx, dy = aim_x - s[0], aim_y - s[1]
    fwd = dx * np.cos(yaw) + dy * np.sin(yaw)
    rgt = -dx * np.sin(yaw) + dy * np.cos(yaw)
    off = np.degrees(np.arctan2(rgt, fwd))
    env.reversing = np.where(~env.reversing & (np.abs(off) > enter_deg), True, np.where(env.reversing & (np.abs(off) < exit_deg), False, env.reversing))
    fb = np.degrees(np.arctan2(rgt, np.maximum(fwd, 20.0)))
    rb = np.degrees(np.arctan2(-rgt, np.maximum(-fwd, 20.0)))
    desired = np.where(env.reversing, np.clip(rb * 0.6, -55, 55), np.clip(-fb * 0.6, -55, 55))
    steer = np.clip((desired - s[4]) / 15.0, -0.5, 0.5)
    final = dist < 250.0
    thr = np.where(env.reversing, -0.35, np.where(final | (np.abs(fb) > 25), 0.2, 0.55))
    # lidar stop: anything on the straight path within 150 cm (the real controller uses the roof lidar; here the exact clearance)
    c = fs.clearance_gt(env.boxes, s[0], s[1], s[2])
    ahead = np.where(env.reversing, 36, 0)                      # bin 36 = straight behind (5 deg bins)
    idx = (ahead[:, None] + np.arange(-8, 9)[None, :]) % 72
    blocked = np.take_along_axis(c, idx, 1).min(1) < 150.0
    thr = np.where(blocked, 0.0, thr)
    return np.stack([thr, steer], 1).astype(np.float32)


def run_classical(level, episodes=256, seed=0, n_envs=64):
    """Success / collision / time of the classical controller over `episodes` episodes at a curriculum level."""
    env = BatchEnv(n_envs, seed=seed, level=level)
    done_n, succ, coll, tout, times = 0, 0, 0, 0, []
    while done_n < episodes:
        a = classical_action(env)
        _, _, done, info = env.step(a)
        for k in np.flatnonzero(done):
            done_n += 1
            succ += int(info["success"][k]); coll += int(info["collided"][k]); tout += int(info["timeout"][k])
            if info["success"][k]:
                times.append(info["t"][k] / 10.0)
    return dict(level=level, episodes=done_n, success=succ / done_n, collision=coll / done_n, timeout=tout / done_n, mean_time=float(np.mean(times)) if times else float("nan"))


if __name__ == "__main__":
    import time
    for lv in range(len(LEVELS)):
        t = time.time()
        print(run_classical(lv, episodes=192), f"({time.time() - t:.0f}s)", flush=True)
