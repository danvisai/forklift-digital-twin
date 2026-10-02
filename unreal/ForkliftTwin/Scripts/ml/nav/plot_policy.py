"""Render Saved/NavRuns/<run>/metrics.jsonl (policy training) to curves.png - a static fallback for the live dashboard."""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

run = sys.argv[1] if len(sys.argv) > 1 else "policy"
d = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "Saved", "NavRuns", run))
rows = [json.loads(l) for l in open(os.path.join(d, "metrics.jsonl")) if l.strip()]
it = [r for r in rows if r["type"] == "iter"]
x = [r["iter"] for r in it]
fig, ax = plt.subplots(2, 2, figsize=(12, 7))
ax[0, 0].plot(x, [100 * r["success"] for r in it], label="learned"); ax[0, 0].plot(x, [100 * r["base_success"] for r in it], "--", label="classical")
ax[0, 0].set_title("success (%)"); ax[0, 0].legend()
ax[0, 1].plot(x, [100 * r["collision"] for r in it], label="collision", color="tab:red"); ax[0, 1].plot(x, [100 * r["timeout"] for r in it], label="timeout", color="tab:orange")
ax[0, 1].plot(x, [100 * r["base_collision"] for r in it], "--", color="tab:red", label="classical collision"); ax[0, 1].set_title("collision / timeout (%)"); ax[0, 1].legend()
ax[1, 0].plot(x, [r["reward"] for r in it], color="tab:green"); ax[1, 0].set_title("mean reward per step")
ax[1, 1].step(x, [r["level"] for r in it], label="level"); a2 = ax[1, 1].twinx(); a2.plot(x, [r["time"] if r["time"] is not None else float("nan") for r in it], color="tab:green")
ax[1, 1].set_title("curriculum level (blue) / time to goal s (green)")
for a in ax.ravel():
    a.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(d, "curves.png"), dpi=100)
print(os.path.join(d, "curves.png"))
