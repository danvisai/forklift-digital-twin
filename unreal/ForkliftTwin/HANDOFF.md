# Handoff — forklift ML detection pipeline

Snapshot as of **2026-09-22**, end of session. Read this before `README.md` if you're picking this
project back up — it's the "what's actually going on right now" doc; the README is the reference
manual. Everything below lives in `unreal/ForkliftTwin/`, which is **untracked by git** (`?? unreal/`
at repo root) — nothing here is committed yet.

## TL;DR

Goal: teach an ML model to find pallets (empty/loaded), their fork pockets, totes and racks from the
forklift's own camera, as a first step toward automated pick-and-place. Sensors, a synthetic-data
generator, a trained detector, and a live in-game overlay are all working. The forklift's own physical
pallet pickup is **not** working yet, but that doesn't block anything above — the data generator places
pallets directly in the scene, it never needs the forklift to actually lift one.

## What's running right now (check before assuming state)

- **Detector server**: `python.exe` PID 17364 (may have exited since — check with
  `curl http://127.0.0.1:8765/health`), serving `Saved/ml/runs/pallet_demo_v2/weights/best.pt`
  (the current, tightened-box model). Start it yourself with:
  ```
  %USERPROFILE%\.venvs\forklift-ml\Scripts\python.exe Scripts\ml\serve_detector.py
  ```
  (defaults now point at `pallet_demo_v2`, no flag needed).
- **UnrealEditor.exe**: not running as of this note.
- Nothing else backgrounded.

## Vehicle

`AForkliftVehicle` drives both Isaac rigs from one class; a per-model table in `ForkliftVehicle.cpp`
holds the mesh/wheel/tuning differences. **`EForkliftModel::ForkliftB` is the default now** (was C
earlier this project). Force C with `-ForkliftModel=C` (e.g. for the regression self-test).

**Known open bug, unfixed:** the scripted pallet pickup (`-ForkliftPalletTest`) reproducibly fails.
The forks insert to a good depth (~113–116 cm) but the pallet skews 20–25° in yaw during entry, then
doesn't rise with the lift and drags ~338 cm of slip when carried. Confirmed deterministic across
repeated runs (same fixed-timestep numbers every time), so it's a real geometry/insertion issue, not
noise. **This does not block the ML pipeline** — the synthetic generator places pallets directly in
the scene for rendering/labeling, no physical pickup involved.

## Sensors

- `UForkliftCameraComponent`: 640×480 @ 10 Hz, fixed exposure, optional aligned metric depth
  (`bCaptureDepth`, writes float16 `.npy`). Mounted on the fork carriage, rises/falls with the lift.
- `ULidarSensorComponent`: 16 channels × 360 steps, ±15° vertical, ray-traced (UE has no built-in
  lidar), mounted on the chassis (fixed, not on the moving lift).
- `AForkliftHUD`: `V` toggles a debug overlay — fork camera thumbnail (now with live detection boxes
  drawn on it) + a top-down lidar view.

## ML pipeline

**Classes (5):** `0 pallet, 1 loaded_pallet, 2 pallet_pocket, 3 tote, 4 rack`.

**Generator:** `SyntheticDataGenerator.cpp`, a `UWorldSubsystem` that spins up when the game is
launched with `-ForkliftDataGen`. Randomizes pallet/tote/rack placement, lighting, camera pose
(matched to real fork-camera geometry), and partial occlusion; writes YOLO-format labels + depth +
per-object JSON meta + `data.yaml`. ~11 images/sec on this (thermally throttled) machine.

**Label-box fix (2026-09-22):** boxes were originally the screen-projection of each object's full 3D
bounding box (all 8 corners) — this reliably oversized the box for anything at an oblique angle to the
camera (racks were worst, sometimes covering half the frame with empty space). Fixed to build the box
from only the *actually visible* sample points (reusing the per-point depth-test visibility grid the
code already computed for occlusion filtering), falling back to the old 8-corner method only when
fewer than 4 points are visible — those objects get rejected by the existing visibility-ratio gate
anyway, so their box geometry never matters. Verified visually (same frames, before/after) — dramatic
tightening on racks/totes, modest on pallets/pockets.

**Datasets** (`Saved/Dataset/<name>/`):
- `demo` — original, pre-fix, 3000 images, seed 11. **Loose/oversized boxes — don't train on this.**
- `demo_v2` — same seed/scenes, tightened boxes. **This is the current one.**

**Training** (`Scripts/ml/train_detector.py`, YOLO11n, 40 epochs, 640px, batch 16):

| Class | v1 (`pallet_demo`, loose boxes) mAP50 | v2 (`pallet_demo_v2`, tight boxes) mAP50 |
|---|--:|--:|
| pallet | 0.943 | 0.952 |
| loaded_pallet | 0.964 | 0.952 |
| pallet_pocket | 0.781 | 0.802 |
| tote | 0.827 | 0.834 |
| rack | 0.960 | 0.952 |
| **all** | **0.895** | **0.898** |

mAP50-95 dropped (0.684 → 0.656) between the two, which is *expected*, not a regression: that metric
rewards very-high-IoU matches, and the v2 ground truth is now genuinely tighter, so it's a stricter
target even though detection quality (mAP50) held steady or improved.

v2's first training attempt was **killed by the harness at epoch 38/40** (system memory pressure while
the session was idle — not a training bug). Resumed cleanly via
`YOLO('.../last.pt').train(resume=True)`, which picks up the optimizer state and finishes the
remaining epochs — no need to restart from scratch if this happens again.

**Live overlay:** `ForkliftDetectionClient.h/.cpp` (new), wired into `ForkliftHUD.cpp`. The HUD posts
the fork camera's raw pixels to `http://127.0.0.1:8765/detect` at 5 Hz and draws whatever boxes come
back on the camera thumbnail. Needs the `HTTP` UBT module (already added to `ForkliftTwin.Build.cs`).
**The server does not start itself and is not persistent across sessions** — see "what's running
right now" above.

## Mouse-look / camera (tuned today)

- `AForkliftVehicle::bMouseLook` (default on): free-look chase camera, additive on top of the truck's
  own heading (the spring arm still inherits body yaw).
- Fixed a "very slow" complaint: the spring arm had `bEnableCameraRotationLag = true` at
  `CameraRotationLagSpeed = 4` — meant for smoothing the third-person follow, but since mouse-look now
  sets the arm's rotation directly every tick, that lag was double-smoothing every mouse move into a
  sluggish drift. **Now off.**
- `MouseSensitivity` 0.15 → 0.3 deg/px, `CameraArm->TargetArmLength` 900 → 500 cm (camera closer).
  Both are just constructor defaults in `ForkliftVehicle.cpp`/`.h` — trivial to retune further.

## Environment

- ML venv: `%USERPROFILE%\.venvs\forklift-ml` (torch 2.5.1+cu124, ultralytics 8.4.158) — separate
  from system Python, which is CPU-only.
- This laptop's GPU idles around 90°C / 210 MHz even at rest (thermal throttling, not a new problem —
  don't chase it as if it were a regression).
- UE 5.8 at `C:\Program Files\Epic Games\UE_5.8`.
- Before any `UnrealEditor(-Cmd).exe` run: check `Get-Process UnrealEditor` first. An editor left open
  (yours, or an orphaned one from a prior `-unattended` automated run that didn't fully exit) will
  block the next build with a DLL lock.

## Reproduce / key commands

```powershell
# Build
& "C:\Program Files\Epic Games\UE_5.8\Engine\Build\BatchFiles\Build.bat" ForkliftTwinEditor Win64 Development -Project="<uproject>" -WaitMutex -NoHotReload -NoHotReloadFromIDE

# Regression self-test (both rigs)
UnrealEditor-Cmd.exe <uproject> /Game/Maps/Warehouse -game -nullrhi -unattended -ForkliftSelfTest [-ForkliftModel=C]

# Generate synthetic data (batch of 500, repeat with -DataStart=500,1000,... for more)
UnrealEditor.exe <uproject> /Game/Maps/Warehouse -game -windowed -ResX=320 -ResY=180 -unattended -nosound -ForkliftDataGen -DataName=<name> -DataFrames=500 -DataSeed=11 -DataStart=0

# Train
%USERPROFILE%\.venvs\forklift-ml\Scripts\python.exe Scripts\ml\train_detector.py --data Saved\Dataset\demo_v2\data.yaml --epochs 40 --imgsz 640 --batch 16 --name <run_name> --device 0

# Serve the current model (for the live HUD overlay)
%USERPROFILE%\.venvs\forklift-ml\Scripts\python.exe Scripts\ml\serve_detector.py
```

## Next steps / open decisions

1. **Fix the physical pallet pickup** (see "known open bug" above) — needed before any real
   automation, not needed for more detector work.
2. **Scale up training data** if pocket accuracy (weakest class, mAP50 0.80) needs to improve —
   current set is 3000 images; a remote A100 was discussed as an option (only data generation needs
   Windows/UE, `train_detector.py` is portable as-is — copy `images/`, `labels/`, `data.yaml`, fix the
   absolute Windows `path:` line, skip `depth/`/`meta/` to save bandwidth). Deferred, not started.
3. **LOCO** (real-world CC0 pallet dataset) was researched as a future held-out real-data test but
   never actually used — everything trained/evaluated so far is 100% synthetic, same renderer for
   train/val/test. Worth doing before trusting this on a real camera.
4. Minor: a test-set composition drift was noticed between the `demo` and `demo_v2` full runs despite
   the same seed (instance counts differ slightly) — not root-caused, didn't block anything, but flag
   it if exact reproducibility ever matters.
5. `unreal/` is still fully untracked in git — nothing here has been committed.
