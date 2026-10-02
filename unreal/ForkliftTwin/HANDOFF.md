# Handoff — forklift ML detection pipeline

Snapshot as of **2026-09-26**, end of session. Read this before `README.md` if you're picking this
project back up — it's the "what's actually going on right now" doc; the README is the reference
manual. Everything below lives in `unreal/ForkliftTwin/`, which is **untracked by git** (`?? unreal/`
at repo root) — nothing here is committed yet.

## TL;DR

Goal: teach an ML model to find pallets (empty/loaded), their fork pockets, totes and racks from the
forklift's cameras, and use those detections for a full autonomous mission (find → navigate → pick up
→ carry → place). Sensors, a synthetic-data generator, a trained detector, live overlays, a low RGB-D
tine camera, and an align/insert/lift controller are working. **No ML/detector training has happened
since 2026-09-22** (v2, `pallet_demo_v2` weights, is still the current model) — this session's work was
all on the pickup controller and warehouse layout, in C++, not the training pipeline.

## Session 2026-09-26: drop zones, pathfinding decision, closed-loop insertion

1. **Warehouse drop slots** (`Warehouse.h/.cpp`, `FPalletSlot`): a west zone (x=-12 m, blue pads) for
   empty pallets and an east zone (x=+12 m, green pads) for loaded ones, 4 slots each, 2 m pitch. Each
   slot carries a final pose and a pre-pose `PrePoseStandOffCm` (350 cm) back toward the aisle. API:
   `GetSlots()`, `GetSlotsFor(bLoaded)`, `IsSlotOccupied(slot)`, `FindFreeSlot(bLoaded, from, out)`. Two
   rows per zone line up with the rack banks on purpose, so a router has to go around them. **Not wired
   into anything yet** — no planner reads these slots, they just exist and render.
2. **Pathfinding decision (design only, no code)** — see `RESEARCH_HANDOFF.md`'s "Decision (2026-09-25):
   pathfinding approach" section for the full writeup. Researched what NVIDIA's own Isaac stack actually
   uses (Nav2 Smac Hybrid-A*, opennav_docking, nvblox, FoundationPose) rather than guessing. Landed on:
   classical Hybrid-A* (Reeds-Shepp motion, footprint collision check) for the global route to a slot's
   pre-pose, pure pursuit to follow it, lidar built into a local costmap as a safety/replan layer (it
   can't see floor-level pallets close-up — that stays on the tine depth camera + detector), and learning
   introduced only later, narrowly, for final alignment/placement residuals. **No planner/tracker code
   exists yet** — next concrete step if this is picked back up.
3. **Closed-loop pallet insertion** (`ForkliftAutoPickupComponent.h/.cpp`) — the actual code change this
   session. The old controller went blind at insertion (pure odometry, no yaw correction), which matched
   the known 20-25° skew failure in the pallet test. Now:
   - The pallet's insertion axis is tracked as a **world-frame yaw** (`PalletAxisYawWorld`), not a
     per-frame camera-relative angle, so it survives the truck turning and the pockets leaving view.
   - Yaw is estimated from `EstimateFaceYawDeg`: a line fit (3-pass, outlier-trimmed) across the *whole*
     pallet-face depth profile (~120 cm wide), not just two pocket-depth samples ~40 cm apart — far less
     noisy (was reading 15-22° for an actual 8° error).
   - During `Inserting`, pure pursuit steers the tine tips onto that axis using `CrossTrackCm` +
     `LastYawErrorDeg`, instead of driving straight and hoping.
   - New `EAutoPickupState::Withdrawing`: if cross-track exceeds `InsertionCrossTrackAbortCm` (10 cm) or
     yaw exceeds `InsertionYawAbortDeg` (9°) after 15 cm of travel, the truck backs straight out instead
     of ramming in skewed.
   - Modeled on the Nav2 `opennav_docking` pattern (stay closed-loop through contact, don't switch to
     open-loop odometry once "aligned") — see the research summary in the conversation, not yet copied
     into a doc file.
4. **Test harness rework** — `-ForkliftAutoPickupTest` now takes `-PickupOffsetY=<cm>`,
   `-PickupYawDeg=<deg>`, `-PickupBackCm=<cm run-up>` (`ForkliftGameMode.cpp`) for reproducible
   misalignment cases. Running each case as a fresh `-game` process works but is slow/noisy; running
   each as a fresh Editor process crashes UnrealEd on shutdown-after-PIE when scripted. Fixed by adding
   `Content/Python/init_unreal.py` + `Saved/pickup_batch.json`: **one editor process, one Play session,
   reset the scene in-place between cases** (teleport truck, restore pallet transform, lower lift, centre
   steering) rather than ending/restarting Play. Batch results land in `Saved/AutoPickupBatch.txt`.
   `AForkliftGameMode` gained `ReadAutoPickupCaseArgs`, `PlaceAutoPickupCase`, `PollAutoPickupRun`,
   `FinishAutoPickupTest`, and an `EndPlay` override that un-forces the fixed timestep.

**Latest 8-case batch result** (`Saved/AutoPickupBatch.txt`, 120 cm run-up unless noted):

| Case | Result |
|---|---|
| square | PASS (rise 32.6 cm) |
| yaw +8° | PASS |
| yaw -8° | PASS |
| offset +25 cm | PASS |
| offset -25 cm | PASS |
| offset -25, yaw +8° | **FAIL** — pockets lost during alignment |
| offset +25, yaw -8° | **FAIL** — too close (gap -4, lateral 9.1, yaw -3.8) when insertion should've started |
| yaw +15° (160 cm run-up) | **FAIL** — too close (lateral -23.5, yaw -7.1), didn't converge in time |

Single-axis errors (yaw-only or offset-only, up to the tested magnitudes) pass cleanly now. Combined
errors and larger single yaw (15°) still fail — the approach phase runs out of distance to converge, or
loses the pockets outright, before insertion should start. **Not yet root-caused or fixed.**

### Next steps (this thread)

1. Fix the two/three failing combined-misalignment cases — likely needs either more approach distance,
   a slower/more aggressive alignment gain at large errors, or widening the pocket-lock detection window.
2. Write the Hybrid-A*/pure-pursuit planner and wire it to `AWarehouse::FindFreeSlot` — currently just a
   design decision, no code.
3. Regression-test that the insertion changes didn't reintroduce the old pallet-drag/slip bug from the
   `ForkliftPalletTest` (not re-run this session).
4. `unreal/` is still fully untracked in git — nothing from this session is committed either.

## What's running right now (check before assuming state)

- **Detector server**: was started this session (`serve_detector.py`, port 8765) and was still healthy
  (`curl http://127.0.0.1:8765/health`) as of this note — check, it may have exited since. Serves
  `Saved/ml/runs/pallet_demo_v2/weights/best.pt`. Start it yourself with:
  ```
  %USERPROFILE%\.venvs\forklift-ml\Scripts\python.exe Scripts\ml\serve_detector.py
  ```
  (defaults now point at `pallet_demo_v2`, no flag needed).
- **UnrealEditor.exe**: not running as of this note (the batch-test editor session from this session has
  since closed).
- Nothing else backgrounded.

## Vehicle

`AForkliftVehicle` drives both Isaac rigs from one class; a per-model table in `ForkliftVehicle.cpp`
holds the mesh/wheel/tuning differences. **`EForkliftModel::ForkliftB` is the default now** (was C
earlier this project). Force C with `-ForkliftModel=C` (e.g. for the regression self-test).

**Pickup fix (2026-09-23):** the generated warehouse pallet asset was stale and still had one solid
100x120x12 cm collision box, physically sealing the visible fork pockets. Re-running the existing
`IMPORT_ONLY=pallet` path in `Scripts/import_assets.py` installed the intended five analytic boxes.
The rendered `-ForkliftAutoPickupTest` now passes: the low camera detected both pockets, the controller
aligned/inserted, and the physical pallet rose 32.6 cm. The generated `.uasset` and the import script both
carry the hollow collision definition.

## Sensors

- `UForkliftCameraComponent`: 640×480 @ 10 Hz, fixed exposure, optional aligned metric depth
  (`bCaptureDepth`, writes float16 `.npy`). Mounted on the fork carriage, rises/falls with the lift.
- `TineCamera`: 640×480 RGB-D, 78° FOV, mounted low between the tines and hiding the forklift itself from
  RGB/depth. Its synchronized frames feed `UForkliftAutoPickupComponent`.
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

1. Expand auto-pick validation beyond the square test pose: lateral/yaw offsets, loaded pallets, racks,
   and recovery/back-out behavior after a safety abort.
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
