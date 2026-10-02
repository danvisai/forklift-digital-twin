# Forklift Digital Twin - Unreal Engine 5.8

Port of the Omniverse Kit forklift app (`source/extensions/my_company.forklift_twin.core`) to UE 5.8:
the same warehouse, the same NVIDIA ForkliftC, spawned at the same point, driven with the same keys.

## Run it

Needs Windows, UE 5.8, Visual Studio 2022 (C++ workload) and Python 3 with
`pip install usd-core numpy pillow`. Close any running Unreal Editor first (Live Coding blocks the build).

```
python Scripts/setup.py            # ~5 min: download -> bake -> build C++ -> import -> build map
```

Then open `ForkliftTwin.uproject` (it opens `/Game/Maps/Warehouse`) and press Play, or run
`UnrealEditor.exe ForkliftTwin.uproject /Game/Maps/Warehouse -game`.
Use `--ue "D:/UE_5.8"` or `UE_ROOT` if the engine isn't in the default launcher location.

The NVIDIA meshes are **not committed**: `setup.py` downloads the same USD files the Kit app referenced,
and everything under `Content/` is generated from them (see `.gitignore`).

## Controls

| Key | Action |
|---|---|
| W / S (or Up / Down) | drive forward / back |
| A / D (or Left / Right) | steer left / right (rear-wheel steering) |
| R / F | raise / lower the forks (0-2 m) |
| P | start / abort vision-guided pallet alignment, insertion and lift |
| Mouse | free-look chase camera (stacks on top of the truck's own heading; `AForkliftVehicle::bMouseLook`) |
| V | show / hide the sensor view (fork camera + top-down lidar) |

Scripted control (what an AI/RL controller would use): `AForkliftVehicle::SetDriveInput(throttle, steer, lift)`,
each -1..1, with `bKeyboardControl = false`.

## Sensors (for the learning pipeline)

| Sensor | Where | Output |
|---|---|---|
| `ForkCamera` (`UForkliftCameraComponent`) | top of the carriage backrest, looking down; rises and falls with the forks | RGB 640x480, 90 deg FOV, 10 Hz, fixed exposure |
| `TineCamera` (`UForkliftCameraComponent`) | low and centred between the tines; rises and falls with them | aligned RGB + metric depth, 78 deg FOV, 10 Hz |
| `Lidar` (`ULidarSensorComponent`) | cab roof, 10 cm above the overhead guard | 16 channels x 360 azimuth steps, -15..+15 deg, 0.3-30 m, 10 Hz |

Get them with `AForkliftVehicle::GetForkCamera()` / `GetTineCamera()` / `GetLidar()`. The cameras hand out pixels (`ReadPixelsBgra`,
`SavePng`); the lidar keeps `Ranges` (channel-major, cm, -1 = no return) and `Points` (sensor frame, cm), and both
can be written as `.npy` (`SaveRangesNpy`, `SavePointsNpy`) for `np.load`. Rates, resolution, FOV, noise and
mount transforms are properties, editable per instance. UE has no built-in lidar, so this one is ray traces:
about 9-12 ms per scan on the game thread with the default 5760 rays.

The low camera feeds `UForkliftAutoPickupComponent`. Press `P` (or launch with `-ForkliftAutoPickup`) while two
`pallet_pocket` detections are visible. The controller requires both pockets, uses their synchronized depth to
correct lateral/yaw/height error, approaches slowly, inserts a bounded odometry distance after the openings leave
view, settles, and raises the load. It aborts on target loss, unsafe close-range misalignment, or timeout. Run
`Scripts/ml/serve_detector.py` first; the existing model and `/detect` endpoint are reused for both cameras.
For a reproducible end-to-end run, use `-ForkliftAutoPickupTest`; it parks square to the loose pallet, runs at a
fixed 60 Hz step, logs `[AutoPickup]` state changes, and exits 0 on completion (nonzero on abort/timeout).

Things to know: the lidar's rays ignore the truck's own body (no self-occlusion, optimistic against a real
unit); a roof lidar limited to +-15 deg cannot see anything on the floor within ~8 m, so pallets and bins are
invisible to it (the camera sees them); the hall has no ceiling, so the upper channels return nothing.

## Kit -> UE mapping

| Kit (`extension.py`) | Unreal |
|---|---|
| `_create_factory` (floor, 4 walls, racks + collider boxes, pallets, bins, guard rails, test box) | `AWarehouse` (`Warehouse.cpp`). Regenerates in `OnConstruction`; **Rebuild** button in Details = old Create/Reset Factory |
| Same coordinates, meters, right-handed | cm, left-handed: X, Z x100, **Y negated** (Kit y=+4.5 is UE y=-450) |
| `_add_forklift` at (0, -6.75, 0), `/World/Vehicles/Forklift` | `AForkliftVehicle` placed in the map at (0, 675, 5); `AForkliftGameMode` spawns one if a map has none |
| Enable/Disable Driving, WASD/RF | always on; same keys |
| Lights: Dome 650 + Distant 2500 | Sun + Sky Atmosphere + Sky Light (Lumen GI) |
| 0.01 scale on every NVIDIA reference (cm assets in a metre stage) | not needed, UE is cm |

## What is different, and why

- **No mast tilt (T/G).** `forklift_c.usd` has no tilt joint. Reading its rig showed 7 joints: 4 wheels,
  2 rear `*_rotator_joint` (vertical axis, +-45 deg) and `lift_joint` (0-2 m). The Kit keyword matcher for
  "steer" and "tilt" could not match any of them either (nothing is named steer/tilt), so those two controls
  never worked in the Kit prototype.
- **Different physics.** The 7 PhysX joints as a Chaos constraint chain were unstable (20-60 kg wheels on an
  1800 kg body blew up). The truck is now one rigid body, each wheel a raycast spring + friction tyre, the
  front wheels driven at the Kit's 360 deg/s, rear wheels steered +-30 deg at 60 deg/s, and the mast a
  kinematic body (pallets still collide with the forks). Same layout and controls; not the same solver.
- Steering **auto-centres** on key release (the Kit held the last angle). Fork lift rate is the Kit's 0.4 m/s.
- Materials are approximated: the NVIDIA MDL materials and UDIM atlas don't import, so the forklift's colours
  are sampled from its own albedo/ORM atlas (paint uses the asset's yellow variant tint) and the props use
  hand-picked colours. Decals are dropped.
- The loose pickup pallet uses five analytic boxes that preserve its two fork openings; other pallets, bins and
  guard rails use simple box collision (the Kit had none), and racks keep the Kit's proxy boxes.
- GI/reflections use Lumen and virtual shadow maps are off (`Config/DefaultEngine.ini`). If it's too heavy,
  set `r.DynamicGlobalIlluminationMethod=0`, `r.ReflectionMethod=2` - shadows get much darker without GI.

## Verifying

Headless drive test (writes `[SelfTest]` lines: distance, yaw, lift for forward / turns / reverse / lift):

```
UnrealEditor-Cmd.exe ForkliftTwin.uproject /Game/Maps/Warehouse -game -nullrhi -unattended -ForkliftSelfTest -stdout
```

Screenshot the real game view (presets 0 chase, 1 front-left, 2 rear-right, 3 whole hall):

```
UnrealEditor.exe ForkliftTwin.uproject /Game/Maps/Warehouse -game -windowed -ResX=1600 -ResY=900 -ForkliftShot=1 -ForkliftShotViewport
```

Sensor check (needs a real renderer, not `-nullrhi`): parks the truck facing the pallets, writes a camera PNG and
low-camera RGB/depth plus lidar `.npy` files to `Saved/SensorTest/` at fork height 0 and ~1 m, logs `[SensorTest]`
lines and quits:

```
UnrealEditor.exe ForkliftTwin.uproject /Game/Maps/Warehouse -game -windowed -ResX=960 -ResY=540 -ForkliftSensorTest
```

End-to-end auto-pick check (requires `Scripts/ml/serve_detector.py` and a real renderer):

```
UnrealEditor-Cmd.exe ForkliftTwin.uproject /Game/Maps/Warehouse -game -RenderOffscreen -unattended -ForkliftAutoPickupTest -stdout
```

Output lands in `Saved/Screenshots/WindowsEditor/`. (Git Bash rewrites `/Game/...`; set `MSYS_NO_PATHCONV=1`.)

## Layout

```
Source/ForkliftTwin/   AForkliftVehicle, AWarehouse, AForkliftGameMode (+ self-test / photo mode / sensor test),
                       UForkliftCameraComponent, ULidarSensorComponent, AForkliftHUD
Scripts/setup.py       runs the chain below
  download_assets.py   NVIDIA USDs + forklift textures -> ExternalAssets/
  bake_usd.py          flatten each forklift part / prop into one mesh in a shared frame (pxr, numpy)
  import_assets.py     (in UE) import, materials, collision, Nanite -> /Game/Forklift, /Game/Warehouse
  build_level.py       (in UE) lights + warehouse + forklift -> /Game/Maps/Warehouse
```

Gotchas found while porting, in case you touch the pipeline: the USD importer leaves
`bNeverNeedsCookedCollisionData` set, which silently discards convex-hull physics bodies (cleared in
`import_assets.py`); `unreal.Rotator` takes `(roll, pitch, yaw)`; script-spawned lights are Static (black
without baked lighting) so they are set Movable; `new_level` and re-importing over existing assets fail, so
`setup.py` wipes `Content/{Forklift,Warehouse,Maps}` first.
