# Research Handoff — Autonomous Pickup, Transport, and Placement

Last updated: **2026-09-23**

## Objective

Extend the existing vision-guided pallet pickup into an autonomous material-handling mission:

> find/select a pallet → navigate to it → align and pick it up → carry it safely → place it in a dedicated location → withdraw and verify placement

The main design question is how much of this should use classical planning/control versus learned policies trained through category-based scoring.

## Main finding

The research supports a **hybrid hierarchical system**, not a single end-to-end policy that learns recognition, navigation, pickup, transport, and placement simultaneously.

Recommended mission flow:

```text
Find pallet
  → Navigate to pickup pre-pose
  → Visual alignment
  → Insert
  → Lift and verify
  → Navigate to drop pre-pose
  → Placement alignment
  → Lower
  → Withdraw
  → Verify placement
```

Use conventional planning and bounded controllers for long-range motion and safety. Introduce learning first where uncertainty is highest: final approach, alignment, recovery, and placement refinement.

## Relevant papers and findings

### Forklift navigation and system architecture

1. **Autonomous Forklift Navigation Inside a Cluttered Logistics Factory** (Lucet et al., 2024)
   - Paper: https://www.scitepress.org/Papers/2024/130676/130676.pdf
   - Decomposes navigation into a global road graph, local A* obstacle planning, constrained model-predictive path tracking, and a finite-state mission controller.
   - Validated in simulation and on a real prototype.
   - Closest architectural template for warehouse navigation in this project.

2. **ADAPT: An Autonomous Forklift for Construction Site Operation** (Huemer et al., 2025; revised 2026)
   - Paper: https://arxiv.org/abs/2503.14331
   - Combines learned perception with traditional decision-making, planning, and control.
   - Real-world testing reported near-human operational performance in difficult outdoor conditions.
   - Strong evidence against making the complete forklift mission one end-to-end learned policy.

3. **Path Following of Autonomous Forklifts via Nonlinear Model Predictive Control** (Zhang et al., 2025)
   - DOI: https://doi.org/10.1109/TMECH.2025.3573067
   - A real laboratory forklift achieved position accuracy better than 1 cm and angular accuracy within 0.5 degrees during pallet-loading positioning.
   - Indicates that precise low-level forklift motion has a strong model-based baseline; learning can be added as a residual or for recovery rather than replacing it immediately.

### Pallet perception, pickup, and placement

4. **Pallet Detection and Estimation for Fork Insertion with RGB-D Camera** (Kubo et al., 2021)
   - DOI: https://doi.org/10.1109/ICMA52036.2021.9512641
   - Uses RGB semantic segmentation plus depth to estimate pallet position and orientation during insertion, including tilted terrain.
   - Suggests evolving this project's two-pocket box centers into a more explicit relative pallet/fork pose estimate.

5. **Pallet Recognition and Localization Using an RGB-D Camera** (Xiao et al., 2017)
   - Paper: https://doi.org/10.1177/1729881417737799
   - Depth-based localization guided fork insertion with measured horizontal and vertical error below 3 cm.
   - Highlights camera-to-fork calibration and yaw accuracy as critical.

6. **Closed-loop Pallet Manipulation in Unstructured Environments** (Walter et al., 2010)
   - Paper: https://agile.mit.edu/publications/walter10a.pdf
   - Coupled perception and control enabled a real forklift to detect, pick up, transport, and place previously unknown palletized cargo.
   - Placement should be closed-loop rather than simply driving to fixed coordinates and lowering.

7. **Robotic Forklift for Stacking Multiple Pallets with RGB-D Cameras** (Iinuma et al., 2021)
   - Paper: https://www.fujipress.jp/jrm/rb/robot003300061265/
   - Multiple RGB-D cameras plus path-following control successfully aligned and stacked pallets on a real forklift.
   - A dedicated drop-zone view or additional placement camera may improve final accuracy.

8. **ICP-Based Pallet Tracking for Unloading on Inclined Surfaces by Autonomous Forklifts** (Kato and Morisawa, 2026)
   - Paper: https://arxiv.org/abs/2602.16744
   - Tracks the carried pallet during lowering, aligns the fork to the target surface, and controls withdrawal to prevent dragging.
   - Real-forklift trials succeeded with perception operating at approximately 2.5–5 Hz.
   - The placement skill needs load tracking, unload/contact verification, controlled withdrawal, and final verification.

### Learning, rewards, curriculum, and safety

9. **HRL4IN: Hierarchical Reinforcement Learning for Interactive Navigation with Mobile Manipulators** (Li et al., 2020)
   - Paper: https://proceedings.mlr.press/v100/li20a.html
   - Separating long-horizon work into navigation and manipulation subpolicies outperformed flat PPO.
   - Supports stage-specific forklift skills controlled by a mission supervisor.

10. **Hindsight Experience Replay** (Andrychowicz et al., 2017)
    - Paper: https://proceedings.neurips.cc/paper/2017/hash/453fadbd8a1a3af50a9df4df899537b5-Abstract.html
    - Re-labels failed attempts with goals actually achieved, making sparse goal-conditioned manipulation tasks much more trainable.
    - Relevant when pickup and placement targets vary by pose; pairs naturally with off-policy algorithms such as SAC or TD3.

11. **Reverse Curriculum Generation for Reinforcement Learning** (Florensa et al., 2017)
    - Paper: https://proceedings.mlr.press/v78/florensa17a.html
    - Begins near successful terminal states and expands the start-state distribution as competence improves.
    - Directly motivates training placement from an already loaded and nearly aligned forklift before full missions.

12. **Policy Invariance Under Reward Transformations: Theory and Application to Reward Shaping** (Ng, Harada, and Russell, 1999)
    - Paper: https://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/NgHaradaRussell-shaping-ICML1999.pdf
    - Potential-based progress rewards can speed learning without changing the desired optimal policy.
    - Poorly designed dense rewards can produce loops and reward exploitation.

13. **Constrained Policy Optimization** (Achiam et al., 2017)
    - Paper: https://proceedings.mlr.press/v70/achiam17a
    - Models safety as explicit constraints instead of relying only on large negative rewards.
    - Collision, load drop, clearance, and dangerous speed should be constraint signals or safety overrides.

14. **Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World** (Tobin et al., 2017)
    - Paper: https://arxiv.org/abs/1703.06907
    - Demonstrates simulated perception transferring to real robotic localization through randomized rendering.
    - Later training should vary pallet geometry, lighting, textures, friction, mass, camera calibration, depth noise, and detector errors.

## Recommended architecture

### 1. Mission supervisor

Implement a deterministic finite-state machine or behavior tree with explicit states for:

- target selection;
- navigation to pickup pre-pose;
- pickup acquisition/alignment/insertion/lift;
- load verification;
- loaded navigation;
- placement alignment/lowering/withdrawal;
- placement verification;
- recovery or abort.

The supervisor selects skills. It should initially remain deterministic even if individual skills become learned policies.

### 2. Global navigation

- Represent warehouse aisles and maneuvering regions with a navigation graph or occupancy grid.
- Use Hybrid A* or a state-lattice planner so paths respect rear-wheel steering, turning radius, direction changes, and the full forklift footprint.
- Plan to a **pre-pose**, not directly to the pallet or drop point. The local manipulation controller takes over from that pose.
- Account for the larger swept footprint of the forks and carried pallet.

### 3. Path following

- Start with a deterministic pure-pursuit or Stanley-style controller.
- Move to MPC when tighter path, footprint, steering-rate, acceleration, or carry-stability constraints are needed.
- Use separate speed profiles for empty travel, carrying, turning, final approach, and reverse withdrawal.

### 4. Pickup and placement skills

- Retain the current pickup controller as the baseline and an expert demonstration source.
- Add a separate placement controller with drop-zone pose estimation, lowering, unload verification, straight withdrawal, and post-placement verification.
- Prefer explicit relative poses over raw detector boxes where possible.

### 5. Learned policies

Introduce learning incrementally:

- first as a residual correction to a deterministic controller;
- then as a stage-local policy for approach/alignment or recovery;
- only later consider learned skill selection or an image-to-action policy.

The existing action interface already provides normalized throttle, steering, and lift commands through `AForkliftVehicle::SetDriveInput`.

## Scoring and evaluation design

Do **not** mix object identification into the same reward used for vehicle control. Perception should be trained and evaluated with supervised metrics such as precision/recall, AP, confidence calibration, depth error, relative-pose error, and detection dropout.

For control, log a reward vector with distinct categories and scalarize it only for training.

### Approach score

- decrease in distance to a valid pre-pose;
- reduction in heading error;
- path progress;
- obstacle and footprint clearance;
- steering and acceleration smoothness.

### Alignment score

- reduction in lateral fork-to-pocket error;
- reduction in vertical error;
- reduction in yaw error;
- correct longitudinal stand-off;
- stable detection of both openings.

### Insertion score

- insertion-depth progress;
- both tines entering valid openings;
- low contact impulse;
- no pallet displacement or tine collision;
- correct fork-height hold.

### Lift score

- physical load rise rather than fork motion alone;
- load centered and supported;
- no sliding, rotation, or drop;
- achievement of safe carry height.

### Transport score

- route progress toward drop pre-pose;
- collision and clearance margin;
- stable pallet pose relative to the forklift;
- low acceleration and jerk;
- efficient travel time and path length.

### Placement score

- reduction in pallet position and orientation error relative to the target zone;
- correct support/contact after lowering;
- pallet stability after the forks unload;
- clean tine withdrawal without dragging;
- final pallet pose within tolerance.

### Terminal success

Award success only when the pallet:

- is within position/yaw tolerance;
- is supported by the floor or destination rather than the forks;
- remains stable for a settling period;
- has clear separation from the withdrawn tines.

### Safety constraints

Treat these as hard aborts, safety overrides, or separate constrained-RL costs:

- vehicle or fork collision;
- excessive contact force/impulse;
- dropped or unstable load;
- excessive speed near a pallet, destination, person, or obstacle;
- leaving a permitted navigation corridor;
- invalid fork height while traveling;
- timeout or prolonged perception loss.

Use reward structure of the form:

```text
terminal success + change in goal potential - time/energy/smoothness costs
```

Reward the **change** in distance/alignment rather than being near a goal, to avoid hovering or cycling for reward.

## Training curriculum

Train in approximately the reverse order of the complete mission:

1. Place an already carried pallet from a nearly correct pose.
2. Expand placement offsets in position, yaw, and fork height.
3. Pick up from a nearly aligned pose.
4. Expand pickup offsets, heights, pallet types, and partial occlusion.
5. Navigate to a visible pickup pre-pose.
6. Carry along increasingly complex routes.
7. Chain pickup, transport, and placement.
8. Replace privileged simulator poses with noisy detector/depth-derived estimates.
9. Add obstacles, detector failures, recovery behaviors, and domain randomization.

Start policy observations with compact state features:

- goal-relative position and orientation;
- pallet/pocket-relative pose;
- lift height;
- forward speed and steering angle;
- lidar sectors or local clearance features;
- carried-pallet pose relative to the forks;
- current mission stage.

Do not start by training directly from full camera pixels. Images can enter through the existing detector and a later relative-pose estimator once the control task is working and measurable.

## Current project fit

The project already contains most of the local pickup foundation:

- `ForkliftAutoPickupComponent` has acquisition, alignment, insertion, settling, and lifting states.
- The tine RGB-D camera and detector produce relative lateral, vertical, longitudinal, and yaw errors.
- `AForkliftVehicle::SetDriveInput(throttle, steer, lift)` is suitable as either a scripted or RL action interface.
- `ForkliftAutoPickupTest` already provides a fixed-step automated test and verifies physical pallet rise.

Missing capabilities are:

- target/destination selection;
- warehouse localization and route planning;
- obstacle-aware path following with the full swept footprint;
- loaded driving behavior;
- drop-zone perception;
- placement, unload, withdrawal, and final-pose verification;
- randomized episode reset, metrics, and training interfaces.

## Simulator limitations that affect learning

1. **Kinematic lift and unrealistic load dynamics**
   - Carried pallet weight is not transmitted to the chassis.
   - The vehicle cannot currently learn realistic rollover or load-induced handling effects.
   - There is no mast-tilt joint.
   - Load-stability and safety results will therefore be optimistic.

2. **Near-ground lidar blind region**
   - The roof lidar's ±15 degree vertical range cannot see floor-level pallets and bins at close range.
   - Use it for walls, racks, and general navigation, but keep near-field manipulation and pallet safety camera/depth-based unless another low sensor is added.

3. **Fixed timestep required**
   - Data-generation and learning runs should use a fixed physics step, as the existing pallet and auto-pick tests do.

## Proposed first implementation milestone

Build a deterministic and measurable end-to-end baseline before training RL:

- one selectable pallet;
- one fixed, floor-level destination zone;
- static warehouse obstacles;
- navigation to a pickup pre-pose;
- existing vision-guided pickup;
- carry-height and carry-speed limiting;
- navigation to a placement pre-pose;
- closed-loop lower → unload → withdraw → verify placement;
- randomized test harness covering at least 100 pickup/drop configurations.

Track at minimum:

- success rate per stage and end-to-end;
- final pallet position and yaw error;
- collisions and peak contact impulse;
- pallet drops/slips;
- path length and mission time;
- pickup/placement retries and recovery rate;
- detector and pose-estimation dropout.

Once this baseline produces stage-level failure data, choose the first learned skill based on measured weakness. The likely first candidates are recovery from pickup misalignment and final placement alignment, not global route planning.

## Decisions to make before implementation

1. Is the first destination a marked floor zone, a pallet stack, or a rack location?
2. Is the first scope a known static warehouse map, or must the forklift discover/map it?
3. Are privileged simulator poses acceptable during early training, followed by perception noise and eventual visual observations?
4. Is the goal simulation autonomy only, or eventual real-forklift transfer?
5. Should the first navigation baseline be a simple warehouse waypoint graph or a full Hybrid A* occupancy-grid planner?


## Decision (2026-09-25): pathfinding approach

**Use classical planning, not a learned pathfinder.** Stack, in build order:

1. **Global planner: Hybrid A\*** over a 2D occupancy grid (10 cm cells, 36 x 24 m). State = (x, y, heading, direction),
   motion primitives from the forklift's rear-steer kinematics (min turn radius from `ForkliftVehicle`), footprint
   check with body + tines (+ carried pallet when loaded). Goal = slot **pre-pose** (`FPalletSlot::PrePoseLocation/Yaw`),
   never the pallet itself. Reeds-Shepp analytic expansion for the last stretch.
2. **Tracker: pure pursuit** with separate speed profiles (empty / loaded / turning / reverse), outputs
   `SetDriveInput(throttle, steer, lift)`. Swap to MPC only if measured tracking error demands it.
3. **Lidar = local costmap + safety layer.** Project the 16-ch scan to a rolling 2D grid, inflate by footprint, merge
   with the static map, replan when the path is blocked. Independent speed limit / stop from the nearest return in the
   swept-footprint sector. Blind spot (lidar +-15 deg does not see floor-level pallets/bins): the tine RGB-D depth and
   the detector boxes feed the same grid for the near field; slot occupancy comes from `AWarehouse::IsSlotOccupied`.
4. **Localization:** use simulator ground-truth pose now; add scan-matching only for real-vehicle transfer.
5. **Learning later, and narrow:** residual/recovery policy for final alignment and placement, per the curriculum above.

**Why not RL/learned navigation now:** the map is known and small, the vehicle is non-holonomic (a planner handles
that exactly), no safety guarantees or debuggability from a learned policy, and it would need a large training
harness before it matches a 200-line planner. Research (Lucet 2024, ADAPT 2025) uses exactly this split.

**Drop zones (implemented):** `AWarehouse::BuildDropZones`. West zone x=-12 m = 4 EMPTY slots (blue), east zone
x=+12 m = 4 LOADED slots (green), Kit y = 5, 3, 1, -1 m (UE y negated), 2 m pitch. API: `GetSlots`, `GetSlotsFor`,
`IsSlotOccupied`, `FindFreeSlot`. Empty pallets enter heading west (yaw 180), loaded heading east (yaw 0). Pre-pose is
`PrePoseStandOffCm` (350) from the slot toward the aisle. Rows y=3 and y=-1 line up with rack banks, so routes to them
must go around the racks (intentional planner test).
