# Classical baseline (hub-graph Nav + pure pursuit + vision pickup), 2026-09-30

The numbers the learned navigation has to beat. All headless, fixed 60 Hz step, detector server on, one pallet per mission.

## Mission: `-ForkliftMissionTest -MissionPallets=1 -MissionEpisodes=5 -MissionJitter -MissionSeed=7`

Pallet at UE (250, 450) (Kit (2.5, -4.5)), jittered up to +-20 cm / +-5 deg per episode; truck spawns at (0, 675) facing +X; drop slot `Empty_4`.

| episode | result | score | pickup attempts | set-down error | sim time |
|---|---|---|---|---|---|
| 0 | pass | 76.8 | 1 | 12.4 cm, 1.8 deg | 134 s |
| 1 | pass | 77.5 | 1 | 9.9 cm, 2.4 deg | 128 s |
| 2 | pass | 76.1 | 1 | 11.0 cm, 2.5 deg | 134 s |
| 3 | pass | 76.8 | 1 | 13.9 cm, 1.4 deg | 134 s |
| 4 | pass | 76.8 | 1 | 14.6 cm, 1.2 deg | 135 s |

Mean score 76.8, 5/5 delivered, set-down error <= 15 cm / 2.5 deg. Raw records: `Saved/Missions/deliveries.jsonl`, `episodes.jsonl`.

## Navigation: `-ForkliftNavTest` (spawn -> west-zone slot `Empty_4` pre-pose)

Passes in 47.4 sim-s (was 95 s before the reverse manoeuvre and axis-following last leg).

## What the classical stack does (and where it is limited)

* Global route: hub graph + Dijkstra (`UForkliftNavComponent::BuildGraph`), plus a line-up point 250 cm out on the destination heading axis.
* Local driving: pure pursuit; reverses toward a target more than 100 deg off the nose (switches back below 70); the last leg chases a
  120 cm carrot on the destination heading axis.
* Safety: roof-lidar stop (forward or rear corridor, ignores anything beyond the target). The lidar cannot see floor-level objects
  closer than ~2.5 m (roof, -25..+10 deg), which is what the low lidar + learned clearance net are for.
* Known limits: no obstacle avoidance beyond the stop; loose pallets are not in the planner; pallet spot and staging distance were tuned
  by hand (the staging point must stay east of x ~ -450 because the west rack is ~1.2 m behind x = -525).
