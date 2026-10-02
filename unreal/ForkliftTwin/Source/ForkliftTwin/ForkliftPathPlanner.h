// Forklift digital twin - kinematically feasible, footprint-aware path planner (hybrid A*).
#pragma once

#include "CoreMinimal.h"

/** One pose on a planned path, in the truck's ORIGIN frame (the actor origin, what the follower tracks). */
struct FPlanPose
{
	FVector2D P = FVector2D::ZeroVector;   // world cm
	float YawDeg = 0.f;                    // UE yaw (deg, from +X toward +Y)
	int8 Dir = 1;                          // +1 driving forward, -1 reversing
	float SteerDeg = 0.f;                  // planned rear-wheel angle on this arc (left positive): feed-forward for the tracker
};

struct FPlannerConfig
{
	// Truck geometry, from the system identification (Scripts/ml/nav/vehicle_model.py) and the rig (AForkliftVehicle):
	// the rear wheel steers and drives, the fixed rollers sit AxleAheadCm in front of the actor origin and do not slide sideways,
	// so the point on the fixed axle (R) moves along the heading and the heading changes at  -u * tan(steer) / Wheelbase.
	float WheelbaseCm = 141.f;
	float AxleAheadCm = 88.f;
	float FrontCm = 215.f;        // origin to the fork tips (a carried pallet extends this, see bCarrying)
	float RearCm = 85.f;
	float HalfWidthCm = 55.f;
	float MarginCm = 15.f;        // clearance kept around the outline
	float CarryExtraFrontCm = 60.f;
	float CarryExtraHalfWidthCm = 8.f;
	float CarryExtraMarginCm = 20.f;   // a pallet on the forks swings wide: keep further from racks

	// left steer positive; the hardware limit is 60 deg, but the measured turns at full lock are the least repeatable (the truck slips),
	// so the planner only uses arcs the truck can really follow: radius of the fixed axle >= L / tan(35 deg) = 2.0 m.
	TArray<float> SteerDeg = { -35.f, -18.f, 0.f, 18.f, 35.f };
	float StepCm = 80.f;          // length of one primitive (signed distance travelled by the fixed-axle point)

	float ReversePenalty = 1.35f;
	float SteerChangePenalty = 30.f;
	float CuspPenalty = 120.f;
	float HeuristicWeight = 1.3f;

	FVector2D BoundsMin = FVector2D(-1700.f, -1100.f);
	FVector2D BoundsMax = FVector2D(1700.f, 1100.f);
	float CellCm = 25.f;
	float YawBinDeg = 10.f;
	float GoalPosTolCm = 20.f;
	float GoalYawTolDeg = 5.f;
	bool bRequireGoalYaw = true;
	bool bCarrying = false;
	int32 MaxExpansions = 120000;
	double MaxSeconds = 4.0;
};

/**
 * Hybrid A*: searches the continuous poses reachable with the truck's real steering (forward and reverse arcs of StepCm), keeps a
 * discretised closed set, and rejects any primitive whose swept truck outline (body + forks, optionally + a carried pallet,
 * plus a margin) touches an obstacle box. A final analytic "shot" along a single arc connects to the goal pose once the search is close.
 * Start / goal / path poses are origin poses. Obstacles are world AABBs in XY. Returns false if no feasible path exists.
 */
class FForkliftPathPlanner
{
public:
	/** Which obstacle boxes (indices into Obstacles) the truck outline at this ORIGIN pose overlaps; empty = the pose is free. Diagnostics. */
	static TArray<int32> BoxesHitAt(const FPlannerConfig& Cfg, const TArray<FBox2D>& Obstacles, const FVector2D& Origin, float YawDeg);

	static bool Plan(const FPlannerConfig& Cfg, const TArray<FBox2D>& Obstacles, const FVector2D& StartOrigin, float StartYawDeg,
		const FVector2D& GoalOrigin, float GoalYawDeg, TArray<FPlanPose>& OutPath, int32* OutExpansions = nullptr);
};
