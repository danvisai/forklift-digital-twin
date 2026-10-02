// Forklift digital twin - lightweight waypoint-graph path planner + pure-pursuit follower for
// autonomous navigation between named points in the warehouse (pallet pre-poses, drop-zone pre-poses).
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ForkliftPathPlanner.h"
#include "ForkliftNavComponent.generated.h"

class AForkliftVehicle;
class AWarehouse;

UENUM(BlueprintType)
enum class ENavState : uint8
{
	Idle,
	Planning,
	Following,
	Arrived,
	Blocked,
	Failed,
};

/**
 * Drives AForkliftVehicle from its current position to a destination pose without hitting the rack
 * runs or outer walls. Not a general-purpose planner: it works specifically for this warehouse's
 * layout, where the two rack runs act as two long walls splitting the floor into three lanes
 * (west/centre/east) connected only around their north/south ends. PlanRouteTo() builds a small graph
 * from a handful of hand-anchored hub waypoints (the clear spine north and south of the racks) plus
 * the current location and the destination itself (expected to already be a sensible stand-off pose,
 * e.g. FPalletSlot::PrePoseLocation, not a slot/pallet to dock into directly); connects any pair whose
 * straight segment (inflated by VehicleHalfWidthCm) doesn't cross AWarehouse::GetObstacleBoxes(); and
 * runs Dijkstra over that graph (well under 20 nodes) for the shortest ordered waypoint list.
 * TickComponent then drives it with pure pursuit, with a lidar-based forward safety stop. Arrival is
 * position-only (see TickComponent) - it does not try to guarantee the final heading matches
 * DestYawDeg, so a caller that needs a precise final pose (pickup alignment, placement approach) does
 * that fine alignment itself once Nav reports Arrived.
 *
 * Deliberately not Hybrid-A-star / Reeds-Shepp (see RESEARCH_HANDOFF.md's "Decision (2026-09-25)" for that
 * fuller design) - this room's geometry (a couple of long straight corridors and a clear spine) doesn't
 * need it, and a full planner is a much larger, riskier build to get right on this machine. Revisit if
 * pallets/slots ever need to be reachable somewhere this graph can't route to, or if dynamic obstacles
 * need more than a stop-and-wait response.
 */
UCLASS(ClassGroup = (Forklift), meta = (BlueprintSpawnableComponent))
class FORKLIFTTWIN_API UForkliftNavComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UForkliftNavComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Plans and starts driving a route to DestLoc, ending pointed at DestYawDeg. False if no route exists
	 *  or a route is already active. */
	UFUNCTION(BlueprintCallable, Category = "Forklift|Nav")
	bool PlanRouteTo(const FVector& DestLoc, float DestYawDeg);

	UFUNCTION(BlueprintCallable, Category = "Forklift|Nav")
	void CancelRoute();

	/** Planner obstacles come from the racks/walls and from every pallet the camera has found. Pallets within Radius of Point are NOT obstacles
	 *  (the one the truck is about to pick up). Radius 0 clears it. */
	void SetPlanIgnore(const FVector& Point, float Radius) { PlanIgnorePoint = Point; PlanIgnoreRadius = Radius; }

	/** True while a pallet is on the forks: the planner then uses the longer, wider outline. */
	void SetCarrying(bool bCarry) { bCarryingLoad = bCarry; }

	/** The path the truck follows: footprint-aware hybrid A* over its real steering (forward and reverse arcs), tracked pose by pose.
	 *  Off = the old hub graph + pure pursuit. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav")
	bool bUseHybridPlanner = true;

	/** Extra clearance around the map's rack / wall boxes in the planner (the real racks have guards and feet that stick out past the collider). */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "0.0"))
	float StaticInflateCm = 45.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "30.0"))
	float LookaheadForwardCm = 150.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "30.0"))
	float LookaheadReverseCm = 110.f;

	// ---- Dynamic steering / speed envelope (tracker) -------------------------------------------------------------------
	/** Comfortable lateral acceleration, cm/s^2, empty and with a pallet on the forks. The wheel-angle limit follows from it:
	 *  delta_max = atan(L * a_lat * turn_response / v^2), so full lock is allowed when crawling and less as speed rises. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav|Dynamic limits", meta = (ClampMin = "10.0"))
	float LateralAccelEmptyCmS2 = 110.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav|Dynamic limits", meta = (ClampMin = "10.0"))
	float LateralAccelLoadedCmS2 = 70.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav|Dynamic limits", meta = (ClampMin = "5.0"))
	float MinSteerLimitDeg = 18.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav|Dynamic limits", meta = (ClampMin = "5.0"))
	float MaxSteerLimitDeg = 55.f;

	/** Sideways tracking error (cm) at which the speed governor starts easing off, and the error at which it reaches its floor. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav|Dynamic limits")
	float SlowOnErrorStartCm = 25.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav|Dynamic limits")
	float SlowOnErrorFullCm = 120.f;

	/** Measured / predicted yaw rate, low-passed (1 = the truck turns as the model says, < 1 = understeer / slip). */
	float GetTurnResponse() const { return TurnResponse; }

	/** Move the final destination (and the last waypoint) while the route is running, e.g. when the camera refines where a pallet is. */
	void UpdateFinalDestination(const FVector& NewDest);

	UFUNCTION(BlueprintPure, Category = "Forklift|Nav")
	ENavState GetState() const { return State; }

	/** True while a route is being planned, followed, or temporarily blocked (i.e. still working on it). */
	UFUNCTION(BlueprintPure, Category = "Forklift|Nav")
	bool IsBusy() const { return State == ENavState::Planning || State == ENavState::Following || State == ENavState::Blocked; }

	UFUNCTION(BlueprintPure, Category = "Forklift|Nav")
	FString GetStatus() const;

	/** Remaining waypoints (world cm), next first, for HUD/debug drawing. */
	const TArray<FVector>& GetCurrentPath() const { return CurrentPath; }

	// --- Tuning ---------------------------------------------------------------------------------

	/** Half-width used to inflate obstacle boxes when checking a path segment for clearance - roughly
	 *  the vehicle's own track half-width (~50cm) plus a working buffer, not a full carried-pallet
	 *  half-width. Keep this at 70-80cm: both the hub waypoints (the ~4m gap between the rack runs at
	 *  |Y|=8.15m and the outer walls at |Y|=12m) AND the drop-zone pre-poses (only ~95cm from the west
	 *  rack's raw edge by the warehouse's own existing geometry - PrePoseStandOffCm was tuned for the
	 *  pickup/placement skills, not for this planner's clearance) get swallowed by their own nearby
	 *  obstacle above roughly 95-100cm, making every route through them fail to plan at all. The lidar
	 *  safety stop is the real-time collision layer; this margin only needs to keep the planned
	 *  centreline geometrically plausible. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "40.0"))
	float VehicleHalfWidthCm = 70.f;

	/** Distance to a waypoint at which it counts as reached and the follower advances to the next one. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "20.0"))
	float WaypointToleranceCm = 100.f;

	/** Distance to the final destination at which the route counts as Arrived. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "10.0"))
	float ArrivalToleranceCm = 40.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float CruiseThrottle = 0.55f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float FinalApproachThrottle = 0.2f;

	/** Forward look-ahead distance for the lidar safety stop. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "50.0"))
	float SafetyStopLookaheadCm = 150.f;

	/** Half-width of the forward safety corridor the lidar check uses. Narrowed from an initial 90cm:
	 *  at 90cm, a rack corner off to the side while the truck was mid-turn (not on a collision course,
	 *  just nearby) was enough to trigger Blocked - see HubSpotsM's comment in the .cpp for the measured
	 *  case. 55cm is a little over the vehicle's own half-track width, so it still catches a genuine
	 *  head-on obstacle. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "20.0"))
	float SafetyStopCorridorHalfWidthCm = 55.f;

	/** The low lidar (30 cm up, 6 channels from -12 to +3 deg) sees floor-level objects the roof lidar cannot see closer than ~2.5 m
	 *  (a flat pallet, a tote, a person's feet). The truck stops if one is within this distance beyond the fork tips. The tines and a
	 *  carried load are masked out of its returns. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "20.0"))
	float LowLidarStopBeyondTipsCm = 70.f;

	/** How long the path ahead can stay Blocked before the route gives up outright. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "1.0"))
	float BlockedTimeoutSec = 12.f;

	/** Overall route timeout, from PlanRouteTo to Arrived/Failed. A multi-leg route across the room
	 *  (e.g. around a rack run to a far drop slot) is ~24m of path at FinalApproachThrottle through most
	 *  of the turns; the west-zone test route measured 95 s, so this needs real headroom over that. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "1.0"))
	float RouteTimeoutSec = 420.f;

	/** Before the last leg the route passes a line-up point this far from the destination ON its heading axis (behind
	 *  it when the truck will arrive driving forward, ahead of it when it will arrive reversing - whichever side is
	 *  nearer and clear). The last leg is then a straight run along the axis, so the truck arrives pointing the
	 *  right way instead of orbiting a target that sits inside its turning circle. 0 disables it. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "0.0"))
	float LineUpCm = 250.f;

	/** Look-ahead of the carrot the last leg chases along the destination's heading axis (0 = chase the point). */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "0.0"))
	float AxisCarrotCm = 120.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "2.0"))
	float StuckSec = 6.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "0.5"))
	float RecoverSec = 3.f;

	/** A rear-steered truck cannot turn in place: a target more than this far off the nose is reached by reversing
	 *  toward it (rear-steer reverses like a car going forward) instead of a wide forward U-turn that sweeps whatever
	 *  stands beside the path. It switches back to forward below ReverseExitDeg (hysteresis). */
	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "60.0", ClampMax = "180.0"))
	float ReverseEnterDeg = 100.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "20.0", ClampMax = "120.0"))
	float ReverseExitDeg = 70.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Nav", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float ReverseThrottle = 0.35f;

	/** When true, the follower also servos the fork lift to HoldLiftCm while driving (otherwise it leaves
	 *  the lift command at zero). Set by the caller before PlanRouteTo: e.g. lowered when heading to a
	 *  pallet, the current carry height when transporting one. */
	bool bHoldLift = false;
	float HoldLiftCm = 0.f;

private:
	bool BuildGraph(const FVector& Start, const FVector& Dest, float DestYawDeg, TArray<FVector>& OutWaypoints) const;
	bool IsSegmentClear(const FVector& A, const FVector& B) const;
	bool IsLidarPathBlocked(bool bReverse, float MaxAheadCm) const;
	bool PlanHybrid(const FVector& DestLoc, float DestYawDeg);
	void TickDense(float DeltaTime);
	bool IsLowLidarBlocked() const;
	bool IsLowLidarBlockedAlongPath() const;
	float LowScanTimer = 0.f;
	mutable bool bLoggedLowBlock = false;
	mutable bool bLoggedBlock = false;
	void SetState(ENavState NewState, const FString& Detail = FString());
	void HoldStill();
	void Finish(ENavState EndState, const FString& Detail);

	TWeakObjectPtr<AForkliftVehicle> Vehicle;
	TWeakObjectPtr<AWarehouse> Warehouse;

	ENavState State = ENavState::Idle;
	FString StatusDetail;

	TArray<FVector> CurrentPath;   // remaining waypoints, front = next target
	FVector FinalDestination = FVector::ZeroVector;
	float FinalYawDeg = 0.f;

	float StateTime = 0.f;
	float RouteTime = 0.f;
	float BlockedTime = 0.f;
	bool bReversing = false;

	TArray<FPlanPose> DensePath;   // hybrid-A* path in origin poses; empty when the old waypoint follower is in use
	int32 DenseIdx = 0;
	float CuspHold = 0.f;
	TArray<FBox2D> DynamicObstacles;   // obstacles the low lidar revealed on the path (not yet seen by the camera): the planner avoids them from now on
	mutable FVector LastBlockPoint = FVector::ZeroVector;
	int32 DynReplans = 0;
	TArray<FBox2D> PlanObstacles;   // what the current plan already avoids (racks, walls, detected pallets): the lidar stop is for everything else
	float TurnResponse = 1.f;
	float PrevYawDeg = 0.f;
	bool bHavePrevYaw = false;
	float DynSteerLimitDeg = 55.f;
	float OffPathTime = 0.f;
	FVector PlanIgnorePoint = FVector::ZeroVector;
	float PlanIgnoreRadius = 0.f;
	bool bCarryingLoad = false;

	// Stuck recovery: the truck can wedge itself (forks against a wall after a corner it cannot turn). If it has barely moved
	// for StuckSec while being driven, back out the way it came for RecoverSec with the wheel swung alternately, then resume.
	FVector StuckRefPos = FVector::ZeroVector;
	float StuckTimer = 0.f;
	float RecoveryTimer = 0.f;
	float RecoveryThrottle = 0.f;
	float RecoverySteer = 0.f;
	float LastThrottle = 0.f;
	int32 RecoveryCount = 0;
};
