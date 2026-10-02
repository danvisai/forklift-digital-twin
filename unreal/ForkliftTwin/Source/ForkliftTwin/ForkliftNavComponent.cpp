#include "ForkliftNavComponent.h"

#include "EngineUtils.h"
#include "Engine/OverlapResult.h"
#include "ForkliftVehicle.h"
#include "LidarSensorComponent.h"
#include "ForkliftPalletFinderComponent.h"
#include "Warehouse.h"

DEFINE_LOG_CATEGORY_STATIC(LogForkliftNav, Log, All);

namespace
{
	const TCHAR* StateName(ENavState State)
	{
		switch (State)
		{
		case ENavState::Idle: return TEXT("idle");
		case ENavState::Planning: return TEXT("planning");
		case ENavState::Following: return TEXT("following");
		case ENavState::Arrived: return TEXT("arrived");
		case ENavState::Blocked: return TEXT("blocked");
		case ENavState::Failed: return TEXT("failed");
		default: return TEXT("unknown");
		}
	}

	// Kit meters (right-handed) -> UE cm (left-handed), matching Warehouse.cpp's convention.
	FVector UsdToUe(double XM, double YM) { return FVector(XM * 100.0, -YM * 100.0, 0.0); }

	// Hub waypoints spanning the full room width at Y = +-9.5 m (Kit) - between the rack runs
	// (|Y|=8.15m raw) and the outer walls (|Y|=12m raw). This is a real-vehicle-turning compromise, not
	// just a straight-line clearance check: a pure-pursuit follower cuts corners while turning onto and
	// off of a hub, so the truck's actual swept path passes closer to both the rack's end corners and
	// the wall than the waypoint-to-waypoint straight-line distances suggest - confirmed by the lidar
	// safety stop firing, at |Y|=10.5m, against the real wall while turning onto a hub, and, at |Y|=9.5m,
	// against the west rack's actual NW corner while turning off one (SafetyStopCorridorHalfWidthCm was
	// narrowed to compensate for the latter - see its comment). |Y|=10.0m made the wall case worse
	// rather than splitting the difference, so this isn't a simple linear tradeoff; 9.5m is the better
	// of the two measured points. A cleaner fix would give corner hubs their own pull-in point, wide
	// enough that no single leg needs a turn sharp enough to cut a corner - worth doing if this still
	// doesn't generalize to other routes.
	const FVector2D HubSpotsM[] = {
		FVector2D(-13.0, 9.5), FVector2D(-4.0, 9.5), FVector2D(4.0, 9.5), FVector2D(13.0, 9.5),
		FVector2D(-13.0, -9.5), FVector2D(-4.0, -9.5), FVector2D(4.0, -9.5), FVector2D(13.0, -9.5),
	};
}

UForkliftNavComponent::UForkliftNavComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UForkliftNavComponent::BeginPlay()
{
	Super::BeginPlay();
	Vehicle = Cast<AForkliftVehicle>(GetOwner());
	for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
	{
		Warehouse = *It;
		break;
	}
}

bool UForkliftNavComponent::IsSegmentClear(const FVector& A, const FVector& B) const
{
	const AWarehouse* WH = Warehouse.Get();
	if (!WH)
	{
		return true;
	}
	for (const FBox& Obstacle : WH->GetObstacleBoxes())
	{
		const FBox Inflated = Obstacle.ExpandBy(VehicleHalfWidthCm);
		FVector HitLocation, HitNormal;
		float HitTime = 0.f;
		if (FMath::LineExtentBoxIntersection(Inflated, A, B, FVector::ZeroVector, HitLocation, HitNormal, HitTime))
		{
			return false;
		}
	}
	return true;
}

bool UForkliftNavComponent::BuildGraph(const FVector& Start, const FVector& Dest, float DestYawDeg, TArray<FVector>& OutWaypoints) const
{
	const AWarehouse* WH = Warehouse.Get();
	if (!WH)
	{
		return false;
	}

	// Note: Dest is expected to be a pre-pose (already placed well clear of racks/walls by whoever
	// asked for the route - e.g. FPalletSlot::PrePoseLocation), not the slot/pallet itself, so it is
	// checked for clearance exactly like every other node below. An earlier version inserted a
	// synthetic "final approach" node a fixed distance behind Dest along DestYawDeg and exempted that
	// last hop from obstacle checking, meant to guarantee the last leg arrives pointed the right way;
	// for a destination on the far side of a rack run from that synthetic node, the exempted hop cut
	// straight through the rack (caught by the lidar safety stop, but the route itself was never
	// valid). Final heading isn't enforced on arrival (see TickComponent) - the caller's own skill
	// (pickup alignment, placement approach) does that fine alignment once Nav has it in the area.
	TArray<FVector> Nodes;
	Nodes.Add(Start);   // index 0
	for (const FVector2D& Hub : HubSpotsM)
	{
		Nodes.Add(WH->GetActorTransform().TransformPosition(UsdToUe(Hub.X, Hub.Y)));
	}
	const int32 DestIndex = Nodes.Add(Dest);

	const int32 N = Nodes.Num();
	TArray<TArray<float>> Cost;
	Cost.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		Cost[i].Init(-1.f, N);
	}
	for (int32 i = 0; i < N; ++i)
	{
		for (int32 j = i + 1; j < N; ++j)
		{
			if (IsSegmentClear(Nodes[i], Nodes[j]))
			{
				const float D = FVector::Dist2D(Nodes[i], Nodes[j]);
				Cost[i][j] = D;
				Cost[j][i] = D;
			}
		}
	}

	TArray<float> Dist;
	TArray<int32> Prev;
	TArray<bool> Visited;
	Dist.Init(BIG_NUMBER, N);
	Prev.Init(INDEX_NONE, N);
	Visited.Init(false, N);
	Dist[0] = 0.f;
	for (int32 Iter = 0; Iter < N; ++Iter)
	{
		int32 Best = INDEX_NONE;
		float BestDist = BIG_NUMBER;
		for (int32 i = 0; i < N; ++i)
		{
			if (!Visited[i] && Dist[i] < BestDist)
			{
				BestDist = Dist[i];
				Best = i;
			}
		}
		if (Best == INDEX_NONE)
		{
			break;
		}
		Visited[Best] = true;
		for (int32 j = 0; j < N; ++j)
		{
			if (Cost[Best][j] >= 0.f && !Visited[j] && Dist[Best] + Cost[Best][j] < Dist[j])
			{
				Dist[j] = Dist[Best] + Cost[Best][j];
				Prev[j] = Best;
			}
		}
	}

	if (Dist[DestIndex] >= BIG_NUMBER)
	{
		return false;
	}

	TArray<int32> PathIndices;
	for (int32 At = DestIndex; At != INDEX_NONE; At = Prev[At])
	{
		PathIndices.Insert(At, 0);
	}
	OutWaypoints.Reset();
	for (int32 Idx : PathIndices)
	{
		OutWaypoints.Add(Nodes[Idx]);
	}
	if (OutWaypoints.Num() > 0)
	{
		OutWaypoints.RemoveAt(0);   // drop the start node - we're already there
	}
	if (OutWaypoints.Num() == 0)
	{
		return false;
	}

	// Line-up point on the destination's heading axis, on whichever side the truck reaches first.
	if (LineUpCm > 0.f)
	{
		const FVector PrevPt = OutWaypoints.Num() >= 2 ? OutWaypoints[OutWaypoints.Num() - 2] : Start;
		const FVector Axis = FRotator(0.f, DestYawDeg, 0.f).Vector();
		float BestDist = BIG_NUMBER;
		FVector Best = FVector::ZeroVector;
		for (const float Side : { -1.f, 1.f })
		{
			const FVector Cand = Dest + Axis * (Side * LineUpCm);
			if (FVector::Dist2D(PrevPt, Dest) > LineUpCm * 1.2f && IsSegmentClear(PrevPt, Cand) && IsSegmentClear(Cand, Dest) && FVector::Dist2D(PrevPt, Cand) < BestDist)
			{
				BestDist = FVector::Dist2D(PrevPt, Cand);
				Best = Cand;
			}
		}
		if (BestDist < BIG_NUMBER)
		{
			OutWaypoints.Insert(Best, OutWaypoints.Num() - 1);
		}
	}
	return true;
}

bool UForkliftNavComponent::PlanRouteTo(const FVector& DestLoc, float DestYawDeg)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	if (!Truck || IsBusy())
	{
		return false;
	}
	SetState(ENavState::Planning);
	DynamicObstacles.Reset();
	DynReplans = 0;
	if (bUseHybridPlanner && PlanHybrid(DestLoc, DestYawDeg))
	{
		FinalDestination = DestLoc;
		FinalYawDeg = DestYawDeg;
		RouteTime = 0.f;
		BlockedTime = 0.f;
		bReversing = false;
		StuckRefPos = Truck->GetActorLocation();
		StuckTimer = 0.f;
		RecoveryTimer = 0.f;
		RecoveryCount = 0;
		LastThrottle = 0.f;
		SetState(ENavState::Following, FString::Printf(TEXT("%d poses"), DensePath.Num()));
		return true;
	}
	TArray<FVector> Waypoints;
	if (!BuildGraph(Truck->GetActorLocation(), DestLoc, DestYawDeg, Waypoints))
	{
		Finish(ENavState::Failed, TEXT("no route found"));
		return false;
	}
	CurrentPath = Waypoints;
	bReversing = false;
	StuckRefPos = Truck->GetActorLocation();
	StuckTimer = 0.f;
	RecoveryTimer = 0.f;
	RecoveryCount = 0;
	LastThrottle = 0.f;
	FinalDestination = DestLoc;
	FinalYawDeg = DestYawDeg;
	RouteTime = 0.f;
	BlockedTime = 0.f;
	SetState(ENavState::Following, FString::Printf(TEXT("%d waypoints"), CurrentPath.Num()));
	UE_LOG(LogForkliftNav, Log, TEXT("[Nav] route planned: %d waypoints to %s"), CurrentPath.Num(), *DestLoc.ToString());
	return true;
}

bool UForkliftNavComponent::PlanHybrid(const FVector& DestLoc, float DestYawDeg)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	const AWarehouse* WH = Warehouse.Get();
	if (!Truck || !WH)
	{
		return false;
	}
	TArray<FBox2D> Obstacles;
	for (const FBox& B : WH->GetObstacleBoxes())
	{
		// The map's rack boxes are the collider of the rack run; the real rack has end guards, shields and column feet that stick out
		// past it (the truck wedged on the north-west rack corner with 15 cm 'free' in the plan), so keep a further StaticInflateCm.
		Obstacles.Add(FBox2D(FVector2D(B.Min.X - StaticInflateCm, B.Min.Y - StaticInflateCm), FVector2D(B.Max.X + StaticInflateCm, B.Max.Y + StaticInflateCm)));
	}
	int32 PalletObstacles = 0;
	if (const UForkliftPalletFinderComponent* Finder = Truck->GetFinder())
	{
		for (const FPalletCandidate& C : Finder->GetCandidates())
		{
			if (C.Hits < 2 || (PlanIgnoreRadius > 0.f && FVector::Dist2D(C.Pos, PlanIgnorePoint) < PlanIgnoreRadius))
			{
				continue;
			}
			Obstacles.Add(FBox2D(FVector2D(C.Pos.X - 65.f, C.Pos.Y - 60.f), FVector2D(C.Pos.X + 65.f, C.Pos.Y + 60.f)));
			++PalletObstacles;
		}
	}
	for (const FBox2D& D : DynamicObstacles)
	{
		Obstacles.Add(D);
	}
	FPlannerConfig Cfg;
	Cfg.bCarrying = bCarryingLoad;
	TArray<FPlanPose> Path;
	int32 Expansions = 0;
	const double T0 = FPlatformTime::Seconds();
	const bool bOk = FForkliftPathPlanner::Plan(Cfg, Obstacles, FVector2D(Truck->GetActorLocation().X, Truck->GetActorLocation().Y), Truck->GetActorRotation().Yaw,
		FVector2D(DestLoc.X, DestLoc.Y), DestYawDeg, Path, &Expansions);
	UE_LOG(LogForkliftNav, Log, TEXT("[Nav] hybrid A* %s: %d expansions, %.2f s, %d obstacle boxes (%d detected pallets), %d poses%s"), bOk ? TEXT("planned") : TEXT("found no path"),
		Expansions, FPlatformTime::Seconds() - T0, Obstacles.Num(), PalletObstacles, Path.Num(), Cfg.bCarrying ? TEXT(", carrying") : TEXT(""));
	if (!bOk)
	{
		const FVector2D S(Truck->GetActorLocation().X, Truck->GetActorLocation().Y);
		const TArray<int32> StartHits = FForkliftPathPlanner::BoxesHitAt(Cfg, Obstacles, S, Truck->GetActorRotation().Yaw);
		const TArray<int32> GoalHits = FForkliftPathPlanner::BoxesHitAt(Cfg, Obstacles, FVector2D(DestLoc.X, DestLoc.Y), DestYawDeg);
		const auto Describe = [&Obstacles](const TArray<int32>& Hits)
		{
			FString S2;
			for (int32 I : Hits)
			{
				S2 += FString::Printf(TEXT("[%.0f,%.0f..%.0f,%.0f] "), Obstacles[I].Min.X, Obstacles[I].Min.Y, Obstacles[I].Max.X, Obstacles[I].Max.Y);
			}
			return S2.IsEmpty() ? FString(TEXT("none")) : S2;
		};
		UE_LOG(LogForkliftNav, Log, TEXT("[Nav] no-path diagnosis: start (%.0f, %.0f yaw %.0f) overlaps %s| goal (%.0f, %.0f yaw %.0f) overlaps %s"),
			S.X, S.Y, Truck->GetActorRotation().Yaw, *Describe(StartHits), DestLoc.X, DestLoc.Y, DestYawDeg, DestLoc.X * 0.f + 0.f == 0.f ? *Describe(GoalHits) : TEXT(""));
		return false;
	}
	DensePath = Path;
	PlanObstacles = Obstacles;
	DenseIdx = 0;
	CuspHold = 0.f;
	OffPathTime = 0.f;
	TurnResponse = 1.f;
	bHavePrevYaw = false;
	CurrentPath.Reset();                       // HUD route: every 6th pose
	for (int32 I = 5; I < DensePath.Num(); I += 6)
	{
		CurrentPath.Add(FVector(DensePath[I].P.X, DensePath[I].P.Y, DestLoc.Z));
	}
	CurrentPath.Add(DestLoc);
	int32 Cusps = 0;
	for (int32 I = 1; I < DensePath.Num(); ++I)
	{
		Cusps += DensePath[I].Dir != DensePath[I - 1].Dir ? 1 : 0;
	}
	UE_LOG(LogForkliftNav, Log, TEXT("[Nav] path: %d direction changes, ends at %s yaw %.0f"), Cusps, *FVector(DensePath.Last().P.X, DensePath.Last().P.Y, 0.f).ToString(), DensePath.Last().YawDeg);
	return true;
}

void UForkliftNavComponent::CancelRoute()
{
	if (IsBusy())
	{
		Finish(ENavState::Idle, TEXT("cancelled"));
	}
}

void UForkliftNavComponent::UpdateFinalDestination(const FVector& NewDest)
{
	if (State != ENavState::Following && State != ENavState::Blocked)
	{
		return;
	}
	if (DensePath.Num() > 0)
	{
		if (FVector::Dist2D(NewDest, FinalDestination) > 12.f && PlanHybrid(NewDest, FinalYawDeg))
		{
			FinalDestination = NewDest;
		}
		return;
	}
	FinalDestination = NewDest;
	if (CurrentPath.Num() > 0)
	{
		CurrentPath.Last() = NewDest;
	}
}

void UForkliftNavComponent::HoldStill()
{
	if (AForkliftVehicle* Truck = Vehicle.Get())
	{
		Truck->SetDriveInput(0.f, 0.f, 0.f);
	}
}

void UForkliftNavComponent::SetState(ENavState NewState, const FString& Detail)
{
	State = NewState;
	StateTime = 0.f;
	StatusDetail = Detail;
}

void UForkliftNavComponent::Finish(ENavState EndState, const FString& Detail)
{
	HoldStill();
	CurrentPath.Reset();
	DensePath.Reset();
	bReversing = false;
	SetState(EndState, Detail);
	UE_LOG(LogForkliftNav, Log, TEXT("[Nav] %s%s%s"), StateName(EndState), Detail.IsEmpty() ? TEXT("") : TEXT(": "), *Detail);
}

bool UForkliftNavComponent::IsLowLidarBlockedAlongPath() const
{
	// The planner already steers around everything the camera has found. This is the safety net for what it has NOT found: any
	// floor-level return the low lidar sees inside the truck's outline at a pose of the next ~2.5 m of planned forward path.
	const AForkliftVehicle* Truck = Vehicle.Get();
	const ULidarSensorComponent* Low = Truck ? Truck->GetLowLidar() : nullptr;
	if (!Low || DensePath.Num() == 0 || DensePath[DenseIdx].Dir < 0)
	{
		return false;
	}
	const FTransform BodyXf = Truck->GetActorTransform();
	const TArray<FVector>& Pts = Low->GetPointsRef();
	const TArray<float> Ranges = Low->GetRangesCm();
	const int32 Steps = FMath::Max(Low->HorizontalSteps, 1);
	TArray<float, TInlineAllocator<512>> FloorZ;
	{
		int32 K = 0;
		for (int32 I = 0; I < Ranges.Num() && K <= Pts.Num(); ++I)
		{
			if (Ranges[I] < 0.f)
			{
				continue;
			}
			if (I / Steps <= 1 && K < Pts.Num())
			{
				FloorZ.Add(Low->GetComponentTransform().TransformPosition(Pts[K]).Z);
			}
			++K;
		}
	}
	float FloorBias = 0.f;
	if (FloorZ.Num() > 20)
	{
		FloorZ.Sort();
		FloorBias = FloorZ[FloorZ.Num() / 2];
	}
	int32 SegEnd = DenseIdx;
	while (SegEnd + 1 < DensePath.Num() && DensePath[SegEnd + 1].Dir == DensePath[DenseIdx].Dir)
	{
		++SegEnd;
	}
	const float Front = 215.f + (bCarryingLoad ? 60.f : 0.f) + 30.f, Rear = 95.f, Half = 55.f + (bCarryingLoad ? 8.f : 0.f) + 15.f;
	for (const FVector& P : Pts)
	{
		const FVector W = Low->GetComponentTransform().TransformPosition(P);
		if (W.Z - FloorBias < 8.f)
		{
			continue;   // floor
		}
		const FVector B = BodyXf.InverseTransformPosition(W);
		if (B.X > 35.f && B.X < 300.f && FMath::Abs(B.Y) < 80.f && B.Z < 200.f)
		{
			continue;   // forks and what they carry
		}
		if (PlanIgnoreRadius > 0.f && FVector::Dist2D(W, PlanIgnorePoint) < PlanIgnoreRadius)
		{
			continue;   // the pallet we are going to insert into
		}
		bool bKnown = false;   // already in the plan's obstacle set (with 40 cm of slack): the planner is routing around it on purpose
		for (const FBox2D& KB : PlanObstacles)
		{
			if (W.X > KB.Min.X - 40.f && W.X < KB.Max.X + 40.f && W.Y > KB.Min.Y - 40.f && W.Y < KB.Max.Y + 40.f)
			{
				bKnown = true;
				break;
			}
		}
		if (bKnown)
		{
			continue;
		}
		for (int32 K = 1; K <= 2; ++K)   // poses 60 and 120 cm ahead along the plan: roughly the truck's stopping distance
		{
			const FPlanPose& Pose = DensePath[FMath::Min(DenseIdx + K * 4, SegEnd)];
			const float Yaw = FMath::DegreesToRadians(Pose.YawDeg);
			const FVector2D D(W.X - Pose.P.X, W.Y - Pose.P.Y);
			const float Lx = D.X * FMath::Cos(Yaw) + D.Y * FMath::Sin(Yaw);
			const float Ly = -D.X * FMath::Sin(Yaw) + D.Y * FMath::Cos(Yaw);
			if (Lx > -Rear && Lx < Front && FMath::Abs(Ly) < Half)
			{
				LastBlockPoint = W;
				if (!bLoggedLowBlock)
				{
					bLoggedLowBlock = true;
					FHitResult Hit;
					FCollisionQueryParams QP(SCENE_QUERY_STAT(NavPathProbe), false, Truck);
					const FVector From = Low->GetComponentLocation();
					const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, From, From + (W - From) * 1.05f, Low->TraceChannel, QP);
					UE_LOG(LogForkliftNav, Log, TEXT("[Nav] low-lidar path stop: point %s (z-bias %.1f) inside planned pose +%d (%s); probe %s; truck %s yaw %.0f; ignore point %s r=%.0f"),
						*W.ToString(), W.Z - FloorBias, K * 4, *FVector(Pose.P.X, Pose.P.Y, 0.f).ToString(), bHit ? *Hit.GetComponent()->GetFullName() : TEXT("no hit"),
						*Truck->GetActorLocation().ToString(), Truck->GetActorRotation().Yaw, *PlanIgnorePoint.ToString(), PlanIgnoreRadius);
				}
				return true;
			}
		}
	}
	return false;
}

bool UForkliftNavComponent::IsLowLidarBlocked() const
{
	const AForkliftVehicle* Truck = Vehicle.Get();
	const ULidarSensorComponent* Low = Truck ? Truck->GetLowLidar() : nullptr;
	if (!Low)
	{
		return false;
	}
	const FTransform BodyXf = Truck->GetActorTransform();
	// Floor reference: the two lowest channels (-12 and -9 deg) mostly hit the floor, so their median world height is the floor level as
	// this scan sees it (the sprung truck pitches and rolls, which shifts floor returns by several cm at 2-3 m). Subtract it.
	const TArray<FVector>& Pts = Low->GetPointsRef();
	const TArray<float> Ranges = Low->GetRangesCm();
	const int32 Steps = FMath::Max(Low->HorizontalSteps, 1);
	TArray<float, TInlineAllocator<512>> FloorZ;
	{
		int32 K = 0;
		for (int32 I = 0; I < Ranges.Num() && K <= Pts.Num(); ++I)
		{
			if (Ranges[I] < 0.f)
			{
				continue;
			}
			if (I / Steps <= 1 && K < Pts.Num())
			{
				FloorZ.Add(Low->GetComponentTransform().TransformPosition(Pts[K]).Z);
			}
			++K;
		}
	}
	float FloorBias = 0.f;
	if (FloorZ.Num() > 20)
	{
		FloorZ.Sort();
		FloorBias = FloorZ[FloorZ.Num() / 2];
	}
	for (const FVector& P : Pts)
	{
		const FVector WorldP = Low->GetComponentTransform().TransformPosition(P);
		const FVector B = BodyXf.InverseTransformPosition(WorldP);   // truck frame: x forward, y right, z up from the truck origin
		if (WorldP.Z - FloorBias < 8.f)
		{
			continue;   // the floor itself
		}
		if (B.X > 35.f && B.X < 300.f && FMath::Abs(B.Y) < 80.f && B.Z < 200.f)
		{
			continue;   // tines and whatever they carry
		}
		if (B.X >= 300.f && B.X < 300.f + LowLidarStopBeyondTipsCm && FMath::Abs(B.Y) < 70.f)
		{
			if (!bLoggedLowBlock)
			{
				bLoggedLowBlock = true;
				FHitResult Hit;
				FCollisionQueryParams QP(SCENE_QUERY_STAT(NavLowProbe), false, Truck);
				const FVector From = Low->GetComponentLocation();
				const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, From, From + (WorldP - From) * 1.05f, Low->TraceChannel, QP);
				{
					TArray<FHitResult> Down;
					FCollisionQueryParams QP2(SCENE_QUERY_STAT(NavLowProbe2), false, Truck);
					GetWorld()->LineTraceMultiByChannel(Down, WorldP + FVector(0.f, 0.f, 200.f), WorldP - FVector(0.f, 0.f, 30.f), Low->TraceChannel, QP2);
					for (const FHitResult& H : Down)
					{
						UE_LOG(LogForkliftNav, Log, TEXT("[Nav]   vertical probe at the stop point hits %s at z=%.1f"), *H.GetComponent()->GetFullName(), H.ImpactPoint.Z);
					}
				}
				UE_LOG(LogForkliftNav, Log, TEXT("[Nav] low-lidar stop: truck-frame %s world %s; %s; truck %s yaw %.0f"), *B.ToString(), *WorldP.ToString(),
					bHit ? *Hit.GetComponent()->GetFullName() : TEXT("no hit"), *Truck->GetActorLocation().ToString(), Truck->GetActorRotation().Yaw);
			}
			return true;
		}
	}
	bLoggedLowBlock = false;
	return false;
}

bool UForkliftNavComponent::IsLidarPathBlocked(bool bReverse, float MaxAheadCm) const
{
	const AForkliftVehicle* Truck = Vehicle.Get();
	const ULidarSensorComponent* Lidar = Truck ? Truck->GetLidar() : nullptr;
	if (!Lidar)
	{
		return false;
	}
	for (const FVector& P : Lidar->GetPointsRef())   // sensor-local: +X forward, +Y right, cm
	{
		// Reversing looks behind the sensor (mounted 50 cm behind the body origin, so the rear is ~30 cm from it).
		const float Ahead = bReverse ? -P.X - 30.f : P.X;
		if (Ahead > 0.f && Ahead < FMath::Min(FMath::Clamp(50.f + 1.3f * FMath::Abs(Truck->GetForwardSpeedCmPerSec()), 70.f, SafetyStopLookaheadCm), MaxAheadCm) && FMath::Abs(P.Y) < SafetyStopCorridorHalfWidthCm)
		{
			if (!bLoggedBlock)
			{
				bLoggedBlock = true;   // once per stop: what did the lidar see, and where
				FHitResult Hit;
				FCollisionQueryParams QP(SCENE_QUERY_STAT(NavBlockProbe), false, Truck);
				const FVector From = Lidar->GetComponentLocation();
				const FVector To = Lidar->GetComponentTransform().TransformPosition(P) ;
				const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, From, From + (To - From) * 1.05f, Lidar->TraceChannel, QP);
				UE_LOG(LogForkliftNav, Log, TEXT("[Nav] lidar stop probe: %s %s"), bHit ? *Hit.GetComponent()->GetFullName() : TEXT("no hit"), bHit ? *Hit.ImpactPoint.ToString() : TEXT(""));
				UE_LOG(LogForkliftNav, Log, TEXT("[Nav] lidar stop (%s): return at sensor-local %s = world %s, %.0f cm %s; truck %s yaw %.0f"),
					bReverse ? TEXT("reversing") : TEXT("forward"), *P.ToString(), *Lidar->GetComponentTransform().TransformPosition(P).ToString(), Ahead,
					bReverse ? TEXT("behind") : TEXT("ahead"), *Truck->GetActorLocation().ToString(), Truck->GetActorRotation().Yaw);
			}
			return true;
		}
	}
	bLoggedBlock = false;
	return false;
}

void UForkliftNavComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	AForkliftVehicle* Truck = Vehicle.Get();
	if (!Truck || (State != ENavState::Following && State != ENavState::Blocked))
	{
		return;
	}
	StateTime += DeltaTime;
	RouteTime += DeltaTime;
	if (RouteTime > RouteTimeoutSec)
	{
		Finish(ENavState::Failed, TEXT("route timeout"));
		return;
	}

	// Only obstacles between the truck and the point it is driving to matter: a rack just beyond the destination is not
	// in the way (reversing into a staging point 1 m in front of a rack otherwise stops the truck short, permanently).
	float RemainingCm = FVector::Dist2D(Truck->GetActorLocation(), CurrentPath.Num() > 0 ? CurrentPath[0] : FinalDestination);
	if (DensePath.Num() > 0)
	{
		// planner mode: what matters is how far the truck still drives in this gear, up to the next gear change or the goal
		int32 SegEndIdx = DenseIdx;
		while (SegEndIdx + 1 < DensePath.Num() && DensePath[SegEndIdx + 1].Dir == DensePath[DenseIdx].Dir)
		{
			++SegEndIdx;
		}
		RemainingCm = FVector2D::Distance(FVector2D(Truck->GetActorLocation().X, Truck->GetActorLocation().Y), DensePath[SegEndIdx].P);
	}
	LowScanTimer += DeltaTime;
	if (LowScanTimer > 0.1f)
	{
		LowScanTimer = 0.f;
		if (ULidarSensorComponent* Low = Truck->GetLowLidar())
		{
			Low->Scan();
		}
	}
	// The low-lidar stop only applies when the truck is actually heading toward what it sees: steering away from an obstacle that is
	// merely ahead of the nose (a parked pallet 2 m in front while the path turns left) must not freeze the truck.
	const FVector ToWaypoint = (CurrentPath.Num() > 0 ? CurrentPath[0] : FinalDestination) - Truck->GetActorLocation();
	const float HeadingToTargetDeg = FMath::Abs(FMath::RadiansToDegrees(FMath::Atan2(FVector::DotProduct(ToWaypoint, Truck->GetActorRightVector()), FVector::DotProduct(ToWaypoint, Truck->GetActorForwardVector()))));
	const bool bRoofBlocked = IsLidarPathBlocked(bReversing, RemainingCm + 40.f);
	bool bLowBlocked = false;
	if (DensePath.Num() > 0)
	{
		bLowBlocked = RemainingCm > 120.f && IsLowLidarBlockedAlongPath();
		if (bLowBlocked && !bRoofBlocked && DynReplans < 6)
		{
			// Something the camera has not seen yet is on the path: treat it as an obstacle (a pallet-sized box around the return) and replan.
			DynamicObstacles.Add(FBox2D(FVector2D(LastBlockPoint.X - 70.f, LastBlockPoint.Y - 65.f), FVector2D(LastBlockPoint.X + 70.f, LastBlockPoint.Y + 65.f)));
			++DynReplans;
			if (PlanHybrid(FinalDestination, FinalYawDeg))
			{
				UE_LOG(LogForkliftNav, Log, TEXT("[Nav] unplanned obstacle at %s - replanned around it (%d)"), *LastBlockPoint.ToString(), DynReplans);
				bLowBlocked = false;
			}
			else
			{
				DynamicObstacles.Pop();   // no way around it: do not keep piling boxes up; the stop below holds the truck instead
				DynReplans = 6;           // and stop trying until the next route
			}
		}
	}
	else
	{
		bLowBlocked = !bReversing && RemainingCm > 120.f && HeadingToTargetDeg < 40.f && IsLowLidarBlocked();
	}
	if (bRoofBlocked || bLowBlocked)
	{
		BlockedTime += DeltaTime;
		HoldStill();
		if (State != ENavState::Blocked)
		{
			SetState(ENavState::Blocked, TEXT("obstacle ahead"));
		}
		if (BlockedTime > BlockedTimeoutSec)
		{
			Finish(ENavState::Failed, TEXT("blocked timeout"));
		}
		return;
	}
	if (State == ENavState::Blocked)
	{
		SetState(ENavState::Following, TEXT("path clear, resuming"));
	}
	BlockedTime = 0.f;

	// Recovery manoeuvre in progress: drive it out, ignore the path.
	if (RecoveryTimer > 0.f)
	{
		RecoveryTimer -= DeltaTime;
		const float SteerCmd = FMath::Clamp((RecoverySteer - Truck->GetSteerTargetDeg()) / 15.f, -0.6f, 0.6f);
		Truck->SetDriveInput(RecoveryThrottle, SteerCmd, 0.f);
		if (RecoveryTimer <= 0.f)
		{
			StuckRefPos = Truck->GetActorLocation();
			StuckTimer = 0.f;
			bReversing = false;
		}
		return;
	}
	if (FVector::Dist2D(Truck->GetActorLocation(), StuckRefPos) > 25.f)
	{
		StuckRefPos = Truck->GetActorLocation();
		StuckTimer = 0.f;
	}
	else if (FMath::Abs(LastThrottle) > 0.05f)
	{
		StuckTimer += DeltaTime;
		if (StuckTimer > StuckSec)
		{
			// Back out opposite to the direction it was pushing, wheel swung one way then the other on successive tries.
			++RecoveryCount;
			RecoveryThrottle = LastThrottle > 0.f ? -0.35f : 0.35f;
			RecoverySteer = (RecoveryCount % 2 ? 1.f : -1.f) * 35.f;
			RecoveryTimer = RecoverSec;
			StuckTimer = 0.f;
			UE_LOG(LogForkliftNav, Log, TEXT("[Nav] stuck at %s - backing out (%s, attempt %d)"), *Truck->GetActorLocation().ToString(), RecoveryThrottle < 0.f ? TEXT("reverse") : TEXT("forward"), RecoveryCount);
			return;
		}
	}

	if (DensePath.Num() > 0)
	{
		TickDense(DeltaTime);
		return;
	}

	const bool bFinalLeg = CurrentPath.Num() <= 1;
	const FVector TargetPoint = CurrentPath.Num() > 0 ? CurrentPath[0] : FinalDestination;
	const FVector Loc = Truck->GetActorLocation();
	const FVector Forward = Truck->GetActorForwardVector();
	const FVector Right = Truck->GetActorRightVector();

	const float DistToTarget = FVector::Dist2D(Loc, TargetPoint);
	const float Tolerance = bFinalLeg ? ArrivalToleranceCm : WaypointToleranceCm;

	if (FMath::FloorToInt(StateTime * 2.f) != FMath::FloorToInt((StateTime - DeltaTime) * 2.f))
	{
		UE_LOG(LogForkliftNav, Log, TEXT("[Nav] trace t=%.1f loc=%s target=%s dist=%.0f tol=%.0f waypointsLeft=%d"),
			RouteTime, *Loc.ToString(), *TargetPoint.ToString(), DistToTarget, Tolerance, CurrentPath.Num());
	}

	if (DistToTarget <= Tolerance)
	{
		if (bFinalLeg)
		{
			Finish(ENavState::Arrived, TEXT("reached destination"));
			return;
		}
		CurrentPath.RemoveAt(0);
		return;   // re-evaluate next tick against the new target
	}

	// Last leg: pursue a carrot on the destination's heading axis instead of the destination itself, so lateral error
	// and heading converge onto the axis before arrival (a plain point chase arrives ~50 cm off and ~15 deg skewed,
	// which the pickup's vision alignment then cannot recover from in the last metre).
	FVector AimPoint = TargetPoint;
	if (bFinalLeg && AxisCarrotCm > 0.f && DistToTarget < LineUpCm * 1.6f + 150.f)
	{
		const FVector Axis = FRotator(0.f, FinalYawDeg, 0.f).Vector();
		const float S = FVector::DotProduct(Loc - TargetPoint, Axis);   // position along the axis, 0 at the destination
		// Behind the destination (S < 0): drive forward onto it; ahead of it (S > 0): reverse onto it.
		const float CarrotS = S <= 0.f ? FMath::Min(S + AxisCarrotCm, 0.f) : FMath::Max(S - AxisCarrotCm, 0.f);
		AimPoint = TargetPoint + Axis * CarrotS;
	}
	const FVector ToTarget = AimPoint - Loc;
	// Angle of the target off the nose, full +-180 (no clamp): decides forward vs reverse with hysteresis.
	const float OffNoseDeg = FMath::RadiansToDegrees(FMath::Atan2(FVector::DotProduct(ToTarget, Right), FVector::DotProduct(ToTarget, Forward)));
	if (!bReversing && FMath::Abs(OffNoseDeg) > ReverseEnterDeg)
	{
		bReversing = true;
		UE_LOG(LogForkliftNav, Log, TEXT("[Nav] target %.0f deg off the nose - reversing toward it"), OffNoseDeg);
	}
	else if (bReversing && FMath::Abs(OffNoseDeg) < ReverseExitDeg)
	{
		bReversing = false;
		UE_LOG(LogForkliftNav, Log, TEXT("[Nav] target %.0f deg off the nose - driving forward"), OffNoseDeg);
	}

	float Steer, Throttle;
	if (bReversing)
	{
		// Bearing in the reversed frame (tail = nose). With the rear wheel steering, reversing with a left wheel angle
		// swings the tail left and the nose right, so the wheel angle follows the target bearing with the OPPOSITE sign
		// from driving forward (checked against the open-loop -ForkliftIdentTest model in Scripts/ml/nav/vehicle_model.py).
		const float RevBearing = FMath::RadiansToDegrees(FMath::Atan2(-FVector::DotProduct(ToTarget, Right), FMath::Max(-FVector::DotProduct(ToTarget, Forward), 20.f)));
		const float DesiredSteerDeg = FMath::Clamp(RevBearing * 0.6f, -55.f, 55.f);
		Steer = FMath::Clamp((DesiredSteerDeg - Truck->GetSteerTargetDeg()) / 15.f, -0.5f, 0.5f);
		Throttle = -ReverseThrottle;
	}
	else
	{
		const float Bearing = FMath::RadiansToDegrees(FMath::Atan2(FVector::DotProduct(ToTarget, Right), FMath::Max(FVector::DotProduct(ToTarget, Forward), 20.f)));
		// SetDriveInput's steer channel is a wheel-angle rate, not an absolute angle (same convention as
		// ForkliftAutoPickupComponent) - track a desired rear-wheel angle and close the gap gradually so a
		// momentary large bearing error cannot slam the steering to its limit. The +-55deg clamp is
		// deliberately close to ForkliftB's own 60deg max (ForkliftC's lower 30deg max clamps it further
		// internally in SetDriveInput, so this is safe for either model).
		const float DesiredSteerDeg = FMath::Clamp(-Bearing * 0.6f, -55.f, 55.f);
		Steer = FMath::Clamp((DesiredSteerDeg - Truck->GetSteerTargetDeg()) / 15.f, -0.5f, 0.5f);
		const bool bSharpTurn = FMath::Abs(Bearing) > 25.f;
		Throttle = (bFinalLeg || bSharpTurn) ? FinalApproachThrottle : CruiseThrottle;
	}

	const float LiftCmd = bHoldLift ? FMath::Clamp((HoldLiftCm - Truck->GetLiftHeightCm()) / 5.f, -1.f, 1.f) : 0.f;
	LastThrottle = Throttle;
	Truck->SetDriveInput(Throttle, Steer, LiftCmd);
}

void UForkliftNavComponent::TickDense(float DeltaTime)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	const FVector Loc3 = Truck->GetActorLocation();
	const FVector2D Pos(Loc3.X, Loc3.Y);
	const int32 Num = DensePath.Num();

	// Turn response: yaw rate actually achieved vs the kinematic prediction -u * tan(delta) / L from the measured speed and wheel angle.
	{
		const float YawNow = Truck->GetActorRotation().Yaw;
		if (bHavePrevYaw && DeltaTime > 1.e-4f)
		{
			const float MeasDegS = FRotator::NormalizeAxis(YawNow - PrevYawDeg) / DeltaTime;
			const float U = Truck->GetForwardSpeedCmPerSec();
			const float PredDegS = FMath::RadiansToDegrees(-U * FMath::Tan(FMath::DegreesToRadians(Truck->GetSteerTargetDeg())) / 141.f);
			if (FMath::Abs(PredDegS) > 6.f && FMath::Abs(U) > 15.f)
			{
				const float Ratio = FMath::Clamp(MeasDegS / PredDegS, 0.2f, 1.4f);
				TurnResponse = FMath::Lerp(TurnResponse, Ratio, FMath::Min(1.f, 1.5f * DeltaTime));
			}
		}
		PrevYawDeg = YawNow;
		bHavePrevYaw = true;
	}

	// progress along the path: nearest pose within a window ahead of the last one
	int32 Best = DenseIdx;
	float BestD = BIG_NUMBER;
	for (int32 I = DenseIdx; I < FMath::Min(Num, DenseIdx + 40); ++I)
	{
		const float D = FVector2D::Distance(Pos, DensePath[I].P);
		if (D < BestD)
		{
			BestD = D;
			Best = I;
		}
	}
	DenseIdx = Best;

	// lost the path (pushed, slipped, a manoeuvre went wide): plan again from here
	OffPathTime = BestD > 130.f ? OffPathTime + DeltaTime : 0.f;
	if (OffPathTime > 2.5f)
	{
		OffPathTime = 0.f;
		UE_LOG(LogForkliftNav, Log, TEXT("[Nav] %.0f cm off the planned path - replanning"), BestD);
		if (!PlanHybrid(FinalDestination, FinalYawDeg))
		{
			Finish(ENavState::Failed, TEXT("replanning found no path"));
		}
		return;
	}

	const int8 Gear = DensePath[DenseIdx].Dir;
	int32 SegEnd = DenseIdx;
	while (SegEnd + 1 < Num && DensePath[SegEnd + 1].Dir == Gear)
	{
		++SegEnd;
	}
	const bool bLastSeg = SegEnd == Num - 1;
	const float DistToSegEnd = FVector2D::Distance(Pos, DensePath[SegEnd].P);

	if (CuspHold > 0.f)   // standing at a gear change
	{
		CuspHold -= DeltaTime;
		HoldStill();
		if (CuspHold <= 0.f)
		{
			DenseIdx = FMath::Min(SegEnd + 1, Num - 1);
		}
		return;
	}
	if (DistToSegEnd < (bLastSeg ? ArrivalToleranceCm : 30.f))
	{
		if (bLastSeg)
		{
			Finish(ENavState::Arrived, TEXT("reached destination"));
		}
		else
		{
			CuspHold = 0.7f;
			HoldStill();
		}
		return;
	}

	// Tracking: the planner already knows which wheel angle each arc needs (feed-forward), and the truck's rear-steer kinematics give
	// the corrections. With e = sideways offset of the truck to the right of the path, eps = its heading minus the path heading, and
	// u the signed speed:  e' = u*sin(eps),  eps' = -u*tan(delta)/L.  Driving e and eps to zero gives  delta = k_e*e + Dir*k_h*eps,
	// i.e. the sideways term has the same sign forward and in reverse, the heading term flips (derivation in the nav notes).
	bReversing = Gear < 0;
	const FPlanPose& Cur = DensePath[DenseIdx];
	const float PathYaw = FMath::DegreesToRadians(Cur.YawDeg);
	const FVector2D RightOfPath(-FMath::Sin(PathYaw), FMath::Cos(PathYaw));
	const float E = FVector2D::DotProduct(Pos - Cur.P, RightOfPath);
	const float Eps = FRotator::NormalizeAxis(Truck->GetActorRotation().Yaw - Cur.YawDeg);
	const int32 Preview = FMath::Min(DenseIdx + 5, SegEnd);   // the wheel needs time to swing: use the angle a little way ahead
	const float Speed = FMath::Abs(Truck->GetForwardSpeedCmPerSec());
	const float ALat = (bCarryingLoad ? LateralAccelLoadedCmS2 : LateralAccelEmptyCmS2) * FMath::Clamp(TurnResponse, 0.5f, 1.f);
	DynSteerLimitDeg = FMath::Clamp(FMath::RadiansToDegrees(FMath::Atan(141.f * ALat / FMath::Max(Speed * Speed, 1.f))), MinSteerLimitDeg, MaxSteerLimitDeg);
	const float DeltaDeg = FMath::Clamp(DensePath[Preview].SteerDeg + 0.16f * E + Gear * 1.0f * Eps, -DynSteerLimitDeg, DynSteerLimitDeg);
	const float Steer = FMath::Clamp((DeltaDeg - Truck->GetSteerTargetDeg()) / 10.f, -1.f, 1.f);
	// Speed governor: ease off as the sideways error grows or the truck stops turning the way the model says it should.
	const float ErrScale = 1.f - FMath::Clamp((FMath::Abs(E) - SlowOnErrorStartCm) / FMath::Max(SlowOnErrorFullCm - SlowOnErrorStartCm, 1.f), 0.f, 1.f) * 0.65f;
	const float Governor = FMath::Clamp(ErrScale * FMath::Clamp(TurnResponse, 0.6f, 1.f), 0.35f, 1.f);
	float Throttle;
	if (Gear < 0)
	{
		Throttle = -(DistToSegEnd < 120.f ? 0.22f : ReverseThrottle) * Governor;
	}
	else
	{
		const bool bTight = FMath::Abs(DensePath[Preview].SteerDeg) > 20.f;
		Throttle = ((DistToSegEnd < 150.f || (bLastSeg && DistToSegEnd < 250.f) || bTight) ? FinalApproachThrottle : CruiseThrottle) * Governor;
	}
	if (FMath::FloorToInt(RouteTime * 2.f) != FMath::FloorToInt((RouteTime - DeltaTime) * 2.f))
	{
		UE_LOG(LogForkliftNav, Log, TEXT("[Nav] trace t=%.1f loc=%s path %d/%d gear %s sideways %.0f cm, to segment end %.0f, steer limit %.0f deg, turn response %.2f"), RouteTime, *Loc3.ToString(), DenseIdx, Num, Gear > 0 ? TEXT("fwd") : TEXT("rev"), E, DistToSegEnd, DynSteerLimitDeg, TurnResponse);
	}
	const float LiftCmd = bHoldLift ? FMath::Clamp((HoldLiftCm - Truck->GetLiftHeightCm()) / 5.f, -1.f, 1.f) : 0.f;
	LastThrottle = Throttle;
	Truck->SetDriveInput(Throttle, Steer, LiftCmd);
}

FString UForkliftNavComponent::GetStatus() const
{
	FString Result = FString::Printf(TEXT("nav: %s"), StateName(State));
	if (!StatusDetail.IsEmpty())
	{
		Result += TEXT(" - ") + StatusDetail;
	}
	if (State == ENavState::Following || State == ENavState::Blocked)
	{
		Result += FString::Printf(TEXT("  %d waypoints left"), CurrentPath.Num());
	}
	return Result;
}
