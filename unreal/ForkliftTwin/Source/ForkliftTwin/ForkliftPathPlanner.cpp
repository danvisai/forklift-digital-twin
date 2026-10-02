#include "ForkliftPathPlanner.h"

#include "Algo/Reverse.h"
#include "HAL/PlatformTime.h"

namespace
{
	struct FNode
	{
		FVector2D R;          // fixed-axle reference point, world cm
		float Yaw = 0.f;      // radians
		float G = 0.f;
		float F = 0.f;
		int32 Parent = INDEX_NONE;
		int8 Dir = 1;
		int8 SteerIdx = 0;
		int32 Key = 0;
		float PrimLen = 0.f;
		float PrimSteerDeg = 0.f;
	};

	inline FVector2D Fwd(float Yaw) { return FVector2D(FMath::Cos(Yaw), FMath::Sin(Yaw)); }

	// Pose after travelling signed distance S with steer Delta (left positive): the fixed axle moves along the heading, heading
	// changes by -S * tan(delta) / L (UE yaw is clockwise, so a left turn lowers it).
	void Advance(const FPlannerConfig& C, const FVector2D& R, float Yaw, float SteerRad, float S, FVector2D& OutR, float& OutYaw)
	{
		const float K = -FMath::Tan(SteerRad) / C.WheelbaseCm;   // curvature along the travel direction
		if (FMath::Abs(K) < 1.e-5f)
		{
			OutR = R + Fwd(Yaw) * S;
			OutYaw = Yaw;
			return;
		}
		const float NewYaw = Yaw + K * S;
		OutR = FVector2D(R.X + (FMath::Sin(NewYaw) - FMath::Sin(Yaw)) / K, R.Y - (FMath::Cos(NewYaw) - FMath::Cos(Yaw)) / K);
		OutYaw = NewYaw;
	}

	// Oriented rectangle (outline in the R frame) against an AABB: separating-axis test.
	bool RectHitsBox(const FVector2D& Centre, const FVector2D& Ax, const FVector2D& Ay, float Hx, float Hy, const FBox2D& B)
	{
		const FVector2D BC = (B.Min + B.Max) * 0.5f, BH = (B.Max - B.Min) * 0.5f;
		const FVector2D D = BC - Centre;
		// axes of the box (world x, y)
		const float RectExtentX = FMath::Abs(Ax.X) * Hx + FMath::Abs(Ay.X) * Hy;
		if (FMath::Abs(D.X) > BH.X + RectExtentX) return false;
		const float RectExtentY = FMath::Abs(Ax.Y) * Hx + FMath::Abs(Ay.Y) * Hy;
		if (FMath::Abs(D.Y) > BH.Y + RectExtentY) return false;
		// axes of the rectangle
		const float BoxOnAx = FMath::Abs(Ax.X) * BH.X + FMath::Abs(Ax.Y) * BH.Y;
		if (FMath::Abs(FVector2D::DotProduct(D, Ax)) > Hx + BoxOnAx) return false;
		const float BoxOnAy = FMath::Abs(Ay.X) * BH.X + FMath::Abs(Ay.Y) * BH.Y;
		if (FMath::Abs(FVector2D::DotProduct(D, Ay)) > Hy + BoxOnAy) return false;
		return true;
	}

	struct FCollider
	{
		const FPlannerConfig& C;
		const TArray<FBox2D>& Boxes;
		float Hx, Hy, CentreOffset;   // outline half sizes and the outline centre's offset ahead of R
		TArray<bool> IgnoreNearStart;   // boxes the start pose already overlaps (the truck is leaving them)
		FVector2D StartR = FVector2D::ZeroVector;
		float IgnoreRadius = 150.f;

		FCollider(const FPlannerConfig& Cfg, const TArray<FBox2D>& Obst) : C(Cfg), Boxes(Obst)
		{
			const float Margin = C.MarginCm + (C.bCarrying ? C.CarryExtraMarginCm : 0.f);
			const float Front = C.FrontCm + (C.bCarrying ? C.CarryExtraFrontCm : 0.f) + Margin;
			const float Rear = C.RearCm + Margin;
			const float HalfW = C.HalfWidthCm + (C.bCarrying ? C.CarryExtraHalfWidthCm : 0.f) + Margin;
			// extents relative to R (the fixed axle is AxleAheadCm in front of the origin)
			const float XMin = -(Rear + C.AxleAheadCm), XMax = Front - C.AxleAheadCm;
			Hx = 0.5f * (XMax - XMin);
			Hy = HalfW;
			CentreOffset = 0.5f * (XMax + XMin);
		}

		void MarkStart(const FVector2D& R, float Yaw)
		{
			StartR = R;
			const FVector2D Ax = Fwd(Yaw), Ay(-Ax.Y, Ax.X);
			const FVector2D Centre = R + Ax * CentreOffset;
			IgnoreNearStart.Init(false, Boxes.Num());
			for (int32 I = 0; I < Boxes.Num(); ++I)
			{
				IgnoreNearStart[I] = RectHitsBox(Centre, Ax, Ay, Hx, Hy, Boxes[I]);
			}
		}

		bool Hits(const FVector2D& R, float Yaw) const
		{
			const FVector2D Ax = Fwd(Yaw), Ay(-Ax.Y, Ax.X);
			const FVector2D Centre = R + Ax * CentreOffset;
			if (Centre.X - Hx - Hy < C.BoundsMin.X || Centre.X + Hx + Hy > C.BoundsMax.X || Centre.Y - Hx - Hy < C.BoundsMin.Y || Centre.Y + Hx + Hy > C.BoundsMax.Y)
			{
				// only a hard bounds violation by the outline's corners matters; test the corners precisely
				for (int32 I = 0; I < 4; ++I)
				{
					const FVector2D Corner = Centre + Ax * ((I & 1) ? Hx : -Hx) + Ay * ((I & 2) ? Hy : -Hy);
					if (Corner.X < C.BoundsMin.X || Corner.X > C.BoundsMax.X || Corner.Y < C.BoundsMin.Y || Corner.Y > C.BoundsMax.Y)
					{
						return true;
					}
				}
			}
			const float Reach = Hx + Hy;
			const bool bNearStart = FVector2D::Distance(R, StartR) < IgnoreRadius;
			for (int32 Bi = 0; Bi < Boxes.Num(); ++Bi)
			{
				const FBox2D& B = Boxes[Bi];
				if (bNearStart && IgnoreNearStart.IsValidIndex(Bi) && IgnoreNearStart[Bi])
				{
					continue;
				}
				if (Centre.X + Reach < B.Min.X || Centre.X - Reach > B.Max.X || Centre.Y + Reach < B.Min.Y || Centre.Y - Reach > B.Max.Y)
				{
					continue;
				}
				if (RectHitsBox(Centre, Ax, Ay, Hx, Hy, B))
				{
					return true;
				}
			}
			return false;
		}
	};

	int32 MakeKey(const FPlannerConfig& C, const FVector2D& R, float Yaw)
	{
		const int32 Ix = FMath::FloorToInt((R.X - C.BoundsMin.X) / C.CellCm);
		const int32 Iy = FMath::FloorToInt((R.Y - C.BoundsMin.Y) / C.CellCm);
		const float YawDeg = FMath::Fmod(FMath::RadiansToDegrees(Yaw) + 3600.f, 360.f);
		const int32 Iyaw = FMath::FloorToInt(YawDeg / C.YawBinDeg) % FMath::Max(1, FMath::RoundToInt(360.f / C.YawBinDeg));
		return (Ix & 0x3FF) | ((Iy & 0x3FF) << 10) | ((Iyaw & 0x3F) << 20);
	}

	float YawDiffDeg(float A, float B) { return FMath::Abs(FRotator::NormalizeAxis(FMath::RadiansToDegrees(A - B))); }
}

TArray<int32> FForkliftPathPlanner::BoxesHitAt(const FPlannerConfig& Cfg, const TArray<FBox2D>& Obstacles, const FVector2D& Origin, float YawDeg)
{
	TArray<int32> Out;
	const FCollider Col(Cfg, Obstacles);
	const float Yaw = FMath::DegreesToRadians(YawDeg);
	const FVector2D R = Origin + Fwd(Yaw) * Cfg.AxleAheadCm;
	const FVector2D Ax = Fwd(Yaw), Ay(-Ax.Y, Ax.X);
	const FVector2D Centre = R + Ax * Col.CentreOffset;
	for (int32 I = 0; I < Obstacles.Num(); ++I)
	{
		if (RectHitsBox(Centre, Ax, Ay, Col.Hx, Col.Hy, Obstacles[I]))
		{
			Out.Add(I);
		}
	}
	return Out;
}

bool FForkliftPathPlanner::Plan(const FPlannerConfig& Cfg, const TArray<FBox2D>& Obstacles, const FVector2D& StartOrigin, float StartYawDeg,
	const FVector2D& GoalOrigin, float GoalYawDeg, TArray<FPlanPose>& OutPath, int32* OutExpansions)
{
	OutPath.Reset();
	FCollider Col(Cfg, Obstacles);
	const float StartYaw = FMath::DegreesToRadians(StartYawDeg), GoalYaw = FMath::DegreesToRadians(GoalYawDeg);
	const FVector2D StartR = StartOrigin + Fwd(StartYaw) * Cfg.AxleAheadCm;
	const FVector2D GoalR = GoalOrigin + Fwd(GoalYaw) * Cfg.AxleAheadCm;
	const double T0 = FPlatformTime::Seconds();
	Col.MarkStart(StartR, StartYaw);

	TArray<FNode> Nodes;
	Nodes.Reserve(60000);
	TArray<int32> Open;           // binary min-heap of node indices by F
	TMap<int32, float> BestG;
	auto Less = [&Nodes](int32 A, int32 B) { return Nodes[A].F < Nodes[B].F; };

	auto Heur = [&](const FVector2D& R, float Yaw)
	{
		float H = FVector2D::Distance(R, GoalR);
		if (Cfg.bRequireGoalYaw)
		{
			H += 0.25f * FMath::DegreesToRadians(YawDiffDeg(Yaw, GoalYaw)) * 100.f;   // turning the last degrees is not free
		}
		return Cfg.HeuristicWeight * H;
	};
	auto InGoal = [&](const FVector2D& R, float Yaw)
	{
		return FVector2D::Distance(R, GoalR) <= Cfg.GoalPosTolCm && (!Cfg.bRequireGoalYaw || YawDiffDeg(Yaw, GoalYaw) <= Cfg.GoalYawTolDeg);
	};

	{
		FNode N;
		N.R = StartR; N.Yaw = StartYaw; N.G = 0.f; N.F = Heur(StartR, StartYaw); N.Dir = 0; N.Key = MakeKey(Cfg, StartR, StartYaw);
		Nodes.Add(N);
		Open.HeapPush(0, Less);
		BestG.Add(N.Key, 0.f);
	}

	// A single-arc analytic connection from a node to the goal pose (steer fine set, both directions).
	auto Shot = [&](const FNode& N, TArray<FPlanPose>& Seg) -> bool
	{
		const float Dist = FVector2D::Distance(N.R, GoalR);
		if (Dist > 450.f || Dist < 1.f)
		{
			return false;
		}
		if (Cfg.bRequireGoalYaw && YawDiffDeg(N.Yaw, GoalYaw) > 60.f)
		{
			return false;
		}
		for (int32 Dir = 1; Dir >= -1; Dir -= 2)
		{
			for (float SteerDeg = -35.f; SteerDeg <= 35.f; SteerDeg += 1.5f)
			{
				const float Steer = FMath::DegreesToRadians(SteerDeg);
				float Best = BIG_NUMBER;
				float BestS = 0.f;
				for (float S = 10.f; S <= 480.f; S += 5.f)
				{
					FVector2D R; float Y;
					Advance(Cfg, N.R, N.Yaw, Steer, Dir * S, R, Y);
					if (InGoal(R, Y))
					{
						Best = S;
						BestS = S;
						break;
					}
				}
				if (Best == BIG_NUMBER)
				{
					continue;
				}
				bool bClear = true;
				for (float S = 10.f; S <= BestS && bClear; S += 10.f)
				{
					FVector2D R; float Y;
					Advance(Cfg, N.R, N.Yaw, Steer, Dir * S, R, Y);
					bClear = !Col.Hits(R, Y);
				}
				if (bClear)
				{
					Seg.Reset();
					for (float S = 15.f; S <= BestS; S += 15.f)
					{
						FVector2D R; float Y;
						Advance(Cfg, N.R, N.Yaw, Steer, Dir * S, R, Y);
						FPlanPose P;
						P.P = R - Fwd(Y) * Cfg.AxleAheadCm;
						P.YawDeg = FMath::RadiansToDegrees(Y);
						P.Dir = static_cast<int8>(Dir);
						P.SteerDeg = SteerDeg;
						Seg.Add(P);
					}
					return true;
				}
			}
		}
		return false;
	};

	int32 Expansions = 0;
	int32 GoalNode = INDEX_NONE;
	TArray<FPlanPose> ShotSeg;
	while (Open.Num() > 0 && Expansions < Cfg.MaxExpansions)
	{
		if ((Expansions & 255) == 0 && FPlatformTime::Seconds() - T0 > Cfg.MaxSeconds)
		{
			break;
		}
		int32 Cur;
		Open.HeapPop(Cur, Less);
		++Expansions;
		const FNode N = Nodes[Cur];   // copy: Nodes may reallocate below
		if (const float* Known = BestG.Find(N.Key); Known && *Known + 0.01f < N.G)
		{
			continue;   // a cheaper route to this cell was found after this entry was queued
		}
		if (InGoal(N.R, N.Yaw))
		{
			GoalNode = Cur;
			break;
		}
		if (Expansions % 3 == 0 && Shot(N, ShotSeg))
		{
			GoalNode = Cur;
			break;
		}
		for (int32 Dir = 1; Dir >= -1; Dir -= 2)
		{
			for (int32 Si = 0; Si < Cfg.SteerDeg.Num(); ++Si)
			{
				const float SteerDeg = Cfg.SteerDeg[Si];
				FVector2D R2; float Y2;
				bool bClear = true;
				for (float S = 20.f; S <= Cfg.StepCm + 0.1f && bClear; S += 20.f)
				{
					Advance(Cfg, N.R, N.Yaw, FMath::DegreesToRadians(SteerDeg), Dir * S, R2, Y2);
					bClear = !Col.Hits(R2, Y2);
				}
				if (!bClear)
				{
					continue;
				}
				float Cost = Cfg.StepCm * (Dir < 0 ? Cfg.ReversePenalty : 1.f);
				if (N.Dir != 0 && N.Dir != Dir)
				{
					Cost += Cfg.CuspPenalty;
				}
				if (N.Dir != 0 && Si != N.SteerIdx)
				{
					Cost += Cfg.SteerChangePenalty * FMath::Abs(SteerDeg - Cfg.SteerDeg[N.SteerIdx]) / 35.f;
				}
				FNode M;
				M.R = R2; M.Yaw = Y2; M.G = N.G + Cost; M.Parent = Cur; M.Dir = static_cast<int8>(Dir); M.SteerIdx = static_cast<int8>(Si);
				M.PrimLen = Cfg.StepCm; M.PrimSteerDeg = SteerDeg;
				M.Key = MakeKey(Cfg, R2, Y2);
				M.F = M.G + Heur(R2, Y2);
				if (const float* Prev = BestG.Find(M.Key); Prev && *Prev <= M.G)
				{
					continue;
				}
				BestG.Add(M.Key, M.G);
				Nodes.Add(M);
				Open.HeapPush(Nodes.Num() - 1, Less);
			}
		}
	}
	if (OutExpansions)
	{
		*OutExpansions = Expansions;
	}
	if (GoalNode == INDEX_NONE)
	{
		return false;
	}

	// Unwind to the start, then re-sample every primitive finely (the follower tracks a dense path).
	TArray<int32> Chain;
	for (int32 I = GoalNode; I != INDEX_NONE; I = Nodes[I].Parent)
	{
		Chain.Add(I);
	}
	Algo::Reverse(Chain);
	for (int32 C = 1; C < Chain.Num(); ++C)
	{
		const FNode& Parent = Nodes[Chain[C - 1]];
		const FNode& Child = Nodes[Chain[C]];
		for (float S = 15.f; S <= Child.PrimLen + 0.1f; S += 15.f)
		{
			FVector2D R; float Y;
			Advance(Cfg, Parent.R, Parent.Yaw, FMath::DegreesToRadians(Child.PrimSteerDeg), Child.Dir * S, R, Y);
			FPlanPose P;
			P.P = R - Fwd(Y) * Cfg.AxleAheadCm;
			P.YawDeg = FMath::RadiansToDegrees(Y);
			P.Dir = Child.Dir;
			P.SteerDeg = Child.PrimSteerDeg;
			OutPath.Add(P);
		}
	}
	if (ShotSeg.Num() > 0)
	{
		OutPath.Append(ShotSeg);
	}
	return OutPath.Num() > 0;
}
