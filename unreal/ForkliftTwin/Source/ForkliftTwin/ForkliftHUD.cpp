#include "ForkliftHUD.h"

#include "Engine/Canvas.h"
#include "Engine/TextureRenderTarget2D.h"
#include "ForkliftCameraComponent.h"
#include "ForkliftAutoPickupComponent.h"
#include "ForkliftDetectionClient.h"
#include "ForkliftMissionComponent.h"
#include "ForkliftPalletFinderComponent.h"
#include "ForkliftNavComponent.h"
#include "ForkliftVehicle.h"
#include "GameFramework/PlayerController.h"
#include "Components/StaticMeshComponent.h"
#include "InputCoreTypes.h"
#include "Warehouse.h"
#include "LidarSensorComponent.h"

namespace
{
	// One fixed colour per class id (pallet, loaded_pallet, pallet_pocket, tote, rack - see data.yaml); repeats if a
	// server with more classes is pointed at this HUD.
	const FLinearColor ClassColors[] = {
		FLinearColor(0.2f, 0.9f, 0.2f), FLinearColor(1.0f, 0.55f, 0.0f), FLinearColor(1.0f, 0.15f, 0.15f),
		FLinearColor(0.25f, 0.6f, 1.0f), FLinearColor(0.85f, 0.2f, 0.9f),
	};
}

AForkliftHUD::AForkliftHUD()
{
	PrimaryActorTick.bCanEverTick = true;  // polls the detector server independently of DrawHUD
}

void AForkliftHUD::BeginPlay()
{
	Super::BeginPlay();
	Detector = NewObject<UForkliftDetectionClient>(this);
	Detector->ServerUrl = DetectorUrl;
	Detector->DetectionRateHz = DetectionRateHz;
}

void AForkliftHUD::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	const AForkliftVehicle* Truck = Cast<AForkliftVehicle>(GetOwningPawn());

	if (bShowDetections && Detector && Truck)
	{
		Detector->Poll(Truck->GetForkCamera(), DeltaSeconds);
	}

	// Depth readback is a GPU stall (see UpdateDepthColorTexture), so only pay for it while the view is on
	// screen, and throttled independent of frame rate - same reasoning as the detector's own poll rate above.
	if (bShowDepth && Truck)
	{
		TimeSinceDepthUpdate += DeltaSeconds;
		const float Period = 1.f / FMath::Max(DepthUpdateRateHz, 0.1f);
		if (TimeSinceDepthUpdate >= Period)
		{
			TimeSinceDepthUpdate = FMath::Fmod(TimeSinceDepthUpdate, Period);
			if (UForkliftCameraComponent* Cam = Truck->GetForkCamera())
			{
				Cam->UpdateDepthColorTexture(ForkDepthNearCm, ForkDepthFarCm);
			}
			// The operationally meaningful one: UForkliftAutoPickupComponent uses this same camera's depth to
			// judge fork-to-pocket range while inserting.
			if (UForkliftCameraComponent* TineCam = Truck->GetTineCamera())
			{
				TineCam->UpdateDepthColorTexture(TineDepthNearCm, TineDepthFarCm);
			}
		}
	}
}

void AForkliftHUD::DrawWorldBox(const FBox& Box, const FLinearColor& Color, float Thickness)
{
	FVector P[8];
	for (int32 I = 0; I < 8; ++I)
	{
		P[I] = Project(FVector((I & 1) ? Box.Max.X : Box.Min.X, (I & 2) ? Box.Max.Y : Box.Min.Y, (I & 4) ? Box.Max.Z : Box.Min.Z));
	}
	static const int32 Edges[12][2] = { {0,1},{2,3},{4,5},{6,7},{0,2},{1,3},{4,6},{5,7},{0,4},{1,5},{2,6},{3,7} };
	for (const int32 (&E)[2] : Edges)
	{
		if (P[E[0]].Z > 0.f && P[E[1]].Z > 0.f)
		{
			DrawLine(P[E[0]].X, P[E[0]].Y, P[E[1]].X, P[E[1]].Y, Color, Thickness);
		}
	}
}

void AForkliftHUD::DrawWorldLabel(const FVector& World, const FString& Text, const FLinearColor& Color)
{
	const FVector S = Project(World);
	if (S.Z > 0.f)
	{
		DrawText(Text, Color, S.X - 4.f * Text.Len(), S.Y);
	}
}

void AForkliftHUD::DrawTargetMarkers(const AForkliftVehicle& Truck)
{
	const UForkliftMissionComponent* Mission = Truck.GetMission();
	const AWarehouse* WH = Mission ? Mission->GetWarehouse() : nullptr;
	if (!Mission || !WH || !Mission->IsMissionActive())
	{
		return;
	}
	const EMissionState State = Mission->GetState();
	const bool bCarrying = State == EMissionState::NavigatingToDrop || State == EMissionState::Placing || State == EMissionState::Lowering ||
		State == EMissionState::Withdrawing || State == EMissionState::Verifying;

	// Pallets the truck's own camera has found (vision search): a box per confirmed detection, the chosen one in magenta.
	if (const UForkliftPalletFinderComponent* Finder = Truck.GetFinder(); Finder && Mission->IsVisionSearch())
	{
		for (const FPalletCandidate& C : Finder->GetCandidates())
		{
			if (C.Hits < 2)
			{
				continue;
			}
			const bool bChosen = Mission->HasTarget() && !bCarrying && FVector::Dist2D(C.Pos, Mission->GetTargetCentre()) < 120.f;
			const FLinearColor Col = bChosen ? FLinearColor(1.f, 0.f, 1.f) : (C.IsLoaded() ? FLinearColor(0.3f, 0.7f, 1.f) : FLinearColor(0.85f, 0.85f, 0.85f));
			const FBox Box(FVector(C.Pos.X - 60.f, C.Pos.Y - 50.f, 0.f), FVector(C.Pos.X + 60.f, C.Pos.Y + 50.f, C.IsLoaded() ? 110.f : 15.f));
			DrawWorldBox(Box, Col, bChosen ? 3.f : 1.5f);
			DrawWorldLabel(FVector(C.Pos.X, C.Pos.Y, Box.Max.Z + 25.f),
				FString::Printf(TEXT("%s%s pallet (detected, %d frames, %.0f%%)"), bChosen ? TEXT("TARGET  ") : TEXT(""), C.IsLoaded() ? TEXT("loaded") : TEXT("empty"), C.Hits, C.BestConfidence * 100.f), Col);
		}
		if (Mission->GetState() == EMissionState::Searching)
		{
			DrawText(TEXT("searching: patrolling the aisle, looking for pallets with the fork camera"), FLinearColor(1.f, 0.9f, 0.2f), 20.f, Canvas->SizeY - 30.f);
		}
	}

	// Where the truck is heading for the pickup: staging point, then the pre-pose, with the approach direction.
	if (State == EMissionState::NavigatingToPickup || State == EMissionState::NavigatingToDrop)
	{
		const FVector Dir = FRotator(0.f, Mission->GetApproachYawDeg(), 0.f).Vector();
		const FLinearColor C = FLinearColor(0.f, 0.9f, 1.f);
		for (int32 I = 0; I < 2; ++I)
		{
			const FVector P = (I == 0 ? Mission->GetStagePos() : Mission->GetPrePos()) + FVector(0.f, 0.f, 8.f);
			DrawWorldBox(FBox(P - FVector(25.f, 25.f, 0.f), P + FVector(25.f, 25.f, 10.f)), C, 2.f);
			const FVector S0 = Project(P), S1 = Project(P + Dir * 80.f);
			if (S0.Z > 0.f && S1.Z > 0.f)
			{
				DrawLine(S0.X, S0.Y, S1.X, S1.Y, C, 3.f);
			}
			DrawWorldLabel(P + FVector(0.f, 0.f, 40.f), I == 0 ? TEXT("stage") : TEXT("pre-pose"), C);
		}
	}

	// Drop slot once the pallet is on the forks.
	if (bCarrying)
	{
		const FVector L = Mission->GetTargetSlot().Location + FVector(0.f, 0.f, 4.f);
		DrawWorldBox(FBox(L - FVector(60.f, 50.f, 0.f), L + FVector(60.f, 50.f, 6.f)), FLinearColor(1.f, 0.6f, 0.f), 3.f);
		DrawWorldLabel(L + FVector(0.f, 0.f, 40.f), FString::Printf(TEXT("DROP  %s"), *Mission->GetTargetSlot().Id.ToString()), FLinearColor(1.f, 0.6f, 0.f));
	}

	// Vision lock during the pickup: a green cross where the detector thinks the pocket centre is.
	if (const UForkliftAutoPickupComponent* Auto = Truck.GetAutoPickup(); Auto && Auto->HasFreshVisionTarget())
	{
		const FVector S = Project(Auto->GetVisionTargetWorld());
		if (S.Z > 0.f)
		{
			DrawLine(S.X - 10.f, S.Y, S.X + 10.f, S.Y, FLinearColor::Green, 2.f);
			DrawLine(S.X, S.Y - 10.f, S.X, S.Y + 10.f, FLinearColor::Green, 2.f);
			DrawText(TEXT("vision lock"), FLinearColor::Green, S.X + 12.f, S.Y - 6.f);
		}
	}
}

void AForkliftHUD::DrawMissionOverlay(const AForkliftVehicle& Truck)
{
	DrawTargetMarkers(Truck);
	const UForkliftMissionComponent* Mission = Truck.GetMission();
	const UForkliftNavComponent* Nav = Truck.GetNav();
	if (!Mission || !Nav)
	{
		return;
	}

	// The route being followed, drawn on the floor in the 3D view.
	if (Nav->IsBusy())
	{
		FVector From = Truck.GetActorLocation() + FVector(0.f, 0.f, 25.f);
		for (const FVector& Point : Nav->GetCurrentPath())
		{
			const FVector To = Point + FVector(0.f, 0.f, 25.f);
			const FVector A = Project(From);
			const FVector B = Project(To);
			if (A.Z > 0.f && B.Z > 0.f)
			{
				DrawLine(A.X, A.Y, B.X, B.Y, FLinearColor::Yellow, 3.f);
				DrawRect(FLinearColor::Yellow, B.X - 4.f, B.Y - 4.f, 8.f, 8.f);
			}
			From = To;
		}
	}

	// Status panel, top right.
	const float X = Canvas->SizeX - 560.f;
	float Y = 16.f;
	const bool bActive = Mission->IsMissionActive();
	DrawText(Mission->GetStatus(), bActive ? FLinearColor::Green : FLinearColor::White, X, Y);
	Y += 16.f;
	if (Nav->IsBusy())
	{
		DrawText(Nav->GetStatus(), FLinearColor::Yellow, X, Y);
		Y += 16.f;
	}
	if (Mission->GetRecords().Num() > 0)
	{
		DrawText(FString::Printf(TEXT("score: last %.1f   mean %.1f   over %d deliveries (%d skipped)"), Mission->GetLastScore(), Mission->GetMeanScore(),
			Mission->GetRecords().Num(), Mission->GetSkippedCount()), FLinearColor::White, X, Y);
	}
}

void AForkliftHUD::DrawHUD()
{
	Super::DrawHUD();

	if (PlayerOwner && PlayerOwner->WasInputKeyJustPressed(EKeys::V))
	{
		bShowSensors = !bShowSensors;
	}
	if (PlayerOwner && PlayerOwner->WasInputKeyJustPressed(EKeys::L))
	{
		bShowDepth = !bShowDepth;
	}
	const AForkliftVehicle* Truck = Cast<AForkliftVehicle>(GetOwningPawn());
	if (Truck && Canvas)
	{
		DrawMissionOverlay(*Truck);
	}
	if (!bShowSensors || !Truck || !Canvas)
	{
		return;
	}

	constexpr float Margin = 16.f;
	float Y = Margin;

	if (const UForkliftCameraComponent* Cam = Truck->GetForkCamera(); Cam && Cam->GetRenderTarget())
	{
		// The capture's alpha is 0, so this must not alpha-blend.
		const float W = 320.f, H = W * Cam->Height / Cam->Width;
		if (bShowDepth && Cam->GetDepthColorTexture())
		{
			DrawTexture(Cam->GetDepthColorTexture(), Margin, Y, W, H, 0.f, 0.f, 1.f, 1.f, FLinearColor::White, BLEND_Opaque);
			DrawText(FString::Printf(TEXT("fork camera DEPTH  %.0f-%.0f cm  (L: RGB)"), ForkDepthNearCm, ForkDepthFarCm), FLinearColor::White, Margin + 4.f, Y + 4.f);
			Y += H + Margin;
		}
		else
		{
			DrawTexture(Cam->GetRenderTarget(), Margin, Y, W, H, 0.f, 0.f, 1.f, 1.f, FLinearColor::White, BLEND_Opaque);
			DrawText(FString::Printf(TEXT("fork camera  %dx%d @ %.0f Hz  (L: depth)"), Cam->Width, Cam->Height, Cam->CaptureRateHz), FLinearColor::White, Margin + 4.f, Y + 4.f);

			if (bShowDetections && Detector && Detector->GetImageWidth() > 0)
			{
				// Boxes are in the pixel space of the frame the client sent (Cam->Width x Cam->Height); scale to the
				// on-screen thumbnail (W x H).
				const float Sx = W / Detector->GetImageWidth(), Sy = H / Detector->GetImageHeight();
				for (const FForkliftDetectionBox& Box : Detector->GetBoxes())
				{
					const FLinearColor Color = ClassColors[Box.ClassId % UE_ARRAY_COUNT(ClassColors)];
					const float Bx1 = Margin + Box.X1 * Sx, By1 = Y + Box.Y1 * Sy;
					const float Bx2 = Margin + Box.X2 * Sx, By2 = Y + Box.Y2 * Sy;
					constexpr float Thick = 2.f;
					DrawRect(Color, Bx1, By1, Bx2 - Bx1, Thick);                    // top
					DrawRect(Color, Bx1, By2 - Thick, Bx2 - Bx1, Thick);            // bottom
					DrawRect(Color, Bx1, By1, Thick, By2 - By1);                    // left
					DrawRect(Color, Bx2 - Thick, By1, Thick, By2 - By1);            // right
					DrawText(FString::Printf(TEXT("%s %.0f%%"), *Box.ClassName, Box.Confidence * 100.f), Color, Bx1 + 2.f, FMath::Max(By1 - 14.f, Y));
				}
				DrawText(Detector->GetStatus(), Detector->IsConnected() ? FLinearColor::White : FLinearColor(1.f, 0.6f, 0.2f), Margin + W - 120.f, Y + H - 14.f);
			}
			Y += H + Margin;
		}
	}

	if (const UForkliftCameraComponent* Cam = Truck->GetTineCamera(); Cam && Cam->GetRenderTarget())
	{
		const float W = 320.f, H = W * Cam->Height / Cam->Width;
		const UForkliftAutoPickupComponent* Auto = Truck->GetAutoPickup();
		if (bShowDepth && Cam->GetDepthColorTexture())
		{
			DrawTexture(Cam->GetDepthColorTexture(), Margin, Y, W, H, 0.f, 0.f, 1.f, 1.f, FLinearColor::White, BLEND_Opaque);
			DrawText(FString::Printf(TEXT("tine RGB-D DEPTH  %.0f-%.0f cm  (L: RGB)"), TineDepthNearCm, TineDepthFarCm), FLinearColor::White, Margin + 4.f, Y + 4.f);
			if (Auto)
			{
				DrawText(Auto->GetStatus(), Auto->IsPickupActive() ? FLinearColor::Green : FLinearColor::White, Margin + 4.f, Y + H - 16.f);
			}
			Y += H + Margin;
		}
		else
		{
			DrawTexture(Cam->GetRenderTarget(), Margin, Y, W, H, 0.f, 0.f, 1.f, 1.f, FLinearColor::White, BLEND_Opaque);
			DrawText(FString::Printf(TEXT("tine RGB-D  %dx%d @ %.0f Hz  (L: depth)"), Cam->Width, Cam->Height, Cam->CaptureRateHz), FLinearColor::White, Margin + 4.f, Y + 4.f);

			const UForkliftDetectionClient* TineDetector = Auto ? Auto->GetDetector() : nullptr;
			if (bShowDetections && TineDetector && TineDetector->GetImageWidth() > 0)
			{
				const float Sx = W / TineDetector->GetImageWidth(), Sy = H / TineDetector->GetImageHeight();
				for (const FForkliftDetectionBox& Box : TineDetector->GetBoxes())
				{
					const FLinearColor Color = ClassColors[Box.ClassId % UE_ARRAY_COUNT(ClassColors)];
					const float Bx1 = Margin + Box.X1 * Sx, By1 = Y + Box.Y1 * Sy;
					const float Bx2 = Margin + Box.X2 * Sx, By2 = Y + Box.Y2 * Sy;
					constexpr float Thick = 2.f;
					DrawRect(Color, Bx1, By1, Bx2 - Bx1, Thick);
					DrawRect(Color, Bx1, By2 - Thick, Bx2 - Bx1, Thick);
					DrawRect(Color, Bx1, By1, Thick, By2 - By1);
					DrawRect(Color, Bx2 - Thick, By1, Thick, By2 - By1);
					DrawText(FString::Printf(TEXT("%s %.0f%%"), *Box.ClassName, Box.Confidence * 100.f), Color, Bx1 + 2.f, FMath::Max(By1 - 14.f, Y));
				}
			}
			if (Auto)
			{
				DrawText(Auto->GetStatus(), Auto->IsPickupActive() ? FLinearColor::Green : FLinearColor::White, Margin + 4.f, Y + H - 16.f);
			}
			Y += H + Margin;
		}
	}

	if (const ULidarSensorComponent* Lidar = Truck->GetLidar())
	{
		const float Cx = Margin + RadarRadiusPx, Cy = Y + RadarRadiusPx;
		DrawRect(FLinearColor(0.f, 0.f, 0.f, 0.55f), Margin, Y, RadarRadiusPx * 2.f, RadarRadiusPx * 2.f);
		DrawText(FString::Printf(TEXT("lidar  %d returns  %.1f ms  (%.0f m ring)"), Lidar->GetPointsRef().Num(), Lidar->GetScanCostMs(), RadarRangeCm * 0.01f),
			FLinearColor::White, Margin + 4.f, Y + 4.f);

		const float Scale = RadarRadiusPx / RadarRangeCm;
		for (const FVector& P : Lidar->GetPointsRef())
		{
			// Sensor frame: +X forward -> up the screen, +Y right -> right on the screen.
			const float Px = Cx + P.Y * Scale, Py = Cy - P.X * Scale;
			if (FMath::Abs(Px - Cx) < RadarRadiusPx && FMath::Abs(Py - Cy) < RadarRadiusPx)
			{
				DrawRect(FLinearColor::Green, Px, Py, 2.f, 2.f);
			}
		}
		DrawRect(FLinearColor::Red, Cx - 3.f, Cy - 3.f, 6.f, 6.f);  // the sensor itself
	}
}
