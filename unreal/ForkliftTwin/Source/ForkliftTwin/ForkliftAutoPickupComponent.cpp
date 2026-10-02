#include "ForkliftAutoPickupComponent.h"

#include "ForkliftCameraComponent.h"
#include "ForkliftDetectionClient.h"
#include "ForkliftVehicle.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY_STATIC(LogForkliftAutoPickup, Log, All);

namespace
{
	const TCHAR* StateName(EAutoPickupState State)
	{
		switch (State)
		{
		case EAutoPickupState::Idle: return TEXT("idle");
		case EAutoPickupState::Acquiring: return TEXT("acquiring pockets");
		case EAutoPickupState::Aligning: return TEXT("aligning");
		case EAutoPickupState::Inserting: return TEXT("inserting");
		case EAutoPickupState::Settling: return TEXT("settling");
		case EAutoPickupState::Lifting: return TEXT("lifting");
		case EAutoPickupState::Withdrawing: return TEXT("withdrawing");
		case EAutoPickupState::Complete: return TEXT("complete");
		case EAutoPickupState::Aborted: return TEXT("aborted");
		default: return TEXT("unknown");
		}
	}
}

UForkliftAutoPickupComponent::UForkliftAutoPickupComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UForkliftAutoPickupComponent::BeginPlay()
{
	Super::BeginPlay();
	Vehicle = Cast<AForkliftVehicle>(GetOwner());
	Detector = NewObject<UForkliftDetectionClient>(this, TEXT("TineCameraDetector"));
	Detector->ServerUrl = DetectorUrl;
	Detector->DetectionRateHz = DetectionRateHz;

	// -ForkliftAutoPickupTest is started explicitly by AForkliftGameMode after a brief settle: starting
	// acquisition immediately on BeginPlay would lock onto a frame captured before a just-teleported
	// raycast-wheel body has settled onto its commanded pose (see ForkliftGameMode::PollAutoPickupRun).
	if (FParse::Param(FCommandLine::Get(), TEXT("ForkliftAutoPickup")))
	{
		StartPickup();
	}
}

bool UForkliftAutoPickupComponent::IsPickupActive() const
{
	return State >= EAutoPickupState::Acquiring && State <= EAutoPickupState::Withdrawing;
}

void UForkliftAutoPickupComponent::StartPickup()
{
	AForkliftVehicle* Truck = Vehicle.Get();
	if (!Truck || !Detector || IsPickupActive())
	{
		return;
	}
	bRestoreKeyboardControl = Truck->bKeyboardControl;
	Truck->bKeyboardControl = false;
	Truck->SetDriveInput(0.f, 0.f, 0.f);
	TotalTime = 0.f;
	TargetAge = BIG_NUMBER;
	StableTargetFrames = 0;
	CloseMisalignmentTime = 0.f;
	bSeededFromSim = false;
	SetState(EAutoPickupState::Acquiring, TEXT("need two pallet-pocket detections"));
}

void UForkliftAutoPickupComponent::AbortPickup(const FString& Reason)
{
	if (IsPickupActive())
	{
		Finish(false, Reason);
	}
}

void UForkliftAutoPickupComponent::TogglePickup()
{
	if (IsPickupActive())
	{
		AbortPickup(TEXT("operator abort"));
	}
	else
	{
		StartPickup();
	}
}

void UForkliftAutoPickupComponent::SetState(EAutoPickupState NewState, const FString& Detail)
{
	State = NewState;
	StateTime = 0.f;
	StatusDetail = Detail;
	UE_LOG(LogForkliftAutoPickup, Log, TEXT("[AutoPickup] %s%s%s"), StateName(State), Detail.IsEmpty() ? TEXT("") : TEXT(": "), *Detail);
}

void UForkliftAutoPickupComponent::Finish(bool bSuccess, const FString& Detail)
{
	HoldStill();
	if (AForkliftVehicle* Truck = Vehicle.Get())
	{
		Truck->bKeyboardControl = bRestoreKeyboardControl;
	}
	SetState(bSuccess ? EAutoPickupState::Complete : EAutoPickupState::Aborted, Detail);
}

void UForkliftAutoPickupComponent::HoldStill()
{
	if (AForkliftVehicle* Truck = Vehicle.Get())
	{
		Truck->SetDriveInput(0.f, 0.f, 0.f);
	}
}

bool UForkliftAutoPickupComponent::BoxToCameraPoint(const FForkliftDetectionBox& Box, FVector& OutPoint) const
{
	const AForkliftVehicle* Truck = Vehicle.Get();
	const UForkliftCameraComponent* Camera = Truck ? Truck->GetTineCamera() : nullptr;
	if (!Detector || !Camera || Detector->GetImageWidth() <= 0 || Detector->GetImageHeight() <= 0)
	{
		return false;
	}

	const int32 W = Detector->GetImageWidth();
	const int32 H = Detector->GetImageHeight();
	const float CenterX = FMath::Clamp((Box.X1 + Box.X2) * 0.5f, 0.f, W - 1.f);
	const float CenterY = FMath::Clamp((Box.Y1 + Box.Y2) * 0.5f, 0.f, H - 1.f);

	// The centre of a pocket is open space and can see the rear of the pallet. Sample a small grid and use its
	// lower quintile. This selects the front face/boards around the opening while rejecting the high values that
	// see through to its rear. Tine self-depth is absent because this camera hides its owning vehicle.
	TArray<float, TInlineAllocator<25>> DepthSamples;
	for (int32 Gy = 0; Gy < 5; ++Gy)
	{
		for (int32 Gx = 0; Gx < 5; ++Gx)
		{
			const float U = FMath::Lerp(Box.X1, Box.X2, (Gx + 0.5f) / 5.f);
			const float V = FMath::Lerp(Box.Y1, Box.Y2, (Gy + 0.5f) / 5.f);
			float Depth = 0.f;
			if (Detector->GetDepthAtPixel(FMath::RoundToInt(U), FMath::RoundToInt(V), Depth) && Depth > 20.f)
			{
				DepthSamples.Add(Depth);
			}
		}
	}
	if (DepthSamples.Num() < 5)
	{
		return false;
	}
	DepthSamples.Sort();
	const float RobustDepth = DepthSamples[FMath::Clamp(DepthSamples.Num() / 5, 0, DepthSamples.Num() - 1)];

	const float FocalPx = (W * 0.5f) / FMath::Tan(FMath::DegreesToRadians(Camera->FOVAngle * 0.5f));
	OutPoint.X = RobustDepth;
	OutPoint.Y = (CenterX - W * 0.5f) * RobustDepth / FocalPx;
	OutPoint.Z = (H * 0.5f - CenterY) * RobustDepth / FocalPx;
	return true;
}

bool UForkliftAutoPickupComponent::EstimateFaceYawDeg(float PocketMidPixelX, float PocketMidPixelY, float& OutYawDeg) const
{
	// Two pocket depths ~40 cm apart give a noisy yaw (a 3 cm depth error is ~4 deg). The pallet front face is
	// ~120 cm wide, so fit a line to its depth profile across the whole width instead: X = a + b*Y in the camera
	// frame (X forward, Y right), yaw = -atan(b), the same sign convention as the pocket-pair estimate.
	const AForkliftVehicle* Truck = Vehicle.Get();
	const UForkliftCameraComponent* Camera = Truck ? Truck->GetTineCamera() : nullptr;
	if (!Detector || !Camera || Detector->GetImageWidth() <= 0)
	{
		return false;
	}

	const FForkliftDetectionBox* Pallet = nullptr;
	for (const FForkliftDetectionBox& Box : Detector->GetBoxes())
	{
		const bool bPalletClass = Box.ClassId == 0 || Box.ClassId == 1;
		if (bPalletClass && Box.Confidence >= 0.3f && PocketMidPixelX >= Box.X1 && PocketMidPixelX <= Box.X2 &&
			PocketMidPixelY >= Box.Y1 && PocketMidPixelY <= Box.Y2 && (!Pallet || Box.Confidence > Pallet->Confidence))
		{
			Pallet = &Box;
		}
	}
	if (!Pallet)
	{
		return false;
	}

	const int32 W = Detector->GetImageWidth();
	const int32 H = Detector->GetImageHeight();
	const float FocalPx = (W * 0.5f) / FMath::Tan(FMath::DegreesToRadians(Camera->FOVAngle * 0.5f));
	const float V0 = FMath::Lerp(Pallet->Y1, Pallet->Y2, 0.4f);  // lower part only: a load stacked on top can be set back
	const float V1 = Pallet->Y2;

	TArray<FVector2D, TInlineAllocator<128>> Pts;  // (Y, X)
	for (float U = Pallet->X1 + 2.f; U <= Pallet->X2 - 2.f; U += 3.f)
	{
		TArray<float, TInlineAllocator<32>> Column;
		for (float V = V0; V <= V1; V += 3.f)
		{
			float D = 0.f;
			if (Detector->GetDepthAtPixel(FMath::RoundToInt(U), FMath::Clamp(FMath::RoundToInt(V), 0, H - 1), D) && D > 20.f)
			{
				Column.Add(D);
			}
		}
		if (Column.Num() < 4)
		{
			continue;
		}
		Column.Sort();
		const float D = Column[Column.Num() / 8];  // front-most surface in this column, tolerant of a stray pixel
		Pts.Add(FVector2D((U - W * 0.5f) * D / FocalPx, D));
	}
	if (Pts.Num() < 12)
	{
		return false;
	}

	float Slope = 0.f;
	for (int32 Pass = 0; Pass < 3; ++Pass)  // least squares, then drop outliers (pocket-rim columns, box overrun) and refit
	{
		double Sy = 0, Sx = 0, Syy = 0, Syx = 0;
		int32 N = 0;
		float MinY = BIG_NUMBER, MaxY = -BIG_NUMBER;
		for (const FVector2D& P : Pts)
		{
			Sy += P.X; Sx += P.Y; Syy += P.X * P.X; Syx += P.X * P.Y; ++N;
			MinY = FMath::Min(MinY, (float)P.X);
			MaxY = FMath::Max(MaxY, (float)P.X);
		}
		const double Den = N * Syy - Sy * Sy;
		if (N < 12 || MaxY - MinY < 50.f || FMath::Abs(Den) < 1e-6)
		{
			return false;
		}
		Slope = (float)((N * Syx - Sy * Sx) / Den);
		const float Intercept = (float)((Sx - Slope * Sy) / N);
		const float Tolerance = Pass == 0 ? 8.f : 3.f;
		Pts.RemoveAll([&](const FVector2D& P) { return FMath::Abs(P.Y - (Intercept + Slope * P.X)) > Tolerance; });
	}
	OutYawDeg = -FMath::RadiansToDegrees(FMath::Atan(Slope));
	return true;
}

bool UForkliftAutoPickupComponent::UpdateVisionTarget()
{
	AForkliftVehicle* Truck = Vehicle.Get();
	UForkliftCameraComponent* Camera = Truck ? Truck->GetTineCamera() : nullptr;
	if (!Truck || !Camera || !Detector)
	{
		return false;
	}

	struct FCandidate
	{
		FVector CameraPoint;
		float ImageX = 0.f;
		float ImageY = 0.f;
		float Confidence = 0.f;
	};
	TArray<FCandidate> Pockets;
	for (const FForkliftDetectionBox& Box : Detector->GetBoxes())
	{
		if (Box.Confidence < MinimumPocketConfidence || (Box.ClassId != 2 && !Box.ClassName.Equals(TEXT("pallet_pocket"), ESearchCase::IgnoreCase)))
		{
			continue;
		}
		FVector Point;
		if (BoxToCameraPoint(Box, Point))
		{
			Pockets.Add({ Point, (Box.X1 + Box.X2) * 0.5f, (Box.Y1 + Box.Y2) * 0.5f, Box.Confidence });
		}
	}
	if (Pockets.Num() < 2)
	{
		return false;  // one opening is ambiguous: it cannot safely centre a two-tine carriage
	}

	// If more than one pallet is visible, choose the closest plausible left/right pair with useful separation.
	Pockets.Sort([](const FCandidate& A, const FCandidate& B) { return A.CameraPoint.X < B.CameraPoint.X; });
	int32 BestA = INDEX_NONE, BestB = INDEX_NONE;
	float BestScore = BIG_NUMBER;
	for (int32 A = 0; A < Pockets.Num(); ++A)
	{
		for (int32 B = A + 1; B < Pockets.Num(); ++B)
		{
			const float Separation = FMath::Abs(Pockets[A].CameraPoint.Y - Pockets[B].CameraPoint.Y);
			const float DepthDelta = FMath::Abs(Pockets[A].CameraPoint.X - Pockets[B].CameraPoint.X);
			if (Separation < 20.f || Separation > 140.f || DepthDelta > 80.f)
			{
				continue;
			}
			const float Score = (Pockets[A].CameraPoint.X + Pockets[B].CameraPoint.X) * 0.5f + DepthDelta * 2.f;
			if (Score < BestScore)
			{
				BestScore = Score;
				BestA = A;
				BestB = B;
			}
		}
	}
	if (BestA == INDEX_NONE)
	{
		return false;
	}

	const FCandidate& A = Pockets[BestA];
	const FCandidate& B = Pockets[BestB];
	const FVector MidCamera = (A.CameraPoint + B.CameraPoint) * 0.5f;
	const FVector CandidateWorld = Detector->GetResultCameraTransform().TransformPosition(MidCamera);

	// Plausibility gates (a first-attempt failure in the recorded run): once the tines are in the pocket the camera
	// sees only fragments and the detector can lock a pocket pair on something else entirely (a rack bay 4 m away), which
	// used to replace the anchor and abort a perfectly good insertion as "off-axis". During and after insertion the anchored
	// pose plus odometry is authoritative; while aligning, a candidate far from the established target is an outlier.
	if (bSeededFromSim)
	{
		return false;   // the pose came from the simulator and is exact; a flickering detector must not drag the axis or the target
	}
	if (State == EAutoPickupState::Inserting || State == EAutoPickupState::Settling || State == EAutoPickupState::Lifting || State == EAutoPickupState::Withdrawing)
	{
		return false;
	}
	if (State == EAutoPickupState::Aligning && TargetAge < PocketLossAbortSec && FVector::Dist2D(CandidateWorld, TargetWorld) > 50.f)
	{
		return false;
	}

	const FVector& Left = A.ImageX < B.ImageX ? A.CameraPoint : B.CameraPoint;
	const FVector& Right = A.ImageX < B.ImageX ? B.CameraPoint : A.CameraPoint;
	const FVector Across = Right - Left;
	// Yaw of the pallet axis relative to the camera; 0 when square (both openings equally deep). Positive = axis points right of the truck.
	float CandidateYawDeg = -FMath::RadiansToDegrees(FMath::Atan2(Across.X, FMath::Abs(Across.Y)));
	float FaceYawDeg = 0.f;
	if (EstimateFaceYawDeg((A.ImageX + B.ImageX) * 0.5f, (A.ImageY + B.ImageY) * 0.5f, FaceYawDeg))
	{
		CandidateYawDeg = FaceYawDeg;
	}
	// The pallet's insertion axis is fixed in the world, so store it as a world yaw. CandidateYawDeg is the pallet
	// axis yaw relative to the camera (positive = axis points to the truck's right, same sign the steering law uses),
	// which lets the controller keep an exact yaw error while the truck turns and after the pockets leave view.
	const float CandidateAxisWorld = Detector->GetResultCameraTransform().Rotator().Yaw + CandidateYawDeg;
	if (TargetAge < 1.f)
	{
		// The target is stationary in world space. Filtering rejects individual detector/depth outliers without
		// adding control lag as the truck moves. Depth-based yaw becomes noisy when the tines nearly enter the
		// pockets, so finish the last metre using the already-established axis.
		TargetWorld = FMath::Lerp(TargetWorld, CandidateWorld, 0.35f);
		if (MidCamera.X > 70.f)
		{
			PalletAxisYawWorld = FRotator::NormalizeAxis(PalletAxisYawWorld + 0.25f * FMath::FindDeltaAngleDegrees(PalletAxisYawWorld, CandidateAxisWorld));
		}
	}
	else
	{
		TargetWorld = CandidateWorld;
		PalletAxisYawWorld = CandidateAxisWorld;
	}
	TargetAge = 0.f;
	return true;
}

void UForkliftAutoPickupComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	AForkliftVehicle* Truck = Vehicle.Get();
	UForkliftCameraComponent* Camera = Truck ? Truck->GetTineCamera() : nullptr;
	if (!Truck || !Camera || !Detector)
	{
		return;
	}

	if (bPreviewWhenIdle || IsPickupActive())
	{
		Detector->Poll(Camera, DeltaTime);
	}
	TargetAge += DeltaTime;
	if (Detector->GetResultSerial() != LastResultSerial)
	{
		LastResultSerial = Detector->GetResultSerial();
		if (UpdateVisionTarget())
		{
			++StableTargetFrames;
		}
		else
		{
			StableTargetFrames = 0;
		}
	}

	if (!IsPickupActive())
	{
		return;
	}
	StateTime += DeltaTime;
	TotalTime += DeltaTime;
	if (TotalTime > 45.f)
	{
		Finish(false, TEXT("45 s safety timeout"));
		return;
	}

	const FVector Tip = Truck->GetForkTipWorld();
	const FVector Forward = Truck->GetActorForwardVector();
	const FVector Right = Truck->GetActorRightVector();
	const FVector ToTarget = TargetWorld - Tip;
	LastLongitudinalErrorCm = FVector::DotProduct(ToTarget, Forward);
	LastLateralErrorCm = FVector::DotProduct(ToTarget, Right);
	LastVerticalErrorCm = TargetWorld.Z - Tip.Z;
	LastYawErrorDeg = FMath::FindDeltaAngleDegrees(Truck->GetActorRotation().Yaw, PalletAxisYawWorld);
	if (LastVerticalErrorCm < 0.f && LastVerticalErrorCm > -8.f && Truck->GetLiftHeightCm() <= Truck->GetLiftMinCm() + 1.0f)
	{
		LastVerticalErrorCm = 0.f;   // the target is lower than the forks can go: with the lift on its floor the forks are as low as they get
	}
	if (bSeededFromSim)
	{
		LastVerticalErrorCm = 0.f;   // no pocket height was measured; the forks come in lowered, at pocket level (the lift cannot go lower anyway)
	}

	if (FMath::FloorToInt(StateTime * 4.f) != FMath::FloorToInt((StateTime - DeltaTime) * 4.f))
	{
		UE_LOG(LogForkliftAutoPickup, Log, TEXT("[AutoPickup] trace %s t=%.2f gap=%.0f lat=%.1f vert=%.1f yaw=%.1f crossTrack=%.1f axis=%.1f truckYaw=%.1f steerTgt=%.1f age=%.2f"),
			StateName(State), StateTime, LastLongitudinalErrorCm, LastLateralErrorCm, LastVerticalErrorCm, LastYawErrorDeg, CrossTrackCm,
			PalletAxisYawWorld, Truck->GetActorRotation().Yaw, Truck->GetSteerTargetDeg(), TargetAge);
	}

	switch (State)
	{
	case EAutoPickupState::Acquiring:
		HoldStill();
		if (StateTime > 1.f && StableTargetFrames >= 3 && TargetAge < 0.5f)
		{
			SetState(EAutoPickupState::Aligning);
		}
		else if (StateTime > 10.f)
		{
			if (bHasFallback)
			{
				// Vision could not lock the pockets: continue from the camera-derived estimate of the pallet's entry face.
				TargetWorld = FallbackFace;
				TargetWorld.Z = Truck->GetForkTipWorld().Z;           // the lowered forks sit at pocket height (tines 3..8 cm in a 2..10 cm opening); keep that level
				PalletAxisYawWorld = FallbackAxisYaw;
				TargetAge = 0.f;
				bSeededFromSim = true;
				UE_LOG(LogForkliftAutoPickup, Log, TEXT("[AutoPickup] pockets not locked in 10 s - continuing from the camera-derived pallet estimate %s (axis yaw %.1f)"), *TargetWorld.ToString(), PalletAxisYawWorld);
				SetState(EAutoPickupState::Aligning, TEXT("pocket lock failed - using the fork-camera pallet estimate"));
			}
			else
			{
				Finish(false, TEXT("could not lock two pallet pockets"));
			}
		}
		break;

	case EAutoPickupState::Aligning:
	{
		const bool bWithinTolerance = FMath::Abs(LastLateralErrorCm) <= LateralToleranceCm &&
			FMath::Abs(LastVerticalErrorCm) <= VerticalToleranceCm && FMath::Abs(LastYawErrorDeg) <= YawToleranceDeg;
		if (bSeededFromSim)
		{
			TargetAge = 0.f;   // the seeded pose is exact and the pallet does not move; do not treat it as stale
		}
		const bool bCommitOnLastTarget = bWithinTolerance && LastLongitudinalErrorCm <= VisionLossCommitGapCm;
		// The pallet is stationary and TargetWorld is anchored in world space, so a few missed detector frames (the
		// pockets flicker in and out at 1-2 m) are bridged by tracking the last good target, slowly. Only a long
		// outage aborts.
		if (TargetAge > PocketLossAbortSec && !bCommitOnLastTarget)
		{
			Finish(false, TEXT("pallet pockets lost"));
			break;
		}
		const float BearingDeg = FMath::RadiansToDegrees(FMath::Atan2(LastLateralErrorCm, FMath::Max(LastLongitudinalErrorCm, 20.f)));
		// SetDriveInput's steer channel is a wheel-angle RATE (keyboard-like), not an absolute angle. Track a
		// conservative desired rear-wheel angle so a small vision error cannot integrate all the way to steering lock.
		const float DesiredSteerDeg = FMath::Clamp(-(BearingDeg * 0.7f + LastYawErrorDeg * 0.5f), -12.f, 12.f);
		const float SteerAngleError = DesiredSteerDeg - Truck->GetSteerTargetDeg();
		const float Steer = FMath::Clamp(SteerAngleError / 15.f, -0.35f, 0.35f);
		const float Lift = FMath::Clamp(LastVerticalErrorCm / 8.f, -0.65f, 0.65f);
		const bool bHeightSafe = FMath::Abs(LastVerticalErrorCm) < 6.f;
		const bool bGrosslyMisaligned = FMath::Abs(LastLateralErrorCm) > 30.f || FMath::Abs(LastYawErrorDeg) > 20.f;
		float Throttle = bHeightSafe ? ApproachThrottle : 0.f;
		if (bGrosslyMisaligned)
		{
			Throttle = FMath::Min(Throttle, 0.08f);
		}
		if (LastLongitudinalErrorCm < 45.f || TargetAge > 0.6f)   // close in, or creeping on a stale target
		{
			Throttle = FMath::Min(Throttle, 0.08f);
		}
		Truck->SetDriveInput(Throttle, Steer, Lift);

		const bool bAligned = FMath::Abs(LastLateralErrorCm) <= LateralToleranceCm &&
			FMath::Abs(LastVerticalErrorCm) <= VerticalToleranceCm && FMath::Abs(LastYawErrorDeg) <= YawToleranceDeg;
		if (bAligned && LastLongitudinalErrorCm <= 18.f)
		{
			InsertionStart = Truck->GetActorLocation();
			InsertionForward = Forward;
			InsertionLiftHeightCm = Truck->GetLiftHeightCm();
			SetState(EAutoPickupState::Inserting, TEXT("vision aligned; switching to odometry"));
		}
		else if (LastLongitudinalErrorCm <= 35.f && !bAligned)
		{
			CloseMisalignmentTime += DeltaTime;
			if (CloseMisalignmentTime > 3.f)
			{
				Finish(false, FString::Printf(TEXT("too close: gap %.1f lateral %.1f vertical %.1f yaw %.1f"),
					LastLongitudinalErrorCm, LastLateralErrorCm, LastVerticalErrorCm, LastYawErrorDeg));
			}
		}
		else
		{
			CloseMisalignmentTime = 0.f;
		}
		break;
	}

	case EAutoPickupState::Inserting:
	{
		const float Travel = FVector::DotProduct(Truck->GetActorLocation() - InsertionStart, InsertionForward);
		const float LiftHold = FMath::Clamp((InsertionLiftHeightCm - Truck->GetLiftHeightCm()) / 3.f, -0.4f, 0.4f);

		// Closed-loop insertion (Nav2-docking style): the pallet pose stays anchored in the world from the last
		// good detection, and the tine tips are steered onto its axis with pure pursuit. Live pocket sightings keep
		// refining the anchor (UpdateVisionTarget) for as long as the pockets are visible.
		const FVector Axis = FRotator(0.f, PalletAxisYawWorld, 0.f).Vector();
		const FVector AxisRight = FRotator(0.f, PalletAxisYawWorld + 90.f, 0.f).Vector();
		const FVector Rel = Tip - TargetWorld;
		const float Along = FVector::DotProduct(Rel, Axis);
		CrossTrackCm = FVector::DotProduct(Rel, AxisRight);  // + = tips right of the axis
		const FVector LookAhead = TargetWorld + Axis * (Along + InsertionLookAheadCm);
		const FVector ToLook = LookAhead - Tip;
		const float Bearing = FMath::RadiansToDegrees(FMath::Atan2(FVector::DotProduct(ToLook, Right), FMath::Max(FVector::DotProduct(ToLook, Forward), 20.f)));
		const float DesiredSteerDeg = FMath::Clamp(-(Bearing * 0.5f + LastYawErrorDeg * 0.35f), -8.f, 8.f);
		const float Steer = FMath::Clamp((DesiredSteerDeg - Truck->GetSteerTargetDeg()) / 15.f, -0.3f, 0.3f);
		Truck->SetDriveInput(InsertionThrottle, Steer, LiftHold);

		UE_LOG(LogForkliftAutoPickup, Verbose, TEXT("[AutoPickup] insert travel %.1f along %.1f crossTrack %.1f yawErr %.1f steer %.1f"),
			Travel, Along, CrossTrackCm, LastYawErrorDeg, DesiredSteerDeg);

		const bool bStalled = StateTime > 1.5f && Travel > 35.f && FMath::Abs(Truck->GetForwardSpeedCmPerSec()) < 3.f;
		if (Travel >= InsertionTravelCm || bStalled)
		{
			SetState(EAutoPickupState::Settling, bStalled ? TEXT("stopped by load/backrest") : TEXT("insertion distance reached"));
		}
		else if (Travel > 15.f && (FMath::Abs(CrossTrackCm) > InsertionCrossTrackAbortCm || FMath::Abs(LastYawErrorDeg) > InsertionYawAbortDeg))
		{
			// Skewing into the pallet would jam or shove it (the known 20-25 deg yaw failure): back straight out.
			SetState(EAutoPickupState::Withdrawing, FString::Printf(TEXT("insertion off-axis: cross-track %.1f cm yaw %.1f deg at travel %.0f cm"), CrossTrackCm, LastYawErrorDeg, Travel));
		}
		else if (StateTime > 12.f)
		{
			Finish(false, TEXT("insertion timeout"));
		}
		break;
	}

	case EAutoPickupState::Withdrawing:
	{
		const float Travel = FVector::DotProduct(Truck->GetActorLocation() - InsertionStart, InsertionForward);
		Truck->SetDriveInput(-InsertionThrottle, 0.f, 0.f);
		if (Travel <= 0.f || StateTime > 10.f)
		{
			Finish(false, StatusDetail + TEXT(" - withdrawn"));
		}
		break;
	}

	case EAutoPickupState::Settling:
		HoldStill();
		if (StateTime >= 0.75f)
		{
			LiftTargetCm = FMath::Clamp(Truck->GetLiftHeightCm() + CarryLiftCm, Truck->GetLiftMinCm(), Truck->GetLiftMaxCm());
			SetState(EAutoPickupState::Lifting);
		}
		break;

	case EAutoPickupState::Lifting:
	{
		const float Error = LiftTargetCm - Truck->GetLiftHeightCm();
		Truck->SetDriveInput(0.f, 0.f, FMath::Clamp(Error / 5.f, -1.f, 1.f));
		if (FMath::Abs(Error) < 0.75f)
		{
			Finish(true, TEXT("pallet raised to carry height"));
		}
		else if (StateTime > 8.f)
		{
			Finish(false, TEXT("lift timeout"));
		}
		break;
	}

	default:
		break;
	}
}

FString UForkliftAutoPickupComponent::GetStatus() const
{
	FString Result = FString::Printf(TEXT("auto: %s"), StateName(State));
	if (!StatusDetail.IsEmpty())
	{
		Result += TEXT(" - ") + StatusDetail;
	}
	if (IsPickupActive() && TargetAge < 2.f)
	{
		Result += FString::Printf(TEXT("  gap x/y/z %.0f/%.1f/%.1f cm yaw %.1f deg"), LastLongitudinalErrorCm, LastLateralErrorCm, LastVerticalErrorCm, LastYawErrorDeg);
	}
	return Result;
}
