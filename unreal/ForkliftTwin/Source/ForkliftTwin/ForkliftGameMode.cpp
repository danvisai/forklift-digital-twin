#include "ForkliftGameMode.h"

#include "AssetCompilingManager.h"
#include "Camera/CameraActor.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/SceneCapture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "RenderingThread.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "ForkliftCameraComponent.h"
#include "ForkliftHUD.h"
#include "ForkliftVehicle.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "LidarSensorComponent.h"
#include "Warehouse.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "PhysicsEngine/BodySetup.h"
#include "Components/BoxComponent.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/CoreDelegates.h"
#include "Misc/Parse.h"
#include "ShaderCompiler.h"

DEFINE_LOG_CATEGORY_STATIC(LogForkliftTwin, Log, All);

AForkliftGameMode::AForkliftGameMode()
{
	PrimaryActorTick.bCanEverTick = true;
	// The forklift is the player's pawn; no default spectator pawn.
	DefaultPawnClass = nullptr;
	HUDClass = AForkliftHUD::StaticClass();
}

void AForkliftGameMode::StartPlay()
{
	Super::StartPlay();

	for (TActorIterator<AForkliftVehicle> It(GetWorld()); It; ++It)
	{
		Forklift = *It;
		break;
	}

	if (!Forklift)
	{
		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Forklift = GetWorld()->SpawnActor<AForkliftVehicle>(AForkliftVehicle::StaticClass(), FallbackSpawn, Params);
		UE_LOG(LogForkliftTwin, Log, TEXT("No forklift in the level; spawned one at %s"), *FallbackSpawn.GetLocation().ToString());
	}

	if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
	{
		if (Forklift && PC->GetPawn() != Forklift)
		{
			PC->Possess(Forklift);
		}
	}

	if (GEngine && Forklift && !FParse::Param(FCommandLine::Get(), TEXT("ForkliftSelfTest")))
	{
		GEngine->AddOnScreenDebugMessage(101, 30.f, FColor::Cyan, TEXT("W/S drive   A/D steer (rear-wheel)   R/F mast up/down   mouse look   V sensor view"));
	}

	bSensorTest = FParse::Param(FCommandLine::Get(), TEXT("ForkliftSensorTest"));
	if (bSensorTest && Forklift)
	{
		// Face the loose block pallet from 3.5 m (Kit (-3.5, 2.5) -> UE (-350, -250)), clear of the rack colliders.
		Forklift->bKeyboardControl = false;
		Forklift->SetActorLocationAndRotation(FVector(0.f, -250.f, 15.f), FRotator(0.f, 180.f, 0.f), false, nullptr, ETeleportType::TeleportPhysics);
		SensorTestDir = FPaths::ProjectSavedDir() / TEXT("SensorTest");
		FParse::Value(FCommandLine::Get(), TEXT("ForkliftSensorOut="), SensorTestDir);
		IFileManager::Get().MakeDirectory(*SensorTestDir, true);
	}

	if (FParse::Value(FCommandLine::Get(), TEXT("ForkliftShot="), ShotPreset) && Forklift)
	{
		Forklift->bKeyboardControl = false;
		const FVector Base = Forklift->GetActorLocation();
		const FRotator Yaw(0.f, Forklift->GetActorRotation().Yaw, 0.f);
		FVector Target = Base + FVector(0.f, 0.f, 110.f);
		switch (ShotPreset)
		{
		case 1: ShotPos = Base + Yaw.RotateVector(FVector(520.f, -380.f, 230.f)); break;   // front-left three-quarter
		case 2: ShotPos = Base + Yaw.RotateVector(FVector(-420.f, 460.f, 320.f)); break;   // rear-right three-quarter
		case 3: ShotPos = FVector(-900.f, 1100.f, 900.f); Target = FVector(0.f, 100.f, 80.f); break;  // overview of the hall
		default: ShotPos = Base + Yaw.RotateVector(FVector(-700.f, 0.f, 260.f)); break;    // 0 = behind the truck
		}
		ShotRot = (Target - ShotPos).Rotation();
		if (FParse::Param(FCommandLine::Get(), TEXT("ForkliftShotViewport")))
		{
			// Frame it through the real game view so exposure / GI / post-process match what a player sees.
			if (ACameraActor* Cam = GetWorld()->SpawnActor<ACameraActor>(ShotPos, ShotRot))
			{
				GetWorld()->GetFirstPlayerController()->SetViewTarget(Cam);
			}
		}
	}

	bPalletTest = FParse::Param(FCommandLine::Get(), TEXT("ForkliftPalletTest"));
	if (bPalletTest && Forklift)
	{
		// Same parking spot as the sensor test: 3.5 m from the loose pallet at Kit (-3.5, 2.5) = UE (-350, -250),
		// forks pointing at it (-X), lined up on its centre.
		Forklift->bKeyboardControl = false;
		Forklift->SetActorLocationAndRotation(FVector(0.f, -250.f, 15.f), FRotator(0.f, 180.f, 0.f), false, nullptr, ETeleportType::TeleportPhysics);
		// Fixed 60 Hz steps. The forks are a kinematic body whose velocity Chaos derives from how far it moved since the
		// last physics step; with erratic frame times (an unthrottled headless run) that velocity is mis-scaled every
		// frame and a carried pallet, held by friction alone, creeps toward the tips (6-10 cm per 3 m measured).
		// At a fixed step it does not (0.1-0.2 cm). Data-generation / RL runs should use a fixed step as well.
		FApp::SetUseFixedTimeStep(true);
		FApp::SetFixedDeltaTime(1.0 / 60.0);
		for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
		{
			PalletComp = It->GetLoosePallet();
			break;
		}
		if (!PalletComp.IsValid())
		{
			UE_LOG(LogForkliftTwin, Error, TEXT("[PalletTest] no loose pallet in the warehouse - run Scripts/setup.py"));
			bPalletTest = false;
		}
	}

	bSelfTest = FParse::Param(FCommandLine::Get(), TEXT("ForkliftSelfTest"));
	if (bSelfTest && Forklift)
	{
		Forklift->bKeyboardControl = false;
		UE_LOG(LogForkliftTwin, Log, TEXT("[SelfTest] starting; forklift at %s"), *Forklift->GetActorLocation().ToString());
	}
}

void AForkliftGameMode::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (bSelfTest && Forklift)
	{
		SelfTestTick(DeltaSeconds);
	}
	if (ShotPreset >= 0)
	{
		PhotoModeTick(DeltaSeconds);
	}
	if (bSensorTest && Forklift)
	{
		SensorTestTick(DeltaSeconds);
	}
	if (bPalletTest && Forklift)
	{
		PalletTestTick(DeltaSeconds);
	}
}

void AForkliftGameMode::LogPalletState(const TCHAR* Tag)
{
	UStaticMeshComponent* Pallet = PalletComp.Get();
	const FTransform Truck = Forklift->GetActorTransform();
	const FVector P = Pallet->GetComponentLocation();
	const FRotator R = Pallet->GetComponentRotation();
	const FVector Local = Truck.InverseTransformPosition(P);
	// Depth of the tines inside the pallet: tip x minus the pallet's near edge (its centre less half its 100 cm length), truck frame.
	const float Insertion = Truck.InverseTransformPosition(Forklift->GetForkTipWorld()).X - (Local.X - 50.f);
	MaxInsertionCm = FMath::Max(MaxInsertionCm, Insertion);
	UE_LOG(LogForkliftTwin, Log,
		TEXT("[PalletTest] %-12s t=%5.1f truck=(%6.1f %6.1f yaw %6.1f) speed=%6.1f lift=%6.1f | pallet=(%7.1f %7.1f z=%5.1f yaw=%6.1f pitch=%5.1f roll=%5.1f) inTruckFrame=(%6.1f %6.1f %6.1f) insertion=%6.1f cm"),
		Tag, PalletTestTotal, Truck.GetLocation().X, Truck.GetLocation().Y, Forklift->GetActorRotation().Yaw, Forklift->GetForwardSpeedCmPerSec(), Forklift->GetLiftHeightCm(),
		P.X, P.Y, P.Z, R.Yaw, R.Pitch, R.Roll, Local.X, Local.Y, Local.Z, Insertion);
}

void AForkliftGameMode::PalletTestTick(float DeltaSeconds)
{
	if ((GShaderCompilingManager && GShaderCompilingManager->IsCompiling()) || FAssetCompilingManager::Get().GetNumRemainingAssets() > 0)
	{
		PalletTestFrames = 0;
		return;
	}
	UStaticMeshComponent* Pallet = PalletComp.Get();
	if (!Pallet)
	{
		FPlatformMisc::RequestExit(false);
		return;
	}
	++PalletTestFrames;
	if (PalletTestFrames < 60 && PalletTestState == 0)
	{
		return;  // let the truck and the pallet settle first
	}
	PalletTestTime += DeltaSeconds;
	PalletTestTotal += DeltaSeconds;

	const FTransform Truck = Forklift->GetActorTransform();
	const FVector PalletPos = Pallet->GetComponentLocation();
	const auto Next = [this](const TCHAR* Tag)
	{
		LogPalletState(Tag);
		++PalletTestState;
		PalletTestTime = 0.f;
	};
	constexpr float LiftTargetCm = 30.f;   // fork height above the rest pose while carrying
	constexpr float CarryDistanceCm = 300.f;
	// Fork height that centres a 5.3 cm tine in the pallet's 8 cm pocket (z 2..10): tine centre 20.65 + lift = 6.0.
	constexpr float PocketEntryLiftCm = -14.65f;
	// Closed-loop lift command: the exact fraction of full rate that lands on the target this frame, so the forks stop
	// on the target height instead of overshooting by up to a frame's travel (0.4-0.7 cm; the pocket has ~1.3 cm to spare).
	const auto LiftTo = [this, DeltaSeconds](float TargetCm)
	{
		const float Step = FMath::Max(Forklift->LiftRateCmPerSec * FMath::Clamp(DeltaSeconds, 1.e-4f, 1.f / 20.f), 1.e-3f);
		return FMath::Clamp((TargetCm - Forklift->GetLiftHeightCm()) / Step, -1.f, 1.f);
	};

	// Every 0.25 s while the pallet is on the forks: where it sits in the truck frame, and how fast everything moves.
	PalletLogTimer += DeltaSeconds;
	if (PalletTestState >= 1 && PalletTestState <= 6 && PalletLogTimer >= 0.25f)
	{
		PalletLogTimer = 0.f;
		const FVector L = Truck.InverseTransformPosition(PalletPos);
		UE_LOG(LogForkliftTwin, Log, TEXT("[PalletTest]   trace s%d truckSpeed=%7.1f throttle=%4.1f lift=%5.1f pitch=%5.2f tipZ=%5.2f pallet inTruckFrame=(%6.1f %6.1f %6.1f) palletMoved=%6.1f palletVel=(%7.1f %7.1f %7.1f)"),
			PalletTestState, Forklift->GetForwardSpeedCmPerSec(), Forklift->GetThrottleCmd(), Forklift->GetLiftHeightCm(), Forklift->GetActorRotation().Pitch,
			Forklift->GetForkTipWorld().Z - 2.65f - 0.f, L.X, L.Y, L.Z, FVector::Dist2D(PalletPos, PalletStartPos),
			Pallet->GetPhysicsLinearVelocity().X, Pallet->GetPhysicsLinearVelocity().Y, Pallet->GetPhysicsLinearVelocity().Z);
	}

	if (PalletTestTotal > 120.f && PalletTestState < 99)
	{
		UE_LOG(LogForkliftTwin, Error, TEXT("[PalletTest] TIMEOUT in state %d"), PalletTestState);
		PalletTestState = 99;
	}

	switch (PalletTestState)
	{
	case 0:  // forks to the pocket-entry height (B: tines centred in the 8 cm opening; C would need its rest pose lowered)
		Forklift->SetDriveInput(0.f, 0.f, LiftTo(FMath::Max(PocketEntryLiftCm, Forklift->GetLiftMinCm())));
		if (PalletTestTime > 1.5f && FMath::Abs(Forklift->GetLiftHeightCm() - FMath::Max(PocketEntryLiftCm, Forklift->GetLiftMinCm())) < 0.1f)
		{
			PalletStartPos = PalletPos;
			TruckStartPos = Truck.GetLocation();
			// What the physics engine actually sees: the pallet's collision shapes (should be the 5 hollow boxes).
			if (const UBodySetup* Setup = Pallet->GetStaticMesh() ? Pallet->GetStaticMesh()->GetBodySetup() : nullptr)
			{
				UE_LOG(LogForkliftTwin, Log, TEXT("[PalletTest] pallet collision: boxes=%d convex=%d sphyl=%d sphere=%d traceFlag=%d simulating=%d"),
					Setup->AggGeom.BoxElems.Num(), Setup->AggGeom.ConvexElems.Num(), Setup->AggGeom.SphylElems.Num(), Setup->AggGeom.SphereElems.Num(),
					static_cast<int32>(Setup->CollisionTraceFlag), Pallet->IsSimulatingPhysics() ? 1 : 0);
				for (const FKBoxElem& Box : Setup->AggGeom.BoxElems)
				{
					UE_LOG(LogForkliftTwin, Log, TEXT("[PalletTest]   box center=(%.1f %.1f %.1f) size=(%.1f %.1f %.1f)"), Box.Center.X, Box.Center.Y, Box.Center.Z, Box.X, Box.Y, Box.Z);
				}
			}
			// The tine / platform colliders in the world (min / max corners) and their physics state.
			TInlineComponentArray<UBoxComponent*> Boxes;
			Forklift->GetComponents(Boxes);
			for (const UBoxComponent* Box : Boxes)
			{
				const FBoxSphereBounds B = Box->Bounds;
				UE_LOG(LogForkliftTwin, Log, TEXT("[PalletTest]   %s world min=(%.1f %.1f %.1f) max=(%.1f %.1f %.1f) sim=%d collision=%d objType=%d"), *Box->GetName(),
					B.Origin.X - B.BoxExtent.X, B.Origin.Y - B.BoxExtent.Y, B.Origin.Z - B.BoxExtent.Z, B.Origin.X + B.BoxExtent.X, B.Origin.Y + B.BoxExtent.Y, B.Origin.Z + B.BoxExtent.Z,
					Box->IsSimulatingPhysics() ? 1 : 0, static_cast<int32>(Box->GetCollisionEnabled()), static_cast<int32>(Box->GetCollisionObjectType()));
			}
			Next(TEXT("start"));
		}
		break;
	case 1:  // drive in at half speed until the tines are seated (or the platform stops the pallet)
	{
		const float Insertion = Truck.InverseTransformPosition(Forklift->GetForkTipWorld()).X - (Truck.InverseTransformPosition(PalletPos).X - 50.f);
		// Half speed until the tine tips are ~1 m short of the pallet's near edge (insertion -100), then a slow crawl for
		// the last stretch so the platform doesn't shove the pallet when the truck stops.
		Forklift->SetDriveInput(Insertion < -100.f ? 0.5f : 0.2f, 0.f, LiftTo(FMath::Max(PocketEntryLiftCm, Forklift->GetLiftMinCm())));
		const bool bStalled = PalletTestTime > 3.f && FMath::Abs(Forklift->GetForwardSpeedCmPerSec()) < 5.f;
		if (Insertion >= 95.f || bStalled || PalletTestTime > 25.f)
		{
			Next(TEXT("inserted"));
		}
		break;
	}
	case 2:  // stop and settle
		Forklift->SetDriveInput(0.f, 0.f, 0.f);
		if (PalletTestTime > 1.f)
		{
			Next(TEXT("settled"));
		}
		break;
	case 3:  // lift ~45 cm above the lowest fork position
		Forklift->SetDriveInput(0.f, 0.f, 1.f);
		if (Forklift->GetLiftHeightCm() >= LiftTargetCm || PalletTestTime > 6.f)
		{
			Next(TEXT("lifted"));
		}
		break;
	case 4:  // hold; record where the pallet sits in the truck frame
		Forklift->SetDriveInput(0.f, 0.f, 0.f);
		if (PalletTestTime > 1.f)
		{
			LiftedZCm = PalletPos.Z - PalletStartPos.Z;
			PalletHoldLocal = Truck.InverseTransformPosition(PalletPos);
			CarryStartPos = Truck.GetLocation();
			Next(TEXT("held"));
		}
		break;
	case 5:  // reverse 3 m with the pallet on the forks
	{
		Forklift->SetDriveInput(-0.5f, 0.f, 0.f);
		const float Moved = FVector::Dist2D(Truck.GetLocation(), CarryStartPos);
		if (!bPalletShotTaken && Moved >= CarryDistanceCm * 0.5f && FParse::Param(FCommandLine::Get(), TEXT("ForkliftPalletShot")))
		{
			// Side view of the truck mid-carry, through the real game view so the HUD and lighting match what a player sees.
			bPalletShotTaken = true;
			LogPalletState(TEXT("mid-carry"));
			const FVector Mid = Truck.GetLocation() + FVector(0.f, 0.f, 70.f);
			const FVector CamPos = Mid + FVector(120.f, -380.f, 110.f);
			if (ACameraActor* Cam = GetWorld()->SpawnActor<ACameraActor>(CamPos, (Mid + FVector(-120.f, 0.f, 0.f) - CamPos).Rotation()))
			{
				GetWorld()->GetFirstPlayerController()->SetViewTarget(Cam);
			}
			GetWorld()->GetFirstPlayerController()->ConsoleCommand(TEXT("HighResShot 1600x900"));
		}
		if (Moved >= CarryDistanceCm || PalletTestTime > 15.f)
		{
			Next(TEXT("carried"));
		}
		break;
	}
	case 6:  // stop; how much did the pallet slip on the tines?
		Forklift->SetDriveInput(0.f, 0.f, 0.f);
		if (PalletTestTime > 1.f)
		{
			CarrySlipCm = (Truck.InverseTransformPosition(PalletPos) - PalletHoldLocal).Size();
			Next(TEXT("stopped"));
		}
		break;
	case 7:  // set it down: forks back to the pocket-entry height (pallet on the floor, tines free inside the pockets)
		Forklift->SetDriveInput(0.f, 0.f, LiftTo(FMath::Max(PocketEntryLiftCm, Forklift->GetLiftMinCm())));
		if (FMath::Abs(Forklift->GetLiftHeightCm() - FMath::Max(PocketEntryLiftCm, Forklift->GetLiftMinCm())) < 0.1f || PalletTestTime > 8.f)
		{
			SetDownZCm = PalletPos.Z - PalletStartPos.Z;
			PalletDownPos = PalletPos;
			Next(TEXT("set down"));
		}
		break;
	case 8:  // withdraw the tines (reverse) and see whether the pallet stays where it was put
		Forklift->SetDriveInput(-0.3f, 0.f, LiftTo(FMath::Max(PocketEntryLiftCm, Forklift->GetLiftMinCm())));
		if (PalletTestTime > 3.5f)
		{
			BackOffShiftCm = FVector::Dist2D(PalletPos, PalletDownPos);
			Next(TEXT("backed off"));
		}
		break;
	default:
	{
		Forklift->SetDriveInput(0.f, 0.f, 0.f);
		const bool bInserted = MaxInsertionCm >= 90.f;
		const bool bLifted = LiftedZCm >= 20.f;
		const bool bCarried = CarrySlipCm < 5.f;
		const bool bSetDown = FMath::Abs(SetDownZCm) < 3.f && BackOffShiftCm < 5.f;
		const float Moved = FVector::Dist2D(PalletPos, PalletStartPos);
		if (!bPalletSummaryLogged)
		{
		bPalletSummaryLogged = true;
		UE_LOG(LogForkliftTwin, Log,
			TEXT("[PalletTest] SUMMARY inserted=%s (max %.1f cm) lifted=%s (pallet rose %.1f cm) carried=%s (slip %.1f cm) setDown=%s (z %.1f cm, shifted %.1f cm on withdraw) | pallet moved %.0f cm from start"),
			bInserted ? TEXT("PASS") : TEXT("FAIL"), MaxInsertionCm, bLifted ? TEXT("PASS") : TEXT("FAIL"), LiftedZCm,
			bCarried ? TEXT("PASS") : TEXT("FAIL"), CarrySlipCm, bSetDown ? TEXT("PASS") : TEXT("FAIL"), SetDownZCm, BackOffShiftCm, Moved);
		UE_LOG(LogForkliftTwin, Log, TEXT("[PalletTest] %s"), (bInserted && bLifted && bCarried && bSetDown) ? TEXT("RESULT: PASS") : TEXT("RESULT: FAIL"));
		}
		if (PalletTestTime > (bPalletShotTaken ? 4.f : 0.5f))
		{
			FPlatformMisc::RequestExit(false);
		}
		break;
	}
	}
}

void AForkliftGameMode::SensorTestTick(float DeltaSeconds)
{
	// Same readiness rule as photo mode: don't capture while shaders / meshes are still compiling.
	if ((GShaderCompilingManager && GShaderCompilingManager->IsCompiling()) || FAssetCompilingManager::Get().GetNumRemainingAssets() > 0)
	{
		SensorTestFrames = 0;
		return;
	}
	++SensorTestFrames;

	switch (SensorTestState)
	{
	case 0:  // let the truck settle and the renderer warm up, then sample with the forks down
		if (SensorTestFrames > 120)
		{
			DumpSensors(TEXT("lift0"));
			SensorTestState = 1;
			SensorTestTime = 0.f;
		}
		break;
	case 1:  // raise the forks ~1 m (40 cm/s) and sample again: the camera must have gone up with them
		Forklift->SetDriveInput(0.f, 0.f, 1.f);
		SensorTestTime += DeltaSeconds;
		if (SensorTestTime >= 2.5f)
		{
			Forklift->SetDriveInput(0.f, 0.f, 0.f);
			DumpSensors(TEXT("lift100"));
			SensorTestState = 2;
			SensorTestFrames = 0;
		}
		break;
	default:
		if (SensorTestFrames > 10)
		{
			FPlatformMisc::RequestExit(false);
		}
		break;
	}
}

void AForkliftGameMode::DumpSensors(const TCHAR* Tag)
{
	UForkliftCameraComponent* Cam = Forklift->GetForkCamera();
	ULidarSensorComponent* Lidar = Forklift->GetLidar();
	Cam->Capture();
	Lidar->Scan();
	FlushRenderingCommands();

	const FString Base = SensorTestDir / Tag;
	const bool bPng = Cam->SavePng(Base + TEXT("_camera.png"));
	const bool bRanges = Lidar->SaveRangesNpy(Base + TEXT("_lidar_ranges.npy"));
	const bool bPoints = Lidar->SavePointsNpy(Base + TEXT("_lidar_points.npy"));

	TArray<FColor> Pixels;
	double Sum = 0.0;
	if (Cam->ReadPixelsBgra(Pixels))
	{
		for (const FColor& C : Pixels)
		{
			Sum += (C.R + C.G + C.B) / 3.0;
		}
	}
	float MinR = TNumericLimits<float>::Max(), MaxR = 0.f;
	for (const float R : Lidar->GetRangesCm())
	{
		if (R >= 0.f)
		{
			MinR = FMath::Min(MinR, R);
			MaxR = FMath::Max(MaxR, R);
		}
	}
	const FVector CamPos = Cam->GetComponentLocation();
	UE_LOG(LogForkliftTwin, Log,
		TEXT("[SensorTest] %s: lift=%.1f cm  camera %dx%d z=%.1f cm mean=%.1f/255 png=%d | lidar %d returns of %d rays, range %.0f..%.0f cm, scan %.2f ms npy=%d/%d -> %s"),
		Tag, Forklift->GetLiftHeightCm(), Cam->Width, Cam->Height, CamPos.Z, Pixels.Num() ? Sum / Pixels.Num() : -1.0, bPng,
		Lidar->GetPointsRef().Num(), Lidar->GetRangesCm().Num(), MinR, MaxR, Lidar->GetScanCostMs(), bRanges, bPoints, *SensorTestDir);
}

void AForkliftGameMode::PhotoModeTick(float DeltaSeconds)
{
	// The first frames after load report huge hitches (shader compiles, mesh streaming), so time is
	// counted in frames, and only once shaders and assets have finished compiling.
	const bool bBusy = (GShaderCompilingManager && GShaderCompilingManager->IsCompiling()) || FAssetCompilingManager::Get().GetNumRemainingAssets() > 0;
	if (bBusy)
	{
		ShotIdleFrames = 0;
		return;
	}

	int32 SettleFrames = 90;
	FParse::Value(FCommandLine::Get(), TEXT("ForkliftShotFrames="), SettleFrames);
	++ShotIdleFrames;
	if (!bShotTaken && ShotIdleFrames > SettleFrames)
	{
		bShotTaken = true;
		ShotIdleFrames = 0;
		if (FParse::Param(FCommandLine::Get(), TEXT("ForkliftShotViewport")))
		{
			GetWorld()->GetFirstPlayerController()->ConsoleCommand(TEXT("HighResShot 1600x900"));
			UE_LOG(LogForkliftTwin, Log, TEXT("[Shot] HighResShot (viewport) issued"));
		}
		else
		{
			CaptureShot();
			FPlatformMisc::RequestExit(false);
		}
	}
	else if (bShotTaken && ShotIdleFrames > 150)
	{
		FPlatformMisc::RequestExit(false);
	}
}

// Render from a scene-capture camera straight into a render target and write the PNG ourselves. Unlike
// HighResShot this does not depend on the game window / backbuffer, so it works headless and unattended.
void AForkliftGameMode::CaptureShot()
{
	constexpr int32 Width = 1600, Height = 900;
	UTextureRenderTarget2D* Target = NewObject<UTextureRenderTarget2D>(this);
	Target->InitCustomFormat(Width, Height, PF_B8G8R8A8, false);
	Target->UpdateResourceImmediate(true);

	ASceneCapture2D* Capture = GetWorld()->SpawnActor<ASceneCapture2D>(ShotPos, ShotRot);
	USceneCaptureComponent2D* Comp = Capture->GetCaptureComponent2D();
	Comp->TextureTarget = Target;
	// -ForkliftShotSource=basecolor|hdr|final (default final). Base colour is unlit and exposure independent,
	// which is handy for checking geometry and materials separately from lighting.
	FString Source = TEXT("final");
	FParse::Value(FCommandLine::Get(), TEXT("ForkliftShotSource="), Source);
	Comp->CaptureSource = Source == TEXT("basecolor") ? SCS_BaseColor : Source == TEXT("hdr") ? SCS_FinalToneCurveHDR : SCS_FinalColorLDR;
	// Fixed exposure so the capture doesn't depend on eye-adaptation history (a fresh capture has none).
	FParse::Value(FCommandLine::Get(), TEXT("ForkliftShotEV="), ShotEV);
	Comp->PostProcessSettings.bOverride_AutoExposureMethod = true;
	Comp->PostProcessSettings.AutoExposureMethod = AEM_Manual;
	Comp->PostProcessSettings.bOverride_AutoExposureBias = true;
	Comp->PostProcessSettings.AutoExposureBias = ShotEV;
	Comp->FOVAngle = 60.f;
	Comp->bCaptureEveryFrame = false;
	Comp->bCaptureOnMovement = false;
	Comp->CaptureScene();
	FlushRenderingCommands();

	TArray<FColor> Pixels;
	Target->GameThread_GetRenderTargetResource()->ReadPixels(Pixels);
	double Sum = 0.0;
	for (const FColor& C : Pixels)
	{
		Sum += (C.R + C.G + C.B) / 3.0;
	}
	UE_LOG(LogForkliftTwin, Log, TEXT("[Shot] captured %dx%d, mean brightness %.1f / 255"), Width, Height, Pixels.Num() ? Sum / Pixels.Num() : -1.0);

	for (FColor& C : Pixels)
	{
		C.A = 255;
	}
	TArray64<uint8> Png;
	FImageUtils::PNGCompressImageArray(Width, Height, TArrayView64<const FColor>(Pixels.GetData(), Pixels.Num()), Png);
	FString OutPath = FPaths::ProjectSavedDir() / TEXT("Screenshots") / FString::Printf(TEXT("forklift_shot_%d.png"), ShotPreset);
	FParse::Value(FCommandLine::Get(), TEXT("ForkliftShotOut="), OutPath);
	FFileHelper::SaveArrayToFile(Png, *OutPath);
	UE_LOG(LogForkliftTwin, Log, TEXT("[Shot] wrote %s"), *OutPath);
}

void AForkliftGameMode::SelfTestTick(float DeltaSeconds)
{
	struct FPhase { const TCHAR* Name; float Duration; float Throttle; float Steer; float Lift; };
	static const FPhase Phases[] = {
		{ TEXT("settle"),        1.5f, 0.f, 0.f, 0.f },
		{ TEXT("forward"),       3.0f, 1.f, 0.f, 0.f },
		{ TEXT("coast"),         1.5f, 0.f, 0.f, 0.f },
		{ TEXT("forward+right"), 3.0f, 1.f, -1.f, 0.f },
		{ TEXT("stop"),          1.5f, 0.f, 0.f, 0.f },
		{ TEXT("reverse"),       2.0f, -1.f, 0.f, 0.f },
		{ TEXT("stop"),          1.0f, 0.f, 0.f, 0.f },
		{ TEXT("forward+left"),  3.0f, 1.f, 1.f, 0.f },
		{ TEXT("stop"),          1.5f, 0.f, 0.f, 0.f },
		{ TEXT("lift up"),       4.0f, 0.f, 0.f, 1.f },
		{ TEXT("lift down"),     5.0f, 0.f, 0.f, -1.f },
	};
	constexpr int32 NumPhases = UE_ARRAY_COUNT(Phases);

	const auto Report = [this](const TCHAR* Name)
	{
		const FVector Pos = Forklift->GetActorLocation();
		const FVector Delta = Pos - PhaseStartPos;
		const float DYaw = FMath::FindDeltaAngleDegrees(PhaseStartYaw, Forklift->GetActorRotation().Yaw);
		UE_LOG(LogForkliftTwin, Log,
			TEXT("[SelfTest] %-13s dPos=(%7.1f %7.1f %7.1f) cm  dYaw=%6.1f deg (UE: +=clockwise)  fwdSpeed=%6.1f cm/s  lift=%6.1f cm  steer=%5.1f deg  pitch=%5.1f roll=%5.1f"),
			Name, Delta.X, Delta.Y, Delta.Z, DYaw, Forklift->GetForwardSpeedCmPerSec(), Forklift->GetLiftHeightCm(),
			Forklift->GetSteerTargetDeg(), Forklift->GetActorRotation().Pitch, Forklift->GetActorRotation().Roll);
		UE_LOG(LogForkliftTwin, Log, TEXT("[SelfTest]     %s"), *Forklift->GetDebugSummary());
	};

	if (SelfTestPhase < 0)
	{
		SelfTestPhase = 0;
		SelfTestTime = 0.f;
		PhaseStartPos = Forklift->GetActorLocation();
		PhaseStartYaw = Forklift->GetActorRotation().Yaw;
	}

	SelfTestTime += DeltaSeconds;
	const FPhase& Phase = Phases[SelfTestPhase];
	Forklift->SetDriveInput(Phase.Throttle, Phase.Steer, Phase.Lift);

	if (SelfTestTime >= Phase.Duration)
	{
		Report(Phase.Name);
		++SelfTestPhase;
		SelfTestTime = 0.f;
		PhaseStartPos = Forklift->GetActorLocation();
		PhaseStartYaw = Forklift->GetActorRotation().Yaw;
		if (SelfTestPhase >= NumPhases)
		{
			UE_LOG(LogForkliftTwin, Log, TEXT("[SelfTest] done"));
			bSelfTest = false;
			FPlatformMisc::RequestExit(false);
		}
	}
}
