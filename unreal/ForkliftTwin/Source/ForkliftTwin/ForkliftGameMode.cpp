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
#include "ForkliftDetectionClient.h"
#include "ForkliftAutoPickupComponent.h"
#include "ForkliftNavComponent.h"
#include "ForkliftMissionComponent.h"
#include "ForkliftHUD.h"
#include "ForkliftVehicle.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "LidarSensorComponent.h"
#include "Warehouse.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "PhysicsEngine/BodySetup.h"
#include "Components/BoxComponent.h"
#include "HAL/IConsoleManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/CoreDelegates.h"
#include "Misc/Parse.h"
#include "ShaderCompiler.h"

DEFINE_LOG_CATEGORY_STATIC(LogForkliftTwin, Log, All);

namespace { FString LidarDumpPath(); }
namespace { FString RecordPath(); }

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
		GEngine->AddOnScreenDebugMessage(101, 30.f, FColor::Cyan, TEXT("W/S drive   A/D steer   R/F mast   P auto-pick/abort   mouse look   V sensors   L depth view"));
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
		case 3: ShotPos = FVector(-1350.f, 1650.f, 1350.f); Target = FVector(0.f, 150.f, 80.f); break;  // overview of the (now 36x24 m) hall
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

	bAutoPickupHarness = FParse::Param(FCommandLine::Get(), TEXT("ForkliftAutoPickupTest"));
	if (bAutoPickupHarness && Forklift)
	{
		// Reproducible camera/controller test: start square to the dynamic loose pallet, 3.5 m from its face.
		Forklift->bKeyboardControl = false;
		FApp::SetUseFixedTimeStep(true);
		FApp::SetFixedDeltaTime(1.0 / 60.0);
		for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
		{
			AutoPickupPallet = It->GetLoosePallet();
			break;
		}
		if (AutoPickupPallet.IsValid())
		{
			AutoPickupPalletHome = AutoPickupPallet->GetComponentTransform();
			AutoPickupPalletStartZ = AutoPickupPallet->GetComponentLocation().Z;
		}
		PlaceAutoPickupCase(ReadAutoPickupCaseArgs());
		// Let the teleported body settle (steering centred, lift lowered) before StartPickup() - the
		// same sequence PollAutoPickupRun uses for the PIE batch harness. Starting acquisition
		// immediately after a hard ResetPhysics teleport locks the vision system onto a frame captured
		// before the raycast-wheel body has actually settled onto its commanded pose/yaw.
		bAutoPickupResetting = true;
		AutoPickupResetTime = 0.f;
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

	bIdentTest = FParse::Param(FCommandLine::Get(), TEXT("ForkliftIdentTest"));
	if (bIdentTest && Forklift)
	{
		Forklift->bKeyboardControl = false;
		FApp::SetUseFixedTimeStep(true);
		FApp::SetFixedDeltaTime(1.0 / 60.0);
		IdentSpawnXf = Forklift->GetActorTransform();
		IdentCase = 0;
		IdentSeg = -1;
		bIdentSettling = true;
		IdentSettleTime = 0.f;
		IFileManager::Get().MakeDirectory(*(FPaths::ProjectSavedDir() / TEXT("Ident")), true);
		UE_LOG(LogForkliftTwin, Log, TEXT("[IdentTest] starting from %s"), *IdentSpawnXf.ToString());
	}

	bRecord = FParse::Param(FCommandLine::Get(), TEXT("ForkliftRecord")) || FParse::Value(FCommandLine::Get(), TEXT("ForkliftRecord="), RecordFps);
	if (bRecord && Forklift)
	{
		FParse::Value(FCommandLine::Get(), TEXT("ForkliftRecord="), RecordFps);
		RecordSetup();
	}

	bLidarDump = FParse::Param(FCommandLine::Get(), TEXT("ForkliftLidarDump")) || FParse::Value(FCommandLine::Get(), TEXT("ForkliftLidarDump="), LidarDumpScenes);
	if (bLidarDump && Forklift)
	{
		Forklift->bKeyboardControl = false;
		FApp::SetUseFixedTimeStep(true);
		FApp::SetFixedDeltaTime(1.0 / 60.0);
		FParse::Value(FCommandLine::Get(), TEXT("ForkliftLidarDump="), LidarDumpScenes);
		int32 Seed = 1;
		FParse::Value(FCommandLine::Get(), TEXT("LidarDumpSeed="), Seed);
		LidarDumpRng.Initialize(Seed);
		for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
		{
			LidarDumpStatic = It->GetObstacleBoxes();
			for (const FLoosePallet& P : It->GetLoosePallets())
			{
				if (P.Comp)
				{
					LidarDumpStatic.Add(P.Comp->Bounds.GetBox());
				}
			}
			break;
		}
		IFileManager::Get().MakeDirectory(*(LidarDumpPath()), true);
		// world.json: the static geometry the Python twin (Scripts/ml/nav/forklift_sim.py) loads, so both share one layout.
		{
			const auto Num = [](double V) { return FString::Printf(TEXT("%.1f"), V); };
			FString Json = TEXT("{\n \"obstacles\": [");
			TArray<FBox> Statics;
			for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
			{
				Statics = It->GetObstacleBoxes();
				break;
			}
			for (int32 I = 0; I < Statics.Num(); ++I)
			{
				const FBox& B = Statics[I];
				Json += FString::Printf(TEXT("%s[%s,%s,%s,%s,%s,%s]"), I ? TEXT(",") : TEXT(""), *Num(B.Min.X), *Num(B.Min.Y), *Num(B.Min.Z), *Num(B.Max.X), *Num(B.Max.Y), *Num(B.Max.Z));
			}
			Json += TEXT("],\n \"loose_pallets\": [");
			bool bFirst = true;
			FString SlotsJson;
			for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
			{
				for (const FLoosePallet& P : It->GetLoosePallets())
				{
					if (P.Comp)
					{
						const FVector L = P.Comp->GetComponentLocation();
						Json += FString::Printf(TEXT("%s[%s,%s,%s]"), bFirst ? TEXT("") : TEXT(","), *Num(L.X), *Num(L.Y), *Num(P.Comp->GetComponentRotation().Yaw));
						bFirst = false;
					}
				}
				bool bFirstSlot = true;
				for (const FPalletSlot& S : It->GetSlots())
				{
					SlotsJson += FString::Printf(TEXT("%s{\"id\":\"%s\",\"loaded\":%d,\"x\":%s,\"y\":%s,\"yaw\":%s,\"pre_x\":%s,\"pre_y\":%s,\"pre_yaw\":%s}"),
						bFirstSlot ? TEXT("") : TEXT(","), *S.Id.ToString(), S.bForLoaded ? 1 : 0, *Num(S.Location.X), *Num(S.Location.Y), *Num(S.PalletYawDeg),
						*Num(S.PrePoseLocation.X), *Num(S.PrePoseLocation.Y), *Num(S.PrePoseYawDeg));
					bFirstSlot = false;
				}
				break;
			}
			Json += FString::Printf(TEXT("],\n \"slots\": [%s],\n \"spawn\": [%s,%s,%s]\n}\n"), *SlotsJson, *Num(Forklift->GetActorLocation().X), *Num(Forklift->GetActorLocation().Y), *Num(Forklift->GetActorRotation().Yaw));
			FFileHelper::SaveStringToFile(Json, *(LidarDumpPath() / TEXT("world.json")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
		}
		LidarDumpRows.Add(TEXT("scene,x,y,yaw,clutter,gt0..gt71 (cm, 5 deg bins from the truck heading, toward +Y)"));
		UE_LOG(LogForkliftTwin, Log, TEXT("[LidarDump] %d scenes, %d static boxes"), LidarDumpScenes, LidarDumpStatic.Num());
	}

	bNavTest = FParse::Param(FCommandLine::Get(), TEXT("ForkliftNavTest"));
	if (bNavTest && Forklift)
	{
		Forklift->bKeyboardControl = false;
		FApp::SetUseFixedTimeStep(true);
		FApp::SetFixedDeltaTime(1.0 / 60.0);
		AWarehouse* WH = nullptr;
		for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
		{
			WH = *It;
			break;
		}
		FPalletSlot Slot;
		bool bStarted = false;
		if (!WH)
		{
			UE_LOG(LogForkliftTwin, Error, TEXT("[NavTest] no AWarehouse in the level"));
		}
		else if (!Forklift->GetNav())
		{
			UE_LOG(LogForkliftTwin, Error, TEXT("[NavTest] forklift has no Nav component"));
		}
		else if (!WH->FindFreeSlot(false, Forklift->GetActorLocation(), Slot))
		{
			UE_LOG(LogForkliftTwin, Error, TEXT("[NavTest] no free empty-zone slot"));
		}
		else if (!Forklift->GetNav()->PlanRouteTo(Slot.PrePoseLocation, Slot.PrePoseYawDeg))
		{
			UE_LOG(LogForkliftTwin, Error, TEXT("[NavTest] PlanRouteTo failed for slot %s pre-pose %s (nav status: %s)"),
				*Slot.Id.ToString(), *Slot.PrePoseLocation.ToString(), *Forklift->GetNav()->GetStatus());
		}
		else
		{
			bStarted = true;
			UE_LOG(LogForkliftTwin, Log, TEXT("[NavTest] routing to slot %s pre-pose %s yaw %.1f"),
				*Slot.Id.ToString(), *Slot.PrePoseLocation.ToString(), Slot.PrePoseYawDeg);
		}
		if (!bStarted)
		{
			bNavTest = false;
			FPlatformMisc::RequestExitWithStatus(false, 1);   // fail fast instead of hanging with nothing left to tick
		}
	}

	bMissionTest = FParse::Param(FCommandLine::Get(), TEXT("ForkliftMissionTest"));
	if (bMissionTest && Forklift)
	{
		FApp::SetUseFixedTimeStep(true);
		FApp::SetFixedDeltaTime(1.0 / 60.0);
		int32 Pallets = 0;
		FParse::Value(FCommandLine::Get(), TEXT("MissionPallets="), Pallets);
		int32 Available = 0;
		for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
		{
			Available = It->GetLoosePallets().Num();
			break;
		}
		MissionTestExpected = Pallets > 0 ? FMath::Min(Pallets, Available) : Available;
		FParse::Value(FCommandLine::Get(), TEXT("MissionEpisodes="), MissionEpisodes);
		FParse::Value(FCommandLine::Get(), TEXT("MissionSeed="), MissionSeed);
		bMissionJitter = FParse::Param(FCommandLine::Get(), TEXT("MissionJitter"));
		MissionSpawnXf = Forklift->GetActorTransform();
		UForkliftMissionComponent* Mission = Forklift->GetMission();
		if (!Mission || MissionTestExpected <= 0)
		{
			UE_LOG(LogForkliftTwin, Error, TEXT("[MissionTest] could not start (no mission component or no loose pallets)"));
			bMissionTest = false;
			FPlatformMisc::RequestExitWithStatus(false, 1);
		}
		else
		{
			Mission->MaxDeliveries = MissionTestExpected;
			Mission->StartMission();
			UE_LOG(LogForkliftTwin, Log, TEXT("[MissionTest] started, expecting %d delivery(ies)"), MissionTestExpected);
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
	if (!bFpsCapApplied)
	{
		// Interactive play was rendering uncapped (GPU pinned, machine overheating). Cap it to 30 fps unless a scripted
		// test/data run set a fixed step, where wall-clock speed matters more than heat.
		bFpsCapApplied = true;
		if (!FApp::UseFixedTimeStep())
		{
			if (IConsoleVariable* Cvar = IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS")))
			{
				PrevMaxFps = Cvar->GetFloat();
				Cvar->Set(30.f, ECVF_SetByGameSetting);
			}
		}
	}
	if (bRecord && Forklift)
	{
		RecordTick(DeltaSeconds);
	}
	if (bIdentTest && Forklift)
	{
		IdentTestTick(DeltaSeconds);
	}
	if (bLidarDump && Forklift)
	{
		LidarDumpTick(DeltaSeconds);
	}
	if (bNavTest && Forklift)
	{
		NavTestTick(DeltaSeconds);
	}
	if (bMissionTest && Forklift)
	{
		MissionTestTick(DeltaSeconds);
	}
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
	if (bAutoPickupTest && Forklift)
	{
		AutoPickupTestTick(DeltaSeconds);
	}
	else if (bAutoPickupHarness && Forklift)
	{
		PollAutoPickupRun(DeltaSeconds);
	}
}

namespace
{
	struct FIdentSeg { float Throttle; float Steer; float Seconds; };
	struct FIdentCase { const TCHAR* Name; TArray<FIdentSeg> Segs; };

	// Open-loop step cases. Steer +1 holds the wheel at +MaxSteerDeg; steer 0 re-centres at SteerRateDegPerSec.
	const TArray<FIdentCase>& IdentCases()
	{
		static const TArray<FIdentCase> Cases = {
			{ TEXT("fwd_full"),      { {1.f, 0.f, 3.f}, {0.f, 0.f, 2.f} } },
			{ TEXT("fwd_half"),      { {0.5f, 0.f, 4.f}, {0.f, 0.f, 2.f} } },
			{ TEXT("rev"),           { {-0.6f, 0.f, 3.f}, {0.f, 0.f, 2.f} } },
			{ TEXT("steer_left"),    { {0.6f, 0.f, 0.5f}, {0.6f, 1.f, 4.f}, {0.6f, 0.f, 1.5f}, {0.f, 0.f, 2.f} } },
			{ TEXT("steer_right"),   { {0.6f, 0.f, 0.5f}, {0.6f, -1.f, 4.f}, {0.6f, 0.f, 1.5f}, {0.f, 0.f, 2.f} } },
			{ TEXT("steer_static"),  { {0.f, 1.f, 1.f}, {0.f, 0.f, 1.f}, {0.f, -1.f, 1.f}, {0.f, 0.f, 1.f} } },
			{ TEXT("rev_steer"),     { {-0.6f, 1.f, 4.f}, {-0.6f, 0.f, 1.f}, {0.f, 0.f, 2.f} } },
			{ TEXT("slalom"),        { {0.7f, 1.f, 1.f}, {0.7f, -1.f, 2.f}, {0.7f, 1.f, 2.f}, {0.7f, 0.f, 1.f}, {0.f, 0.f, 2.f} } },
			{ TEXT("brake_from_full"), { {1.f, 0.f, 2.5f}, {-1.f, 0.f, 1.5f}, {0.f, 0.f, 2.f} } },
		};
		return Cases;
	}
}

void AForkliftGameMode::IdentTestTick(float DeltaSeconds)
{
	const TArray<FIdentCase>& Cases = IdentCases();
	if (IdentCase >= Cases.Num())
	{
		UE_LOG(LogForkliftTwin, Log, TEXT("[IdentTest] PASS: wrote %d case file(s) to %s"), Cases.Num(), *(FPaths::ProjectSavedDir() / TEXT("Ident")));
		bIdentTest = false;
		FPlatformMisc::RequestExitWithStatus(false, 0);
		return;
	}
	if (bIdentSettling)
	{
		// Teleport to the spawn pose with everything zeroed and let the suspension settle before each case.
		if (IdentSettleTime == 0.f)
		{
			Forklift->SetActorLocationAndRotation(IdentSpawnXf.GetLocation(), IdentSpawnXf.Rotator(), false, nullptr, ETeleportType::ResetPhysics);
		}
		Forklift->SetDriveInput(0.f, 0.f, 0.f);
		IdentSettleTime += DeltaSeconds;
		if (IdentSettleTime > 2.f)
		{
			bIdentSettling = false;
			IdentSeg = 0;
			IdentSegTime = 0.f;
			IdentTime = 0.f;
			IdentRows.Reset();
			IdentRows.Add(TEXT("t,x,y,yaw,speed,steer_deg,throttle_cmd,steer_cmd"));
		}
		return;
	}
	const FIdentCase& Case = Cases[IdentCase];
	const FIdentSeg& Seg = Case.Segs[IdentSeg];
	Forklift->SetDriveInput(Seg.Throttle, Seg.Steer, 0.f);
	const FVector P = Forklift->GetActorLocation();
	IdentRows.Add(FString::Printf(TEXT("%.4f,%.3f,%.3f,%.4f,%.3f,%.4f,%.3f,%.3f"), IdentTime, P.X, P.Y, Forklift->GetActorRotation().Yaw,
		Forklift->GetForwardSpeedCmPerSec(), Forklift->GetSteerTargetDeg(), Seg.Throttle, Seg.Steer));
	IdentTime += DeltaSeconds;
	IdentSegTime += DeltaSeconds;
	if (IdentSegTime >= Seg.Seconds)
	{
		IdentSegTime = 0.f;
		if (++IdentSeg >= Case.Segs.Num())
		{
			const FString Path = FPaths::ProjectSavedDir() / TEXT("Ident") / FString::Printf(TEXT("case%d_%s.csv"), IdentCase, Case.Name);
			FFileHelper::SaveStringToFile(FString::Join(IdentRows, TEXT("\n")), *Path);
			UE_LOG(LogForkliftTwin, Log, TEXT("[IdentTest] case %d (%s) done: %d rows, end pos (%.1f, %.1f) yaw %.1f"),
				IdentCase, Case.Name, IdentRows.Num() - 1, P.X, P.Y, Forklift->GetActorRotation().Yaw);
			++IdentCase;
			bIdentSettling = true;
			IdentSettleTime = 0.f;
		}
	}
}

namespace
{
	// Output folder of -ForkliftRecord: Saved/Video, or Saved/<-RecordDir=name>.
	FString RecordPath()
	{
		FString Dir = TEXT("Video");
		FParse::Value(FCommandLine::Get(), TEXT("RecordDir="), Dir);
		return FPaths::ProjectSavedDir() / Dir;
	}
}

namespace
{
	// Output folder of -ForkliftLidarDump: Saved/LidarDump, or Saved/<-LidarDumpDir=name>.
	FString LidarDumpPath()
	{
		FString Dir = TEXT("LidarDump");
		FParse::Value(FCommandLine::Get(), TEXT("LidarDumpDir="), Dir);
		return FPaths::ProjectSavedDir() / Dir;
	}
}

namespace
{
	// Distance along a 2D ray (origin O, unit dir D, XY) to the nearest of the boxes; Max if none closer.
	float RayClearance2D(const FVector2D& O, const FVector2D& D, const TArray<FBox>& Boxes, float Max)
	{
		float Best = Max;
		for (const FBox& B : Boxes)
		{
			float T0 = 0.f, T1 = Best;
			bool bHit = true;
			for (int32 Axis = 0; Axis < 2 && bHit; ++Axis)
			{
				const float Lo = Axis == 0 ? B.Min.X : B.Min.Y;
				const float Hi = Axis == 0 ? B.Max.X : B.Max.Y;
				const float Oa = Axis == 0 ? O.X : O.Y;
				const float Da = Axis == 0 ? D.X : D.Y;
				if (FMath::Abs(Da) < 1.e-6f)
				{
					bHit = Oa >= Lo && Oa <= Hi;
				}
				else
				{
					float Ta = (Lo - Oa) / Da, Tb = (Hi - Oa) / Da;
					if (Ta > Tb) { Swap(Ta, Tb); }
					T0 = FMath::Max(T0, Ta);
					T1 = FMath::Min(T1, Tb);
					bHit = T0 <= T1;
				}
			}
			if (bHit)
			{
				Best = FMath::Min(Best, T0);
			}
		}
		return Best;
	}
}

void AForkliftGameMode::LidarDumpTick(float DeltaSeconds)
{
	ULidarSensorComponent* Roof = Forklift->GetLidar();
	ULidarSensorComponent* Low = Forklift->GetLowLidar();
	if (LidarDumpScene >= LidarDumpScenes)
	{
		FFileHelper::SaveStringToFile(FString::Join(LidarDumpRows, TEXT("\n")), *(LidarDumpPath() / TEXT("meta.csv")));
		UE_LOG(LogForkliftTwin, Log, TEXT("[LidarDump] PASS: %d scenes -> %s"), LidarDumpScenes, *(LidarDumpPath()));
		bLidarDump = false;
		FPlatformMisc::RequestExitWithStatus(false, 0);
		return;
	}
	if (LidarDumpFrames == 0)
	{
		// New scene: a random pose in free floor (truck footprint circle clear of every static box), then clutter.
		FVector2D Pos(0.f, 0.f);
		for (int32 Try = 0; Try < 200; ++Try)
		{
			Pos = FVector2D(LidarDumpRng.FRandRange(-1650.f, 1650.f), LidarDumpRng.FRandRange(-1050.f, 1050.f));
			bool bClear = true;
			for (const FBox& B : LidarDumpStatic)
			{
				if (Pos.X > B.Min.X - 260.f && Pos.X < B.Max.X + 260.f && Pos.Y > B.Min.Y - 260.f && Pos.Y < B.Max.Y + 260.f)
				{
					bClear = false;
					break;
				}
			}
			if (bClear) { break; }
		}
		const float StartYaw = LidarDumpRng.FRandRange(-180.f, 180.f);
		Forklift->SetActorLocationAndRotation(FVector(Pos.X, Pos.Y, 15.f), FRotator(0.f, StartYaw, 0.f), false, nullptr, ETeleportType::ResetPhysics);
		Forklift->SetDriveInput(0.f, 0.f, 0.f);

		LidarDumpBoxes = LidarDumpStatic;
		UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
		const int32 NumClutter = LidarDumpRng.RandRange(0, 5);
		for (int32 I = 0; I < NumClutter && Cube; ++I)
		{
			// Floor-level things: flat pallets, totes, cartons, a few tall posts. Placed 1.5..9 m from the truck.
			const float Ang = LidarDumpRng.FRandRange(-PI, PI), Dist = LidarDumpRng.FRandRange(150.f, 900.f);
			const float Kind = LidarDumpRng.FRand();
			const FVector Size = Kind < 0.4f ? FVector(120.f, 100.f, 14.f) : (Kind < 0.75f ? FVector(60.f, 40.f, 28.f) : (Kind < 0.9f ? FVector(50.f, 40.f, 50.f) : FVector(30.f, 30.f, 180.f)));
			const FVector Loc(Pos.X + Dist * FMath::Cos(Ang), Pos.Y + Dist * FMath::Sin(Ang), Size.Z * 0.5f);
			const FRotator Rot(0.f, LidarDumpRng.FRandRange(0.f, 180.f), 0.f);
			if (FMath::Abs(Loc.X) > 1700.f || FMath::Abs(Loc.Y) > 1100.f)
			{
				continue;
			}
			bool bOverlap = false;
			const FBox Fp(Loc - FVector(Size.X, Size.Y, 0.f) * 0.75f, Loc + FVector(Size.X, Size.Y, 0.f) * 0.75f);
			for (const FBox& B : LidarDumpBoxes)
			{
				if (FMath::Min(B.Max.X, Fp.Max.X) > FMath::Max(B.Min.X, Fp.Min.X) && FMath::Min(B.Max.Y, Fp.Max.Y) > FMath::Max(B.Min.Y, Fp.Min.Y))
				{
					bOverlap = true;
					break;
				}
			}
			if (bOverlap) { continue; }
			AStaticMeshActor* A = GetWorld()->SpawnActor<AStaticMeshActor>(Loc, Rot);
			if (!A) { continue; }
			A->SetMobility(EComponentMobility::Movable);
			A->GetStaticMeshComponent()->SetStaticMesh(Cube);
			A->SetActorScale3D(Size / 100.f);
			A->GetStaticMeshComponent()->SetCollisionProfileName(TEXT("BlockAll"));
			LidarDumpClutter.Add(A);
			// Conservative axis-aligned bound of the rotated box for the 2D ground truth (slightly pessimistic when rotated).
			const float R = FMath::Abs(FMath::Cos(FMath::DegreesToRadians(Rot.Yaw))), S = FMath::Abs(FMath::Sin(FMath::DegreesToRadians(Rot.Yaw)));
			const FVector2D Half(0.5f * (Size.X * R + Size.Y * S), 0.5f * (Size.X * S + Size.Y * R));
			LidarDumpBoxes.Add(FBox(FVector(Loc.X - Half.X, Loc.Y - Half.Y, 0.f), FVector(Loc.X + Half.X, Loc.Y + Half.Y, Size.Z)));
		}
	}
	++LidarDumpFrames;
	int32 SettleFrames = 45;
	FParse::Value(FCommandLine::Get(), TEXT("LidarDumpSettle="), SettleFrames);
	if (LidarDumpFrames < SettleFrames)
	{
		return;   // let the teleported body settle and the new colliders register
	}
	Roof->Scan();
	Low->Scan();
	const FVector P = Forklift->GetActorLocation();
	const float Yaw = Forklift->GetActorRotation().Yaw;
	FString Row = FString::Printf(TEXT("%d,%.2f,%.2f,%.3f,%d"), LidarDumpScene, P.X, P.Y, Yaw, LidarDumpClutter.Num());
	for (int32 K = 0; K < 72; ++K)
	{
		const float A = FMath::DegreesToRadians(Yaw + K * 5.f);
		Row += FString::Printf(TEXT(",%.1f"), RayClearance2D(FVector2D(P.X, P.Y), FVector2D(FMath::Cos(A), FMath::Sin(A)), LidarDumpBoxes, 3000.f));
	}
	LidarDumpRows.Add(Row);
	const FString Base = LidarDumpPath() / FString::Printf(TEXT("scene%03d"), LidarDumpScene);
	Roof->SaveRangesNpy(Base + TEXT("_roof.npy"));
	Low->SaveRangesNpy(Base + TEXT("_low.npy"));
	UE_LOG(LogForkliftTwin, Log, TEXT("[LidarDump] scene %d at (%.0f, %.0f) yaw %.0f with %d clutter; roof %d returns, low %d returns"),
		LidarDumpScene, P.X, P.Y, Yaw, LidarDumpClutter.Num(), Roof->GetPointsRef().Num(), Low->GetPointsRef().Num());
	for (const TWeakObjectPtr<AActor>& A : LidarDumpClutter)
	{
		if (A.IsValid()) { A->Destroy(); }
	}
	LidarDumpClutter.Reset();
	LidarDumpFrames = 0;
	++LidarDumpScene;
}

void AForkliftGameMode::RecordSetup()
{
	FApp::SetUseFixedTimeStep(true);
	FApp::SetFixedDeltaTime(1.0 / 60.0);
	constexpr int32 W = 960, H = 540;
	float RecordEV = 3.5f;   // -RecordEV=: manual exposure of the scene-capture cameras
	FParse::Value(FCommandLine::Get(), TEXT("RecordEV="), RecordEV);
	RecordNames = { TEXT("chase"), TEXT("top"), TEXT("side"), TEXT("wide") };
	for (const FString& Name : RecordNames)
	{
		UTextureRenderTarget2D* Target = NewObject<UTextureRenderTarget2D>(this);
		Target->InitCustomFormat(W, H, PF_B8G8R8A8, false);
		Target->UpdateResourceImmediate(true);
		ASceneCapture2D* Cap = GetWorld()->SpawnActor<ASceneCapture2D>(FVector::ZeroVector, FRotator::ZeroRotator);
		USceneCaptureComponent2D* Comp = Cap->GetCaptureComponent2D();
		Comp->TextureTarget = Target;
		Comp->CaptureSource = SCS_FinalColorLDR;
		Comp->bCaptureEveryFrame = false;
		Comp->bCaptureOnMovement = false;
		Comp->FOVAngle = Name == TEXT("top") ? 80.f : (Name == TEXT("wide") ? 85.f : 65.f);
		// Fixed exposure, as in CaptureShot: a fresh capture has no eye-adaptation history.
		Comp->PostProcessSettings.bOverride_AutoExposureMethod = true;
		Comp->PostProcessSettings.AutoExposureMethod = AEM_Manual;
		Comp->PostProcessSettings.bOverride_AutoExposureBias = true;
		Comp->PostProcessSettings.AutoExposureBias = RecordEV;
		// Exposure overrides on a capture are ignored unless it keeps its rendering state (see ForkliftVehicle's camera
		// constructors): switch that on and re-register so the manual exposure actually applies.
		Comp->bAlwaysPersistRenderingState = true;
		Comp->UnregisterComponent();
		Comp->RegisterComponent();
		RecordCams.Add(Cap);
		RecordTargets.Add(Target);
		IFileManager::Get().MakeDirectory(*(RecordPath() / Name), true);
	}
	IFileManager::Get().MakeDirectory(*(RecordPath() / TEXT("tine")), true);
	RecordSidePos = Forklift->GetActorLocation() + Forklift->GetActorRightVector() * 550.f + FVector(0.f, 0.f, 160.f);
	RecordCaptions.Add(TEXT("frame,sim_t,mission,nav,pickup"));
	UE_LOG(LogForkliftTwin, Log, TEXT("[Record] %d cameras at %.0f fps -> %s"), RecordNames.Num() + 1, RecordFps, *(RecordPath()));
}

void AForkliftGameMode::RecordFlushCaptions()
{
	if (RecordSensorLines.Num() > 0)
	{
		FFileHelper::SaveStringToFile(FString::Join(RecordSensorLines, TEXT("\n")) + TEXT("\n"), *(RecordPath() / TEXT("sensors.jsonl")),
			FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
		RecordSensorLines.Reset();
	}
	FFileHelper::SaveStringToFile(FString::Join(RecordCaptions, TEXT("\n")), *(RecordPath() / TEXT("captions.csv")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

void AForkliftGameMode::RecordTick(float DeltaSeconds)
{
	RecordAccum += DeltaSeconds;
	const float Period = 1.f / FMath::Max(RecordFps, 1.f);
	if (RecordAccum < Period)
	{
		return;
	}
	RecordAccum -= Period;

	const FVector Truck = Forklift->GetActorLocation();
	const FVector Fwd = Forklift->GetActorForwardVector();
	const FVector Look = Truck + FVector(0.f, 0.f, 90.f);
	const auto LookAt = [](const FVector& From, const FVector& To) { return (To - From).Rotation(); };

	// Camera poses (cm). Side camera follows the truck smoothly from a fixed side offset instead of being rigid.
	RecordSidePos = FMath::Lerp(RecordSidePos, Truck + FVector(0.f, -600.f, 170.f), 0.06f);
	const FVector Poses[4] = {
		Truck - Fwd * 520.f + FVector(0.f, 0.f, 560.f),   // chase
		Truck + FVector(0.f, 0.f, 820.f),                 // top (under the 9 m ceiling)
		RecordSidePos,                                    // side
		FVector(900.f, 1000.f, 520.f),                    // wide: fixed corner overview of the pickup zone
	};
	const FVector Targets[4] = { Look, Truck, Look, FVector(-300.f, 250.f, 40.f) };
	for (int32 I = 0; I < RecordCams.Num(); ++I)
	{
		FRotator Rot = LookAt(Poses[I], Targets[I]);
		if (RecordNames[I] == TEXT("top"))
		{
			Rot = FRotator(-90.f, Forklift->GetActorRotation().Yaw, 0.f);   // straight down, truck heading up the screen
		}
		RecordCams[I]->SetActorLocationAndRotation(Poses[I], Rot);
		RecordCams[I]->GetCaptureComponent2D()->CaptureScene();
	}
	if (UForkliftCameraComponent* Tine = Forklift->GetTineCamera())
	{
		Tine->Capture();
	}
	FlushRenderingCommands();

	IImageWrapperModule& Wrappers = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
	const auto SaveJpeg = [&](const TArray<FColor>& Pixels, int32 W, int32 H, const FString& Path)
	{
		TSharedPtr<IImageWrapper> Wrapper = Wrappers.CreateImageWrapper(EImageFormat::JPEG);
		if (Wrapper.IsValid() && Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), W, H, ERGBFormat::BGRA, 8))
		{
			FFileHelper::SaveArrayToFile(Wrapper->GetCompressed(88), *Path);
		}
	};
	for (int32 I = 0; I < RecordCams.Num(); ++I)
	{
		TArray<FColor> Pixels;
		RecordTargets[I]->GameThread_GetRenderTargetResource()->ReadPixels(Pixels);
		for (FColor& C : Pixels) { C.A = 255; }
		SaveJpeg(Pixels, RecordTargets[I]->SizeX, RecordTargets[I]->SizeY,
			RecordPath() / RecordNames[I] / FString::Printf(TEXT("f%05d.jpg"), RecordFrame));
	}
	if (UForkliftCameraComponent* Tine = Forklift->GetTineCamera())
	{
		TArray<FColor> Pixels;
		if (Tine->ReadPixelsBgra(Pixels))
		{
			for (FColor& C : Pixels) { C.A = 255; }
			SaveJpeg(Pixels, Tine->Width, Tine->Height, RecordPath() / TEXT("tine") / FString::Printf(TEXT("f%05d.jpg"), RecordFrame));
		}
	}

	// Sensor sidecar for the video overlay: lidar returns in the sensor frame (roof: every 3rd ray, low: all) and the
	// tine detector's current boxes, so make_video.py can draw the radar and detections exactly as the truck saw them.
	{
		FString Line = FString::Printf(TEXT("{\"f\":%d,\"roof\":["), RecordFrame);
		ULidarSensorComponent* Roof = Forklift->GetLidar();
		ULidarSensorComponent* Low = Forklift->GetLowLidar();
		if (Low) { Low->Scan(); }
		bool bFirst = true;
		if (Roof)
		{
			const TArray<FVector>& Pts = Roof->GetPointsRef();
			for (int32 I = 0; I < Pts.Num(); I += 3)
			{
				Line += FString::Printf(TEXT("%s%d,%d,%d"), bFirst ? TEXT("") : TEXT(","), FMath::RoundToInt(Pts[I].X), FMath::RoundToInt(Pts[I].Y), FMath::RoundToInt(Pts[I].Z));
				bFirst = false;
			}
		}
		Line += TEXT("],\"low\":[");
		bFirst = true;
		if (Low)
		{
			for (const FVector& P : Low->GetPointsRef())
			{
				Line += FString::Printf(TEXT("%s%d,%d,%d"), bFirst ? TEXT("") : TEXT(","), FMath::RoundToInt(P.X), FMath::RoundToInt(P.Y), FMath::RoundToInt(P.Z));
				bFirst = false;
			}
		}
		Line += TEXT("],\"det\":[");
		const UForkliftAutoPickupComponent* AutoRec = Forklift->GetAutoPickup();
		const UForkliftDetectionClient* Det = AutoRec ? AutoRec->GetDetector() : nullptr;
		int32 DetW = 0, DetH = 0;
		if (Det)
		{
			DetW = Det->GetImageWidth();
			DetH = Det->GetImageHeight();
			bFirst = true;
			for (const FForkliftDetectionBox& B : Det->GetBoxes())
			{
				Line += FString::Printf(TEXT("%s[\"%s\",%.2f,%.0f,%.0f,%.0f,%.0f]"), bFirst ? TEXT("") : TEXT(","), *B.ClassName, B.Confidence, B.X1, B.Y1, B.X2, B.Y2);
				bFirst = false;
			}
		}
		Line += FString::Printf(TEXT("],\"det_wh\":[%d,%d]}"), DetW, DetH);
		RecordSensorLines.Add(Line);
	}

	const UForkliftMissionComponent* Mission = Forklift->GetMission();
	const UForkliftNavComponent* Nav = Forklift->GetNav();
	const UForkliftAutoPickupComponent* Auto = Forklift->GetAutoPickup();
	RecordCaptions.Add(FString::Printf(TEXT("%d,%.2f,\"%s\",\"%s\",\"%s\""), RecordFrame, RecordFrame / RecordFps,
		Mission ? *Mission->GetStatus().Replace(TEXT("\""), TEXT("'")) : TEXT(""),
		Nav && Nav->IsBusy() ? *Nav->GetStatus().Replace(TEXT("\""), TEXT("'")) : TEXT(""),
		Auto && Auto->IsPickupActive() ? *Auto->GetStatus().Replace(TEXT("\""), TEXT("'")) : TEXT("")));
	++RecordFrame;
	if (RecordFrame % 24 == 0)
	{
		RecordFlushCaptions();
	}
}

void AForkliftGameMode::AutoPickupTestTick(float DeltaSeconds)
{
	AutoPickupTestTime += DeltaSeconds;
	if (AutoPickupTestTime < 1.f && AutoPickupPallet.IsValid())
	{
		// Refresh while the pallet settles onto the floor; this is the actual lift baseline.
		AutoPickupPalletStartZ = AutoPickupPallet->GetComponentLocation().Z;
	}
	UForkliftAutoPickupComponent* Auto = Forklift->GetAutoPickup();
	if (!Auto)
	{
		UE_LOG(LogForkliftTwin, Error, TEXT("[AutoPickupTest] controller missing"));
		FPlatformMisc::RequestExitWithStatus(false, 2);
		return;
	}
	const EAutoPickupState State = Auto->GetState();
	if (State == EAutoPickupState::Complete || State == EAutoPickupState::Aborted)
	{
		const float PalletRiseCm = AutoPickupPallet.IsValid() ? AutoPickupPallet->GetComponentLocation().Z - AutoPickupPalletStartZ : -1.f;
		const bool bLoadRaised = PalletRiseCm >= 20.f;
		const bool bPassed = State == EAutoPickupState::Complete && bLoadRaised;
		if (AutoPickupPallet.IsValid())
		{
			const FVector PalletLocal = Forklift->GetActorTransform().InverseTransformPosition(AutoPickupPallet->GetComponentLocation());
			UE_LOG(LogForkliftTwin, Log, TEXT("[AutoPickupTest] final truck=%s yaw=%.1f lift=%.1f tip=%s palletLocal=%s palletRot=%s"),
				*Forklift->GetActorLocation().ToString(), Forklift->GetActorRotation().Yaw, Forklift->GetLiftHeightCm(),
				*Forklift->GetActorTransform().InverseTransformPosition(Forklift->GetForkTipWorld()).ToString(), *PalletLocal.ToString(),
				*AutoPickupPallet->GetComponentRotation().ToString());
		}
		if (UForkliftCameraComponent* TineCam = Forklift->GetTineCamera())
		{
			TineCam->Capture();
			FlushRenderingCommands();
			TineCam->SavePng(FPaths::ProjectSavedDir() / TEXT("SensorTest/auto_pick_final.png"));
		}
		if (bPassed)
		{
			UE_LOG(LogForkliftTwin, Log, TEXT("[AutoPickupTest] %s | pallet rise %.1f cm (PASS)"), *Auto->GetStatus(), PalletRiseCm);
		}
		else
		{
			UE_LOG(LogForkliftTwin, Error, TEXT("[AutoPickupTest] %s | pallet rise %.1f cm (FAIL)"), *Auto->GetStatus(), PalletRiseCm);
		}
		FinishAutoPickupTest(bPassed ? 0 : (State == EAutoPickupState::Aborted ? 3 : 5), bPassed, PalletRiseCm, Auto->GetStatus());
	}
	else if (AutoPickupTestTime > 55.f)
	{
		Auto->AbortPickup(TEXT("test harness timeout"));
		UE_LOG(LogForkliftTwin, Error, TEXT("[AutoPickupTest] %s"), *Auto->GetStatus());
		FinishAutoPickupTest(4, false, -1.f, Auto->GetStatus());
	}
}

FString AForkliftGameMode::ReadAutoPickupCaseArgs() const
{
	// Case parameters (PickupOffsetY=<cm> PickupYawDeg=<deg> PickupBackCm=<cm run-up>) come from the command line and,
	// so that one already-open editor can run many cases, from Saved/AutoPickupCase.txt (same syntax).
	FString Args = FCommandLine::Get();
	FString FromFile;
	if (FFileHelper::LoadFileToString(FromFile, *(FPaths::ProjectSavedDir() / TEXT("AutoPickupCase.txt"))))
	{
		Args += TEXT(" ") + FromFile;
	}
	return Args;
}

void AForkliftGameMode::PlaceAutoPickupCase(const FString& Args)
{
	float PickupOffsetY = 0.f, PickupYawDeg = 0.f, PickupBackCm = 0.f;
	FParse::Value(*Args, TEXT("PickupOffsetY="), PickupOffsetY);
	FParse::Value(*Args, TEXT("PickupYawDeg="), PickupYawDeg);
	FParse::Value(*Args, TEXT("PickupBackCm="), PickupBackCm);
	Forklift->SetActorLocationAndRotation(FVector(PickupBackCm, -250.f + PickupOffsetY, 15.f), FRotator(0.f, 180.f + PickupYawDeg, 0.f), false, nullptr, ETeleportType::ResetPhysics);
	if (UPrimitiveComponent* Body = Cast<UPrimitiveComponent>(Forklift->GetRootComponent()))
	{
		Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
		Body->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
	}
}

void AForkliftGameMode::PollAutoPickupRun(float DeltaSeconds)
{
	// Editor batch mode (Content/Python/init_unreal.py): a new case is requested by writing Saved/AutoPickupRun.txt.
	// The scene is reset inside the running Play session (ending Play between cases crashes UnrealEd here).
	const FString RunFile = FPaths::ProjectSavedDir() / TEXT("AutoPickupRun.txt");
	UForkliftAutoPickupComponent* Auto = Forklift->GetAutoPickup();
	if (!Auto)
	{
		return;
	}
	if (!bAutoPickupResetting)
	{
		if (!IFileManager::Get().FileExists(*RunFile))
		{
			return;
		}
		IFileManager::Get().Delete(*RunFile);
		if (AutoPickupPallet.IsValid())
		{
			AutoPickupPallet->SetWorldTransform(AutoPickupPalletHome, false, nullptr, ETeleportType::ResetPhysics);
			AutoPickupPallet->SetPhysicsLinearVelocity(FVector::ZeroVector);
			AutoPickupPallet->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
		}
		PlaceAutoPickupCase(ReadAutoPickupCaseArgs());
		Forklift->bKeyboardControl = false;
		bAutoPickupResetting = true;
		AutoPickupResetTime = 0.f;
		return;
	}

	// Lower the lift and centre the steering, then let everything settle before the controller starts.
	AutoPickupResetTime += DeltaSeconds;
	const float LiftError = Forklift->GetLiftMinCm() - Forklift->GetLiftHeightCm();
	Forklift->SetDriveInput(0.f, FMath::Clamp(-Forklift->GetSteerTargetDeg() / 15.f, -0.5f, 0.5f), FMath::Clamp(LiftError / 5.f, -1.f, 0.f));
	if (AutoPickupResetTime > 1.5f && LiftError > -1.f && FMath::Abs(Forklift->GetSteerTargetDeg()) < 1.f)
	{
		Forklift->SetDriveInput(0.f, 0.f, 0.f);
		if (AutoPickupPallet.IsValid())
		{
			AutoPickupPalletStartZ = AutoPickupPallet->GetComponentLocation().Z;
		}
		bAutoPickupResetting = false;
		AutoPickupTestTime = 0.f;
		bAutoPickupTest = true;
		Auto->StartPickup();
	}
}

void AForkliftGameMode::FinishAutoPickupTest(int32 ExitCode, bool bPassed, float RiseCm, const FString& Status)
{
	if ((GetWorld() && GetWorld()->WorldType == EWorldType::PIE))
	{
		// Keep the editor open: record the result for the Python batch driver (Content/Python/init_unreal.py), which
		// ends Play and starts the next case. Stop ticking the test so it reports once.
		bAutoPickupTest = false;
		const FString Line = FString::Printf(TEXT("%s rise=%.1f status=%s"), bPassed ? TEXT("PASS") : TEXT("FAIL"), RiseCm, *Status);
		FFileHelper::SaveStringToFile(Line, *(FPaths::ProjectSavedDir() / TEXT("AutoPickupResult.txt")));
		return;
	}
	FPlatformMisc::RequestExitWithStatus(false, ExitCode);
}

void AForkliftGameMode::MissionTestTick(float DeltaSeconds)
{
	UForkliftMissionComponent* Mission = Forklift->GetMission();

	if (bMissionResetting)
	{
		// Between episodes: lower the lift, centre the steering and let the reset pallets settle before restarting.
		MissionResetTime += DeltaSeconds;
		const float LiftError = Forklift->GetLiftMinCm() - Forklift->GetLiftHeightCm();
		Forklift->SetDriveInput(0.f, FMath::Clamp(-Forklift->GetSteerTargetDeg() / 15.f, -0.5f, 0.5f), FMath::Clamp(LiftError / 5.f, -1.f, 0.f));
		if (MissionResetTime > 2.5f)
		{
			bMissionResetting = false;
			MissionTestTime = 0.f;
			MissionTestLastLoggedState = -1;
			Mission->EpisodeIndex = MissionEpisode;
			Mission->StartMission();
			UE_LOG(LogForkliftTwin, Log, TEXT("[MissionTest] episode %d started"), MissionEpisode);
		}
		return;
	}

	MissionTestTime += DeltaSeconds;
	const int32 CurrentState = static_cast<int32>(Mission->GetState());
	if (CurrentState != MissionTestLastLoggedState)
	{
		MissionTestLastLoggedState = CurrentState;
		UE_LOG(LogForkliftTwin, Log, TEXT("[MissionTest] t=%.1f %s truck=%s"), MissionTestTime, *Mission->GetStatus(), *Forklift->GetActorLocation().ToString());
	}

	const EMissionState S = Mission->GetState();
	const bool bTimedOut = MissionTestTime > 1500.f * FMath::Max(1, MissionTestExpected);
	if (bTimedOut && Mission->IsMissionActive())
	{
		Mission->AbortMission(TEXT("harness timeout"));
		return;
	}
	if (S != EMissionState::Complete && S != EMissionState::Failed && S != EMissionState::Idle)
	{
		return;
	}

	// The episode is over: score it.
	const bool bPassed = S == EMissionState::Complete && Mission->GetDeliveredCount() >= MissionTestExpected;
	if (bPassed)
	{
		++MissionEpisodesPassed;
	}
	UE_LOG(LogForkliftTwin, Log, TEXT("[MissionTest] episode %d %s | %s | delivered %d of %d in %.0f sim-s | running mean score %.1f over %d deliveries"),
		MissionEpisode, bPassed ? TEXT("PASS") : TEXT("FAIL"), *Mission->GetStatus(), Mission->GetDeliveredCount(), MissionTestExpected,
		MissionTestTime, Mission->GetMeanScore(), Mission->GetRecords().Num());
	{
		FString Line = FString::Printf(TEXT("{\"episode\":%d,\"pass\":%s,\"delivered\":%d,\"expected\":%d,\"sim_sec\":%.1f,\"mean_score\":%.2f,\"status\":\"%s\"}\n"),
			MissionEpisode, bPassed ? TEXT("true") : TEXT("false"), Mission->GetDeliveredCount(), MissionTestExpected, MissionTestTime,
			Mission->GetMeanScore(), *Mission->GetStatus().ReplaceCharWithEscapedChar());
		const FString Dir = FPaths::ProjectSavedDir() / TEXT("Missions");
		IFileManager::Get().MakeDirectory(*Dir, true);
		FFileHelper::SaveStringToFile(Line, *(Dir / TEXT("episodes.jsonl")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
	}

	++MissionEpisode;
	if (MissionEpisodes > 0 && MissionEpisode >= MissionEpisodes)
	{
		const bool bAllPassed = MissionEpisodesPassed == MissionEpisodes;
		UE_LOG(LogForkliftTwin, Log, TEXT("[MissionTest] FINISHED: %d of %d episodes passed, mean delivery score %.1f (%s)"),
			MissionEpisodesPassed, MissionEpisodes, Mission->GetMeanScore(), bAllPassed ? TEXT("PASS") : TEXT("FAIL"));
		FPlatformMisc::RequestExitWithStatus(false, bAllPassed ? 0 : 5);
		return;
	}

	// Next episode: put the pallets and the truck back, then let the reset block above start it.
	Mission->ResetForNewEpisode();
	for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
	{
		It->ResetLoosePallets(bMissionJitter, MissionSeed + MissionEpisode);
		break;
	}
	Forklift->SetActorLocationAndRotation(MissionSpawnXf.GetLocation(), MissionSpawnXf.Rotator(), false, nullptr, ETeleportType::ResetPhysics);
	if (UPrimitiveComponent* Body = Cast<UPrimitiveComponent>(Forklift->GetRootComponent()))
	{
		Body->SetPhysicsLinearVelocity(FVector::ZeroVector);
		Body->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
	}
	Forklift->bKeyboardControl = false;
	bMissionResetting = true;
	MissionResetTime = 0.f;
}

void AForkliftGameMode::NavTestTick(float DeltaSeconds)
{
	NavTestTime += DeltaSeconds;
	UForkliftNavComponent* Nav = Forklift->GetNav();
	if (!Nav)
	{
		UE_LOG(LogForkliftTwin, Error, TEXT("[NavTest] nav component missing"));
		FPlatformMisc::RequestExitWithStatus(false, 2);
		return;
	}
	const int32 CurrentState = static_cast<int32>(Nav->GetState());
	if (CurrentState != NavTestLastLoggedState)
	{
		NavTestLastLoggedState = CurrentState;
		UE_LOG(LogForkliftTwin, Log, TEXT("[NavTest] t=%.1f %s truck=%s"), NavTestTime, *Nav->GetStatus(), *Forklift->GetActorLocation().ToString());
	}
	if (Nav->GetState() == ENavState::Arrived)
	{
		UE_LOG(LogForkliftTwin, Log, TEXT("[NavTest] %s at t=%.1fs (PASS)"), *Nav->GetStatus(), NavTestTime);
		FPlatformMisc::RequestExitWithStatus(false, 0);
	}
	else if (Nav->GetState() == ENavState::Failed)
	{
		UE_LOG(LogForkliftTwin, Error, TEXT("[NavTest] %s at t=%.1fs (FAIL)"), *Nav->GetStatus(), NavTestTime);
		FPlatformMisc::RequestExitWithStatus(false, 3);
	}
	else if (NavTestTime > 220.f)   // safety net above UForkliftNavComponent's own RouteTimeoutSec
	{
		UE_LOG(LogForkliftTwin, Error, TEXT("[NavTest] harness timeout: %s (FAIL)"), *Nav->GetStatus());
		FPlatformMisc::RequestExitWithStatus(false, 4);
	}
}

void AForkliftGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if ((GetWorld() && GetWorld()->WorldType == EWorldType::PIE))
	{
		FApp::SetUseFixedTimeStep(false);  // the tests force a fixed step; do not leave the editor stuck on it
	}
	if (bFpsCapApplied && !FApp::UseFixedTimeStep())
	{
		if (IConsoleVariable* Cvar = IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS")))
		{
			Cvar->Set(PrevMaxFps, ECVF_SetByGameSetting);
		}
	}
	if (bRecord)
	{
		RecordFlushCaptions();
	}
	Super::EndPlay(EndPlayReason);
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
	UForkliftCameraComponent* TineCam = Forklift->GetTineCamera();
	ULidarSensorComponent* Lidar = Forklift->GetLidar();
	Cam->Capture();
	TineCam->Capture();
	Lidar->Scan();
	FlushRenderingCommands();

	const FString Base = SensorTestDir / Tag;
	const bool bPng = Cam->SavePng(Base + TEXT("_camera.png"));
	const bool bTinePng = TineCam->SavePng(Base + TEXT("_tine_camera.png"));
	const bool bTineDepth = TineCam->SaveDepthNpy(Base + TEXT("_tine_depth.npy"), true);
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
		TEXT("[SensorTest] %s: lift=%.1f cm  overview %dx%d z=%.1f mean=%.1f png=%d | tine RGB-D z=%.1f png/depth=%d/%d | lidar %d/%d range %.0f..%.0f cm %.2f ms npy=%d/%d -> %s"),
		Tag, Forklift->GetLiftHeightCm(), Cam->Width, Cam->Height, CamPos.Z, Pixels.Num() ? Sum / Pixels.Num() : -1.0, bPng,
		TineCam->GetComponentLocation().Z, bTinePng, bTineDepth,
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
