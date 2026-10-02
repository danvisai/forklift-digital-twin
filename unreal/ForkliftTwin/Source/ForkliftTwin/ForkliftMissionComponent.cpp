#include "ForkliftMissionComponent.h"

#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "EngineUtils.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "ForkliftAutoPickupComponent.h"
#include "ForkliftNavComponent.h"
#include "ForkliftPalletFinderComponent.h"
#include "ForkliftVehicle.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY_STATIC(LogForkliftMission, Log, All);

namespace
{
	const TCHAR* StateName(EMissionState State)
	{
		switch (State)
		{
		case EMissionState::Idle: return TEXT("idle");
		case EMissionState::SelectingPallet: return TEXT("selecting pallet");
		case EMissionState::Searching: return TEXT("searching for a pallet");
		case EMissionState::NavigatingToPickup: return TEXT("navigating to pickup");
		case EMissionState::PickingUp: return TEXT("picking up");
		case EMissionState::NavigatingToDrop: return TEXT("navigating to drop");
		case EMissionState::Placing: return TEXT("placing");
		case EMissionState::Lowering: return TEXT("lowering");
		case EMissionState::Withdrawing: return TEXT("withdrawing");
		case EMissionState::Verifying: return TEXT("verifying");
		case EMissionState::Complete: return TEXT("complete");
		case EMissionState::Failed: return TEXT("failed");
		default: return TEXT("unknown");
		}
	}

	// 0 for a failed delivery. Otherwise 100 minus: 0.1 per second taken, 0.5 per cm the pallet ended off the
	// slot centre, 2 per degree off the slot yaw, 10 per extra pickup attempt. Deliberately simple and monotone so
	// runs are comparable; the raw metrics are stored alongside it for any other weighting.
	float ComputeScore(const FMissionDeliveryRecord& R)
	{
		if (!R.bSuccess)
		{
			return 0.f;
		}
		return FMath::Clamp(100.f - 0.1f * R.TotalSec - 0.5f * R.PosErrCm - 2.f * R.YawErrDeg - 10.f * FMath::Max(0, R.PickupAttempts - 1), 0.f, 100.f);
	}
}

UForkliftMissionComponent::UForkliftMissionComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	// Patrol route through the storage aisle (the free floor between the two rack runs), world cm. These are map waypoints,
	// not pallet positions: the truck drives them until its camera has found a pallet.
	SearchWaypoints = { FVector(0.f, 450.f, 0.f), FVector(0.f, -100.f, 0.f), FVector(0.f, -450.f, 0.f), FVector(0.f, 100.f, 0.f) };
}

void UForkliftMissionComponent::BeginPlay()
{
	Super::BeginPlay();
	Vehicle = Cast<AForkliftVehicle>(GetOwner());
	for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
	{
		Warehouse = *It;
		break;
	}
	bVisionSearch = !FParse::Param(FCommandLine::Get(), TEXT("MissionOracle"));
	bLoadedOnly = FParse::Param(FCommandLine::Get(), TEXT("MissionLoaded"));
	bAlternateKinds = FParse::Param(FCommandLine::Get(), TEXT("MissionMixed"));
	bNextLoaded = bLoadedOnly && !bAlternateKinds;
	if (FParse::Param(FCommandLine::Get(), TEXT("ForkliftMission")))
	{
		StartMission();
	}
}

void UForkliftMissionComponent::SetState(EMissionState NewState, const FString& Detail)
{
	State = NewState;
	StateTime = 0.f;
	StatusDetail = Detail;
	UE_LOG(LogForkliftMission, Log, TEXT("[Mission] %s%s%s"), StateName(State), Detail.IsEmpty() ? TEXT("") : TEXT(": "), *Detail);
}

void UForkliftMissionComponent::HoldStill()
{
	if (AForkliftVehicle* Truck = Vehicle.Get())
	{
		Truck->SetDriveInput(0.f, 0.f, 0.f);
	}
}

void UForkliftMissionComponent::StartMission()
{
	AForkliftVehicle* Truck = Vehicle.Get();
	if (!Truck || !Truck->GetNav() || !Truck->GetAutoPickup() || !Warehouse.IsValid() || IsMissionActive())
	{
		UE_LOG(LogForkliftMission, Error, TEXT("[Mission] cannot start (missing vehicle/nav/pickup/warehouse, or already running)"));
		return;
	}
	bRestoreKeyboardControl = Truck->bKeyboardControl;
	Truck->bKeyboardControl = false;   // held for the whole mission; AutoPickup's own save/restore is then a no-op inside it
	Truck->SetDriveInput(0.f, 0.f, 0.f);
	DeliveredCount = 0;
	Skipped.Reset();
	CurrentPallet.Reset();
	SetState(EMissionState::SelectingPallet);
}

void UForkliftMissionComponent::RecordDelivery(bool bSuccess, const FString& Reason)
{
	if (!bDeliveryOpen)
	{
		return;
	}
	bDeliveryOpen = false;
	const UStaticMeshComponent* Pallet = CurrentPallet.Get();

	FMissionDeliveryRecord R;
	R.Pallet = Pallet ? Pallet->GetName() : FString(TEXT("?"));
	R.Episode = EpisodeIndex;
	R.bSuccess = bSuccess;
	R.Reason = Reason;
	for (float Seconds : PhaseSeconds)
	{
		R.TotalSec += Seconds;
	}
	R.NavPickupSec = PhaseSeconds[static_cast<int32>(EMissionState::NavigatingToPickup)];
	R.PickupSec = PhaseSeconds[static_cast<int32>(EMissionState::PickingUp)];
	R.NavDropSec = PhaseSeconds[static_cast<int32>(EMissionState::NavigatingToDrop)];
	R.PlaceSec = PhaseSeconds[static_cast<int32>(EMissionState::Placing)] + PhaseSeconds[static_cast<int32>(EMissionState::Lowering)] +
		PhaseSeconds[static_cast<int32>(EMissionState::Withdrawing)] + PhaseSeconds[static_cast<int32>(EMissionState::Verifying)];
	R.PickupAttempts = PickupAttempts;
	if (bSuccess && Pallet)
	{
		R.PosErrCm = FVector::Dist2D(Pallet->GetComponentLocation(), CurrentSlot.Location);
		const float YawDelta = FMath::Abs(FMath::FindDeltaAngleDegrees(Pallet->GetComponentRotation().Yaw, CurrentSlot.PalletYawDeg));
		R.YawErrDeg = FMath::Min(YawDelta, 180.f - YawDelta);   // an empty pallet is symmetric end-for-end
	}
	R.Score = ComputeScore(R);
	Records.Add(R);

	UE_LOG(LogForkliftMission, Log, TEXT("[Mission] RESULT episode %d %s: %s score %.1f (total %.0fs, pickup attempts %d, pos err %.1f cm, yaw err %.1f deg)%s%s"),
		R.Episode, *R.Pallet, R.bSuccess ? TEXT("SUCCESS") : TEXT("FAILURE"), R.Score, R.TotalSec, R.PickupAttempts, R.PosErrCm, R.YawErrDeg,
		R.Reason.IsEmpty() ? TEXT("") : TEXT(" - "), *R.Reason);

	const TSharedRef<FJsonObject> Json = MakeShared<FJsonObject>();
	Json->SetStringField(TEXT("pallet"), R.Pallet);
	Json->SetNumberField(TEXT("episode"), R.Episode);
	Json->SetBoolField(TEXT("success"), R.bSuccess);
	Json->SetStringField(TEXT("reason"), R.Reason);
	Json->SetNumberField(TEXT("score"), R.Score);
	Json->SetNumberField(TEXT("total_sec"), R.TotalSec);
	Json->SetNumberField(TEXT("nav_pickup_sec"), R.NavPickupSec);
	Json->SetNumberField(TEXT("pickup_sec"), R.PickupSec);
	Json->SetNumberField(TEXT("nav_drop_sec"), R.NavDropSec);
	Json->SetNumberField(TEXT("place_sec"), R.PlaceSec);
	Json->SetNumberField(TEXT("pickup_attempts"), R.PickupAttempts);
	Json->SetNumberField(TEXT("pos_err_cm"), R.PosErrCm);
	Json->SetNumberField(TEXT("yaw_err_deg"), R.YawErrDeg);
	FString Line;
	const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Line);
	FJsonSerializer::Serialize(Json, Writer);
	const FString Dir = FPaths::ProjectSavedDir() / TEXT("Missions");
	IFileManager::Get().MakeDirectory(*Dir, true);
	FFileHelper::SaveStringToFile(Line + TEXT("\n"), *(Dir / TEXT("deliveries.jsonl")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
}

float UForkliftMissionComponent::GetMeanScore() const
{
	float Sum = 0.f;
	for (const FMissionDeliveryRecord& R : Records)
	{
		Sum += R.Score;
	}
	return Records.Num() > 0 ? Sum / Records.Num() : 0.f;
}

void UForkliftMissionComponent::ResetForNewEpisode()
{
	if (IsMissionActive())
	{
		EndMission(EMissionState::Idle, TEXT("reset"));
	}
	Skipped.Reset();
	CurrentPallet.Reset();
	DeliveredCount = 0;
	bNextLoaded = bLoadedOnly && !bAlternateKinds;
	bDeliveryOpen = false;
	SetState(EMissionState::Idle, TEXT("ready"));
}

void UForkliftMissionComponent::EndMission(EMissionState EndState, const FString& Detail)
{
	if (EndState != EMissionState::Complete)
	{
		RecordDelivery(false, Detail);   // a delivery still open when the mission stops did not succeed
	}
	if (AForkliftVehicle* Truck = Vehicle.Get())
	{
		if (UForkliftNavComponent* Nav = Truck->GetNav())
		{
			Nav->CancelRoute();
			Nav->bHoldLift = false;
		}
		if (UForkliftAutoPickupComponent* Auto = Truck->GetAutoPickup())
		{
			Auto->AbortPickup(TEXT("mission ended"));
		}
		Truck->SetDriveInput(0.f, 0.f, 0.f);
		Truck->bKeyboardControl = bRestoreKeyboardControl;
	}
	SetState(EndState, Detail);
}

void UForkliftMissionComponent::AbortMission(const FString& Reason)
{
	if (IsMissionActive())
	{
		EndMission(EMissionState::Idle, Reason);
	}
}

void UForkliftMissionComponent::ToggleMission()
{
	if (IsMissionActive())
	{
		AbortMission();
	}
	else
	{
		StartMission();
	}
}

bool UForkliftMissionComponent::SelectNextPallet()
{
	const AWarehouse* WH = Warehouse.Get();
	const AForkliftVehicle* Truck = Vehicle.Get();
	if (!WH || !Truck)
	{
		return false;
	}
	UStaticMeshComponent* Best = nullptr;
	float BestDist = BIG_NUMBER;
	for (const FLoosePallet& Pallet : WH->GetLoosePallets())
	{
		UStaticMeshComponent* Comp = Pallet.Comp;
		if (!Comp || Pallet.bDelivered || Pallet.bLoaded != bNextLoaded || Skipped.Contains(Comp))
		{
			continue;
		}
		const float Dist = FVector::Dist2D(Comp->GetComponentLocation(), Truck->GetActorLocation());
		if (Dist < BestDist)
		{
			BestDist = Dist;
			Best = Comp;
		}
	}
	CurrentPallet = Best;
	return Best != nullptr;
}

bool UForkliftMissionComponent::StartNavLeg(const FVector& Dest, float DestYawDeg)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	return Truck && Truck->GetNav() && Truck->GetNav()->PlanRouteTo(Dest, DestYawDeg);
}

void UForkliftMissionComponent::FinishPalletFailed(const FString& Why)
{
	// Only used before a pallet is on the forks: give up on this pallet and move on to the next one.
	UE_LOG(LogForkliftMission, Warning, TEXT("[Mission] skipping pallet: %s"), *Why);
	if (bVisionSearch && bHasTarget)
	{
		if (AForkliftVehicle* T = Vehicle.Get(); T && T->GetFinder())
		{
			T->GetFinder()->ExcludeNear(TargetCentre, 150.f);
		}
	}
	RecordDelivery(false, Why);
	if (CurrentPallet.IsValid())
	{
		Skipped.AddUnique(CurrentPallet);
	}
	CurrentPallet.Reset();
	if (AForkliftVehicle* Truck = Vehicle.Get())
	{
		Truck->GetNav()->CancelRoute();
	}
	SetState(EMissionState::SelectingPallet, Why);
}

void UForkliftMissionComponent::BeginSearch()
{
	AForkliftVehicle* Truck = Vehicle.Get();
	UForkliftPalletFinderComponent* Finder = Truck ? Truck->GetFinder() : nullptr;
	if (!Truck || !Finder)
	{
		EndMission(EMissionState::Failed, TEXT("no pallet finder"));
		return;
	}
	PickupAttempts = 0;
	FMemory::Memzero(PhaseSeconds, sizeof(PhaseSeconds));
	bDeliveryOpen = true;
	CurrentPallet.Reset();
	bHasTarget = false;
	Finder->ClearCandidates();
	Finder->SetSearching(true);
	SearchIndex = 0;
	SearchTime = 0.f;
	Truck->GetNav()->CancelRoute();
	SetState(EMissionState::Searching, bNextLoaded ? TEXT("looking for a loaded pallet") : TEXT("looking for an empty pallet"));
}

void UForkliftMissionComponent::TickSearching(float DeltaTime)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	UForkliftPalletFinderComponent* Finder = Truck->GetFinder();
	UForkliftNavComponent* Nav = Truck->GetNav();
	SearchTime += DeltaTime;

	FVector Centre;
	if (Finder->GetBestCandidate(bNextLoaded, Truck->GetActorLocation(), Centre))
	{
		Nav->CancelRoute();
		HoldStill();
		TargetCentre = Centre;
		bHasTarget = true;
		UE_LOG(LogForkliftMission, Log, TEXT("[Mission] detected a %s pallet at %s - going to it"), bNextLoaded ? TEXT("loaded") : TEXT("empty"), *Centre.ToString());
		BeginNavigateToPickup();
		return;
	}
	if (SearchTime < 3.f)
	{
		HoldStill();   // look around from here first
		return;
	}
	if (Nav->IsBusy())
	{
		return;
	}
	if (SearchTime > 600.f || SearchIndex >= SearchWaypoints.Num() * 2)
	{
		Finder->SetSearching(false);
		EndMission(EMissionState::Complete, FString::Printf(TEXT("no %s pallet found, %d delivered"), bNextLoaded ? TEXT("loaded") : TEXT("empty"), DeliveredCount));
		return;
	}
	const FVector Dest = SearchWaypoints[SearchIndex % SearchWaypoints.Num()];
	++SearchIndex;
	const FVector D = Dest - Truck->GetActorLocation();
	Truck->GetNav()->bHoldLift = true;
	Truck->GetNav()->HoldLiftCm = Truck->GetLiftMinCm();
	Truck->GetNav()->SetCarrying(false);
	Truck->GetNav()->SetPlanIgnore(FVector::ZeroVector, 0.f);
	if (!StartNavLeg(Dest, FMath::RadiansToDegrees(FMath::Atan2(D.Y, D.X))))
	{
		UE_LOG(LogForkliftMission, Warning, TEXT("[Mission] no route to search waypoint %s"), *Dest.ToString());
	}
}

void UForkliftMissionComponent::ResolveCarriedPallet()
{
	// For scoring only: which real pallet did the forks end up carrying? (The control above never used this.)
	AForkliftVehicle* Truck = Vehicle.Get();
	const AWarehouse* WH = Warehouse.Get();
	CurrentPallet.Reset();
	if (!Truck || !WH)
	{
		return;
	}
	const FVector Tip = Truck->GetForkTipWorld();
	float Best = 150.f;
	for (const FLoosePallet& P : WH->GetLoosePallets())
	{
		if (P.Comp && !P.bDelivered)
		{
			const float D = FVector::Dist2D(P.Comp->GetComponentLocation(), Tip - Truck->GetActorForwardVector() * 50.f);
			if (D < Best)
			{
				Best = D;
				CurrentPallet = P.Comp;
			}
		}
	}
}

void UForkliftMissionComponent::BeginNavigateToPickup()
{
	AForkliftVehicle* Truck = Vehicle.Get();
	if (!Truck || !bHasTarget)
	{
		FinishPalletFailed(TEXT("no target pallet"));
		return;
	}
	// Approach along the pallet's X axis (its pockets open on both X ends) from the side facing the room centre:
	// the racks stand at |X| ~ 7 m, so a pallet on the west half is entered heading west and vice versa.
	const FVector P = TargetCentre;
	const float Dir = P.X <= 0.f ? 1.f : -1.f;
	PrePos = FVector(P.X + Dir * PickupPreDistCm, P.Y, 0.f);
	StagePos = FVector(P.X + Dir * (PickupPreDistCm + PickupStageRunCm), P.Y, 0.f);
	ApproachYawDeg = Dir > 0.f ? 180.f : 0.f;

	UForkliftNavComponent* Nav = Truck->GetNav();
	Nav->bHoldLift = true;
	Nav->HoldLiftCm = Truck->GetLiftMinCm();
	Nav->SetCarrying(false);
	if (bVisionSearch)
	{
		Nav->SetPlanIgnore(TargetCentre, 220.f);   // the pallet we are about to insert into is not an obstacle
	}
	NavLeg = 0;
	// A longer straight run gives the truck more room to settle onto the pallet axis, but the staging point has to
	// clear the (inflated) racks: fall back to shorter runs until a route exists.
	bool bRouted = false;
	for (const float RunCm : { PickupStageRunCm, 0.5f * (PickupStageRunCm + 200.f), 200.f })
	{
		StagePos = FVector(P.X + Dir * (PickupPreDistCm + RunCm), P.Y, 0.f);
		if (StartNavLeg(StagePos, ApproachYawDeg))
		{
			bRouted = true;
			break;
		}
	}
	if (!bRouted)
	{
		FinishPalletFailed(TEXT("no route to pickup staging point"));
		return;
	}
	SetState(EMissionState::NavigatingToPickup, FString::Printf(TEXT("pallet at %s"), *P.ToString()));
}

void UForkliftMissionComponent::BeginNavigateToDrop()
{
	AForkliftVehicle* Truck = Vehicle.Get();
	const AWarehouse* WH = Warehouse.Get();
	if (!Truck || !WH || !WH->FindFreeSlot(bNextLoaded, Truck->GetActorLocation(), CurrentSlot))
	{
		EndMission(EMissionState::Failed, TEXT("no free empty-pallet slot"));
		return;
	}
	// Enter the slot from the aisle side along its X axis; the drop zones sit far enough out that the staging run
	// fits between the rack run and the slot (see AWarehouse::BuildDropZones).
	const float Dir = CurrentSlot.Location.X < 0.f ? 1.f : -1.f;
	PrePos = FVector(CurrentSlot.Location.X + Dir * DropPreDistCm, CurrentSlot.Location.Y, 0.f);
	StagePos = FVector(CurrentSlot.Location.X + Dir * (DropPreDistCm + DropStageRunCm), CurrentSlot.Location.Y, 0.f);
	ApproachYawDeg = Dir > 0.f ? 180.f : 0.f;

	UForkliftNavComponent* Nav = Truck->GetNav();
	Nav->SetPlanIgnore(FVector::ZeroVector, 0.f);
	Nav->SetCarrying(true);
	Nav->bHoldLift = true;
	Nav->HoldLiftCm = Truck->GetLiftHeightCm();   // keep the load at carry height
	NavLeg = 0;
	if (!StartNavLeg(StagePos, ApproachYawDeg))
	{
		EndMission(EMissionState::Failed, TEXT("no route to drop staging point"));
		return;
	}
	SetState(EMissionState::NavigatingToDrop, FString::Printf(TEXT("slot %s"), *CurrentSlot.Id.ToString()));
}

void UForkliftMissionComponent::TickNavigating(bool bPickup)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	UForkliftNavComponent* Nav = Truck->GetNav();
	if (bPickup && bVisionSearch)
	{
		// The camera keeps seeing the pallet while the truck closes in, and close-range depth is much more accurate than the first
		// far glimpse: fold the refined position into the plan (the pre-pose follows it laterally; the stage stays put).
		if (UForkliftPalletFinderComponent* Finder = Truck->GetFinder())
		{
			FVector Refined;
			if (Finder->RefineNear(TargetCentre, 200.f, Refined) && FVector::Dist2D(Refined, TargetCentre) > 10.f)
			{
				TargetCentre = FMath::Lerp(TargetCentre, Refined, 0.5f);
				const float Dir = ApproachYawDeg == 0.f ? -1.f : 1.f;
				PrePos = FVector(TargetCentre.X + Dir * PickupPreDistCm, TargetCentre.Y, 0.f);
				if (NavLeg == 1)
				{
					Nav->UpdateFinalDestination(PrePos);
				}
			}
		}
	}
	switch (Nav->GetState())
	{
	case ENavState::Arrived:
		if (NavLeg == 0)
		{
			NavLeg = 1;
			if (!StartNavLeg(PrePos, ApproachYawDeg))
			{
				bPickup ? FinishPalletFailed(TEXT("no route to pre-pose")) : EndMission(EMissionState::Failed, TEXT("no route to drop pre-pose"));
			}
		}
		else
		{
			Nav->bHoldLift = false;
			HoldStill();
			bPickupStarted = false;
			SetState(bPickup ? EMissionState::PickingUp : EMissionState::Placing);
		}
		break;
	case ENavState::Failed:
		bPickup ? FinishPalletFailed(FString::Printf(TEXT("navigation failed: %s"), *Nav->GetStatus()))
				: EndMission(EMissionState::Failed, FString::Printf(TEXT("navigation to drop failed: %s"), *Nav->GetStatus()));
		break;
	default:
		break;
	}
}

void UForkliftMissionComponent::TickPickingUp(float DeltaTime)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	UForkliftAutoPickupComponent* Auto = Truck->GetAutoPickup();
	if (!bPickupStarted)
	{
		bPickupStarted = true;
		++PickupAttempts;
		if (UForkliftPalletFinderComponent* Finder = Truck->GetFinder())
		{
			FVector Refined;
			if (Finder->RefineNear(TargetCentre, 150.f, Refined))
			{
				TargetCentre = Refined;   // the closer the truck gets, the better the camera's estimate
			}
		}
		{
			const FVector Axis = FRotator(0.f, ApproachYawDeg, 0.f).Vector();
			Auto->SetFallbackEstimate(TargetCentre - Axis * 50.f, ApproachYawDeg);
		}
		Auto->StartPickup();
		return;
	}
	switch (Auto->GetState())
	{
	case EAutoPickupState::Complete:
		if (UForkliftPalletFinderComponent* Finder = Truck->GetFinder(); Finder && bVisionSearch)
		{
			Finder->RemoveNear(TargetCentre, 220.f);   // it is on the forks now, not on the floor where it was seen
			Finder->SetSearching(false);
		}
		ResolveCarriedPallet();
		BeginNavigateToDrop();
		break;
	case EAutoPickupState::Aborted:
		if (PickupAttempts < MaxPickupAttempts)
		{
			UE_LOG(LogForkliftMission, Warning, TEXT("[Mission] pickup attempt %d failed (%s), re-approaching"), PickupAttempts, *Auto->GetStatus());
			BeginNavigateToPickup();
		}
		else
		{
			FinishPalletFailed(FString::Printf(TEXT("pickup failed after %d attempts: %s"), PickupAttempts, *Auto->GetStatus()));
		}
		break;
	default:
		if (StateTime > 90.f)
		{
			Auto->AbortPickup(TEXT("mission pickup timeout"));
		}
		break;
	}
}

void UForkliftMissionComponent::TickPlacing(float DeltaTime)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	UStaticMeshComponent* Pallet = CurrentPallet.Get();
	if (!Pallet)
	{
		EndMission(EMissionState::Failed, TEXT("lost track of the carried pallet"));
		return;
	}

	// Pure pursuit of the fork tips onto the slot axis (same law as the pickup insertion phase), creeping in until
	// the carried pallet's centre reaches the slot centre.
	const FVector Axis = FRotator(0.f, ApproachYawDeg, 0.f).Vector();
	const FVector Forward = Truck->GetActorForwardVector();
	const FVector Right = Truck->GetActorRightVector();
	const FVector Tip = Truck->GetForkTipWorld();
	const float TipAlong = FVector::DotProduct(Tip - CurrentSlot.Location, Axis);
	const FVector LookAhead = CurrentSlot.Location + Axis * (TipAlong + PlaceLookAheadCm);
	const FVector ToLook = LookAhead - Tip;
	const float Bearing = FMath::RadiansToDegrees(FMath::Atan2(FVector::DotProduct(ToLook, Right), FMath::Max(FVector::DotProduct(ToLook, Forward), 20.f)));
	const float YawErr = FMath::FindDeltaAngleDegrees(Truck->GetActorRotation().Yaw, ApproachYawDeg);
	const float DesiredSteerDeg = FMath::Clamp(-(Bearing * 0.5f + YawErr * 0.35f), -20.f, 20.f);
	const float Steer = FMath::Clamp((DesiredSteerDeg - Truck->GetSteerTargetDeg()) / 15.f, -0.4f, 0.4f);
	Truck->SetDriveInput(PlaceThrottle, Steer, 0.f);

	const float PalletAlong = FVector::DotProduct(Pallet->GetComponentLocation() - CurrentSlot.Location, Axis);
	if (FMath::FloorToInt(StateTime * 2.f) != FMath::FloorToInt((StateTime - DeltaTime) * 2.f))
	{
		const float Cross = FVector::DotProduct(Pallet->GetComponentLocation() - CurrentSlot.Location, FRotator(0.f, ApproachYawDeg + 90.f, 0.f).Vector());
		UE_LOG(LogForkliftMission, Log, TEXT("[Mission] place trace t=%.1f palletAlong=%.0f cross=%.1f yawErr=%.1f bearing=%.1f"), StateTime, PalletAlong, Cross, YawErr, Bearing);
	}
	if (PalletAlong >= -PlaceStopCm)
	{
		HoldStill();
		SetState(EMissionState::Lowering);
	}
	else if (StateTime > 60.f)
	{
		EndMission(EMissionState::Failed, TEXT("placement timeout"));
	}
}

void UForkliftMissionComponent::TickLowering(float DeltaTime)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	const float LiftError = Truck->GetLiftMinCm() - Truck->GetLiftHeightCm();
	Truck->SetDriveInput(0.f, 0.f, FMath::Clamp(LiftError / 5.f, -1.f, 0.f));
	if (LiftError > -1.f && StateTime > 1.f)
	{
		HoldStill();
		SetState(EMissionState::Withdrawing);
	}
	else if (StateTime > 15.f)
	{
		EndMission(EMissionState::Failed, TEXT("lowering timeout"));
	}
}

void UForkliftMissionComponent::TickWithdrawing(float DeltaTime)
{
	AForkliftVehicle* Truck = Vehicle.Get();
	UStaticMeshComponent* Pallet = CurrentPallet.Get();
	const FVector Axis = FRotator(0.f, ApproachYawDeg, 0.f).Vector();
	const float TipAlong = Pallet ? FVector::DotProduct(Truck->GetForkTipWorld() - Pallet->GetComponentLocation(), Axis) : -BIG_NUMBER;
	const float Steer = FMath::Clamp(-Truck->GetSteerTargetDeg() / 15.f, -0.4f, 0.4f);
	Truck->SetDriveInput(-0.15f, Steer, 0.f);
	if (TipAlong <= -WithdrawClearCm || StateTime > 15.f)
	{
		HoldStill();
		SetState(EMissionState::Verifying);
	}
}

void UForkliftMissionComponent::TickVerifying(float DeltaTime)
{
	HoldStill();
	if (StateTime < 1.f)
	{
		return;   // let the pallet settle
	}
	UStaticMeshComponent* Pallet = CurrentPallet.Get();
	AWarehouse* WH = Warehouse.Get();
	const bool bNear = Pallet && FVector::Dist2D(Pallet->GetComponentLocation(), CurrentSlot.Location) <= PlaceToleranceCm;
	const bool bOnFloor = Pallet && Pallet->GetComponentLocation().Z < 30.f;
	if (Pallet && WH && bNear && bOnFloor && WH->IsSlotOccupied(CurrentSlot))
	{
		RecordDelivery(true, FString());
		WH->MarkPalletDelivered(Pallet);
		++DeliveredCount;
		if (bAlternateKinds)
		{
			bNextLoaded = !bNextLoaded;   // empty -> loaded -> empty ...
		}
		UE_LOG(LogForkliftMission, Log, TEXT("[Mission] delivered %d: pallet at %s in slot %s"), DeliveredCount, *Pallet->GetComponentLocation().ToString(), *CurrentSlot.Id.ToString());
		CurrentPallet.Reset();
		if (MaxDeliveries > 0 && DeliveredCount >= MaxDeliveries)
		{
			EndMission(EMissionState::Complete, FString::Printf(TEXT("%d delivered"), DeliveredCount));
		}
		else
		{
			SetState(EMissionState::SelectingPallet);
		}
	}
	else
	{
		EndMission(EMissionState::Failed, Pallet
			? FString::Printf(TEXT("placement verification failed: pallet at %s, slot centre %s"), *Pallet->GetComponentLocation().ToString(), *CurrentSlot.Location.ToString())
			: FString(TEXT("placement verification failed: pallet lost")));
	}
}

void UForkliftMissionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!Vehicle.IsValid() || !IsMissionActive())
	{
		return;
	}
	StateTime += DeltaTime;
	PhaseSeconds[static_cast<int32>(State)] += DeltaTime;
	switch (State)
	{
	case EMissionState::SelectingPallet:
		if (bVisionSearch)
		{
			BeginSearch();
		}
		else if (SelectNextPallet())
		{
			PickupAttempts = 0;
			FMemory::Memzero(PhaseSeconds, sizeof(PhaseSeconds));
			bDeliveryOpen = true;
			TargetCentre = CurrentPallet->GetComponentLocation();
			bHasTarget = true;
			BeginNavigateToPickup();
		}
		else
		{
			EndMission(EMissionState::Complete, FString::Printf(TEXT("no pallets left, %d delivered, %d skipped"), DeliveredCount, Skipped.Num()));
		}
		break;
	case EMissionState::Searching: TickSearching(DeltaTime); break;
	case EMissionState::NavigatingToPickup: TickNavigating(true); break;
	case EMissionState::PickingUp: TickPickingUp(DeltaTime); break;
	case EMissionState::NavigatingToDrop: TickNavigating(false); break;
	case EMissionState::Placing: TickPlacing(DeltaTime); break;
	case EMissionState::Lowering: TickLowering(DeltaTime); break;
	case EMissionState::Withdrawing: TickWithdrawing(DeltaTime); break;
	case EMissionState::Verifying: TickVerifying(DeltaTime); break;
	default: break;
	}
}

FString UForkliftMissionComponent::GetStatus() const
{
	FString Result = FString::Printf(TEXT("mission: %s"), StateName(State));
	if (!StatusDetail.IsEmpty())
	{
		Result += TEXT(" - ") + StatusDetail;
	}
	Result += FString::Printf(TEXT("  delivered %d"), DeliveredCount);
	return Result;
}
