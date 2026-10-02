// Forklift digital twin - top-level autonomous mission: find a loose pallet, drive to it, pick it up with the
// vision-guided pocket-insertion skill, carry it to a free drop slot, set it down, back out and verify.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Warehouse.h"
#include "ForkliftMissionComponent.generated.h"

class AForkliftVehicle;
class UStaticMeshComponent;

UENUM(BlueprintType)
enum class EMissionState : uint8
{
	Idle,
	SelectingPallet,
	Searching,
	NavigatingToPickup,
	PickingUp,
	NavigatingToDrop,
	Placing,
	Lowering,
	Withdrawing,
	Verifying,
	Complete,
	Failed,
};

/** One scored attempt to deliver one pallet (successful or not). Also written as a JSON line to Saved/Missions/deliveries.jsonl. */
struct FMissionDeliveryRecord
{
	FString Pallet;
	int32 Episode = 0;
	bool bSuccess = false;
	FString Reason;
	float TotalSec = 0.f;
	float NavPickupSec = 0.f;
	float PickupSec = 0.f;
	float NavDropSec = 0.f;
	float PlaceSec = 0.f;   // placing + lowering + withdrawing + verifying
	int32 PickupAttempts = 0;
	float PosErrCm = 0.f;   // set-down pallet centre to slot centre (2D)
	float YawErrDeg = 0.f;  // set-down pallet yaw to slot yaw, modulo the pallet's 180 deg symmetry
	float Score = 0.f;      // 0 for a failure; otherwise 100 minus penalties (see ComputeScore in the .cpp)
};

/**
 * Deterministic mission supervisor. It owns no driving skill of its own except placement: long-range motion is
 * UForkliftNavComponent, insertion/lift is UForkliftAutoPickupComponent (unchanged), and the set-down / withdraw
 * sequence lives here because nothing else provides it. Pallet and slot poses are read from the simulator
 * (AWarehouse) rather than perceived - the truck's own detector is only used inside the pickup skill.
 *
 * Approach geometry is chosen so the truck is lined up before the last skill starts: it first navigates to a
 * staging point on the pallet's / slot's insertion axis, then makes a short straight run in to the pre-pose.
 *
 * bKeyboardControl is taken once at StartMission and restored when the mission ends or aborts.
 */
UCLASS(ClassGroup = (Forklift), meta = (BlueprintSpawnableComponent))
class FORKLIFTTWIN_API UForkliftMissionComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UForkliftMissionComponent();


	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "Forklift|Mission")
	void StartMission();

	UFUNCTION(BlueprintCallable, Category = "Forklift|Mission")
	void AbortMission(const FString& Reason = TEXT("operator abort"));

	UFUNCTION(BlueprintCallable, Category = "Forklift|Mission")
	void ToggleMission();

	UFUNCTION(BlueprintPure, Category = "Forklift|Mission")
	EMissionState GetState() const { return State; }

	UFUNCTION(BlueprintPure, Category = "Forklift|Mission")
	bool IsMissionActive() const { return State != EMissionState::Idle && State != EMissionState::Complete && State != EMissionState::Failed; }

	UFUNCTION(BlueprintPure, Category = "Forklift|Mission")
	int32 GetDeliveredCount() const { return DeliveredCount; }

	UFUNCTION(BlueprintPure, Category = "Forklift|Mission")
	int32 GetSkippedCount() const { return Skipped.Num(); }

	UFUNCTION(BlueprintPure, Category = "Forklift|Mission")
	FString GetStatus() const;

	// ---- Read-only view of the mission's current choices, for the HUD ----
	const UStaticMeshComponent* GetTargetPallet() const { return CurrentPallet.Get(); }
	bool IsPalletSkipped(const UStaticMeshComponent* Pallet) const { return Skipped.ContainsByPredicate([Pallet](const TWeakObjectPtr<UStaticMeshComponent>& P) { return P.Get() == Pallet; }); }
	const FPalletSlot& GetTargetSlot() const { return CurrentSlot; }
	const FVector& GetStagePos() const { return StagePos; }
	const FVector& GetPrePos() const { return PrePos; }
	float GetApproachYawDeg() const { return ApproachYawDeg; }
	const AWarehouse* GetWarehouse() const { return Warehouse.Get(); }
	bool IsVisionSearch() const { return bVisionSearch; }
	const FVector& GetTargetCentre() const { return TargetCentre; }
	bool HasTarget() const { return bHasTarget; }

	/** Back to Idle with per-episode state cleared (delivered count, skipped pallets). Scored records are kept. */
	UFUNCTION(BlueprintCallable, Category = "Forklift|Mission")
	void ResetForNewEpisode();

	/** Tag written into each delivery record (set by the episode loop). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Mission")
	int32 EpisodeIndex = 0;

	const TArray<FMissionDeliveryRecord>& GetRecords() const { return Records; }
	float GetMeanScore() const;
	float GetLastScore() const { return Records.Num() > 0 ? Records.Last().Score : 0.f; }

	/** Stop after this many successful deliveries (0 = keep going until no undelivered pallet is left). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Mission", meta = (ClampMin = "0"))
	int32 MaxDeliveries = 0;

	/** Pickup: truck origin this far from the pallet centre when the vision skill takes over. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float PickupPreDistCm = 350.f;

	/** Pickup: extra straight run before the pre-pose, so the truck arrives already lined up on the pallet axis. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float PickupStageRunCm = 350.f;

	/** Drop: truck origin this far from the slot centre when the placement skill starts. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float DropPreDistCm = 250.f;

	/** Drop: extra straight run before the pre-pose (see the drop-zone X in AWarehouse::BuildDropZones for why it fits). */
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float DropStageRunCm = 200.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float PlaceThrottle = 0.15f;

	/** Lookahead along the slot axis for the placement pure pursuit. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float PlaceLookAheadCm = 70.f;

	/** The pallet centre must get within this distance of the slot centre along the axis before it is lowered. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float PlaceStopCm = 8.f;

	/** Delivered when the set-down pallet centre ends within this (2D) distance of the slot centre. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float PlaceToleranceCm = 35.f;

	/** Withdraw until the fork tips are this far back from the pallet centre along the axis (pallet is 100 cm long). */
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	float WithdrawClearCm = 75.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	int32 MaxPickupAttempts = 2;

	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	int32 MaxPlaceAttempts = 2;

private:
	void SetState(EMissionState NewState, const FString& Detail = FString());
	void EndMission(EMissionState EndState, const FString& Detail);
	void HoldStill();

	bool SelectNextPallet();
	void BeginSearch();
	void TickSearching(float DeltaTime);
	void ResolveCarriedPallet();
	void BeginNavigateToPickup();
	void BeginNavigateToDrop();
	bool StartNavLeg(const FVector& Dest, float DestYawDeg);

	void TickNavigating(bool bPickup);
	void TickPickingUp(float DeltaTime);
	void TickPlacing(float DeltaTime);
	void TickLowering(float DeltaTime);
	void TickWithdrawing(float DeltaTime);
	void TickVerifying(float DeltaTime);
	void FinishPalletFailed(const FString& Why);
	void RecordDelivery(bool bSuccess, const FString& Reason);

	TWeakObjectPtr<AForkliftVehicle> Vehicle;
	TWeakObjectPtr<AWarehouse> Warehouse;

	EMissionState State = EMissionState::Idle;
	FString StatusDetail;
	float StateTime = 0.f;
	bool bRestoreKeyboardControl = true;
	bool bLoadedOnly = false;   // -MissionLoaded: move the loaded pallets to the loaded (east) zone instead of the empty ones to the west zone
	bool bAlternateKinds = false;   // -MissionMixed: empty pallet -> west zone, then loaded pallet -> east zone, alternating
	bool bNextLoaded = false;       // the kind of pallet the mission is currently after (and which zone it goes to)

	// Vision search: by default the mission is NOT told where the pallets are. It patrols a route through the storage floor (map
	// knowledge: the aisle), the fork-camera pallet finder reports pallets it has detected, and the mission drives to the nearest one
	// of the wanted kind. -MissionOracle restores the old behaviour (positions read from the simulator) for A/B tests.
	bool bVisionSearch = true;
	bool bHasTarget = false;
	FVector TargetCentre = FVector::ZeroVector;
	int32 SearchIndex = 0;
	float SearchTime = 0.f;
	UPROPERTY(EditAnywhere, Category = "Forklift|Mission")
	TArray<FVector> SearchWaypoints;

	TWeakObjectPtr<UStaticMeshComponent> CurrentPallet;
	TArray<TWeakObjectPtr<UStaticMeshComponent>> Skipped;
	FPalletSlot CurrentSlot;

	// Two-leg approach: leg 0 drives to the staging point, leg 1 the short straight run to the pre-pose.
	int32 NavLeg = 0;
	FVector StagePos = FVector::ZeroVector;
	FVector PrePos = FVector::ZeroVector;
	float ApproachYawDeg = 0.f;

	int32 PickupAttempts = 0;
	int32 PlaceAttempts = 0;
	int32 DeliveredCount = 0;
	bool bPickupStarted = false;

	// Scoring: seconds spent per state since the current pallet was selected, and the finished records.
	float PhaseSeconds[16] = {};
	bool bDeliveryOpen = false;
	TArray<FMissionDeliveryRecord> Records;
};
