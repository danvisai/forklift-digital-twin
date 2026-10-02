// Vision-guided pallet pickup using the low camera mounted between the forklift tines.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ForkliftAutoPickupComponent.generated.h"

class AForkliftVehicle;
class UStaticMeshComponent;
class UForkliftCameraComponent;
class UForkliftDetectionClient;
struct FForkliftDetectionBox;

UENUM(BlueprintType)
enum class EAutoPickupState : uint8
{
	Idle,
	Acquiring,
	Aligning,
	Inserting,
	Settling,
	Lifting,
	Withdrawing,
	Complete,
	Aborted,
};

/**
 * A deliberately slow, safety-bounded pallet pickup controller. The detector finds both pallet pockets in the
 * low camera, aligned depth turns their image centres into 3D targets, and the controller corrects heading,
 * lateral offset and fork height. Once the fork tips cross the pallet face the pockets disappear, so final
 * insertion is measured from truck odometry instead of trying to extrapolate a stale image.
 *
 * Press P to start/abort (handled by AForkliftVehicle), or call StartPickup/AbortPickup from Blueprint.
 */
UCLASS(ClassGroup = (Forklift), meta = (BlueprintSpawnableComponent))
class FORKLIFTTWIN_API UForkliftAutoPickupComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UForkliftAutoPickupComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, Category = "Forklift|Auto Pickup")
	void StartPickup();

	UFUNCTION(BlueprintCallable, Category = "Forklift|Auto Pickup")
	void AbortPickup(const FString& Reason = TEXT("operator abort"));

	UFUNCTION(BlueprintCallable, Category = "Forklift|Auto Pickup")
	void TogglePickup();

	UFUNCTION(BlueprintPure, Category = "Forklift|Auto Pickup")
	EAutoPickupState GetState() const { return State; }

	UFUNCTION(BlueprintPure, Category = "Forklift|Auto Pickup")
	bool IsPickupActive() const;

	UFUNCTION(BlueprintPure, Category = "Forklift|Auto Pickup")
	FString GetStatus() const;

	UForkliftDetectionClient* GetDetector() const { return Detector; }

	/** Camera-derived estimate of the pallet's entry face (world cm, middle of the face, floor height ignored) and its insertion axis
	 *  yaw, supplied by the mission from the fork-camera pallet finder. If the pocket detector cannot lock two pockets in time (e.g.
	 *  on a pallet stacked with cartons), the pickup carries on from this estimate instead of giving up. Never a simulator pose. */
	void SetFallbackEstimate(const FVector& EntryFace, float AxisYawDeg) { FallbackFace = EntryFace; FallbackAxisYaw = AxisYawDeg; bHasFallback = true; }
	void ClearFallbackEstimate() { bHasFallback = false; }
	bool IsSeededFromEstimate() const { return bSeededFromSim; }

	/** Where the vision system currently believes the pallet's pocket centre is (world cm); valid while a pickup runs. */
	bool HasFreshVisionTarget() const { return IsPickupActive() && TargetAge < 1.f; }
	const FVector& GetVisionTargetWorld() const { return TargetWorld; }

	/** Keep detecting for the HUD while manual driving is active. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup")
	bool bPreviewWhenIdle = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup")
	FString DetectorUrl = TEXT("http://127.0.0.1:8765/detect");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "0.5"))
	float DetectionRateHz = 5.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float MinimumPocketConfidence = 0.4f;

	/** Distance driven after the aligned fork tips reach the pallet face. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "20.0"))
	float InsertionTravelCm = 105.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "1.0"))
	float CarryLiftCm = 35.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "0.05", ClampMax = "0.5"))
	float ApproachThrottle = 0.18f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "0.05", ClampMax = "0.3"))
	float InsertionThrottle = 0.10f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup")
	float LateralToleranceCm = 5.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup")
	float VerticalToleranceCm = 2.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup")
	float YawToleranceDeg = 7.5f;

	/** While inserting, back out if the tine tips drift this far off the pallet's centre axis. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "2.0"))
	float InsertionCrossTrackAbortCm = 10.f;

	/** While inserting, back out if the truck heading differs from the pallet axis by more than this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "2.0"))
	float InsertionYawAbortDeg = 9.f;

	/** Look-ahead distance along the pallet axis for the insertion pure-pursuit steering. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "20.0"))
	float InsertionLookAheadCm = 70.f;

	/** Abort the approach when no valid pocket pair has been seen for this long (seconds). Shorter gaps are bridged by
	 *  tracking the world-anchored last target. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "0.5"))
	float PocketLossAbortSec = 4.f;

	/** Within this fork-tip gap, if the truck is already within the alignment tolerances, losing the pockets
	 *  no longer aborts: the pallet is anchored in world space and, this close, the pockets legitimately leave
	 *  the low camera's useful view (they merge into one wide box a few tens of cm out), so keep creeping in
	 *  on the last good target and let the normal insertion phase take over at the insertion gap. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Auto Pickup", meta = (ClampMin = "0.0"))
	float VisionLossCommitGapCm = 45.f;

private:
	bool UpdateVisionTarget();
	bool BoxToCameraPoint(const FForkliftDetectionBox& Box, FVector& OutPoint) const;
	bool EstimateFaceYawDeg(float PocketMidPixelX, float PocketMidPixelY, float& OutYawDeg) const;
	void SetState(EAutoPickupState NewState, const FString& Detail = FString());
	void Finish(bool bSuccess, const FString& Detail);
	void HoldStill();

	UPROPERTY(Transient)
	TObjectPtr<UForkliftDetectionClient> Detector;

	TWeakObjectPtr<AForkliftVehicle> Vehicle;
	EAutoPickupState State = EAutoPickupState::Idle;
	int32 LastResultSerial = 0;
	int32 StableTargetFrames = 0;
	float StateTime = 0.f;
	float TotalTime = 0.f;
	float TargetAge = BIG_NUMBER;
	float CloseMisalignmentTime = 0.f;
	FVector TargetWorld = FVector::ZeroVector;
	FVector FallbackFace = FVector::ZeroVector;
	float FallbackAxisYaw = 0.f;
	bool bHasFallback = false;
	bool bSeededFromSim = false;   // (name kept: 'seeded' = target set without a pocket lock; the estimate is camera-derived)
	/** World yaw (deg) of the pallet's insertion axis, i.e. the direction the tines must travel. */
	float PalletAxisYawWorld = 0.f;
	float CrossTrackCm = 0.f;
	float LastLateralErrorCm = 0.f;
	float LastVerticalErrorCm = 0.f;
	float LastLongitudinalErrorCm = 0.f;
	float LastYawErrorDeg = 0.f;
	FVector InsertionStart = FVector::ZeroVector;
	FVector InsertionForward = FVector::ForwardVector;
	float InsertionLiftHeightCm = 0.f;
	float LiftTargetCm = 0.f;
	bool bRestoreKeyboardControl = true;
	FString StatusDetail;
};
