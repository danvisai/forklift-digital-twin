// Forklift digital twin - game mode: forklift init + optional headless self-test.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "ForkliftGameMode.generated.h"

class AForkliftVehicle;
class UStaticMeshComponent;

/**
 * Handles "forklift init": makes sure a forklift exists and is possessed. If the map already has one
 * placed (the shipped Warehouse map does) it is used; otherwise one is spawned at the Kit prototype's
 * spawn point, centred on the south side of the aisle facing +X.
 *
 * Launching with -ForkliftSelfTest drives the truck through a scripted sequence and logs the result,
 * which is how the joint directions and gains were validated without a display.
 *
 * -ForkliftSensorTest parks the truck facing the pallets, writes one fork-camera frame and one lidar scan
 * (at fork height 0 and again at ~1 m) to Saved/SensorTest, logs what they contain and quits. Needs a real
 * renderer (not -nullrhi) because the camera has to render.
 *
 * -ForkliftPalletTest makes the loose block pallet dynamic and drives the truck through a scripted pick, carry and
 * set-down of it, logging [PalletTest] lines with the pallet's pose at each step and a PASS/FAIL summary. Add
 * -ForkliftPalletShot (real renderer only) to grab a viewport screenshot mid-carry.
 */
UCLASS()
class FORKLIFTTWIN_API AForkliftGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AForkliftGameMode();

	virtual void StartPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Kit: FORKLIFT_POSITION = (0, -6.75, 0) m, converted to UE (Y negated, cm). */
	UPROPERTY(EditAnywhere, Category = "Forklift")
	FTransform FallbackSpawn = FTransform(FRotator::ZeroRotator, FVector(0.f, 675.f, 15.f));

private:
	void SelfTestTick(float DeltaSeconds);
	void PhotoModeTick(float DeltaSeconds);
	void SensorTestTick(float DeltaSeconds);
	void DumpSensors(const TCHAR* Tag);
	void PalletTestTick(float DeltaSeconds);
	void LogPalletState(const TCHAR* Tag);
	void CaptureShot();

	UPROPERTY(Transient)
	TObjectPtr<AForkliftVehicle> Forklift;

	// -ForkliftShot=N: frame the forklift from a preset camera, take a screenshot and quit.
	int32 ShotPreset = -1;
	int32 ShotIdleFrames = 0;
	float ShotEV = 12.f;
	FVector ShotPos = FVector::ZeroVector;
	FRotator ShotRot = FRotator::ZeroRotator;
	bool bShotTaken = false;

	bool bSensorTest = false;
	int32 SensorTestState = 0;
	int32 SensorTestFrames = 0;
	float SensorTestTime = 0.f;
	FString SensorTestDir;

	// -ForkliftPalletTest state machine
	bool bPalletTest = false;
	int32 PalletTestState = 0;
	int32 PalletTestFrames = 0;
	float PalletTestTime = 0.f;
	float PalletTestTotal = 0.f;
	bool bPalletShotTaken = false;
	bool bPalletSummaryLogged = false;
	float PalletLogTimer = 0.f;
	FVector PalletStartPos = FVector::ZeroVector;
	FVector TruckStartPos = FVector::ZeroVector;
	FVector CarryStartPos = FVector::ZeroVector;
	FVector PalletHoldLocal = FVector::ZeroVector;   // pallet centre in the truck frame once lifted
	FVector PalletDownPos = FVector::ZeroVector;
	float MaxInsertionCm = 0.f;
	float LiftedZCm = 0.f;
	float CarrySlipCm = 0.f;
	float SetDownZCm = 0.f;
	float BackOffShiftCm = 0.f;
	TWeakObjectPtr<UStaticMeshComponent> PalletComp;

	bool bSelfTest = false;
	float SelfTestTime = 0.f;
	int32 SelfTestPhase = -1;
	FVector PhaseStartPos = FVector::ZeroVector;
	float PhaseStartYaw = 0.f;
};
