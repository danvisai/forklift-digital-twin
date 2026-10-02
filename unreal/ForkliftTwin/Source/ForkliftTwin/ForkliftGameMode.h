// Forklift digital twin - game mode: forklift init + optional headless self-test.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "ForkliftGameMode.generated.h"

class AForkliftVehicle;
class UStaticMeshComponent;
class ASceneCapture2D;
class UTextureRenderTarget2D;

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
 *
 * -ForkliftAutoPickupTest parks at the same pallet and runs the low-camera detector/controller. It requires the
 * local detector server and a real renderer, verifies that the pallet itself rose at least 20 cm, then exits.
 *
 * -ForkliftNavTest drives AForkliftVehicle's UForkliftNavComponent from its spawn point to the nearest free
 * empty-zone drop slot's pre-pose (on the far side of a rack run from spawn, so a route must actually use the
 * north/south spine rather than driving straight there) and logs [NavTest] lines on state changes; PASS when Nav
 * reports Arrived within the timeout.
 *
 * -ForkliftIdentTest drives scripted open-loop throttle/steer step cases from the spawn pose at a fixed 60 Hz step and
 * writes time, pose, speed, steer angle and the commands to Saved/Ident/caseN.csv (vehicle system-ID for the Python
 * twin in Scripts/ml/nav), then quits.
 *
 * -ForkliftLidarDump[=N] drops the truck at N random poses with random floor-level clutter and writes both lidars' raw
 * ranges plus an exact per-bearing clearance ground truth to Saved/LidarDump.
 *
 * -ForkliftMissionTest [-MissionPallets=N] runs the whole autonomous mission (navigate to a loose pallet, vision-
 * guided pickup, carry, set down in a free empty-zone slot, withdraw, verify) for N pallets (default: all six) at a
 * fixed 60 Hz step, logs [MissionTest] lines and PASSes when that many are verified delivered. Needs the detector
 * server (Scripts/ml/serve_detector.py) and a real renderer, like -ForkliftAutoPickupTest.
 * Add -MissionEpisodes=N to repeat it N times (0 = forever), resetting the truck and pallets between episodes
 * (-MissionJitter nudges the pallets, -MissionSeed=S varies that). Every delivery is scored and appended to
 * Saved/Missions/deliveries.jsonl; each episode's summary goes to Saved/Missions/episodes.jsonl.
 */
UCLASS()
class FORKLIFTTWIN_API AForkliftGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AForkliftGameMode();

	virtual void StartPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** Kit: FORKLIFT_POSITION = (0, -6.75, 0) m, converted to UE (Y negated, cm). */
	UPROPERTY(EditAnywhere, Category = "Forklift")
	FTransform FallbackSpawn = FTransform(FRotator::ZeroRotator, FVector(0.f, 675.f, 15.f));

private:
	void SelfTestTick(float DeltaSeconds);
	void PhotoModeTick(float DeltaSeconds);
	void SensorTestTick(float DeltaSeconds);
	void DumpSensors(const TCHAR* Tag);
	void PalletTestTick(float DeltaSeconds);
	void AutoPickupTestTick(float DeltaSeconds);
	FString ReadAutoPickupCaseArgs() const;
	void PlaceAutoPickupCase(const FString& Args);
	void PollAutoPickupRun(float DeltaSeconds);
	void FinishAutoPickupTest(int32 ExitCode, bool bPassed, float RiseCm, const FString& Status);
	void NavTestTick(float DeltaSeconds);
	void IdentTestTick(float DeltaSeconds);
	void RecordSetup();
	void RecordTick(float DeltaSeconds);
	void RecordFlushCaptions();
	void LidarDumpTick(float DeltaSeconds);
	void MissionTestTick(float DeltaSeconds);
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

	bool bAutoPickupTest = false;
	bool bAutoPickupHarness = false;
	bool bAutoPickupResetting = false;
	float AutoPickupResetTime = 0.f;
	FTransform AutoPickupPalletHome;
	float AutoPickupTestTime = 0.f;
	float AutoPickupPalletStartZ = 0.f;
	TWeakObjectPtr<UStaticMeshComponent> AutoPickupPallet;

	// -ForkliftIdentTest: scripted open-loop throttle/steer cases from the spawn pose, logged to Saved/Ident/caseN.csv
	bool bIdentTest = false;
	int32 IdentCase = 0;
	int32 IdentSeg = 0;
	float IdentSegTime = 0.f;
	float IdentSettleTime = 0.f;
	float IdentTime = 0.f;
	bool bIdentSettling = false;
	FTransform IdentSpawnXf;
	TArray<FString> IdentRows;

	// -ForkliftLidarDump[=N scenes] [-LidarDumpSeed=S]: random truck poses + floor-level clutter, both lidars' raw ranges and
	// an exact 72-bin clearance ground truth to Saved/LidarDump (perception training/validation data for Scripts/ml/nav).
	// -ForkliftRecord[=fps]: while any scripted run is going, render several fixed/tracking cameras at fps (sim time) to
	// Saved/Video/<camera>/fNNNNN.jpg plus captions.csv (mission/nav status per frame); Scripts/ml/nav/make_video.py encodes them.
	bool bRecord = false;
	float RecordFps = 12.f;
	float RecordAccum = 0.f;
	int32 RecordFrame = 0;
	TArray<FString> RecordNames;
	UPROPERTY(Transient)
	TArray<TObjectPtr<ASceneCapture2D>> RecordCams;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UTextureRenderTarget2D>> RecordTargets;
	FVector RecordSidePos = FVector::ZeroVector;
	TArray<FString> RecordCaptions;
	TArray<FString> RecordSensorLines;   // per-frame lidar points + tine-detector boxes, appended to Saved/Video/sensors.jsonl

	bool bLidarDump = false;
	bool bFpsCapApplied = false;
	float PrevMaxFps = 0.f;
	int32 LidarDumpScenes = 40;
	int32 LidarDumpScene = 0;
	int32 LidarDumpFrames = 0;
	FRandomStream LidarDumpRng;
	TArray<TWeakObjectPtr<AActor>> LidarDumpClutter;
	TArray<FBox> LidarDumpStatic;     // racks + walls + loose pallets, world cm
	TArray<FBox> LidarDumpBoxes;      // static + this scene's clutter
	TArray<FString> LidarDumpRows;

	bool bNavTest = false;
	float NavTestTime = 0.f;
	int32 NavTestLastLoggedState = -1;

	bool bMissionTest = false;
	float MissionTestTime = 0.f;
	int32 MissionTestLastLoggedState = -1;
	int32 MissionTestExpected = 0;
	int32 MissionEpisodes = 1;          // -MissionEpisodes=N (0 = loop until the window is closed)
	int32 MissionEpisode = 0;
	int32 MissionEpisodesPassed = 0;
	int32 MissionSeed = 1;              // -MissionSeed=N: base seed for the per-episode pallet jitter
	bool bMissionJitter = false;        // -MissionJitter: nudge the pallets a little each episode
	bool bMissionResetting = false;
	float MissionResetTime = 0.f;
	FTransform MissionSpawnXf;

	bool bSelfTest = false;
	float SelfTestTime = 0.f;
	int32 SelfTestPhase = -1;
	FVector PhaseStartPos = FVector::ZeroVector;
	float PhaseStartYaw = 0.f;
};
