// Forklift digital twin - finds pallets on the floor with the truck's own sensors.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ForkliftPalletFinderComponent.generated.h"

class AForkliftVehicle;
class AWarehouse;
class UForkliftDetectionClient;

/** One pallet the finder believes is on the floor, tracked across detector frames. World cm, XY of the pallet centre. */
struct FPalletCandidate
{
	FVector Pos = FVector::ZeroVector;
	int32 Hits = 0;
	int32 FullHits = 0;          // detections whose box was entirely inside the image (not cut off by the border)
	float WeightSum = 0.f;       // sum of range weights: closer (more accurate) frames count for more
	int32 LoadedVotes = 0;       // frames the detector called it "loaded_pallet" (the rest said "pallet")
	float BestConfidence = 0.f;
	double LastSeen = 0.0;
	bool bExcluded = false;      // the mission tried it and gave up

	bool IsLoaded() const { return LoadedVotes * 2 > Hits; }
};

/**
 * Perception for "where is the pallet?": the fork camera's frames go to the same local detector the pickup uses (class
 * "pallet" / "loaded_pallet"), the depth image under each box gives a 3D point on the pallet's front face, and that point is
 * tracked over frames. A point is accepted only if it is on the floor (low in z, so a pallet sitting on a rack shelf is
 * rejected), inside the room and not inside a known rack/wall, which is the facility map, not pallet knowledge. Nothing here
 * reads a pallet position from the simulator.
 */
UCLASS(ClassGroup = (Forklift), meta = (BlueprintSpawnableComponent))
class FORKLIFTTWIN_API UForkliftPalletFinderComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UForkliftPalletFinderComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** The finder polls the detector only while active (the mission switches it on for the search and the approach). */
	void SetSearching(bool bOn) { bSearching = bOn; }
	bool IsSearching() const { return bSearching; }
	void ClearCandidates() { Candidates.Reset(); }   // (excluded spots are kept)

	/** Nearest confirmed candidate of the wanted kind to From (not excluded). */
	bool GetBestCandidate(bool bWantLoaded, const FVector& From, FVector& OutCentre, int32* OutIndex = nullptr) const;

	/** Latest averaged position of the candidate nearest Approx within Radius (to refine an estimate as the truck gets closer). */
	bool RefineNear(const FVector& Approx, float Radius, FVector& OutCentre) const;

	void ExcludeNear(const FVector& Pos, float Radius);

	/** Forget candidates near Pos (the pallet the truck just picked up is on its forks now, not on the floor where it was seen). */
	void RemoveNear(const FVector& Pos, float Radius);

	const TArray<FPalletCandidate>& GetCandidates() const { return Candidates; }
	UForkliftDetectionClient* GetDetector() const { return Detector; }

	UPROPERTY(EditAnywhere, Category = "Forklift|Finder")
	FString DetectorUrl = TEXT("http://127.0.0.1:8765/detect");

	UPROPERTY(EditAnywhere, Category = "Forklift|Finder", meta = (ClampMin = "0.5"))
	float DetectionRateHz = 4.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Finder", meta = (ClampMin = "0.1", ClampMax = "1.0"))
	float MinConfidence = 0.4f;

	/** Frames a candidate must be seen in before the mission may act on it. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Finder", meta = (ClampMin = "1"))
	int32 MinHits = 4;

	/** ...and in at least this many frames the whole pallet must have been in view (a cut-off box biases the position). */
	UPROPERTY(EditAnywhere, Category = "Forklift|Finder", meta = (ClampMin = "0"))
	int32 MinFullHits = 3;

	/** Ignore points further than this from the camera (depth and projection both get noisy). */
	UPROPERTY(EditAnywhere, Category = "Forklift|Finder")
	float MaxRangeCm = 900.f;

	/** A pallet's front-face point must be below this height to count as floor-standing. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Finder")
	float MaxFaceHeightCm = 45.f;

	/** Half the pallet depth: the front-face point is moved this far away from the camera to estimate the centre. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Finder")
	float HalfDepthCm = 50.f;

private:
	void Integrate();
	void DebugReport();
	float DebugTimer = 0.f;
	int32 DebugLines = 0;

	UPROPERTY(Transient)
	TObjectPtr<UForkliftDetectionClient> Detector;

	TWeakObjectPtr<AForkliftVehicle> Vehicle;
	TWeakObjectPtr<AWarehouse> Warehouse;
	TArray<FPalletCandidate> Candidates;
	TArray<FVector> ExcludedZones;   // spots the mission gave up on; a pallet re-detected there stays excluded
	bool bSearching = false;
	int32 LastSerial = 0;
};
