// Forklift digital twin - ray-cast lidar (UE has no built-in lidar sensor).
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "LidarSensorComponent.generated.h"

/**
 * Multi-channel spinning-lidar model: every scan traces Channels x HorizontalSteps rays from this
 * component, spread over [VerticalMinDeg, VerticalMaxDeg] and HorizontalFovDeg, and keeps the result.
 *
 * Sensor frame is UE's: +X forward, +Y right, +Z up, in cm. Azimuth is measured from +X and is
 * positive toward +Y (clockwise seen from above); elevation is positive up.
 *
 * Ranges are laid out channel-major: Ranges[Channel * HorizontalSteps + Step], channel 0 = lowest ray.
 * -1 means no return within [MinRangeCm, MaxRangeCm]. Points holds only the returns, in the sensor frame.
 *
 * The scan is a snapshot at ScanRateHz (no motion distortion), fine at forklift speeds. The rays ignore the
 * owning actor, so the truck's own mast and roof never occlude the sensor - optimistic against a real unit.
 */
UCLASS(ClassGroup = (Sensors), meta = (BlueprintSpawnableComponent))
class FORKLIFTTWIN_API ULidarSensorComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	ULidarSensorComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Take a scan now, regardless of ScanRateHz. */
	UFUNCTION(BlueprintCallable, Category = "Lidar")
	void Scan();

	/** Latest scan, Channels x HorizontalSteps ranges in cm (-1 = no return). */
	UFUNCTION(BlueprintPure, Category = "Lidar")
	TArray<float> GetRangesCm() const { return Ranges; }

	/** Latest scan's returns in the sensor frame, cm. */
	UFUNCTION(BlueprintPure, Category = "Lidar")
	TArray<FVector> GetPointsLocal() const { return Points; }

	/** Number of scans taken so far (0 until the first one). */
	UFUNCTION(BlueprintPure, Category = "Lidar")
	int32 GetScanIndex() const { return ScanIndex; }

	/** World time of the latest scan, seconds. */
	UFUNCTION(BlueprintPure, Category = "Lidar")
	double GetScanTimeSeconds() const { return ScanTimeSeconds; }

	/** Wall-clock cost of the latest scan on the game thread, milliseconds. */
	UFUNCTION(BlueprintPure, Category = "Lidar")
	float GetScanCostMs() const { return ScanCostMs; }

	const TArray<FVector>& GetPointsRef() const { return Points; }

	/** Write the latest ranges as a float32 .npy of shape (Channels, HorizontalSteps). */
	bool SaveRangesNpy(const FString& Path) const;

	/** Write the latest returns as a float32 .npy of shape (N, 3), sensor frame, cm. */
	bool SavePointsNpy(const FString& Path) const;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar", meta = (ClampMin = "1"))
	int32 Channels = 16;

	/** Rays per channel over the horizontal FOV. 360 = 1 degree resolution on a full circle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar", meta = (ClampMin = "1"))
	int32 HorizontalSteps = 360;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	float VerticalMinDeg = -15.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	float VerticalMaxDeg = 15.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar", meta = (ClampMin = "1", ClampMax = "360"))
	float HorizontalFovDeg = 360.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	float MinRangeCm = 30.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	float MaxRangeCm = 3000.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar", meta = (ClampMin = "0.1"))
	float ScanRateHz = 10.f;

	/** Gaussian range noise (1 sigma), cm. 0 = perfect. A real unit is around 2-3 cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	float RangeNoiseStdCm = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	/** Skip the owning actor's own colliders. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	bool bIgnoreOwner = true;

	/** Draw the returns as debug points in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar")
	bool bDrawDebug = false;

private:
	TArray<float> Ranges;
	TArray<FVector> Points;
	float TimeSinceScan = 0.f;
	int32 ScanIndex = 0;
	double ScanTimeSeconds = 0.0;
	float ScanCostMs = 0.f;
};
