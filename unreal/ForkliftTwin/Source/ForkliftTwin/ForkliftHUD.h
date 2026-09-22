// Forklift digital twin - on-screen view of the sensors (fork camera + top-down lidar).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "ForkliftHUD.generated.h"

class UForkliftDetectionClient;

/**
 * Draws what the forklift's sensors currently see: the fork camera frame (top-left, with detection boxes from
 * a local detector server if one is running - see Scripts/ml/serve_detector.py) and the lidar returns seen
 * from above (below it, truck forward = up). Press V to hide / show. Purely a debugging aid; the sensors (and
 * the detection requests) run the same whether or not it is on screen.
 */
UCLASS()
class FORKLIFTTWIN_API AForkliftHUD : public AHUD
{
	GENERATED_BODY()

public:
	AForkliftHUD();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void DrawHUD() override;

	UPROPERTY(EditAnywhere, Category = "Forklift|HUD")
	bool bShowSensors = true;

	/** Lidar radar radius on screen, px, and the range that radius represents, cm. */
	UPROPERTY(EditAnywhere, Category = "Forklift|HUD")
	float RadarRadiusPx = 130.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|HUD")
	float RadarRangeCm = 1500.f;

	/** Draw boxes from the local detector server on the fork camera thumbnail. Off = camera/lidar only, as before. */
	UPROPERTY(EditAnywhere, Category = "Forklift|HUD")
	bool bShowDetections = true;

	UPROPERTY(EditAnywhere, Category = "Forklift|HUD")
	FString DetectorUrl = TEXT("http://127.0.0.1:8765/detect");

	UPROPERTY(EditAnywhere, Category = "Forklift|HUD")
	float DetectionRateHz = 5.f;

private:
	UPROPERTY(Transient)
	TObjectPtr<UForkliftDetectionClient> Detector;
};
