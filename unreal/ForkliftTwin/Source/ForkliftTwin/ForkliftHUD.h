// Forklift digital twin - on-screen view of the sensors (fork camera + top-down lidar).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "ForkliftHUD.generated.h"

class UForkliftDetectionClient;
class AForkliftVehicle;

/**
 * Draws what the forklift's sensors currently see: the overview and low tine camera frames (with detection boxes
 * from a local detector server if one is running - see Scripts/ml/serve_detector.py) and the lidar returns seen
 * from above (below it, truck forward = up). Press V to hide / show. Purely a debugging aid; the sensors (and
 * the detection requests) run the same whether or not it is on screen.
 *
 * Press L to swap both the fork-camera and tine-camera thumbnails from RGB to a colorized (turbo) depth view:
 * near = warm/red, far = cool/blue. The tine view is the operationally meaningful one - it is the same low
 * RGB-D camera UForkliftAutoPickupComponent uses to judge fork-to-pocket depth during insertion, just with a
 * much closer near/far range than the wide overview camera. Only updated (a GPU depth readback) while actually
 * shown, at DepthUpdateRateHz, not every tick.
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

private:
	/** Mission/nav status text and the planned route, drawn whether or not the sensor panels (V) are showing. */
	void DrawMissionOverlay(const AForkliftVehicle& Truck);

	/** Pickup-choice markers in the 3D view: every pallet (target / candidate / skipped / delivered), the staging and
	 *  pre-pose the truck is heading for, the drop slot, and the vision lock during insertion. */
	void DrawTargetMarkers(const AForkliftVehicle& Truck);
	void DrawWorldBox(const FBox& Box, const FLinearColor& Color, float Thickness);
	void DrawWorldLabel(const FVector& World, const FString& Text, const FLinearColor& Color);

public:

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

	/** Fork-camera and tine-camera thumbnails show colorized depth instead of RGB. Toggled with L. */
	UPROPERTY(EditAnywhere, Category = "Forklift|HUD|Depth")
	bool bShowDepth = false;

	/** Depth range the fork (overview) camera's turbo colormap is stretched across, cm. */
	UPROPERTY(EditAnywhere, Category = "Forklift|HUD|Depth")
	float ForkDepthNearCm = 50.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|HUD|Depth")
	float ForkDepthFarCm = 2000.f;

	/** Depth range the tine camera's turbo colormap is stretched across, cm - much closer than the overview camera:
	 * this is the sensor used to judge fork-to-pocket depth while inserting, out to about the pickup approach distance. */
	UPROPERTY(EditAnywhere, Category = "Forklift|HUD|Depth")
	float TineDepthNearCm = 5.f;

	UPROPERTY(EditAnywhere, Category = "Forklift|HUD|Depth")
	float TineDepthFarCm = 500.f;

	/** How often the depth view is refreshed while shown (a GPU readback, so throttled independent of frame rate). */
	UPROPERTY(EditAnywhere, Category = "Forklift|HUD|Depth")
	float DepthUpdateRateHz = 5.f;

private:
	UPROPERTY(Transient)
	TObjectPtr<UForkliftDetectionClient> Detector;

	float TimeSinceDepthUpdate = 0.f;
};
