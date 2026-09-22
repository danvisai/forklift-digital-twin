// Forklift digital twin - RGB (+ optional metric depth) camera sensor: rate-limited scene captures with their own render targets.
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "ForkliftCameraComponent.generated.h"

class UTextureRenderTarget2D;

/**
 * Renders the scene from this component at CaptureRateHz into a Width x Height render target, and hands the
 * pixels to CPU code on request. Unlike the stock capture component it never renders every frame or on
 * movement, so its cost is set by CaptureRateHz and the resolution, not by how fast the truck moves.
 *
 * Exposure is fixed (ExposureEV) rather than eye-adapted, so a frame looks the same however the truck got
 * there - important for training data. Looks along +X of the component, like every UE camera. FOVAngle is
 * the horizontal field of view.
 *
 * With bCaptureDepth a second, hidden scene capture at the same pose and FOV renders metric depth (cm along the
 * camera's forward axis, not radial distance) into a float render target; it is captured in the same call as the
 * RGB image, so the two are always aligned. Pixels with no geometry (sky) come back as a very large value.
 * Translucent surfaces (e.g. shrink wrap) do not write depth, so depth shows what is behind them.
 */
UCLASS(ClassGroup = (Sensors), meta = (BlueprintSpawnableComponent))
class FORKLIFTTWIN_API UForkliftCameraComponent : public USceneCaptureComponent2D
{
	GENERATED_BODY()

public:
	UForkliftCameraComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Render a frame now (RGB, plus depth if enabled), regardless of CaptureRateHz. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void Capture();

	/** Copy the latest frame to CPU memory (BGRA, row-major, Width x Height). Stalls until the GPU is done. */
	bool ReadPixelsBgra(TArray<FColor>& OutPixels) const;

	/** Copy the latest depth frame to CPU memory: Width x Height floats, row-major, cm along the camera's forward axis. */
	bool ReadDepthCm(TArray<float>& OutDepth) const;

	/** Write the latest frame as an opaque PNG. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	bool SavePng(const FString& Path) const;

	/** Write the latest depth as a (Height, Width) .npy, float32 (or float16 if bHalf). Returns false without depth. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	bool SaveDepthNpy(const FString& Path, bool bHalf = false) const;

	/** Change the fixed exposure at runtime (EV100). Takes effect on the next capture. */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	void SetExposureEV(float NewEV);

	UFUNCTION(BlueprintPure, Category = "Camera")
	UTextureRenderTarget2D* GetRenderTarget() const { return TextureTarget; }

	UFUNCTION(BlueprintPure, Category = "Camera")
	UTextureRenderTarget2D* GetDepthRenderTarget() const { return DepthTarget; }

	/** Number of frames rendered so far. */
	UFUNCTION(BlueprintPure, Category = "Camera")
	int32 GetFrameIndex() const { return FrameIndex; }

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "16"))
	int32 Width = 640;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "16"))
	int32 Height = 480;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera", meta = (ClampMin = "0.1"))
	float CaptureRateHz = 10.f;

	/** Capture on the CaptureRateHz timer. Turn off to capture only when Capture() is called (dataset generation). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
	bool bAutoCapture = true;

	/** Manual exposure, EV100. 12 matches the project's screenshot preset for the daylight-lit hall. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
	float ExposureEV = 12.f;

	/** Also render aligned metric depth (doubles the render cost, so off by default). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Depth")
	bool bCaptureDepth = false;

private:
	void ApplyExposure();

	UPROPERTY(Transient)
	TObjectPtr<USceneCaptureComponent2D> DepthCapture;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> DepthTarget;

	float TimeSinceCapture = 0.f;
	int32 FrameIndex = 0;
};
