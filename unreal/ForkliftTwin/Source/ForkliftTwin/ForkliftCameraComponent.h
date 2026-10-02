// Forklift digital twin - RGB (+ optional metric depth) camera sensor: rate-limited scene captures with their own render targets.
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "ForkliftCameraComponent.generated.h"

class UTextureRenderTarget2D;
class UTexture2D;

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

	/** World pose of this camera at the moment the latest frame was rendered (the frame is read back some ticks later while the truck keeps moving). */
	const FTransform& GetLastCaptureTransform() const { return LastCaptureXf; }

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

	/**
	 * Colorize the last captured depth frame with a turbo colormap (near = warm/red, far = cool/blue) into a
	 * persistent on-screen texture and return it. Reads back the GPU depth target, same cost as ReadDepthCm(),
	 * so call it only as often as you actually redraw it (e.g. a throttled HUD timer), not every tick. Returns
	 * null without depth enabled (bCaptureDepth) or before the first capture.
	 */
	UFUNCTION(BlueprintCallable, Category = "Camera|Depth")
	UTexture2D* UpdateDepthColorTexture(float NearCm, float FarCm);

	UFUNCTION(BlueprintPure, Category = "Camera|Depth")
	UTexture2D* GetDepthColorTexture() const { return DepthColorTexture; }

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

	/** Manual exposure compensation (AutoExposureBias, see ApplyExposure()). 9.5 matches
	 *  SyntheticDataGenerator's calibrated BaseEV for the current enclosed, artificially-lit hall
	 *  (measured against its own saved frames: mean luma ~131-159, no clipping). Only takes effect if
	 *  bAlwaysPersistRenderingState (inherited from USceneCaptureComponent) is true - see
	 *  ApplyExposure()'s comment. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
	float ExposureEV = 9.5f;

	/** Also render aligned metric depth (doubles the render cost, so off by default). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Depth")
	bool bCaptureDepth = false;

	/** Hide every component owned by the vehicle from RGB and depth (useful for the camera between the tines). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera")
	bool bHideOwner = false;

private:
	void ApplyExposure();

	UPROPERTY(Transient)
	TObjectPtr<USceneCaptureComponent2D> DepthCapture;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> DepthTarget;

	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> DepthColorTexture;

	float TimeSinceCapture = 0.f;
	int32 FrameIndex = 0;

private:
	FTransform LastCaptureXf;
};
