#include "ForkliftCameraComponent.h"

#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Actor.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "NpyWriter.h"
#include "TextureResource.h"

UForkliftCameraComponent::UForkliftCameraComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	// We drive captures ourselves at CaptureRateHz.
	bCaptureEveryFrame = false;
	bCaptureOnMovement = false;
	CaptureSource = SCS_FinalColorLDR;
	FOVAngle = 90.f;
}

void UForkliftCameraComponent::BeginPlay()
{
	Super::BeginPlay();

	UTextureRenderTarget2D* Target = NewObject<UTextureRenderTarget2D>(this);
	Target->InitCustomFormat(Width, Height, PF_B8G8R8A8, false);
	Target->UpdateResourceImmediate(true);
	TextureTarget = Target;
	ApplyExposure();

	if (bCaptureDepth)
	{
		// RGBA32F so the depth (in R, cm) reads back unclamped; a hidden child at zero offset shares this pose.
		DepthTarget = NewObject<UTextureRenderTarget2D>(this);
		DepthTarget->RenderTargetFormat = RTF_RGBA32f;
		DepthTarget->InitAutoFormat(Width, Height);
		DepthTarget->UpdateResourceImmediate(true);

		DepthCapture = NewObject<USceneCaptureComponent2D>(GetOwner(), TEXT("DepthCapture"));
		DepthCapture->SetupAttachment(this);
		DepthCapture->bCaptureEveryFrame = false;
		DepthCapture->bCaptureOnMovement = false;
		DepthCapture->CaptureSource = SCS_SceneDepth;
		DepthCapture->TextureTarget = DepthTarget;
		DepthCapture->FOVAngle = FOVAngle;
		DepthCapture->RegisterComponent();
		DepthCapture->SetRelativeTransform(FTransform::Identity);
	}

	if (AActor* Owner = GetOwner())
	{
		AddTickPrerequisiteActor(Owner);  // capture the pose the owner just finished computing (moving carriage)
	}
	TimeSinceCapture = 1.f / FMath::Max(CaptureRateHz, 0.1f);  // first tick captures immediately
}

void UForkliftCameraComponent::ApplyExposure()
{
	// Fixed exposure: a fresh capture has no eye-adaptation history, and adapting would change the image
	// depending on where the truck has been.
	PostProcessSettings.bOverride_AutoExposureMethod = true;
	PostProcessSettings.AutoExposureMethod = AEM_Manual;
	PostProcessSettings.bOverride_AutoExposureBias = true;
	PostProcessSettings.AutoExposureBias = ExposureEV;
}

void UForkliftCameraComponent::SetExposureEV(float NewEV)
{
	ExposureEV = NewEV;
	ApplyExposure();
}

void UForkliftCameraComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!bAutoCapture)
	{
		return;
	}

	const float Period = 1.f / FMath::Max(CaptureRateHz, 0.1f);
	TimeSinceCapture += DeltaTime;
	if (TimeSinceCapture >= Period)
	{
		TimeSinceCapture = FMath::Fmod(TimeSinceCapture, Period);
		Capture();
	}
}

void UForkliftCameraComponent::Capture()
{
	if (!TextureTarget)
	{
		return;
	}
	CaptureScene();
	if (DepthCapture)
	{
		DepthCapture->FOVAngle = FOVAngle;  // stay aligned if the FOV was changed at runtime
		DepthCapture->CaptureScene();
	}
	++FrameIndex;
}

bool UForkliftCameraComponent::ReadPixelsBgra(TArray<FColor>& OutPixels) const
{
	if (!TextureTarget)
	{
		return false;
	}
	FTextureRenderTargetResource* Resource = TextureTarget->GameThread_GetRenderTargetResource();
	return Resource && Resource->ReadPixels(OutPixels);
}

bool UForkliftCameraComponent::ReadDepthCm(TArray<float>& OutDepth) const
{
	if (!DepthTarget)
	{
		return false;
	}
	FTextureRenderTargetResource* Resource = DepthTarget->GameThread_GetRenderTargetResource();
	TArray<FLinearColor> Pixels;
	// RCM_MinMax returns the stored floats unscaled.
	if (!Resource || !Resource->ReadLinearColorPixels(Pixels, FReadSurfaceDataFlags(RCM_MinMax, CubeFace_MAX)) || Pixels.Num() != Width * Height)
	{
		return false;
	}
	OutDepth.SetNumUninitialized(Pixels.Num());
	for (int32 i = 0; i < Pixels.Num(); ++i)
	{
		OutDepth[i] = Pixels[i].R;
	}
	return true;
}

bool UForkliftCameraComponent::SavePng(const FString& Path) const
{
	TArray<FColor> Pixels;
	if (!ReadPixelsBgra(Pixels) || Pixels.Num() != Width * Height)
	{
		return false;
	}
	for (FColor& C : Pixels)
	{
		C.A = 255;  // final colour leaves alpha at 0; without this the PNG viewer shows it transparent
	}
	TArray64<uint8> Png;
	FImageUtils::PNGCompressImageArray(Width, Height, TArrayView64<const FColor>(Pixels.GetData(), Pixels.Num()), Png);
	return FFileHelper::SaveArrayToFile(Png, *Path);
}

bool UForkliftCameraComponent::SaveDepthNpy(const FString& Path, bool bHalf) const
{
	TArray<float> Depth;
	if (!ReadDepthCm(Depth))
	{
		return false;
	}
	return bHalf ? ForkliftNpy::WriteF16(Path, Depth.GetData(), Height, Width) : ForkliftNpy::WriteF32(Path, Depth.GetData(), Height, Width);
}
