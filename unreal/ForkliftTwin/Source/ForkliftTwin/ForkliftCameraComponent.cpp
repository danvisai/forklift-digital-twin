#include "ForkliftCameraComponent.h"

#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Actor.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "NpyWriter.h"
#include "RenderingThread.h"
#include "TextureResource.h"

namespace
{
	// Google's Turbo colormap (public-domain polynomial fit, degree 5): canonically t=0 -> dark blue/purple,
	// t=1 -> red. Callers invert t (near=1) so the depth view reads warm-near/cool-far, like a heat map.
	FColor TurboColor(float T)
	{
		T = FMath::Clamp(T, 0.f, 1.f);
		const float T2 = T * T, T3 = T2 * T, T4 = T2 * T2, T5 = T4 * T;
		const float R = 0.13572138f + 4.61539260f * T - 42.66032258f * T2 + 132.13108234f * T3 - 152.94239396f * T4 + 59.28637943f * T5;
		const float G = 0.09140261f + 2.19418839f * T + 4.84296658f * T2 - 14.18503333f * T3 + 4.27729857f * T4 + 2.82956604f * T5;
		const float B = 0.10667330f + 12.64194608f * T - 60.58204836f * T2 + 110.36276771f * T3 - 89.90310912f * T4 + 27.34824973f * T5;
		return FColor(
			static_cast<uint8>(FMath::Clamp(R, 0.f, 1.f) * 255.f),
			static_cast<uint8>(FMath::Clamp(G, 0.f, 1.f) * 255.f),
			static_cast<uint8>(FMath::Clamp(B, 0.f, 1.f) * 255.f));
	}
}

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
	if (bHideOwner)
	{
		HiddenActors.AddUnique(GetOwner());
	}

	if (bCaptureDepth)
	{
		// RGBA32F so the depth (in R, cm) reads back unclamped; a hidden child at zero offset shares this pose.
		DepthTarget = NewObject<UTextureRenderTarget2D>(this);
		DepthTarget->RenderTargetFormat = RTF_RGBA32f;
		DepthTarget->InitAutoFormat(Width, Height);
		DepthTarget->UpdateResourceImmediate(true);

		// Name must be unique per instance: two sibling cameras on the same actor (e.g. ForkCamera + TineCamera)
		// both requesting a plain "DepthCapture" under the same Outer (GetOwner()) collide, and the loser's
		// capture never renders (its target reads back all zero) - verified empirically.
		DepthCapture = NewObject<USceneCaptureComponent2D>(GetOwner(), *FString::Printf(TEXT("%s_DepthCapture"), *GetName()));
		DepthCapture->SetupAttachment(this);
		DepthCapture->bCaptureEveryFrame = false;
		DepthCapture->bCaptureOnMovement = false;
		DepthCapture->CaptureSource = SCS_SceneDepth;
		DepthCapture->TextureTarget = DepthTarget;
		DepthCapture->FOVAngle = FOVAngle;
		if (bHideOwner)
		{
			DepthCapture->HiddenActors.AddUnique(GetOwner());
		}
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
	//
	// AutoExposureBias only actually takes effect when this component has a persistent view state -
	// see bAlwaysPersistRenderingState below in the class comment / AForkliftVehicle's constructor.
	// Without one, USceneCaptureComponent::GetViewState() (SceneCaptureComponent.cpp) never allocates a
	// FSceneViewStateInterface, which is where eye-adaptation/exposure history lives; every capture then
	// falls back to some default exposure regardless of AutoExposureBias, Manual-mode physical camera
	// settings, or even pinned Histogram min==max brightness - all measured to have zero effect while
	// bAlwaysPersistRenderingState was false. SyntheticDataGenerator's camera already sets this
	// correctly (Camera->bAlwaysPersistRenderingState = bPersistRendering, in CreateCamera(), before
	// RegisterComponent()) which is why its BaseEV=9.5 calibration has always worked correctly.
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
	LastCaptureXf = GetComponentTransform();
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

UTexture2D* UForkliftCameraComponent::UpdateDepthColorTexture(float NearCm, float FarCm)
{
	if (!DepthTarget)
	{
		return nullptr;
	}
	// A fresh synchronous capture, not whatever the periodic auto-capture last queued: without the flush, reading
	// back right after CaptureScene() can race the render thread and come back all zero (verified empirically).
	Capture();
	FlushRenderingCommands();

	TArray<float> Depth;
	if (!ReadDepthCm(Depth))
	{
		return nullptr;
	}

	if (!DepthColorTexture || DepthColorTexture->GetSizeX() != Width || DepthColorTexture->GetSizeY() != Height)
	{
		DepthColorTexture = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8);
		DepthColorTexture->NeverStream = true;
		DepthColorTexture->SRGB = true;
		DepthColorTexture->Filter = TF_Bilinear;
		DepthColorTexture->CompressionSettings = TC_VectorDisplacementmap;  // uncompressed 8-bit, no mips: written every update
		DepthColorTexture->MipGenSettings = TMGS_NoMipmaps;
		DepthColorTexture->UpdateResource();
	}

	// Clamp to [Near,Far] before normalizing, so sky/no-hit pixels (very large depth, see ReadDepthCm) land at
	// Far (t=0, far/blue end) instead of skewing the whole frame toward it.
	const float Range = FMath::Max(FarCm - NearCm, 1.f);
	TArray<FColor> Pixels;
	Pixels.SetNumUninitialized(Depth.Num());
	for (int32 i = 0; i < Depth.Num(); ++i)
	{
		const float D = FMath::Clamp(Depth[i], NearCm, FarCm);
		Pixels[i] = TurboColor(1.f - (D - NearCm) / Range);
	}

	FTexture2DMipMap& Mip = DepthColorTexture->GetPlatformData()->Mips[0];
	void* MipData = Mip.BulkData.Lock(LOCK_READ_WRITE);
	FMemory::Memcpy(MipData, Pixels.GetData(), Pixels.Num() * sizeof(FColor));
	Mip.BulkData.Unlock();
	DepthColorTexture->UpdateResource();
	return DepthColorTexture;
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
