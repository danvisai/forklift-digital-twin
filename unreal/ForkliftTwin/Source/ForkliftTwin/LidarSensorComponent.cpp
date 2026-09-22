#include "LidarSensorComponent.h"

#include "CollisionQueryParams.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"

namespace
{
	// Standard normal sample (Box-Muller); FMath has no gaussian.
	float RandGauss()
	{
		const float U1 = FMath::Max(FMath::FRand(), 1.e-7f);
		return FMath::Sqrt(-2.f * FMath::Loge(U1)) * FMath::Cos(2.f * PI * FMath::FRand());
	}

	// Minimal .npy v1.0 writer for a C-ordered little-endian float32 matrix, so np.load() reads it directly.
	bool WriteNpyF32(const FString& Path, const float* Data, int32 Rows, int32 Cols)
	{
		FString Dict = FString::Printf(TEXT("{'descr': '<f4', 'fortran_order': False, 'shape': (%d, %d), }"), Rows, Cols);
		// Magic(6) + version(2) + header length(2) + header must total a multiple of 64, header ending in '\n'.
		const int32 Unpadded = 10 + Dict.Len() + 1;
		const int32 Pad = (64 - Unpadded % 64) % 64;
		Dict += FString::ChrN(Pad, TEXT(' ')) + TEXT("\n");

		TArray<uint8> Bytes;
		const uint8 Magic[] = { 0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0 };
		Bytes.Append(Magic, UE_ARRAY_COUNT(Magic));
		const uint16 HeaderLen = static_cast<uint16>(Dict.Len());
		Bytes.Add(HeaderLen & 0xFF);
		Bytes.Add(HeaderLen >> 8);
		const FTCHARToUTF8 Ascii(*Dict);
		Bytes.Append(reinterpret_cast<const uint8*>(Ascii.Get()), Ascii.Length());
		Bytes.Append(reinterpret_cast<const uint8*>(Data), static_cast<int64>(Rows) * Cols * sizeof(float));
		return FFileHelper::SaveArrayToFile(Bytes, *Path);
	}
}

ULidarSensorComponent::ULidarSensorComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	// After the truck has moved for the frame (the owner ticks in this group too, see BeginPlay).
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void ULidarSensorComponent::BeginPlay()
{
	Super::BeginPlay();
	if (AActor* Owner = GetOwner())
	{
		AddTickPrerequisiteActor(Owner);  // scan against the pose the owner just finished computing
	}
	TimeSinceScan = 1.f / FMath::Max(ScanRateHz, 0.1f);  // first tick scans immediately
}

void ULidarSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	const float Period = 1.f / FMath::Max(ScanRateHz, 0.1f);
	TimeSinceScan += DeltaTime;
	if (TimeSinceScan >= Period)
	{
		// Slow frames drop scans rather than queueing catch-up work.
		TimeSinceScan = FMath::Fmod(TimeSinceScan, Period);
		Scan();
	}
}

void ULidarSensorComponent::Scan()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}
	const double T0 = FPlatformTime::Seconds();

	const int32 NumChannels = FMath::Max(Channels, 1);
	const int32 NumSteps = FMath::Max(HorizontalSteps, 1);
	const FTransform Xf = GetComponentTransform();
	const FVector Origin = Xf.GetLocation();
	const FQuat Rot = Xf.GetRotation();

	FCollisionQueryParams Params(SCENE_QUERY_STAT(ForkliftLidar), false, bIgnoreOwner ? GetOwner() : nullptr);
	Params.bReturnPhysicalMaterial = false;

	// A full circle must not repeat its first ray at the end; a partial arc includes both edges.
	const bool bFullCircle = HorizontalFovDeg >= 359.99f;
	const float AzStep = NumSteps > 1 ? HorizontalFovDeg / (bFullCircle ? NumSteps : NumSteps - 1) : 0.f;
	const float AzStart = NumSteps > 1 ? -HorizontalFovDeg * 0.5f : 0.f;

	Ranges.SetNumUninitialized(NumChannels * NumSteps);
	Points.Reset();

	for (int32 C = 0; C < NumChannels; ++C)
	{
		const float Elev = NumChannels > 1 ? FMath::Lerp(VerticalMinDeg, VerticalMaxDeg, static_cast<float>(C) / (NumChannels - 1)) : 0.5f * (VerticalMinDeg + VerticalMaxDeg);
		for (int32 S = 0; S < NumSteps; ++S)
		{
			const FVector LocalDir = FRotator(Elev, AzStart + S * AzStep, 0.f).Vector();
			const FVector Dir = Rot.RotateVector(LocalDir);

			FHitResult Hit;
			float& Out = Ranges[C * NumSteps + S];
			if (World->LineTraceSingleByChannel(Hit, Origin + Dir * MinRangeCm, Origin + Dir * MaxRangeCm, TraceChannel, Params))
			{
				float Dist = MinRangeCm + Hit.Distance;
				if (RangeNoiseStdCm > 0.f)
				{
					Dist = FMath::Clamp(Dist + RangeNoiseStdCm * RandGauss(), MinRangeCm, MaxRangeCm);
				}
				Out = Dist;
				Points.Add(LocalDir * Dist);
#if ENABLE_DRAW_DEBUG
				if (bDrawDebug)
				{
					DrawDebugPoint(World, Origin + Dir * Dist, 5.f, FColor::Green, false, 1.f / FMath::Max(ScanRateHz, 0.1f));
				}
#endif
			}
			else
			{
				Out = -1.f;
			}
		}
	}

	++ScanIndex;
	ScanTimeSeconds = World->GetTimeSeconds();
	ScanCostMs = static_cast<float>((FPlatformTime::Seconds() - T0) * 1000.0);
}

bool ULidarSensorComponent::SaveRangesNpy(const FString& Path) const
{
	return Ranges.Num() > 0 && WriteNpyF32(Path, Ranges.GetData(), FMath::Max(Channels, 1), Ranges.Num() / FMath::Max(Channels, 1));
}

bool ULidarSensorComponent::SavePointsNpy(const FString& Path) const
{
	TArray<float> Flat;
	Flat.Reserve(Points.Num() * 3);
	for (const FVector& P : Points)
	{
		Flat.Add(static_cast<float>(P.X));
		Flat.Add(static_cast<float>(P.Y));
		Flat.Add(static_cast<float>(P.Z));
	}
	return WriteNpyF32(Path, Flat.GetData(), Points.Num(), 3);
}
