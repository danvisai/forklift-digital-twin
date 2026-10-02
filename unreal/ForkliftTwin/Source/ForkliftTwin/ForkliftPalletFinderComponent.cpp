#include "ForkliftPalletFinderComponent.h"

#include "EngineUtils.h"
#include "ForkliftCameraComponent.h"
#include "ForkliftDetectionClient.h"
#include "ForkliftVehicle.h"
#include "Warehouse.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY_STATIC(LogForkliftFinder, Log, All);

UForkliftPalletFinderComponent::UForkliftPalletFinderComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UForkliftPalletFinderComponent::BeginPlay()
{
	Super::BeginPlay();
	Vehicle = Cast<AForkliftVehicle>(GetOwner());
	for (TActorIterator<AWarehouse> It(GetWorld()); It; ++It)
	{
		Warehouse = *It;
		break;
	}
	Detector = NewObject<UForkliftDetectionClient>(this, TEXT("ForkCameraDetector"));
	Detector->ServerUrl = DetectorUrl;
	Detector->DetectionRateHz = DetectionRateHz;
}

void UForkliftPalletFinderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	AForkliftVehicle* Truck = Vehicle.Get();
	if (!bSearching || !Truck || !Detector || !Truck->GetForkCamera())
	{
		return;
	}
	Detector->Poll(Truck->GetForkCamera(), DeltaTime);
	DebugTimer += DeltaTime;
	if (DebugTimer > 4.f)
	{
		DebugTimer = 0.f;
		DebugReport();
	}
	if (Detector->GetResultSerial() != LastSerial)
	{
		LastSerial = Detector->GetResultSerial();
		Integrate();
	}
}

void UForkliftPalletFinderComponent::DebugReport()
{
	// Diagnostics only: compare each confirmed estimate with the nearest real loose pallet. Never used for control.
	const AWarehouse* WH = Warehouse.Get();
	if (!WH)
	{
		return;
	}
	for (const FPalletCandidate& C : Candidates)
	{
		if (C.Hits < MinHits)
		{
			continue;
		}
		float Best = BIG_NUMBER;
		FVector True = FVector::ZeroVector;
		for (const FLoosePallet& P : WH->GetLoosePallets())
		{
			if (P.Comp && FVector::Dist2D(P.Comp->GetComponentLocation(), C.Pos) < Best)
			{
				Best = FVector::Dist2D(P.Comp->GetComponentLocation(), C.Pos);
				True = P.Comp->GetComponentLocation();
			}
		}
		UE_LOG(LogForkliftFinder, Log, TEXT("[Finder] debug: %s est %s hits %d -> nearest real pallet %s, error %.0f cm%s"), C.IsLoaded() ? TEXT("loaded") : TEXT("empty"),
			*C.Pos.ToString(), C.Hits, *True.ToString(), Best, Best > 120.f ? TEXT("  (GHOST)") : TEXT(""));
	}
}

void UForkliftPalletFinderComponent::Integrate()
{
	AForkliftVehicle* Truck = Vehicle.Get();
	const UForkliftCameraComponent* Camera = Truck ? Truck->GetForkCamera() : nullptr;
	if (!Camera || Detector->GetImageWidth() <= 0)
	{
		return;
	}
	const int32 W = Detector->GetImageWidth(), H = Detector->GetImageHeight();
	const float FocalPx = (W * 0.5f) / FMath::Tan(FMath::DegreesToRadians(Camera->FOVAngle * 0.5f));
	const FTransform CamXf = Detector->GetResultCameraTransform();   // pose when THIS frame was rendered, not now
	const AWarehouse* WH = Warehouse.Get();

	for (const FForkliftDetectionBox& Box : Detector->GetBoxes())
	{
		const bool bLoaded = Box.ClassName.Equals(TEXT("loaded_pallet"), ESearchCase::IgnoreCase);
		const bool bEmpty = Box.ClassName.Equals(TEXT("pallet"), ESearchCase::IgnoreCase);
		if ((!bLoaded && !bEmpty) || Box.Confidence < MinConfidence)
		{
			continue;
		}
		// A box cut off by the image border describes only part of the pallet, so its centre is biased toward the visible part
		// (automotive pipelines flag such 'truncated' objects). Keep it so the nearest pallet is noticed early, but at low weight:
		// the estimate is refined as the pallet comes fully into view.
		const bool bTruncated = Box.X1 < 4.f || Box.X2 > W - 4.f || Box.Y2 > H - 4.f;
		// Depth of the pallet's front face: sample the lower half of the box (the part standing on the floor), take a low quantile
		// so a stray far pixel (floor behind it, the gap between boards) cannot push the point back.
		TArray<float, TInlineAllocator<25>> Depths;
		const float V0 = Box.Y1 + 0.5f * (Box.Y2 - Box.Y1);
		for (int32 Gy = 0; Gy < 5; ++Gy)
		{
			for (int32 Gx = 0; Gx < 5; ++Gx)
			{
				const float U = FMath::Lerp(Box.X1, Box.X2, (Gx + 0.5f) / 5.f);
				const float V = FMath::Lerp(V0, Box.Y2, (Gy + 0.5f) / 5.f);
				float D = 0.f;
				if (Detector->GetDepthAtPixel(FMath::RoundToInt(U), FMath::RoundToInt(V), D) && D > 30.f)
				{
					Depths.Add(D);
				}
			}
		}
		if (Depths.Num() < 6)
		{
			continue;
		}
		Depths.Sort();
		const float D = Depths[Depths.Num() / 5];
		const float U = 0.5f * (Box.X1 + Box.X2);
		const float V = FMath::Lerp(V0, Box.Y2, 0.5f);
		const FVector Face = CamXf.TransformPosition(FVector(D, (U - W * 0.5f) * D / FocalPx, (H * 0.5f - V) * D / FocalPx));
		const FVector CamPos = CamXf.GetLocation();
		if (Face.Z > MaxFaceHeightCm || FVector::Dist2D(Face, CamPos) > MaxRangeCm)
		{
			continue;   // on a shelf, or too far for a trustworthy depth
		}
		if (DebugLines < 60 && FParse::Param(FCommandLine::Get(), TEXT("FinderDebug")))
		{
			++DebugLines;
			float Best = BIG_NUMBER;
			FVector True = FVector::ZeroVector;
			if (const AWarehouse* WHd = Warehouse.Get())
			{
				for (const FLoosePallet& P : WHd->GetLoosePallets())
				{
					if (P.Comp && FVector::Dist2D(P.Comp->GetComponentLocation(), Face) < Best)
					{
						Best = FVector::Dist2D(P.Comp->GetComponentLocation(), Face);
						True = P.Comp->GetComponentLocation();
					}
				}
			}
			UE_LOG(LogForkliftFinder, Log, TEXT("[Finder] det %s %.2f box (%.0f,%.0f)-(%.0f,%.0f) img %dx%d depth %.0f -> face %s (cam %s, FOV %.0f) nearest real %s dist %.0f"),
				*Box.ClassName, Box.Confidence, Box.X1, Box.Y1, Box.X2, Box.Y2, W, H, D, *Face.ToString(), *CamPos.ToString(), Camera->FOVAngle, *True.ToString(), Best);
		}
		const FVector2D Dir = FVector2D(Face.X - CamPos.X, Face.Y - CamPos.Y).GetSafeNormal();
		const FVector Centre(Face.X + Dir.X * HalfDepthCm, Face.Y + Dir.Y * HalfDepthCm, 0.f);
		if (FMath::Abs(Centre.X) > 1700.f || FMath::Abs(Centre.Y) > 1100.f)
		{
			continue;
		}
		if (WH)
		{
			bool bInsideMap = false;
			for (const FBox& B : WH->GetObstacleBoxes())
			{
				if (Centre.X > B.Min.X - 90.f && Centre.X < B.Max.X + 90.f && Centre.Y > B.Min.Y - 90.f && Centre.Y < B.Max.Y + 90.f)
				{
					bInsideMap = true;   // known rack / wall: pallets in racks are not floor pallets
					break;
				}
			}
			if (bInsideMap)
			{
				continue;
			}
		}

		FPalletCandidate* Match = nullptr;
		float Best = 90.f;
		for (FPalletCandidate& C : Candidates)
		{
			const float Dist = FVector::Dist2D(C.Pos, Centre);
			if (Dist < Best)
			{
				Best = Dist;
				Match = &C;
			}
		}
		if (!Match)
		{
			FPalletCandidate& C = Candidates.AddDefaulted_GetRef();
			C.Pos = Centre;
			C.WeightSum = 0.f;
			for (const FVector& Z : ExcludedZones)
			{
				if (FVector::Dist2D(Z, Centre) < 150.f)
				{
					C.bExcluded = true;
				}
			}
			Match = &C;
		}
		++Match->Hits;
		Match->FullHits += bTruncated ? 0 : 1;
		Match->LoadedVotes += bLoaded ? 1 : 0;
		Match->BestConfidence = FMath::Max(Match->BestConfidence, Box.Confidence);
		Match->LastSeen = GetWorld()->GetTimeSeconds();
		// Depth error grows with range, so weight each frame by 1 / (1 + (range / 250 cm)^2) and keep a weighted mean.
		const float Range = FVector::Dist2D(Face, CamPos);
		const float Weight = (bTruncated ? 0.25f : 1.f) / (1.f + FMath::Square(Range / 250.f));
		Match->Pos = (Match->Pos * Match->WeightSum + Centre * Weight) / (Match->WeightSum + Weight);
		Match->WeightSum += Weight;
		if (Match->Hits == MinHits)
		{
			UE_LOG(LogForkliftFinder, Log, TEXT("[Finder] confirmed %s pallet at %s (conf %.2f)"), Match->IsLoaded() ? TEXT("loaded") : TEXT("empty"), *Match->Pos.ToString(), Match->BestConfidence);
		}
	}
}

bool UForkliftPalletFinderComponent::GetBestCandidate(bool bWantLoaded, const FVector& From, FVector& OutCentre, int32* OutIndex) const
{
	float Best = BIG_NUMBER;
	int32 BestIdx = INDEX_NONE;
	for (int32 I = 0; I < Candidates.Num(); ++I)
	{
		const FPalletCandidate& C = Candidates[I];
		if (C.bExcluded || C.Hits < MinHits || C.FullHits < MinFullHits || C.IsLoaded() != bWantLoaded)
		{
			continue;
		}
		const float Dist = FVector::Dist2D(C.Pos, From);
		if (Dist < Best)
		{
			Best = Dist;
			BestIdx = I;
		}
	}
	if (BestIdx == INDEX_NONE)
	{
		return false;
	}
	OutCentre = Candidates[BestIdx].Pos;
	if (OutIndex)
	{
		*OutIndex = BestIdx;
	}
	return true;
}

bool UForkliftPalletFinderComponent::RefineNear(const FVector& Approx, float Radius, FVector& OutCentre) const
{
	float Best = Radius;
	bool bFound = false;
	for (const FPalletCandidate& C : Candidates)
	{
		const float Dist = FVector::Dist2D(C.Pos, Approx);
		if (Dist < Best && C.Hits >= 2)
		{
			Best = Dist;
			OutCentre = C.Pos;
			bFound = true;
		}
	}
	return bFound;
}

void UForkliftPalletFinderComponent::ExcludeNear(const FVector& Pos, float Radius)
{
	ExcludedZones.Add(Pos);
	for (FPalletCandidate& C : Candidates)
	{
		if (FVector::Dist2D(C.Pos, Pos) < Radius)
		{
			C.bExcluded = true;
		}
	}
}

void UForkliftPalletFinderComponent::RemoveNear(const FVector& Pos, float Radius)
{
	ExcludedZones.Add(Pos);   // and do not re-add it from a late detection of the load riding on the forks
	Candidates.RemoveAll([&](const FPalletCandidate& C) { return FVector::Dist2D(C.Pos, Pos) < Radius; });
}
