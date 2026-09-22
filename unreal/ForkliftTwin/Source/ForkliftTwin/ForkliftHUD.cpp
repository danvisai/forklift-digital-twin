#include "ForkliftHUD.h"

#include "Engine/Canvas.h"
#include "Engine/TextureRenderTarget2D.h"
#include "ForkliftCameraComponent.h"
#include "ForkliftDetectionClient.h"
#include "ForkliftVehicle.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "LidarSensorComponent.h"

namespace
{
	// One fixed colour per class id (pallet, loaded_pallet, pallet_pocket, tote, rack - see data.yaml); repeats if a
	// server with more classes is pointed at this HUD.
	const FLinearColor ClassColors[] = {
		FLinearColor(0.2f, 0.9f, 0.2f), FLinearColor(1.0f, 0.55f, 0.0f), FLinearColor(1.0f, 0.15f, 0.15f),
		FLinearColor(0.25f, 0.6f, 1.0f), FLinearColor(0.85f, 0.2f, 0.9f),
	};
}

AForkliftHUD::AForkliftHUD()
{
	PrimaryActorTick.bCanEverTick = true;  // polls the detector server independently of DrawHUD
}

void AForkliftHUD::BeginPlay()
{
	Super::BeginPlay();
	Detector = NewObject<UForkliftDetectionClient>(this);
	Detector->ServerUrl = DetectorUrl;
	Detector->DetectionRateHz = DetectionRateHz;
}

void AForkliftHUD::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!bShowDetections || !Detector)
	{
		return;
	}
	if (const AForkliftVehicle* Truck = Cast<AForkliftVehicle>(GetOwningPawn()))
	{
		Detector->Poll(Truck->GetForkCamera(), DeltaSeconds);
	}
}

void AForkliftHUD::DrawHUD()
{
	Super::DrawHUD();

	if (PlayerOwner && PlayerOwner->WasInputKeyJustPressed(EKeys::V))
	{
		bShowSensors = !bShowSensors;
	}
	const AForkliftVehicle* Truck = Cast<AForkliftVehicle>(GetOwningPawn());
	if (!bShowSensors || !Truck || !Canvas)
	{
		return;
	}

	constexpr float Margin = 16.f;
	float Y = Margin;

	if (const UForkliftCameraComponent* Cam = Truck->GetForkCamera(); Cam && Cam->GetRenderTarget())
	{
		// The capture's alpha is 0, so this must not alpha-blend.
		const float W = 320.f, H = W * Cam->Height / Cam->Width;
		DrawTexture(Cam->GetRenderTarget(), Margin, Y, W, H, 0.f, 0.f, 1.f, 1.f, FLinearColor::White, BLEND_Opaque);
		DrawText(FString::Printf(TEXT("fork camera  %dx%d @ %.0f Hz"), Cam->Width, Cam->Height, Cam->CaptureRateHz), FLinearColor::White, Margin + 4.f, Y + 4.f);

		if (bShowDetections && Detector && Detector->GetImageWidth() > 0)
		{
			// Boxes are in the pixel space of the frame the client sent (Cam->Width x Cam->Height); scale to the
			// on-screen thumbnail (W x H).
			const float Sx = W / Detector->GetImageWidth(), Sy = H / Detector->GetImageHeight();
			for (const FForkliftDetectionBox& Box : Detector->GetBoxes())
			{
				const FLinearColor Color = ClassColors[Box.ClassId % UE_ARRAY_COUNT(ClassColors)];
				const float Bx1 = Margin + Box.X1 * Sx, By1 = Y + Box.Y1 * Sy;
				const float Bx2 = Margin + Box.X2 * Sx, By2 = Y + Box.Y2 * Sy;
				constexpr float Thick = 2.f;
				DrawRect(Color, Bx1, By1, Bx2 - Bx1, Thick);                    // top
				DrawRect(Color, Bx1, By2 - Thick, Bx2 - Bx1, Thick);            // bottom
				DrawRect(Color, Bx1, By1, Thick, By2 - By1);                    // left
				DrawRect(Color, Bx2 - Thick, By1, Thick, By2 - By1);            // right
				DrawText(FString::Printf(TEXT("%s %.0f%%"), *Box.ClassName, Box.Confidence * 100.f), Color, Bx1 + 2.f, FMath::Max(By1 - 14.f, Y));
			}
			DrawText(Detector->GetStatus(), Detector->IsConnected() ? FLinearColor::White : FLinearColor(1.f, 0.6f, 0.2f), Margin + W - 120.f, Y + H - 14.f);
		}
		Y += H + Margin;
	}

	if (const ULidarSensorComponent* Lidar = Truck->GetLidar())
	{
		const float Cx = Margin + RadarRadiusPx, Cy = Y + RadarRadiusPx;
		DrawRect(FLinearColor(0.f, 0.f, 0.f, 0.55f), Margin, Y, RadarRadiusPx * 2.f, RadarRadiusPx * 2.f);
		DrawText(FString::Printf(TEXT("lidar  %d returns  %.1f ms  (%.0f m ring)"), Lidar->GetPointsRef().Num(), Lidar->GetScanCostMs(), RadarRangeCm * 0.01f),
			FLinearColor::White, Margin + 4.f, Y + 4.f);

		const float Scale = RadarRadiusPx / RadarRangeCm;
		for (const FVector& P : Lidar->GetPointsRef())
		{
			// Sensor frame: +X forward -> up the screen, +Y right -> right on the screen.
			const float Px = Cx + P.Y * Scale, Py = Cy - P.X * Scale;
			if (FMath::Abs(Px - Cx) < RadarRadiusPx && FMath::Abs(Py - Cy) < RadarRadiusPx)
			{
				DrawRect(FLinearColor::Green, Px, Py, 2.f, 2.f);
			}
		}
		DrawRect(FLinearColor::Red, Cx - 3.f, Cy - 3.f, 6.f, 6.f);  // the sensor itself
	}
}
