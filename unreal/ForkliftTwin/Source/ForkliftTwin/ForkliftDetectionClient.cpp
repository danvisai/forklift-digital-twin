#include "ForkliftDetectionClient.h"

#include "Dom/JsonObject.h"
#include "ForkliftCameraComponent.h"
#include "HAL/PlatformTime.h"
#include "HttpModule.h"
#include "Interfaces/IHttpResponse.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

void UForkliftDetectionClient::Poll(UForkliftCameraComponent* Camera, float DeltaSeconds)
{
	if (!Camera || bRequestInFlight)
	{
		return;
	}
	TimeSinceRequest += DeltaSeconds;
	if (TimeSinceRequest < 1.f / FMath::Max(DetectionRateHz, 0.1f))
	{
		return;
	}
	TArray<FColor> Pixels;
	if (!Camera->ReadPixelsBgra(Pixels) || Pixels.Num() != Camera->Width * Camera->Height)
	{
		return;  // no frame yet (e.g. the very first tick)
	}
	PendingDepthCm.Reset();
	PendingDepthWidth = PendingDepthHeight = 0;
	if (Camera->ReadDepthCm(PendingDepthCm) && PendingDepthCm.Num() == Camera->Width * Camera->Height)
	{
		PendingDepthWidth = Camera->Width;
		PendingDepthHeight = Camera->Height;
	}
	TimeSinceRequest = 0.f;
	PendingCameraXf = Camera->GetLastCaptureTransform();

	TArray<uint8> Bytes;
	Bytes.SetNumUninitialized(Pixels.Num() * sizeof(FColor));
	FMemory::Memcpy(Bytes.GetData(), Pixels.GetData(), Bytes.Num());

	const int32 W = Camera->Width, H = Camera->Height;
	TSharedRef<IHttpRequest> Request = FHttpModule::Get().CreateRequest();
	Request->SetVerb(TEXT("POST"));
	Request->SetURL(ServerUrl);
	Request->SetHeader(TEXT("Content-Type"), TEXT("application/octet-stream"));
	Request->SetHeader(TEXT("X-Width"), FString::FromInt(W));
	Request->SetHeader(TEXT("X-Height"), FString::FromInt(H));
	Request->SetHeader(TEXT("X-Format"), TEXT("BGRA"));
	Request->SetContent(Bytes);
	Request->OnProcessRequestComplete().BindUObject(this, &UForkliftDetectionClient::OnResponse);

	bRequestInFlight = true;
	ImageWidth = W;
	ImageHeight = H;
	RequestStartSeconds = FPlatformTime::Seconds();
	Request->ProcessRequest();
}

void UForkliftDetectionClient::OnResponse(FHttpRequestPtr Request, FHttpResponsePtr Response, bool bConnectedSuccessfully)
{
	bRequestInFlight = false;
	LastRequestMs = static_cast<float>((FPlatformTime::Seconds() - RequestStartSeconds) * 1000.0);

	if (!bConnectedSuccessfully || !Response.IsValid() || Response->GetResponseCode() != 200)
	{
		LastError = bConnectedSuccessfully ? FString::Printf(TEXT("HTTP %d"), Response.IsValid() ? Response->GetResponseCode() : -1)
											: TEXT("no connection (is serve_detector.py running?)");
		++ConsecutiveFailures;
		if (ConsecutiveFailures >= MaxFailuresBeforeClear)
		{
			LastBoxes.Reset();  // the frame these described is long gone; stop showing stale boxes
		}
		return;
	}

	TSharedPtr<FJsonObject> Json;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Response->GetContentAsString());
	if (!FJsonSerializer::Deserialize(Reader, Json) || !Json.IsValid())
	{
		LastError = TEXT("bad JSON from detector");
		++ConsecutiveFailures;
		return;
	}

	TArray<FForkliftDetectionBox> Boxes;
	const TArray<TSharedPtr<FJsonValue>>* BoxArray = nullptr;
	if (Json->TryGetArrayField(TEXT("boxes"), BoxArray))
	{
		for (const TSharedPtr<FJsonValue>& Value : *BoxArray)
		{
			const TSharedPtr<FJsonObject> Obj = Value->AsObject();
			if (!Obj.IsValid())
			{
				continue;
			}
			FForkliftDetectionBox Box;
			Box.ClassId = Obj->GetIntegerField(TEXT("cls"));
			Box.ClassName = Obj->GetStringField(TEXT("name"));
			Box.Confidence = Obj->GetNumberField(TEXT("conf"));
			Box.X1 = Obj->GetNumberField(TEXT("x1"));
			Box.Y1 = Obj->GetNumberField(TEXT("y1"));
			Box.X2 = Obj->GetNumberField(TEXT("x2"));
			Box.Y2 = Obj->GetNumberField(TEXT("y2"));
			Boxes.Add(Box);
		}
	}
	LastBoxes = MoveTemp(Boxes);
	LastCameraXf = PendingCameraXf;
	LastDepthCm = MoveTemp(PendingDepthCm);
	LastDepthWidth = PendingDepthWidth;
	LastDepthHeight = PendingDepthHeight;
	++ResultSerial;
	LastError.Reset();
	ConsecutiveFailures = 0;
	bEverSucceeded = true;
}

bool UForkliftDetectionClient::GetDepthAtPixel(int32 X, int32 Y, float& OutDepthCm) const
{
	if (X < 0 || Y < 0 || X >= LastDepthWidth || Y >= LastDepthHeight || LastDepthCm.Num() != LastDepthWidth * LastDepthHeight)
	{
		return false;
	}
	const float Depth = LastDepthCm[Y * LastDepthWidth + X];
	if (!FMath::IsFinite(Depth) || Depth <= 0.f || Depth > 100000.f)
	{
		return false;
	}
	OutDepthCm = Depth;
	return true;
}

FString UForkliftDetectionClient::GetStatus() const
{
	if (!bEverSucceeded && ConsecutiveFailures > 0)
	{
		return TEXT("detector offline");
	}
	if (ConsecutiveFailures >= MaxFailuresBeforeClear)
	{
		return FString::Printf(TEXT("detector lost (%s)"), *LastError);
	}
	return FString::Printf(TEXT("%d boxes  %.0f ms"), LastBoxes.Num(), LastRequestMs);
}
