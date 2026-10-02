// Forklift digital twin - talks to the local detector (Scripts/ml/serve_detector.py) so the HUD can draw boxes.
#pragma once

#include "CoreMinimal.h"
#include "Interfaces/IHttpRequest.h"
#include "UObject/Object.h"
#include "ForkliftDetectionClient.generated.h"

class UForkliftCameraComponent;
class IHttpRequest;
class IHttpResponse;

USTRUCT(BlueprintType)
struct FForkliftDetectionBox
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly)
	int32 ClassId = 0;
	UPROPERTY(BlueprintReadOnly)
	FString ClassName;
	UPROPERTY(BlueprintReadOnly)
	float Confidence = 0.f;
	// Pixel coordinates in the frame that was sent (ImageWidth x ImageHeight), origin top-left.
	UPROPERTY(BlueprintReadOnly)
	float X1 = 0.f, Y1 = 0.f, X2 = 0.f, Y2 = 0.f;
};

/**
 * Sends the fork camera's latest frame to a local HTTP detector (see Scripts/ml/serve_detector.py) at
 * DetectionRateHz and keeps the most recent result. One request in flight at a time: if the server is slower
 * than the request rate, ticks are skipped rather than queued, so a stalled server cannot pile up requests.
 *
 * The server is optional - with none running, GetLastError() reports it and GetBoxes() stays empty, so the
 * rest of the HUD keeps working without it.
 */
UCLASS()
class FORKLIFTTWIN_API UForkliftDetectionClient : public UObject
{
	GENERATED_BODY()

public:
	/** Call periodically (e.g. from the HUD's Tick) with the truck's fork camera. No-ops if a request is already
	 *  in flight or DetectionRateHz has not elapsed since the last one. */
	void Poll(UForkliftCameraComponent* Camera, float DeltaSeconds);

	const TArray<FForkliftDetectionBox>& GetBoxes() const { return LastBoxes; }
	int32 GetImageWidth() const { return ImageWidth; }
	int32 GetImageHeight() const { return ImageHeight; }
	float GetLastRequestMs() const { return LastRequestMs; }
	bool IsConnected() const { return bEverSucceeded && ConsecutiveFailures == 0; }
	FString GetStatus() const;
	int32 GetResultSerial() const { return ResultSerial; }

	/** Pose of the camera when the frame behind the latest result was rendered. Boxes and depth must be projected with THIS pose,
	 *  not the camera's current one: the answer arrives after the truck has moved on (sensor-pose time synchronisation). */
	const FTransform& GetResultCameraTransform() const { return LastCameraXf; }

	/** Depth captured with the RGB image that produced the latest boxes. Pixel coordinates use that image. */
	bool GetDepthAtPixel(int32 X, int32 Y, float& OutDepthCm) const;

	/** e.g. http://127.0.0.1:8765/detect. */
	UPROPERTY(EditAnywhere)
	FString ServerUrl = TEXT("http://127.0.0.1:8765/detect");

	UPROPERTY(EditAnywhere)
	float DetectionRateHz = 5.f;

	/** Drop stale boxes after this many failed requests in a row (the frame they describe is long gone). */
	UPROPERTY(EditAnywhere)
	int32 MaxFailuresBeforeClear = 3;

private:
	void OnResponse(FHttpRequestPtr Request, FHttpResponsePtr Response, bool bConnectedSuccessfully);

	TArray<FForkliftDetectionBox> LastBoxes;
	FTransform PendingCameraXf;
	FTransform LastCameraXf;
	TArray<float> PendingDepthCm;
	TArray<float> LastDepthCm;
	int32 ImageWidth = 0;
	int32 ImageHeight = 0;
	int32 PendingDepthWidth = 0;
	int32 PendingDepthHeight = 0;
	int32 LastDepthWidth = 0;
	int32 LastDepthHeight = 0;
	int32 ResultSerial = 0;
	float LastRequestMs = 0.f;
	FString LastError;
	bool bRequestInFlight = false;
	bool bEverSucceeded = false;
	int32 ConsecutiveFailures = 0;
	float TimeSinceRequest = 0.f;
	double RequestStartSeconds = 0.0;
};
