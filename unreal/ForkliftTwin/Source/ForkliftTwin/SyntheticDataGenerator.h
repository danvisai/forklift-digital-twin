// Forklift digital twin - synthetic training-data generator for pallet / loaded-pallet / pocket / tote / rack detection.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HAL/ThreadSafeCounter.h"
#include "Math/RandomStream.h"
#include "Subsystems/WorldSubsystem.h"
#include "SyntheticDataGenerator.generated.h"

class UStaticMesh;
class UStaticMeshComponent;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class UForkliftCameraComponent;
class AWarehouse;

/** YOLO class indices, in the order written to data.yaml. */
namespace ForkliftClass
{
	constexpr int32 Pallet = 0;
	constexpr int32 LoadedPallet = 1;
	constexpr int32 PalletPocket = 2;
	constexpr int32 Tote = 3;
	constexpr int32 Rack = 4;
}

/** One fork opening of a pallet, from Content/Pallets/catalog.json (pallet-local UE frame, cm). */
struct FPocketDef
{
	FString Entry;      // side of the pallet the fork enters from: "-X" "+X" "-Y" "+Y"
	FVector Center = FVector::ZeroVector;   // centre of the pocket volume
	FVector Size = FVector::ZeroVector;     // extents along X, Y, Z (depth is along the entry axis)
	FVector Normal = FVector::ZeroVector;   // outward normal of the entry face
	float DepthCm = 0.f;
};

/** One pallet / loaded-pallet / tote asset from the catalogue. */
struct FAssetDef
{
	FString Name;
	FString Standard;   // pallets: "1200x1000", "1200x800", ...
	FString MeshPath;
	int32 ClassId = ForkliftClass::Pallet;  // pallet, loaded_pallet (asset carries its own load) or tote
	FVector BBoxLo = FVector::ZeroVector, BBoxHi = FVector::ZeroVector;    // whole object, pivot frame
	FVector BaseLo = FVector::ZeroVector, BaseHi = FVector::ZeroVector;    // the pallet itself (no load)
	TArray<FPocketDef> Pockets;
	TArray<TPair<int32, FString>> Slots;  // material slot index -> material key
};

/** A pallet placed in the current scene. Its class becomes loaded_pallet if anything is placed on it. */
struct FPlacedPallet
{
	const FAssetDef* Def = nullptr;
	UStaticMeshComponent* Comp = nullptr;
	FTransform Xf;
	float YawDeg = 0.f;
	int32 ClassId = ForkliftClass::Pallet;
	FVector Lo = FVector::ZeroVector, Hi = FVector::ZeroVector;  // label box in pallet-local space (pallet + its load)
	FString Load = TEXT("none");                                 // none | asset | totes | boxes
};

/** Anything else that gets a plain box label: a tote/bin or a rack. */
struct FPlacedProp
{
	int32 ClassId = ForkliftClass::Tote;
	FString Asset;
	UStaticMeshComponent* Comp = nullptr;
	FTransform Xf;
	FVector Lo = FVector::ZeroVector, Hi = FVector::ZeroVector;
};

/** Spawns the generator when the game is launched with -ForkliftDataGen (keeps ForkliftGameMode untouched). */
UCLASS()
class FORKLIFTTWIN_API USyntheticDataSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
};

/**
 * Renders randomised warehouse scenes and writes YOLO-format training data:
 *
 *   Saved/Dataset/<name>/images/<split>/f000000.png    RGB, fixed exposure (jittered per frame)
 *                       depth/<split>/f000000.npy      metric depth, cm along the camera axis, float16 (HxW)
 *                       labels/<split>/f000000.txt     "class cx cy w h" normalised
 *                       meta/<split>/f000000.json      camera, lighting, per-object pose / visibility / pocket centres
 *                       data.yaml, summary.json
 *
 * Classes: 0 pallet, 1 loaded_pallet, 2 pallet_pocket, 3 tote, 4 rack.
 *
 * Labelling rules: a pallet is `loaded_pallet` if anything stands on it - an asset that carries its own load, or
 * totes / boxes this generator stacks on a bare pallet; its box then covers pallet + load. Every tote / bin is also
 * labelled `tote` (loose, stacked, or on a pallet). Loose cardboard boxes are distractors and carry no label. Racks are
 * one box per rack unit (Rack_A01 shelving: the four in the hall plus extras placed as backdrop). A pocket is the
 * opening rectangle of each fork entry whose face points at the camera. A label box hugs the object's actually
 * visible points (a face-sampled grid, depth-tested per point), not its full 3D extent, so a pallet turned at an
 * angle to the camera does not get a box inflated by corners nobody can see; objects are kept only if enough of
 * them is in frame and not occluded (depth test).
 *
 * Command line (all optional): -ForkliftDataGen -DataName=pilot -DataFrames=300 -DataSeed=1 -DataFramesPerScene=4
 * -DataStart=0 (first frame; a multiple of FramesPerScene, to resume or batch) -DataOut=<dir> -DataW=640 -DataH=480
 * -DataDepthF32.
 *
 * The split is decided per SCENE (a layout seen from FramesPerScene camera poses), 80/10/10 by scene index, so views
 * of the same layout never straddle train and val. Everything derives from -DataSeed and the scene index, so a run
 * can be repeated or resumed exactly.
 */
UCLASS()
class FORKLIFTTWIN_API ASyntheticDataGenerator : public AActor
{
	GENERATED_BODY()

public:
	ASyntheticDataGenerator();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	enum class EState : uint8 { WaitReady, Settle, Running, Flushing, Done };

	// A target the camera is aimed at, with the directions its "front" faces.
	struct FTarget
	{
		FVector Pos = FVector::ZeroVector;
		float MidZ = 30.f;
		TArray<FVector2D> FrontDirs;
	};

	// One row of the label file plus its meta, before it is written.
	struct FLabelRow
	{
		int32 ClassId = 0;
		float X0 = 0, Y0 = 0, X1 = 0, Y1 = 0;   // pixels, clipped
		TSharedPtr<class FJsonObject> Meta;
	};

	// Setup
	bool LoadCatalog();
	void PrepareWorld();
	void CreateCamera();
	UStaticMesh* GetMesh(const FString& Path);

	// Scene
	void BuildScene(int32 SceneIndex);
	void ClearScene();
	void RandomiseEnvironment();
	const FAssetDef* PickPallet();
	bool TryPlacePallet(const FVector2D& Center, float Radius);
	void PlaceToteLoad(FPlacedPallet& Pallet);
	void PlaceBoxLoad(FPlacedPallet& Pallet);
	void PlaceLooseTotes(const FVector2D& Center, float Radius);
	void PlaceRack(const FVector2D& Center, float Radius);
	void AddClutter(const FVector2D& Center, float Radius);
	UStaticMeshComponent* AddMeshComp(UStaticMesh* Mesh, const FTransform& Xf);
	UStaticMeshComponent* AddCube(const FVector& Loc, float YawDeg, const FVector& SizeCm, const FLinearColor& Color, bool bTransient = false);
	void RandomiseMaterials(UStaticMeshComponent* Comp, const FAssetDef* Def);
	bool AreaFree(const FVector2D& C, float YawDeg, const FVector& Lo, const FVector& Hi, float Margin) const;
	void Occupy(const FVector2D& C, float YawDeg, const FVector& Lo, const FVector& Hi);

	// Views and frames
	bool SampleView(FVector& OutPos, FRotator& OutRot, int32& OutTarget);
	void PlaceOccluder(const FVector& CamPos, const FTarget& Target);
	bool RenderFrame(int32 FrameIndex);
	void FinishRun();

	// Labels
	bool ProjectPoint(const FTransform& Cam, const FVector& World, FVector2D& OutPx, float& OutZ) const;
	void SamplePoints(const FTransform& Cam, const TArray<float>& Depth, const TArray<FVector>& Points, int32& OutTotal, int32& OutInFrame, int32& OutUnoccluded) const;
	bool LabelBox(const FTransform& Cam, const TArray<float>& Depth, const FTransform& ObjXf, const FVector& Lo, const FVector& Hi,
	              float MinPx, FLabelRow& Out, float& OutInFrame, float& OutUnocc) const;

	// Configuration (command line)
	FString DatasetName = TEXT("pilot");
	FString OutputDir;
	int32 NumFrames = 300;
	int32 Seed = 1;
	int32 FramesPerScene = 4;
	int32 StartFrame = 0;
	int32 ImageW = 640;
	int32 ImageH = 480;
	bool bDepthHalf = true;
	// Keeping the capture's rendering state lets Lumen light the shadows (without it they crush to pure black), but
	// the frame then comes out much brighter, so the manual exposure bias is lower (a higher bias BRIGHTENS here).
	// -DataNoPersist, -DataEV=<bias>.
	bool bPersistRendering = true;
	float BaseEV = 9.5f;

	// Catalogue and cached assets
	TArray<FAssetDef> Pallets;
	TArray<FAssetDef> Totes;
	UPROPERTY(Transient)
	TMap<FString, TObjectPtr<UStaticMesh>> MeshCache;
	UPROPERTY(Transient)
	TObjectPtr<UStaticMesh> CubeMesh;
	UPROPERTY(Transient)
	TObjectPtr<UStaticMesh> RackMesh;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> FlatMaterial;

	// World handles
	UPROPERTY(Transient)
	TObjectPtr<UForkliftCameraComponent> Camera;
	UPROPERTY(Transient)
	TObjectPtr<AWarehouse> Warehouse;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> SceneComps;      // everything spawned for the current scene
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> Occluder;                // per view, replaced with the next view
	UPROPERTY(Transient)
	TMap<FString, TObjectPtr<UMaterialInstanceDynamic>> EnvMats;  // floor and wall material instances by name

	// Scene state
	struct FRect2D { FVector2D C; FVector2D Ax; FVector2D Ay; FVector2D Half; };
	TArray<FRect2D> Occupied;           // keep-out rectangles for the current scene (fixed ones + everything placed)
	TArray<FRect2D> FixedRects;         // rack colliders and guard rails of the hall itself
	TArray<FPlacedPallet> PlacedPallets;
	TArray<FPlacedProp> PlacedProps;    // totes and placed racks
	TArray<FPlacedProp> FixedRacks;     // the hall's own Rack_A01 units, labelled too
	TArray<FTarget> Targets;
	FVector2D SceneCenter = FVector2D::ZeroVector;
	TSharedPtr<class FJsonObject> LightingMeta;

	// Run state
	EState State = EState::WaitReady;
	int32 WaitFrames = 0;
	int32 SettleTicks = 0;
	int32 NextFrame = 0;         // frames written so far (also the file index)
	int32 CurrentScene = -1;
	int32 SceneRetries = 0;
	int32 Rejected = 0;
	double StartTime = 0.0;
	FRandomStream Rand;
	FThreadSafeCounter PendingWrites;

	// Tallies for summary.json
	TMap<FString, int32> SplitFrames;
	TMap<FString, int32> ClassCounts;   // key "<split>/<class>"
	int32 TinyPockets = 0;              // pocket boxes below 8 px on their shorter side
	int32 PocketBoxes = 0;
	int32 NegativeFrames = 0;           // frames with no pallet / loaded_pallet label
};
