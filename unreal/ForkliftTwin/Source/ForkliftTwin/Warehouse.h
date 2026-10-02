// Forklift digital twin - procedural warehouse (port of _create_factory in the Kit extension).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Warehouse.generated.h"

class UStaticMesh;
class UStaticMeshComponent;
class UMaterialInterface;
class UPointLightComponent;

/** A floor slot where one pallet is placed. Poses are world-space UE cm / degrees. */
USTRUCT(BlueprintType)
struct FPalletSlot
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Warehouse")
	FName Id;

	/** True: slot for loaded pallets. False: slot for empty pallets. */
	UPROPERTY(BlueprintReadOnly, Category = "Warehouse")
	bool bForLoaded = false;

	/** Pallet centre on the floor (Z = 0). */
	UPROPERTY(BlueprintReadOnly, Category = "Warehouse")
	FVector Location = FVector::ZeroVector;

	/** Yaw the pallet should end up with. */
	UPROPERTY(BlueprintReadOnly, Category = "Warehouse")
	float PalletYawDeg = 0.f;

	/** Where the forklift should be, heading straight at the slot, before the placement controller takes over. */
	UPROPERTY(BlueprintReadOnly, Category = "Warehouse")
	FVector PrePoseLocation = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Warehouse")
	float PrePoseYawDeg = 0.f;
};

/** One loose, dynamic, pickup-able pallet and whether the mission has already delivered it. */
USTRUCT()
struct FLoosePallet
{
	GENERATED_BODY()

	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> Comp = nullptr;

	UPROPERTY()
	bool bDelivered = false;

	/** World pose the pallet was spawned at, for ResetLoosePallets(). */
	UPROPERTY()
	FTransform Home;

	/** Carries a stack of cartons: goes to the loaded drop zone instead of the empty one. */
	UPROPERTY()
	bool bLoaded = false;
};

/**
 * Builds the same warehouse as the Kit prototype: a 36 x 24 m floor with four 3 m walls, four rack
 * banks with invisible box colliders, two pallets, three bins, three guard rails and a dynamic
 * physics test box. Everything is regenerated from OnConstruction, so it shows up in the editor and
 * can be re-run from the "Rebuild" button in the Details panel (the old "Create/Reset Factory").
 *
 * The shell (floor + walls) was enlarged 50% from the original 24 x 16 m Kit footprint for more
 * navigation/aisle room; everything inside it (racks, pallets, bins, rails) keeps its original Kit
 * coordinates so the loose pallet stays exactly 3.5 m from the fixed test spawn point that
 * ForkliftGameMode's SensorTest/PalletTest/AutoPickupTest rely on.
 *
 * The shell (floor/walls/ceiling) is tiled from the same modular kit Isaac Sim's own "Simple_Warehouse"
 * training scene (warehouse.usd / warehouse_multiple_shelves.usd) is built from - real 6x6 m floor and
 * ceiling panels, 6 m wall tiles stacked 3 high for the kit's true 9 m wall height, and a pendant
 * SM_LampCeilingA_04 fixture per overhead light - instead of flat-colour boxes. If those meshes aren't
 * imported yet (see Scripts/setup.py), Rebuild falls back to the old flat 3 m box shell so the actor
 * never renders empty. The level still places a DirectionalLight/SkyLight/SkyAtmosphere/
 * ExponentialHeightFog (Warehouse.umap) for the old open-sky look; on Rebuild this actor dims the
 * DirectionalLight/SkyLight to a basic level (not hidden - a fully hidden sun/sky left the building's
 * outside pure black) rather than requiring the map itself to be re-edited.
 *
 * Coordinates are the Kit stage's (meters, right-handed, Z up) converted to UE (cm, left-handed):
 * X and Z scale by 100, Y is negated. So Kit y=+4.5 lands at UE y=-450.
 */
UCLASS()
class FORKLIFTTWIN_API AWarehouse : public AActor
{
	GENERATED_BODY()

public:
	AWarehouse();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;

	/** Tear down and recreate all warehouse geometry. */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "Warehouse")
	void Rebuild();

	UPROPERTY(EditAnywhere, Category = "Warehouse")
	bool bSpawnPhysicsTestBox = true;

	/** Make the loose block pallet (Kit (-3.5, 2.5)) a 25 kg dynamic body so manual/automatic pickup can lift it. */
	UPROPERTY(EditAnywhere, Category = "Warehouse")
	bool bDynamicLoosePallet = true;

	/** Cap the shell with a roof at wall height (3 m) instead of leaving it open to the level's sky. */
	UPROPERTY(EditAnywhere, Category = "Warehouse|Environment")
	bool bEnclosedCeiling = true;

	/** Spawn a grid of overhead point lights under the ceiling, like Isaac Sim's warehouse fixtures. */
	UPROPERTY(EditAnywhere, Category = "Warehouse|Environment")
	bool bOverheadLights = true;

	/** Dim the level's DirectionalLight/SkyLight to a basic level so the warehouse's own fixtures are the
	 *  dominant indoor light (the walls/ceiling block them from flooding in anyway), while still giving
	 *  the building's exterior some basic ambient instead of going pure black. */
	UPROPERTY(EditAnywhere, Category = "Warehouse|Environment")
	bool bDisableOutdoorSky = true;

	/** Overhead fixtures cast dynamic shadows. Off by default: 12 shadow-casting lights are expensive and
	 *  this machine already runs hot (see thermal-throttling notes) - flip on for a beefier GPU. */
	UPROPERTY(EditAnywhere, Category = "Warehouse|Environment")
	bool bOverheadLightsCastShadows = false;

	/** Per-fixture brightness and reach. Defaults suit the real kit's 9 m-high shell (fixtures hang
	 *  ~3 m below the ceiling, so the emitter itself sits around 6 m up) - a first pass at 25000 lm blew
	 *  out the shelf boxes, but that turned out to be a placement bug (they sat ~35 cm too high, closer
	 *  to the fixtures than intended); now that they sit on the actual beam/pallet surface, brightness is
	 *  back up from the overcorrected 9000 lm mainly so the outer walls (only lit by these fixtures -
	 *  the level's own sun/sky can't reach inside the shell) actually read their colour. */
	UPROPERTY(EditAnywhere, Category = "Warehouse|Environment", meta = (ClampMin = "0.0"))
	float OverheadLightLumens = 14000.f;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Environment", meta = (ClampMin = "0.0"))
	float OverheadLightRadiusM = 17.f;

	/** The first/original loose pallet (Kit (-3.5, 2.5)), or null. Kept for ForkliftGameMode's
	 *  PalletTest/AutoPickupTest/batch harness, which all assume exactly one loose pallet there. */
	UStaticMeshComponent* GetLoosePallet() const { return LoosePallets.IsValidIndex(0) ? LoosePallets[0].Comp : nullptr; }

	/** All loose, pickup-able pallets and their delivery state, for the mission controller. */
	const TArray<FLoosePallet>& GetLoosePallets() const { return LoosePallets; }

	/** Marks the given loose pallet as delivered (placed in a slot). No-op if Comp isn't a known loose pallet. */
	void MarkPalletDelivered(UStaticMeshComponent* Comp);

	/** Puts every loose pallet back at its spawn pose (delivered flags cleared, velocities zeroed) for a new
	 *  episode. With bJitter each pallet is nudged by up to +-JitterCm in X/Y and +-JitterYawDeg in yaw
	 *  (deterministic for a given Seed) so repeated episodes are not identical. */
	void ResetLoosePallets(bool bJitter, int32 Seed, float JitterCm = 20.f, float JitterYawDeg = 5.f);

	/** Static obstacle boxes (rack runs, outer walls) in world space, for path planning. */
	const TArray<FBox>& GetObstacleBoxes() const { return ObstacleBoxes; }

	/** All drop slots (empty zone on the west side, loaded zone on the east side). */
	const TArray<FPalletSlot>& GetSlots() const { return Slots; }

	/** Slots for loaded (true) or empty (false) pallets. */
	TArray<FPalletSlot> GetSlotsFor(bool bLoaded) const;

	/** True when something solid (pallet, bin, rack, ...) already sits on the slot footprint. */
	bool IsSlotOccupied(const FPalletSlot& Slot) const;

	/** First unoccupied slot of the requested kind, nearest to FromLocation. False when the zone is full. */
	bool FindFreeSlot(bool bLoaded, const FVector& FromLocation, FPalletSlot& OutSlot) const;

	/** Distance from the slot centre back to the pre-pose along the aisle (fork tips + turning margin). */
	UPROPERTY(EditAnywhere, Category = "Warehouse|Drop Zones", meta = (ClampMin = "200.0"))
	float PrePoseStandOffCm = 350.f;

	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> RackMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> LoadedPalletMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> WoodPalletMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> BinMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> GuardRailMesh;

	// Shell + lighting kit (real NVIDIA modular tiles; see the class comment). Any of these missing
	// (not yet imported) drops Rebuild back to the old flat-box shell for that piece.
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> WallTileMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> CeilingTileMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> FloorTileMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> BeamMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> LampMesh;

	// Real modular pallet-rack kit (grey frame + orange beams), replacing RackMesh (Rack_A01, a
	// different single-bay blue-painted asset) as the primary racking. RackMesh is kept as the
	// fallback if these aren't imported yet.
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> RackFrameMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> RackShelfMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> RackShieldMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> PaletteAMesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> CardBoxA01Mesh;
	UPROPERTY(EditAnywhere, Category = "Warehouse|Assets")
	TSoftObjectPtr<UStaticMesh> CardBoxA02Mesh;

private:
	UStaticMeshComponent* AddMesh(const FString& Name, UStaticMesh* Mesh, const FVector& UsdPosM, float UsdYawDeg, const FVector& Scale = FVector::OneVector, bool bDynamic = false, float MassKg = 0.f);
	UStaticMeshComponent* AddBox(const FString& Name, const FVector& UsdPosM, const FVector& SizeM, const FLinearColor& Color, bool bVisible, bool bDynamic = false, float MassKg = 0.f, bool bTrackAsObstacle = false);
	UPointLightComponent* AddPointLight(const FString& Name, const FVector& UsdPosM, float Lumens, float RadiusM, const FLinearColor& Color);
	/** Registers an axis-aligned world-space obstacle box (for path planning) at a Kit-meters centre/size, without spawning anything visible. */
	void AddObstacleBox(const FVector& UsdPosM, const FVector& SizeM);

	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> Spawned;

	UPROPERTY(Transient)
	TArray<TObjectPtr<UPointLightComponent>> SpawnedLights;

	void BuildDropZones();
	bool BuildRealShell();
	void BuildFlatShell();
	void BuildCeilingAndLighting(bool bRealShell);
	void ApplyIndoorLighting();
	void ApplyPostProcessTuning();
	void BuildRacks();
	void BuildRackRun(const FString& Name, double XM, float Yaw);
	void BuildFlatRacks();

	TArray<FPalletSlot> Slots;

	UPROPERTY(Transient)
	TArray<FLoosePallet> LoosePallets;

	TArray<FBox> ObstacleBoxes;

	UPROPERTY(Transient)
	TObjectPtr<UStaticMesh> CubeMesh;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> FlatColorMaterial;
};
