#include "Warehouse.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SceneComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "ForkliftVehicle.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "PhysicsEngine/BodyInstance.h"

namespace
{
	// Kit meters (right-handed) -> UE cm (left-handed).
	FVector UsdToUe(const FVector& M) { return FVector(M.X * 100.0, -M.Y * 100.0, M.Z * 100.0); }
}

AWarehouse::AWarehouse()
{
	PrimaryActorTick.bCanEverTick = false;
	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	// Static children (floor, walls, racks...) can only attach to a Static parent.
	Root->SetMobility(EComponentMobility::Static);
	SetRootComponent(Root);

	// The NVIDIA Digital Twin warehouse assets are authored in cm and UE is cm, so they import 1:1
	// (the Kit prototype needed a 0.01 scale on every reference; that goes away here).
	RackMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_Rack_A01.SM_Rack_A01")));
	LoadedPalletMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_LoadedPallet_A04.SM_LoadedPallet_A04")));
	WoodPalletMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_BlockPallet_A01.SM_BlockPallet_A01")));
	BinMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_Bin_A06.SM_Bin_A06")));
	GuardRailMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_GuardRail_A01.SM_GuardRail_A01")));

	// Shell + lighting kit, pulled from Isaac Sim's own Simple_Warehouse training scene (see the class
	// comment and Scripts/download_assets.py).
	WallTileMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_WallA_6M.SM_WallA_6M")));
	CeilingTileMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_CeilingA_6X6.SM_CeilingA_6X6")));
	FloorTileMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_Floor02.SM_Floor02")));
	BeamMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_BeamA_9M.SM_BeamA_9M")));
	LampMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_LampCeilingA_04.SM_LampCeilingA_04")));

	RackFrameMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_RackFrame_03.SM_RackFrame_03")));
	RackShelfMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_RackShelf_01.SM_RackShelf_01")));
	RackShieldMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_Rackshield_02.SM_Rackshield_02")));
	PaletteAMesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_PaletteA_01.SM_PaletteA_01")));
	CardBoxA01Mesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_CardBoxA_01.SM_CardBoxA_01")));
	CardBoxA02Mesh = TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(TEXT("/Game/Warehouse/Meshes/SM_CardBoxA_02.SM_CardBoxA_02")));
}

void AWarehouse::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	Rebuild();
}

void AWarehouse::BeginPlay()
{
	Super::BeginPlay();

	// The geometry is transient (rebuilt from code, never saved in the level). A standalone game runs
	// OnConstruction when it loads the map, but Play-In-Editor copies the editor world and does NOT rerun
	// it, so the copy has no floor or walls and everything falls out of the world. Rebuild if empty.
	if (Spawned.IsEmpty())
	{
		Rebuild();
	}
}

UStaticMeshComponent* AWarehouse::AddMesh(const FString& Name, UStaticMesh* Mesh, const FVector& UsdPosM, float UsdYawDeg, const FVector& Scale, bool bDynamic, float MassKg)
{
	UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(this, *Name, RF_Transient);
	Comp->SetStaticMesh(Mesh);
	Comp->SetMobility(bDynamic ? EComponentMobility::Movable : EComponentMobility::Static);
	Comp->SetupAttachment(GetRootComponent());
	// USD yaw is CCW about +Z in a right-handed frame; UE yaw is CW, so the sign flips with Y.
	Comp->SetRelativeLocationAndRotation(UsdToUe(UsdPosM), FRotator(0.f, -UsdYawDeg, 0.f));
	Comp->SetRelativeScale3D(Scale);
	if (bDynamic)
	{
		Comp->SetCollisionProfileName(TEXT("PhysicsActor"));
		Comp->SetPhysMaterialOverride(AForkliftVehicle::GetCarryGripMaterial());
	}
	Comp->RegisterComponent();
	if (bDynamic)
	{
		Comp->BodyInstance.SetMassOverride(MassKg, true);
		Comp->SetSimulatePhysics(true);
	}
	Spawned.Add(Comp);
	return Comp;
}

UStaticMeshComponent* AWarehouse::AddBox(const FString& Name, const FVector& UsdPosM, const FVector& SizeM, const FLinearColor& Color, bool bVisible, bool bDynamic, float MassKg, bool bTrackAsObstacle)
{
	if (bTrackAsObstacle)
	{
		AddObstacleBox(UsdPosM, SizeM);
	}
	UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(this, *Name, RF_Transient);
	Comp->SetStaticMesh(CubeMesh);
	Comp->SetMobility(bDynamic ? EComponentMobility::Movable : EComponentMobility::Static);
	Comp->SetupAttachment(GetRootComponent());
	// /Engine/BasicShapes/Cube is 100 cm on a side, matching the USD Cube with size=1 m.
	Comp->SetRelativeLocation(UsdToUe(UsdPosM));
	Comp->SetRelativeScale3D(FVector(SizeM.X, SizeM.Y, SizeM.Z));
	Comp->SetCollisionProfileName(bDynamic ? TEXT("PhysicsActor") : TEXT("BlockAll"));
	Comp->SetVisibility(bVisible);
	Comp->SetCastShadow(bVisible);

	if (FlatColorMaterial)
	{
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(FlatColorMaterial, Comp);
		Mid->SetVectorParameterValue(TEXT("Color"), Color);
		Comp->SetMaterial(0, Mid);
	}

	Comp->RegisterComponent();
	if (bDynamic)
	{
		Comp->BodyInstance.SetMassOverride(MassKg, true);
		Comp->SetSimulatePhysics(true);
	}
	Spawned.Add(Comp);
	return Comp;
}

void AWarehouse::AddObstacleBox(const FVector& UsdPosM, const FVector& SizeM)
{
	const FVector CenterWorld = GetActorTransform().TransformPosition(UsdToUe(UsdPosM));
	// SizeM is the cube's per-axis scale (metres) applied to a 100 cm base cube -> half-extent in cm.
	const FVector HalfExtentWorld = SizeM * 100.0 * 0.5;
	ObstacleBoxes.Add(FBox(CenterWorld - HalfExtentWorld, CenterWorld + HalfExtentWorld));
}

UPointLightComponent* AWarehouse::AddPointLight(const FString& Name, const FVector& UsdPosM, float Lumens, float RadiusM, const FLinearColor& Color)
{
	UPointLightComponent* Light = NewObject<UPointLightComponent>(this, *Name, RF_Transient);
	Light->SetMobility(EComponentMobility::Static);
	Light->SetupAttachment(GetRootComponent());
	Light->SetRelativeLocation(UsdToUe(UsdPosM));
	Light->SetIntensityUnits(ELightUnits::Lumens);
	Light->SetIntensity(Lumens);
	Light->SetAttenuationRadius(RadiusM * 100.0);
	// A point source this close to the shelf-top loads (~4 m) spikes hard under inverse-square falloff
	// and blows them out even though the floor (6-9 m away) reads fine; a bit of source radius softens
	// that near-field peak without changing the light's reach.
	Light->SetSourceRadius(20.0f);
	Light->SetLightColor(Color);
	Light->SetCastShadows(bOverheadLightsCastShadows);
	Light->RegisterComponent();
	SpawnedLights.Add(Light);
	return Light;
}

void AWarehouse::Rebuild()
{
	for (UStaticMeshComponent* Old : Spawned)
	{
		if (Old)
		{
			Old->DestroyComponent();
		}
	}
	Spawned.Reset();

	for (UPointLightComponent* OldLight : SpawnedLights)
	{
		if (OldLight)
		{
			OldLight->DestroyComponent();
		}
	}
	SpawnedLights.Reset();

	ObstacleBoxes.Reset();

	CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	FlatColorMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Warehouse/Materials/M_FlatColor.M_FlatColor"));
	if (!FlatColorMaterial)
	{
		FlatColorMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	}

	// --- Shell (Kit: _create_shell) -------------------------------------------------------------
	// 36 x 24 m: 50% bigger than the original 24 x 16 m Kit footprint (more aisle/navigation room), half
	// extents 18 x 12 m. Real modular tiles when the warehouse-kit meshes are imported (BuildRealShell),
	// otherwise the old flat-colour boxes (BuildFlatShell) so the actor never renders empty.
	const bool bRealShell = BuildRealShell();
	if (!bRealShell)
	{
		BuildFlatShell();
	}
	BuildCeilingAndLighting(bRealShell);
	ApplyIndoorLighting();
	ApplyPostProcessTuning();

	// --- Racks: two multi-bay runs, west and east of the centre aisle -----------------------------
	BuildRacks();

	// Outer walls as obstacle boxes for path planning, registered once regardless of which shell
	// variant (real tiled kit or flat fallback) actually rendered them. Must track BuildRealShell's
	// HalfXM/HalfYM (18, 12); 9 m tall covers both the real 9 m shell and the flat 3 m fallback.
	{
		constexpr double HalfXM = 18.0, HalfYM = 12.0;
		AddObstacleBox(FVector(-HalfXM, 0.0, 4.5), FVector(0.5, HalfYM * 2.0 + 1.0, 9.0));
		AddObstacleBox(FVector(HalfXM, 0.0, 4.5), FVector(0.5, HalfYM * 2.0 + 1.0, 9.0));
		AddObstacleBox(FVector(0.0, HalfYM, 4.5), FVector(HalfXM * 2.0 + 1.0, 0.5, 9.0));
		AddObstacleBox(FVector(0.0, -HalfYM, 4.5), FVector(HalfXM * 2.0 + 1.0, 0.5, 9.0));
	}

	// --- Pallets, bins, guard rails ---------------------------------------------------------------
	// (A static LoadedPallet_01 with a solid box collision used to stand here; it could not be lifted and only confused perception, so the
	//  loaded pallet is now the one liftable loose pallet below.)
	LoosePallets.Reset();
	if (UStaticMesh* Mesh = WoodPalletMesh.LoadSynchronous())
	{
		// A dynamic pallet only in a running game: it would fall over in the editor viewport otherwise.
		const bool bDynamic = (bDynamicLoosePallet || FParse::Param(FCommandLine::Get(), TEXT("ForkliftPalletTest")) ||
			FParse::Param(FCommandLine::Get(), TEXT("ForkliftAutoPickupTest")) ||
			FParse::Param(FCommandLine::Get(), TEXT("ForkliftMissionTest"))) && GetWorld() && GetWorld()->IsGameWorld();

		// Element 0 keeps the exact Kit position (-3.5, 2.5): ForkliftGameMode's PalletTest/
		// AutoPickupTest/batch harness all assume exactly one loose pallet there via GetLoosePallet().
		static const FVector2D PalletSpotsM[] = {
			FVector2D(-3.5, 2.5), FVector2D(-3.5, -2.5), FVector2D(-1.0, 7.5),
			FVector2D(3.0, -6.5), FVector2D(3.5, 2.5), FVector2D(3.5, -2.5),
		};
		// Pure visual variety - the detector already trains on colour-randomised pallets
		// (SyntheticDataGenerator::RandomiseMaterials), so this carries no class/detection meaning.
		// Flattens the mesh's baked wood material to a solid tint via FlatColorMaterial, the same
		// idiom BuildRealShell's TintSlot lambda uses for the floor/walls.
		static const FLinearColor PalletColors[] = {
			FLinearColor(0.36f, 0.20f, 0.08f),  // natural wood
			FLinearColor(0.05f, 0.20f, 0.55f),  // blue
			FLinearColor(0.55f, 0.05f, 0.05f),  // red
			FLinearColor(0.60f, 0.45f, 0.02f),  // amber
			FLinearColor(0.04f, 0.35f, 0.10f),  // green
			FLinearColor(0.08f, 0.08f, 0.09f),  // black
		};

		for (int32 i = 0; i < UE_ARRAY_COUNT(PalletSpotsM); ++i)
		{
			UStaticMeshComponent* Comp = AddMesh(FString::Printf(TEXT("WoodPallet_%02d"), i), Mesh,
				FVector(PalletSpotsM[i].X, PalletSpotsM[i].Y, 0.0), 0.f, FVector::OneVector, bDynamic, 25.f);

			if (FlatColorMaterial)
			{
				const FLinearColor& Color = PalletColors[i % UE_ARRAY_COUNT(PalletColors)];
				for (int32 SlotIdx = 0; SlotIdx < Comp->GetNumMaterials(); ++SlotIdx)
				{
					UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(FlatColorMaterial, Comp);
					Mid->SetVectorParameterValue(TEXT("Color"), Color);
					Comp->SetMaterial(SlotIdx, Mid);
				}
			}

			LoosePallets.Add(FLoosePallet{ Comp, false, Comp->GetComponentTransform() });
		}

		// Loaded pallet: a real wood pallet body (insertable pockets, 220 kg) dressed with the NVIDIA loaded-pallet mesh the detector was
		// trained on. The wood pallet is hidden but keeps its collision; the loaded mesh rides on it as a query-only child (the lidar and
		// the cameras see it, it adds no physics body). It belongs in the east (loaded) drop zone. Kit (2.5, 0) = UE (250, 0).
		{
			UStaticMesh* LoadedVisual = LoadedPalletMesh.LoadSynchronous();
			UStaticMeshComponent* Comp = AddMesh(TEXT("LoadedWoodPallet_00"), Mesh, FVector(2.5, 0.0, 0.0), 0.f, FVector::OneVector, bDynamic, 220.f);
			if (LoadedVisual)
			{
				Comp->SetVisibility(false);
				const FBoxSphereBounds PB = Mesh->GetBounds();
				const FBoxSphereBounds LB = LoadedVisual->GetBounds();
				const double Scale = FMath::Min((PB.BoxExtent.X * 2.0) / (LB.BoxExtent.X * 2.0), (PB.BoxExtent.Y * 2.0) / (LB.BoxExtent.Y * 2.0));
				UStaticMeshComponent* Visual = NewObject<UStaticMeshComponent>(this, TEXT("LoadedPalletVisual"), RF_Transient);
				Visual->SetStaticMesh(LoadedVisual);
				Visual->SetMobility(bDynamic ? EComponentMobility::Movable : EComponentMobility::Static);
				Visual->SetupAttachment(Comp);
				Visual->SetRelativeScale3D(FVector(Scale));
				const FVector Loc(PB.Origin.X - Scale * LB.Origin.X, PB.Origin.Y - Scale * LB.Origin.Y, (PB.Origin.Z - PB.BoxExtent.Z) - Scale * (LB.Origin.Z - LB.BoxExtent.Z));
				Visual->SetRelativeLocationAndRotation(Loc, FRotator::ZeroRotator);
				Visual->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
				Visual->SetCollisionResponseToAllChannels(ECR_Block);
				Visual->RegisterComponent();
				Spawned.Add(Visual);
			}
			FLoosePallet Loaded{ Comp, false, Comp->GetComponentTransform() };
			Loaded.bLoaded = true;
			LoosePallets.Add(Loaded);
		}
	}

	// --- Physics test box (Kit: /World/Props/PhysicsTestBox) -------------------------------------
	// The prototype's 50 cm test cube drops into the middle of the aisle; scripted missions and recordings do not want that obstacle there.
	const bool bScripted = FParse::Param(FCommandLine::Get(), TEXT("ForkliftMissionTest")) || FParse::Param(FCommandLine::Get(), TEXT("ForkliftMission")) ||
		FParse::Param(FCommandLine::Get(), TEXT("ForkliftRecord")) || FCString::Stristr(FCommandLine::Get(), TEXT("-ForkliftRecord=")) != nullptr;
	if (bSpawnPhysicsTestBox && !bScripted && GetWorld() && GetWorld()->IsGameWorld())
	{
		AddBox(TEXT("PhysicsTestBox"), FVector(0.0, 0.0, 3.0), FVector(0.5, 0.5, 0.5), FLinearColor(0.95f, 0.45f, 0.10f), true, true, 5.f);
	}

	BuildDropZones();
}

bool AWarehouse::BuildRealShell()
{
	UStaticMesh* WallTile = WallTileMesh.LoadSynchronous();
	UStaticMesh* CeilingTile = CeilingTileMesh.LoadSynchronous();
	UStaticMesh* FloorTile = FloorTileMesh.LoadSynchronous();
	if (!WallTile || !CeilingTile || !FloorTile)
	{
		return false;
	}

	// Real NVIDIA modular kit - Isaac Sim's own Simple_Warehouse training scene (warehouse.usd /
	// warehouse_multiple_shelves.usd) is built from these exact tiles at these exact proportions: 6x6 m
	// floor/ceiling panels, and 6 m wall tiles stacked 3 high for the kit's true 9 m wall height
	// (WallHeightM) - not the old 3 m flat box. SM_WallA_6M's own pivot sits on its outer face (local X
	// in [0, 0.2 m]) with its length centred (local Y in [-3, 3 m]), so placing it at the shell boundary
	// keeps its thickness bulging outward and its inner face flush with the old boundary - the
	// rack/pallet/dropzone coordinates elsewhere in this file assume that 18 x 12 m clear interior and
	// don't need to change.
	constexpr double HalfXM = 18.0, HalfYM = 12.0, TileM = 6.0, WallTierM = 3.0;
	constexpr int32 WallTiers = 3;
	constexpr double WallHeightM = WallTierM * WallTiers;  // 9 m.
	const int32 CountX = FMath::RoundToInt(HalfXM * 2.0 / TileM);  // 6
	const int32 CountY = FMath::RoundToInt(HalfYM * 2.0 / TileM);  // 4

	// Recolour one wall/floor tile's main surface slot (its material name contains SlotHint) via a MID off
	// the same flat-colour master AddBox uses, rather than trusting the imported MI's own colour - the
	// reference photo's dark blue-grey floor and two-tone (white brick base / orange upper) walls don't
	// match a flat texture-average sample the way the neutral shell materials did.
	const auto TintSlot = [this](UStaticMeshComponent* Comp, const TCHAR* SlotHint, const FLinearColor& Color)
	{
		if (!Comp || !FlatColorMaterial)
		{
			return;
		}
		for (int32 i = 0; i < Comp->GetNumMaterials(); ++i)
		{
			if (UMaterialInterface* Mat = Comp->GetMaterial(i); Mat && Mat->GetName().Contains(SlotHint))
			{
				UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(FlatColorMaterial, Comp);
				Mid->SetVectorParameterValue(TEXT("Color"), Color);
				Comp->SetMaterial(i, Mid);
				return;
			}
		}
	};
	const FLinearColor FloorColor(0.035f, 0.045f, 0.06f);       // dark blue-grey sealed concrete
	const FLinearColor WallBrickColor(0.82f, 0.80f, 0.74f);     // lower tier: white brick
	const FLinearColor WallOrangeColor(0.55f, 0.20f, 0.03f);    // upper tiers: painted/corrugated orange

	for (int32 ix = 0; ix < CountX; ++ix)
	{
		const double X = -HalfXM + TileM * (ix + 0.5);
		for (int32 iy = 0; iy < CountY; ++iy)
		{
			const double Y = -HalfYM + TileM * (iy + 0.5);
			TintSlot(AddMesh(FString::Printf(TEXT("FloorTile_%d_%d"), ix, iy), FloorTile, FVector(X, Y, 0.0), 0.f), TEXT("Floor"), FloorColor);
			if (bEnclosedCeiling)
			{
				AddMesh(FString::Printf(TEXT("CeilingTile_%d_%d"), ix, iy), CeilingTile, FVector(X, Y, WallHeightM), 0.f);
			}
		}
	}

	// Match the orientations used by NVIDIA's Simple_Warehouse scene: the wall tile is one-sided, so its
	// finished face must point into the room. Its 20 cm thickness therefore extends inward from the shell
	// boundary (east/west use 180/0 degrees and north/south use -90/+90). Reversing these yaws
	// makes the structural trim visible but back-face-culls the main wall surface, which looks like a black
	// void even in an unlit base-colour capture.
	for (int32 Tier = 0; Tier < WallTiers; ++Tier)
	{
		const double Z = WallTierM * Tier;
		const FLinearColor& TierColor = (Tier == 0) ? WallBrickColor : WallOrangeColor;
		for (int32 iy = 0; iy < CountY; ++iy)
		{
			const double Y = -HalfYM + TileM * (iy + 0.5);
			TintSlot(AddMesh(FString::Printf(TEXT("EastWall_%d_%d"), Tier, iy), WallTile, FVector(HalfXM, Y, Z), 180.f), TEXT("WallA"), TierColor);
			TintSlot(AddMesh(FString::Printf(TEXT("WestWall_%d_%d"), Tier, iy), WallTile, FVector(-HalfXM, Y, Z), 0.f), TEXT("WallA"), TierColor);
		}
		for (int32 ix = 0; ix < CountX; ++ix)
		{
			const double X = -HalfXM + TileM * (ix + 0.5);
			TintSlot(AddMesh(FString::Printf(TEXT("NorthWall_%d_%d"), Tier, ix), WallTile, FVector(X, HalfYM, Z), -90.f), TEXT("WallA"), TierColor);
			TintSlot(AddMesh(FString::Printf(TEXT("SouthWall_%d_%d"), Tier, ix), WallTile, FVector(X, -HalfYM, Z), 90.f), TEXT("WallA"), TierColor);
		}
	}

	// Roof beams: visual only, spaced along the length and non-uniformly scaled to span the 24 m width
	// (the kit's own 9 m truss doesn't divide that evenly).
	if (bEnclosedCeiling)
	{
		if (UStaticMesh* Beam = BeamMesh.LoadSynchronous())
		{
			const double BeamXs[] = { -15.0, -9.0, -3.0, 3.0, 9.0, 15.0 };
			for (int32 i = 0; i < UE_ARRAY_COUNT(BeamXs); ++i)
			{
				AddMesh(FString::Printf(TEXT("RoofBeam_%d"), i), Beam, FVector(BeamXs[i], -HalfYM, WallHeightM - 0.2), 90.f,
					FVector((HalfYM * 2.0) / 9.0, 1.0, 1.0));
			}
		}
	}

	return true;
}

void AWarehouse::BuildFlatShell()
{
	// Fallback when the real warehouse-kit meshes (Scripts/setup.py) aren't imported yet: the old flat
	// 3 m box shell, so the actor still renders a closed room instead of nothing.
	AddBox(TEXT("Floor"), FVector(0.0, 0.0, -0.1), FVector(36.0, 24.0, 0.2), FLinearColor(0.24f, 0.26f, 0.29f), true);

	struct FWall { const TCHAR* Name; FVector Pos; FVector Size; };
	const FWall Walls[] = {
		{ TEXT("NorthWall"), FVector(0.0, 12.1, 1.5), FVector(36.0, 0.2, 3.0) },
		{ TEXT("SouthWall"), FVector(0.0, -12.1, 1.5), FVector(36.0, 0.2, 3.0) },
		{ TEXT("EastWall"), FVector(18.1, 0.0, 1.5), FVector(0.2, 24.0, 3.0) },
		{ TEXT("WestWall"), FVector(-18.1, 0.0, 1.5), FVector(0.2, 24.0, 3.0) },
	};
	for (const FWall& Wall : Walls)
	{
		AddBox(Wall.Name, Wall.Pos, Wall.Size, FLinearColor(0.50f, 0.52f, 0.55f), true);
	}

	if (bEnclosedCeiling)
	{
		AddBox(TEXT("Ceiling"), FVector(0.0, 0.0, 3.1), FVector(36.0, 24.0, 0.2), FLinearColor(0.14f, 0.15f, 0.17f), true);
	}
}

void AWarehouse::BuildCeilingAndLighting(bool bRealShell)
{
	if (!bOverheadLights)
	{
		return;
	}

	// Matches whichever ceiling actually got built: the real kit's 9 m height, or the old 3 m flat box.
	const double CeilingZM = bRealShell ? 9.0 : 3.1;

	// 4 columns x 3 rows, spanning the aisle and both rack banks.
	const double ColsXM[] = { -12.0, -4.0, 4.0, 12.0 };
	const double RowsYM[] = { -8.0, 0.0, 8.0 };
	const FLinearColor FixtureColor(1.0f, 1.0f, 0.96f);
	UStaticMesh* Lamp = LampMesh.LoadSynchronous();
	int32 Index = 0;
	for (double XM : ColsXM)
	{
		for (double YM : RowsYM)
		{
			++Index;
			if (Lamp)
			{
				// SM_LampCeilingA_04's pivot is its ceiling mount point; it hangs ~3 m down to its lens.
				AddMesh(FString::Printf(TEXT("LightFixture_%02d"), Index), Lamp, FVector(XM, YM, CeilingZM), 0.f);
			}
			else
			{
				AddBox(FString::Printf(TEXT("LightFixture_%02d"), Index), FVector(XM, YM, CeilingZM - 0.1), FVector(1.0, 0.3, 0.08), FixtureColor, true);
			}
			AddPointLight(FString::Printf(TEXT("OverheadLight_%02d"), Index), FVector(XM, YM, CeilingZM - 3.0), OverheadLightLumens, OverheadLightRadiusM, FixtureColor);
		}
	}
}

void AWarehouse::ApplyIndoorLighting()
{
	if (!bDisableOutdoorSky || !GetWorld())
	{
		return;
	}

	// Warehouse.umap places a DirectionalLight/SkyLight/SkyAtmosphere/ExponentialHeightFog for the old
	// open-sky look. Dim (not hide) the sun/sky ambient here rather than requiring the map itself to be
	// re-edited: fully hiding them left the building's OUTSIDE (roof, and anything seen from an exterior
	// camera angle) pure black, since the walls block the interior fixtures from reaching out there too.
	// A low-intensity sun/sky gives the exterior basic, unlit-looking ambient without washing out the
	// enclosed interior, which stays dominated by the warehouse's own fixtures either way (the walls and
	// ceiling block it from getting in). SkyAtmosphere/fog are left alone - they're what actually draws a
	// sky outside the building rather than a void, and don't flood the interior on their own.
	for (TActorIterator<ADirectionalLight> It(GetWorld()); It; ++It)
	{
		if (UDirectionalLightComponent* Comp = It->GetComponent())
		{
			Comp->SetVisibility(true);
			Comp->SetIntensity(1.5f);
		}
	}
	for (TActorIterator<ASkyLight> It(GetWorld()); It; ++It)
	{
		if (USkyLightComponent* Comp = It->GetLightComponent())
		{
			Comp->SetVisibility(true);
			Comp->SetIntensity(0.3f);
		}
	}
}

void AWarehouse::ApplyPostProcessTuning()
{
	if (!GetWorld())
	{
		return;
	}

	// Warehouse.umap has no PostProcessVolume, so the level renders with the engine's generic defaults -
	// tuned for a typical bright outdoor scene, not this small, artificially-lit, mostly-dark interior.
	// Against that backdrop the default Bloom intensity (~0.675) blooms any moderately-lit surface (like
	// the cardboard on a loaded pallet two beam-levels up, closer to a fixture than the floor) into a
	// blown-out white blob even though its base colour is correct (confirmed via an unlit capture).
	// Toning bloom down and nudging exposure compensation negative fixes that without touching the
	// lights themselves. Reuses an existing volume if the level already has one; spawns an unbound one
	// (affects the whole level regardless of its own bounds) otherwise.
	APostProcessVolume* Volume = nullptr;
	for (TActorIterator<APostProcessVolume> It(GetWorld()); It; ++It)
	{
		Volume = *It;
		break;
	}
	if (!Volume)
	{
		Volume = GetWorld()->SpawnActor<APostProcessVolume>();
		if (Volume)
		{
			Volume->SetActorLabel(TEXT("WarehousePostProcess"));
		}
	}
	if (!Volume)
	{
		return;
	}

	Volume->bUnbound = true;
	Volume->BlendWeight = 1.f;
	Volume->Settings.bOverride_BloomIntensity = true;
	Volume->Settings.BloomIntensity = 0.2f;
	Volume->Settings.bOverride_AutoExposureBias = true;
	Volume->Settings.AutoExposureBias = -0.5f;
}

void AWarehouse::BuildRacks()
{
	if (RackFrameMesh.LoadSynchronous() && RackShelfMesh.LoadSynchronous() && RackShieldMesh.LoadSynchronous())
	{
		BuildRackRun(TEXT("Rack_West"), -7.0, 90.f);
		BuildRackRun(TEXT("Rack_East"), 7.0, 90.f);
	}
	else
	{
		BuildFlatRacks();
	}
}

void AWarehouse::BuildRackRun(const FString& Name, double XM, float Yaw)
{
	// A real multi-bay pallet-rack run (Isaac Sim's own Simple_Warehouse "multiple shelves" scene: grey
	// SM_RackFrame_03 uprights, orange SM_RackShelf_01 beams, SM_Rackshield_02 end guards) instead of the
	// single-bay, blue-painted DigitalTwin Rack_A01. 5 uprights spaced 4 m apart (the real modular unit -
	// SM_RackFrame_03 is 1 m deep, SM_RackShelf_01 spans exactly 4 m) give 4 bays; each upright is 2
	// stacked 3 m frame sections (6 m tall) with 3 shelf levels per bay. Depth (Kit X) is the frame's own
	// local Y after the 90 deg yaw; length (Kit Y) is the frame's local X (thickness) / shelf's local X
	// (its 4 m span) after the same rotation - one consistent yaw works for the whole run.
	constexpr double PostSpacingM = 4.0;
	constexpr int32 PostCount = 5;  // 4 bays
	constexpr double RunStartYM = -8.0;
	constexpr double FrameTierM = 3.0;
	constexpr int32 FrameTiers = 2;  // 6 m tall
	// Four levels match the denser vertical rhythm of NVIDIA's reference warehouse. The 1.45 m pitch
	// still leaves enough room for a pallet plus two courses of the 48.4 cm cartons.
	const double ShelfLevelsM[] = { 0.05, 1.50, 2.95, 4.40 };

	UStaticMesh* Frame = RackFrameMesh.LoadSynchronous();
	UStaticMesh* Shelf = RackShelfMesh.LoadSynchronous();
	UStaticMesh* Shield = RackShieldMesh.LoadSynchronous();
	UStaticMesh* Pallet = PaletteAMesh.LoadSynchronous();
	UStaticMesh* Box1 = CardBoxA01Mesh.LoadSynchronous();
	UStaticMesh* Box2 = CardBoxA02Mesh.LoadSynchronous();
	UStaticMesh* Tote = BinMesh.LoadSynchronous();
	UStaticMesh* LoadedPallet = LoadedPalletMesh.LoadSynchronous();

	for (int32 i = 0; i < PostCount; ++i)
	{
		const double Y = RunStartYM + PostSpacingM * i;
		for (int32 Tier = 0; Tier < FrameTiers; ++Tier)
		{
			AddMesh(FString::Printf(TEXT("%s_Frame_%d_%d"), *Name, i, Tier), Frame, FVector(XM, Y, FrameTierM * Tier), Yaw);
		}
	}

	// These assets pivot at their base. Build each bay as a small, deliberately composed storage scene
	// instead of repeating three isolated boxes on every level. The pattern is deterministic (so the
	// training world does not rearrange itself on every construction) but varies by rack, bay and level.
	constexpr double ShelfTopClearanceM = 0.03;  // SM_RackShelf_01's own top surface sits ~2.5 cm above its pivot
	const double PalletHeightM = Pallet ? Pallet->GetBoundingBox().GetSize().Z * 0.01 : 0.211;
	constexpr double CartonHeightM = 0.484;
	const bool bEastRun = XM > 0.0;
	for (int32 Bay = 0; Bay < PostCount - 1; ++Bay)
	{
		const double BayY = RunStartYM + PostSpacingM * Bay + PostSpacingM / 2.0;
		for (int32 Level = 0; Level < UE_ARRAY_COUNT(ShelfLevelsM); ++Level)
		{
			AddMesh(FString::Printf(TEXT("%s_Shelf_%d_%d"), *Name, Bay, Level), Shelf, FVector(XM, BayY, ShelfLevelsM[Level]), Yaw);

			const double ShelfTopZ = ShelfLevelsM[Level] + ShelfTopClearanceM;
			int32 StockIndex = 0;
			auto AddStock = [&](const TCHAR* Kind, UStaticMesh* Mesh, double AlongM, double DepthM,
				double BaseZM, float ItemYaw, const FVector& Scale)
			{
				if (!Mesh)
				{
					return;
				}
				AddMesh(FString::Printf(TEXT("%s_%s_B%d_L%d_%02d"), *Name, Kind, Bay, Level, StockIndex++),
					Mesh, FVector(XM + DepthM, BayY + AlongM, BaseZM), ItemYaw, Scale);
			};
			auto AddPalletAt = [&](double AlongM, double BaseZM, int32 StackCount)
			{
				for (int32 Stack = 0; Stack < StackCount; ++Stack)
				{
					AddStock(TEXT("Pallet"), Pallet, AlongM, 0.0, BaseZM + PalletHeightM * Stack,
						(Stack % 2 == 0) ? 0.f : 180.f, FVector::OneVector);
				}
			};
			auto AddCartonRow = [&](int32 Count, double CenterM, double SpacingM, double BaseZM,
				float RowYaw, const FVector& Scale)
			{
				const double StartM = CenterM - SpacingM * (Count - 1) * 0.5;
				for (int32 Item = 0; Item < Count; ++Item)
				{
					// A few centimetres of stable stagger keeps long rows from looking computer-perfect.
					const double DepthJitterM = ((Item + Bay + Level) % 3 - 1) * 0.018;
					const float YawJitter = static_cast<float>(((Item * 2 + Bay + (bEastRun ? 1 : 0)) % 5) - 2) * 0.6f;
					AddStock(TEXT("Carton"), Box1, StartM + SpacingM * Item, DepthJitterM,
						BaseZM, RowYaw + YawJitter, Scale);
				}
			};
			auto AddToteStack = [&](double CenterM, double BaseZM, int32 Columns, int32 ToteLayers)
			{
				if (!Tote)
				{
					return;
				}

				// Bin_A06's pivot is not guaranteed to be at the bottom. Derive its footprint, height and
				// pivot correction from the imported mesh so every tote physically rests on the surface below.
				// Two complete depth rows replace the old checkerboard layout and form one believable block.
				const FVector ToteScale(0.90, 0.90, 0.90);
				const FBox ToteBounds = Tote->GetBoundingBox();
				const double ToteDepthM = ToteBounds.GetSize().X * ToteScale.X * 0.01;
				const double ToteWidthM = ToteBounds.GetSize().Y * ToteScale.Y * 0.01;
				const double ToteHeightM = ToteBounds.GetSize().Z * ToteScale.Z * 0.01;
				const double PivotToBaseM = -ToteBounds.Min.Z * ToteScale.Z * 0.01;
				const double PivotToCenterXM = ToteBounds.GetCenter().X * ToteScale.X * 0.01;
				const double PivotToCenterYM = ToteBounds.GetCenter().Y * ToteScale.Y * 0.01;
				const double ColumnPitchM = FMath::Max(0.01, ToteWidthM - 0.004);
				const double LayerPitchM = FMath::Max(0.01, ToteHeightM - 0.002);
				for (int32 Layer = 0; Layer < ToteLayers; ++Layer)
				{
					for (int32 Column = 0; Column < Columns; ++Column)
					{
						for (int32 DepthRow = 0; DepthRow < 2; ++DepthRow)
						{
							const double CenterDepthM = (DepthRow == 0 ? -0.5 : 0.5) * ToteDepthM;
							const double CenterAlongM = CenterM + (Column - (Columns - 1) * 0.5) * ColumnPitchM;
							AddStock(TEXT("Tote"), Tote,
								CenterAlongM + PivotToCenterYM,
								CenterDepthM - PivotToCenterXM,
								BaseZM + PivotToBaseM + LayerPitchM * Layer,
								0.f, ToteScale);
						}
					}
				}
			};

			const int32 Variant = (Bay + (bEastRun ? 2 : 0)) % 4;
			if (Level == 3)
			{
				// Top tiers carry the broad, tightly packed carton silhouettes seen in the reference.
				if (Variant == 1 && LoadedPallet)
				{
					AddStock(TEXT("LoadedPallet"), LoadedPallet, -0.72, 0.0, ShelfTopZ, 0.f, FVector(0.88, 0.88, 0.88));
					AddStock(TEXT("LoadedPallet"), LoadedPallet, 0.72, 0.0, ShelfTopZ, 180.f, FVector(0.88, 0.88, 0.88));
				}
				else
				{
					AddPalletAt(-1.18, ShelfTopZ, 1);
					AddPalletAt(0.0, ShelfTopZ, 1);
					AddPalletAt(1.18, ShelfTopZ, 1);
					const int32 BottomCount = (Variant == 3) ? 6 : 7;
					const int32 TopCount = (Variant == 0) ? 5 : 4;
					AddCartonRow(BottomCount, 0.0, 0.51, ShelfTopZ + PalletHeightM, 0.f, FVector::OneVector);
					AddCartonRow(TopCount, (Variant == 2) ? -0.38 : 0.20, 0.51,
						ShelfTopZ + PalletHeightM + CartonHeightM, 0.f, FVector::OneVector);
				}
			}
			else if (Level == 2)
			{
				if (Variant == 0 || Variant == 3)
				{
					AddPalletAt(-0.62, ShelfTopZ, 1);
					AddPalletAt(0.62, ShelfTopZ, 1);
					AddCartonRow(5, -0.28, 0.51, ShelfTopZ + PalletHeightM, 0.f, FVector::OneVector);
					AddCartonRow(3, 0.25, 0.51, ShelfTopZ + PalletHeightM + CartonHeightM,
						0.f, FVector::OneVector);
				}
				else if (Variant == 1 && LoadedPallet)
				{
					AddStock(TEXT("LoadedPallet"), LoadedPallet, -0.72, 0.0, ShelfTopZ, 0.f, FVector(0.88, 0.88, 0.88));
					AddStock(TEXT("LoadedPallet"), LoadedPallet, 0.72, 0.0, ShelfTopZ, 0.f, FVector(0.88, 0.88, 0.88));
				}
				else
				{
					AddPalletAt(-1.05, ShelfTopZ, 1);
					AddCartonRow(4, 0.65, 0.51, ShelfTopZ, 0.f, FVector(0.96, 0.96, 0.96));
					AddCartonRow(2, 0.90, 0.51, ShelfTopZ + CartonHeightM, 0.f, FVector(0.96, 0.96, 0.96));
				}
			}
			else if (Level == 1)
			{
				if (Variant == 0 || Variant == 2)
				{
					AddPalletAt(-0.95, ShelfTopZ, 1);
					AddToteStack(-0.95, ShelfTopZ + PalletHeightM, 3, (Variant == 0) ? 3 : 2);
					AddPalletAt(0.95, ShelfTopZ, 1);
					AddCartonRow(2, 0.95, 0.52, ShelfTopZ + PalletHeightM, 0.f, FVector::OneVector);
				}
				else if (Variant == 1)
				{
					AddPalletAt(-1.12, ShelfTopZ, 2);
					AddCartonRow(4, 0.72, 0.51, ShelfTopZ, 0.f, FVector::OneVector);
					if (Box2)
					{
						AddStock(TEXT("OpenCarton"), Box2, 1.48, 0.02, ShelfTopZ, 180.f, FVector(0.86, 0.86, 0.86));
					}
				}
				else
				{
					AddPalletAt(-0.65, ShelfTopZ, 1);
					AddCartonRow(3, -0.65, 0.51, ShelfTopZ + PalletHeightM, 0.f, FVector::OneVector);
					AddPalletAt(1.12, ShelfTopZ, 1);  // purposeful breathing room / empty pallet
				}
			}
			else
			{
				// Picking height is intentionally lighter: totes, short carton groups and empty pallets.
				if (Variant == 0)
				{
					AddPalletAt(-1.12, ShelfTopZ, 2);
					AddPalletAt(0.15, ShelfTopZ, 1);
					AddToteStack(1.22, ShelfTopZ, 2, 2);
				}
				else if (Variant == 1)
				{
					AddPalletAt(-0.95, ShelfTopZ, 1);
					AddToteStack(-0.95, ShelfTopZ + PalletHeightM, 3, 3);
					AddPalletAt(0.95, ShelfTopZ, 1);
				}
				else if (Variant == 2)
				{
					AddCartonRow(4, -0.30, 0.52, ShelfTopZ, 0.f, FVector(0.92, 0.92, 0.92));
					if (Box2)
					{
						AddStock(TEXT("OpenCarton"), Box2, 1.25, 0.02, ShelfTopZ, 180.f, FVector(0.80, 0.80, 0.80));
					}
				}
				else
				{
					AddPalletAt(-1.20, ShelfTopZ, 1);
					AddCartonRow(3, 0.15, 0.51, ShelfTopZ, 0.f, FVector::OneVector);
					AddToteStack(1.28, ShelfTopZ, 2, 2);
				}
			}
		}
	}

	if (Shield)
	{
		AddMesh(FString::Printf(TEXT("%s_Shield_0"), *Name), Shield, FVector(XM, RunStartYM, 0.0), Yaw);
		AddMesh(FString::Printf(TEXT("%s_Shield_1"), *Name), Shield, FVector(XM, RunStartYM + PostSpacingM * (PostCount - 1), 0.0), Yaw);
	}

	// One box over the whole run's footprint for collision/keep-out ("_Collider", matches what
	// SyntheticDataGenerator avoids placing training props on), and a second, identical but
	// non-colliding one named "_Visual" purely so SyntheticDataGenerator's rack detector - which reads
	// one component's own mesh-local bounding box, looking for a "Rack_*_Visual" name - still finds a
	// sensible whole-run box instead of dozens of small frame/shelf pieces.
	const double RunLengthM = PostSpacingM * (PostCount - 1);
	const FVector RunPosM(XM, RunStartYM + RunLengthM / 2.0, FrameTierM * FrameTiers / 2.0);
	const FVector RunSizeM(1.1, RunLengthM + 0.3, FrameTierM * FrameTiers);
	AddBox(FString::Printf(TEXT("%s_Collider"), *Name), RunPosM, RunSizeM, FLinearColor(1.f, 0.f, 1.f), false, false, 0.f, true);
	UStaticMeshComponent* VisualBox = AddBox(FString::Printf(TEXT("%s_Visual"), *Name), RunPosM, RunSizeM, FLinearColor(1.f, 0.f, 1.f), false);
	VisualBox->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void AWarehouse::BuildFlatRacks()
{
	// Fallback when the real rack kit (Scripts/setup.py) isn't imported yet: the old single-bay
	// Rack_A01 islands, so the actor still shows something rack-shaped.
	struct FRack { const TCHAR* Name; FVector Pos; float Yaw; };
	const FRack Racks[] = {
		{ TEXT("Rack_01"), FVector(-7.0, 3.25, 0.0), 0.f },
		{ TEXT("Rack_02"), FVector(-7.0, -1.25, 0.0), 0.f },
		{ TEXT("Rack_03"), FVector(7.0, 3.25, 0.0), 180.f },
		{ TEXT("Rack_04"), FVector(7.0, -1.25, 0.0), 180.f },
		{ TEXT("Rack_05"), FVector(-7.0, -5.75, 0.0), 0.f },
		{ TEXT("Rack_06"), FVector(7.0, -5.75, 0.0), 180.f },
	};
	const FVector RackColliderSizeM(3.30, 0.60, 2.00);
	UStaticMesh* Rack = RackMesh.LoadSynchronous();
	for (const FRack& R : Racks)
	{
		if (Rack)
		{
			UStaticMeshComponent* Visual = AddMesh(FString::Printf(TEXT("%s_Visual"), R.Name), Rack, R.Pos, R.Yaw);
			Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		}
		AddBox(FString::Printf(TEXT("%s_Collider"), R.Name), FVector(R.Pos.X, R.Pos.Y, RackColliderSizeM.Z / 2.0), RackColliderSizeM, FLinearColor(1.f, 0.f, 1.f), false, false, 0.f, true);
	}

	if (UStaticMesh* LoadedPallet = LoadedPalletMesh.LoadSynchronous())
	{
		const double ShelfLevelsM[] = { 1.20, 1.90 };
		const double PalletOffsetXM = 0.8;
		for (const FRack& R : Racks)
		{
			for (int32 Level = 0; Level < UE_ARRAY_COUNT(ShelfLevelsM); ++Level)
			{
				AddMesh(FString::Printf(TEXT("%s_Load_%d_A"), R.Name, Level), LoadedPallet, FVector(R.Pos.X - PalletOffsetXM, R.Pos.Y, ShelfLevelsM[Level]), 0.f);
				AddMesh(FString::Printf(TEXT("%s_Load_%d_B"), R.Name, Level), LoadedPallet, FVector(R.Pos.X + PalletOffsetXM, R.Pos.Y, ShelfLevelsM[Level]), 0.f);
			}
		}
	}
}

void AWarehouse::BuildDropZones()
{
	Slots.Reset();

	// Two zones in the free floor beyond the rack banks (racks end at |x| = 8.65 m, walls at |x| = 18 m):
	//   empty pallets  -> west zone, x = -12 m, entered heading west (-X)
	//   loaded pallets -> east zone, x = +12 m, entered heading east (+X)
	// Slots are a column along Y, 2 m pitch (pallet 1.2 m + room for the truck to swing in), each reached
	// straight from the aisle side. Two of the four rows line up with the gaps between rack banks and two
	// with the rack rows themselves, so the planner has to route around Rack_01..04 to reach them.
	struct FZone { const TCHAR* Prefix; bool bLoaded; double XM; float ApproachYaw; double AisleDirX; FLinearColor Color; };
	const FZone Zones[] = {
		// |x| = 14 m, not the original 12: the autonomous mission needs a straight run of roughly 2 m plus the
		// truck's own length (~1 m behind its origin) between the rack run's inflated edge (|x| ~ 8.3 m) and the
		// pre-pose, so the truck arrives lined up on the slot axis. At 12 m there was under 4.5 m from slot to rack.
		{ TEXT("Empty"), false, -14.0, 180.f, +1.0, FLinearColor(0.10f, 0.35f, 0.95f) },
		{ TEXT("Loaded"), true, 14.0, 0.f, -1.0, FLinearColor(0.15f, 0.80f, 0.25f) },
	};
	const double RowsYM[] = { 5.0, 3.0, 1.0, -1.0 };
	const FTransform ActorXf = GetActorTransform();

	for (const FZone& Zone : Zones)
	{
		int32 Index = 0;
		for (double YM : RowsYM)
		{
			FPalletSlot Slot;
			Slot.Id = FName(*FString::Printf(TEXT("%s_%d"), Zone.Prefix, Index + 1));
			Slot.bForLoaded = Zone.bLoaded;
			const FVector Local = UsdToUe(FVector(Zone.XM, YM, 0.0));
			Slot.Location = ActorXf.TransformPosition(Local);
			Slot.PalletYawDeg = 0.f;
			Slot.PrePoseLocation = ActorXf.TransformPosition(Local + FVector(Zone.AisleDirX * PrePoseStandOffCm, 0.0, 0.0));
			Slot.PrePoseYawDeg = Zone.ApproachYaw;
			Slots.Add(Slot);

			// Floor marking as painted tape, like a real warehouse: a 10 cm wide rectangle (1.4 x 1.3 m) around the pallet
			// position, flat and non-colliding (lidar and wheels ignore it). Yellow for empty pallets, white for loaded ones.
			const FLinearColor Tape = Zone.bLoaded ? FLinearColor(0.85f, 0.85f, 0.85f) : FLinearColor(0.95f, 0.75f, 0.05f);
			const struct { double Dx, Dy, Sx, Sy; } Strips[] = {
				{ 0.0, +0.60, 1.4, 0.10 }, { 0.0, -0.60, 1.4, 0.10 },   // the two long edges (along X, the insertion axis)
				{ +0.65, 0.0, 0.10, 1.1 }, { -0.65, 0.0, 0.10, 1.1 },   // the two short edges
			};
			int32 StripIdx = 0;
			for (const auto& St : Strips)
			{
				UStaticMeshComponent* Line = AddBox(FString::Printf(TEXT("Slot_%s_Tape%d"), *Slot.Id.ToString(), StripIdx++), FVector(Zone.XM + St.Dx, YM + St.Dy, 0.004), FVector(St.Sx, St.Sy, 0.008), Tape, true);
				Line->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				Line->SetCastShadow(false);
			}
			++Index;
		}
	}
}

void AWarehouse::ResetLoosePallets(bool bJitter, int32 Seed, float JitterCm, float JitterYawDeg)
{
	FRandomStream Rand(Seed);
	for (FLoosePallet& Pallet : LoosePallets)
	{
		Pallet.bDelivered = false;
		if (!Pallet.Comp)
		{
			continue;
		}
		FTransform Xf = Pallet.Home;
		if (bJitter)
		{
			Xf.AddToTranslation(FVector(Rand.FRandRange(-JitterCm, JitterCm), Rand.FRandRange(-JitterCm, JitterCm), 0.0));
			Xf.SetRotation(FQuat(FRotator(0.f, Pallet.Home.Rotator().Yaw + Rand.FRandRange(-JitterYawDeg, JitterYawDeg), 0.f)));
		}
		Pallet.Comp->SetWorldTransform(Xf, false, nullptr, ETeleportType::ResetPhysics);
		Pallet.Comp->SetPhysicsLinearVelocity(FVector::ZeroVector);
		Pallet.Comp->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
	}
}

void AWarehouse::MarkPalletDelivered(UStaticMeshComponent* Comp)
{
	for (FLoosePallet& Pallet : LoosePallets)
	{
		if (Pallet.Comp == Comp)
		{
			Pallet.bDelivered = true;
			return;
		}
	}
}

TArray<FPalletSlot> AWarehouse::GetSlotsFor(bool bLoaded) const
{
	return Slots.FilterByPredicate([bLoaded](const FPalletSlot& S) { return S.bForLoaded == bLoaded; });
}

bool AWarehouse::IsSlotOccupied(const FPalletSlot& Slot) const
{
	if (!GetWorld())
	{
		return false;
	}
	// Box hovers 10 cm above the floor so the floor itself never counts; 80 cm tall catches pallets, loads, bins.
	FCollisionObjectQueryParams Objects;
	Objects.AddObjectTypesToQuery(ECC_WorldStatic);
	Objects.AddObjectTypesToQuery(ECC_WorldDynamic);
	Objects.AddObjectTypesToQuery(ECC_PhysicsBody);
	const FVector Center = Slot.Location + FVector(0.0, 0.0, 50.0);
	const FQuat Rot(FVector::UpVector, FMath::DegreesToRadians(Slot.PalletYawDeg));
	return GetWorld()->OverlapAnyTestByObjectType(Center, Rot, Objects, FCollisionShape::MakeBox(FVector(60.0, 70.0, 40.0)));
}

bool AWarehouse::FindFreeSlot(bool bLoaded, const FVector& FromLocation, FPalletSlot& OutSlot) const
{
	TArray<FPalletSlot> Candidates = GetSlotsFor(bLoaded);
	Candidates.Sort([&FromLocation](const FPalletSlot& A, const FPalletSlot& B)
	{
		return FVector::DistSquared2D(A.Location, FromLocation) < FVector::DistSquared2D(B.Location, FromLocation);
	});
	for (const FPalletSlot& Candidate : Candidates)
	{
		if (!IsSlotOccupied(Candidate))
		{
			OutSlot = Candidate;
			return true;
		}
	}
	return false;
}
