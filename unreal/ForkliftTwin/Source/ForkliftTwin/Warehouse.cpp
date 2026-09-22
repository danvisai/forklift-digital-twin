#include "Warehouse.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
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

UStaticMeshComponent* AWarehouse::AddBox(const FString& Name, const FVector& UsdPosM, const FVector& SizeM, const FLinearColor& Color, bool bVisible, bool bDynamic, float MassKg)
{
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

	CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	FlatColorMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Warehouse/Materials/M_FlatColor.M_FlatColor"));
	if (!FlatColorMaterial)
	{
		FlatColorMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	}

	// --- Shell (Kit: _create_shell) -------------------------------------------------------------
	AddBox(TEXT("Floor"), FVector(0.0, 0.0, -0.1), FVector(24.0, 16.0, 0.2), FLinearColor(0.24f, 0.26f, 0.29f), true);

	struct FWall { const TCHAR* Name; FVector Pos; FVector Size; };
	const FWall Walls[] = {
		{ TEXT("NorthWall"), FVector(0.0, 8.1, 1.5), FVector(24.0, 0.2, 3.0) },
		{ TEXT("SouthWall"), FVector(0.0, -8.1, 1.5), FVector(24.0, 0.2, 3.0) },
		{ TEXT("EastWall"), FVector(12.1, 0.0, 1.5), FVector(0.2, 16.0, 3.0) },
		{ TEXT("WestWall"), FVector(-12.1, 0.0, 1.5), FVector(0.2, 16.0, 3.0) },
	};
	for (const FWall& Wall : Walls)
	{
		AddBox(Wall.Name, Wall.Pos, Wall.Size, FLinearColor(0.50f, 0.52f, 0.55f), true);
	}

	// --- Racks: two banks each side leave a central aisle (Kit: _create_warehouse_assets) --------
	struct FRack { const TCHAR* Name; FVector Pos; float Yaw; };
	const FRack Racks[] = {
		{ TEXT("Rack_01"), FVector(-7.0, 3.25, 0.0), 0.f },
		{ TEXT("Rack_02"), FVector(-7.0, -1.25, 0.0), 0.f },
		{ TEXT("Rack_03"), FVector(7.0, 3.25, 0.0), 180.f },
		{ TEXT("Rack_04"), FVector(7.0, -1.25, 0.0), 180.f },
	};
	const FVector RackColliderSizeM(3.30, 0.60, 2.00);
	UStaticMesh* Rack = RackMesh.LoadSynchronous();
	for (const FRack& R : Racks)
	{
		if (Rack)
		{
			UStaticMeshComponent* Visual = AddMesh(FString::Printf(TEXT("%s_Visual"), R.Name), Rack, R.Pos, R.Yaw);
			Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);  // Visual only; the proxy box collides.
		}
		AddBox(FString::Printf(TEXT("%s_Collider"), R.Name), FVector(R.Pos.X, R.Pos.Y, RackColliderSizeM.Z / 2.0), RackColliderSizeM, FLinearColor(1.f, 0.f, 1.f), false);
	}

	// --- Pallets, bins, guard rails ---------------------------------------------------------------
	if (UStaticMesh* Mesh = LoadedPalletMesh.LoadSynchronous())
	{
		AddMesh(TEXT("LoadedPallet_01"), Mesh, FVector(-3.5, 4.5, 0.0), 0.f);
	}
	LoosePallet = nullptr;
	if (UStaticMesh* Mesh = WoodPalletMesh.LoadSynchronous())
	{
		// A dynamic pallet only in a running game: it would fall over in the editor viewport otherwise.
		const bool bDynamic = (bDynamicLoosePallet || FParse::Param(FCommandLine::Get(), TEXT("ForkliftPalletTest"))) && GetWorld() && GetWorld()->IsGameWorld();
		LoosePallet = AddMesh(TEXT("WoodPallet_01"), Mesh, FVector(-3.5, 2.5, 0.0), 0.f, FVector::OneVector, bDynamic, 25.f);
	}
	if (UStaticMesh* Mesh = BinMesh.LoadSynchronous())
	{
		const double BinY[] = { 4.5, 3.75, 3.0 };
		for (int32 i = 0; i < 3; ++i)
		{
			AddMesh(FString::Printf(TEXT("Bin_%02d"), i + 1), Mesh, FVector(3.5, BinY[i], 0.0), 0.f);
		}
	}
	if (UStaticMesh* Mesh = GuardRailMesh.LoadSynchronous())
	{
		const double RailX[] = { -2.5, 0.0, 2.5 };
		for (int32 i = 0; i < 3; ++i)
		{
			AddMesh(FString::Printf(TEXT("GuardRail_%02d"), i + 1), Mesh, FVector(RailX[i], -5.5, 0.0), 0.f);
		}
	}

	// --- Physics test box (Kit: /World/Props/PhysicsTestBox) -------------------------------------
	if (bSpawnPhysicsTestBox && GetWorld() && GetWorld()->IsGameWorld())
	{
		AddBox(TEXT("PhysicsTestBox"), FVector(0.0, 0.0, 3.0), FVector(0.5, 0.5, 0.5), FLinearColor(0.95f, 0.45f, 0.10f), true, true, 5.f);
	}
}
