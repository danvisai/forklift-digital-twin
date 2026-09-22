// Forklift digital twin - procedural warehouse (port of _create_factory in the Kit extension).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Warehouse.generated.h"

class UStaticMesh;
class UStaticMeshComponent;
class UMaterialInterface;

/**
 * Builds the same warehouse as the Kit prototype: a 24 x 16 m floor with four 3 m walls, four rack
 * banks with invisible box colliders, two pallets, three bins, three guard rails and a dynamic
 * physics test box. Everything is regenerated from OnConstruction, so it shows up in the editor and
 * can be re-run from the "Rebuild" button in the Details panel (the old "Create/Reset Factory").
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

	/** Make the loose block pallet (Kit (-3.5, 2.5)) a 25 kg dynamic body so forks can pick it up. Also on with -ForkliftPalletTest. */
	UPROPERTY(EditAnywhere, Category = "Warehouse")
	bool bDynamicLoosePallet = false;

	/** The loose block pallet, or null. Only simulating when bDynamicLoosePallet / -ForkliftPalletTest is set. */
	UStaticMeshComponent* GetLoosePallet() const { return LoosePallet; }

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

private:
	UStaticMeshComponent* AddMesh(const FString& Name, UStaticMesh* Mesh, const FVector& UsdPosM, float UsdYawDeg, const FVector& Scale = FVector::OneVector, bool bDynamic = false, float MassKg = 0.f);
	UStaticMeshComponent* AddBox(const FString& Name, const FVector& UsdPosM, const FVector& SizeM, const FLinearColor& Color, bool bVisible, bool bDynamic = false, float MassKg = 0.f);

	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> Spawned;

	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> LoosePallet;

	UPROPERTY(Transient)
	TObjectPtr<UStaticMesh> CubeMesh;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> FlatColorMaterial;
};
