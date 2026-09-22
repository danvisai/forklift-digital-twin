#include "SyntheticDataGenerator.h"

#include "AssetCompilingManager.h"
#include "Async/Async.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/LightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "ForkliftCameraComponent.h"
#include "GameFramework/Pawn.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "ImageUtils.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "NpyWriter.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "ShaderCompiler.h"
#include "Warehouse.h"

DEFINE_LOG_CATEGORY_STATIC(LogSynthData, Log, All);

namespace
{
	// Usable floor, cm from the hall centre (the hall is 24 x 16 m; keep clear of the walls).
	constexpr float HallX = 1100.f;
	constexpr float HallY = 700.f;

	const TCHAR* const ClassNames[] = { TEXT("pallet"), TEXT("loaded_pallet"), TEXT("pallet_pocket"), TEXT("tote"), TEXT("rack") };

	FVector JsonVec(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field)
	{
		const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
		if (Obj->TryGetArrayField(Field, Arr) && Arr->Num() >= 3)
		{
			return FVector((*Arr)[0]->AsNumber(), (*Arr)[1]->AsNumber(), (*Arr)[2]->AsNumber());
		}
		return FVector::ZeroVector;
	}

	TArray<TSharedPtr<FJsonValue>> NumArray(std::initializer_list<double> Values)
	{
		TArray<TSharedPtr<FJsonValue>> Out;
		for (const double V : Values)
		{
			Out.Add(MakeShared<FJsonValueNumber>(V));
		}
		return Out;
	}

	TArray<TSharedPtr<FJsonValue>> VecArray(const FVector& V)
	{
		return NumArray({ V.X, V.Y, V.Z });
	}

	FString ToJsonString(const TSharedPtr<FJsonObject>& Obj)
	{
		FString Text;
		const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Text);
		FJsonSerializer::Serialize(Obj.ToSharedRef(), Writer);
		return Text;
	}

	FString SplitOf(int32 Scene)
	{
		const int32 M = Scene % 10;
		return M == 8 ? TEXT("val") : (M == 9 ? TEXT("test") : TEXT("train"));
	}

	float SnapYaw(FRandomStream& Rand)
	{
		// Mostly axis-aligned (how pallets are stored) with a few degrees of slop; sometimes anything.
		if (Rand.FRand() < 0.35f)
		{
			return Rand.FRandRange(0.f, 360.f);
		}
		return 90.f * Rand.RandRange(0, 3) + Rand.FRandRange(-12.f, 12.f);
	}
}

// ---- Subsystem -----------------------------------------------------------------------------------------------

bool USyntheticDataSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
	const UWorld* World = Cast<UWorld>(Outer);
	return World && World->IsGameWorld() && FParse::Param(FCommandLine::Get(), TEXT("ForkliftDataGen")) && Super::ShouldCreateSubsystem(Outer);
}

void USyntheticDataSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	InWorld.SpawnActor<ASyntheticDataGenerator>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
}

// ---- Generator: setup ------------------------------------------------------------------------------------------

ASyntheticDataGenerator::ASyntheticDataGenerator()
{
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("Root")));
}

void ASyntheticDataGenerator::BeginPlay()
{
	Super::BeginPlay();

	const TCHAR* Cmd = FCommandLine::Get();
	FParse::Value(Cmd, TEXT("DataName="), DatasetName);
	FParse::Value(Cmd, TEXT("DataFrames="), NumFrames);
	FParse::Value(Cmd, TEXT("DataSeed="), Seed);
	FParse::Value(Cmd, TEXT("DataFramesPerScene="), FramesPerScene);
	FParse::Value(Cmd, TEXT("DataStart="), StartFrame);
	FParse::Value(Cmd, TEXT("DataW="), ImageW);
	FParse::Value(Cmd, TEXT("DataH="), ImageH);
	bDepthHalf = !FParse::Param(Cmd, TEXT("DataDepthF32"));
	bPersistRendering = !FParse::Param(Cmd, TEXT("DataNoPersist"));
	BaseEV = bPersistRendering ? 9.5f : 12.f;  // measured: mean luma 154 (range 89-218), no black crush or clipping
	FParse::Value(Cmd, TEXT("DataEV="), BaseEV);
	FramesPerScene = FMath::Max(1, FramesPerScene);
	StartFrame = (StartFrame / FramesPerScene) * FramesPerScene;  // batches must start on a scene boundary

	FString Base = FPaths::ProjectSavedDir() / TEXT("Dataset");
	FParse::Value(Cmd, TEXT("DataOut="), Base);
	OutputDir = FPaths::ConvertRelativePathToFull(Base / DatasetName);
	for (const TCHAR* Split : { TEXT("train"), TEXT("val"), TEXT("test") })
	{
		for (const TCHAR* Kind : { TEXT("images"), TEXT("depth"), TEXT("labels"), TEXT("meta") })
		{
			IFileManager::Get().MakeDirectory(*(OutputDir / Kind / Split), true);
		}
	}
	NextFrame = StartFrame;
	UE_LOG(LogSynthData, Log, TEXT("[DataGen] %s: frames %d..%d, seed %d, %d views per scene, %dx%d, out=%s"),
		*DatasetName, StartFrame, StartFrame + NumFrames - 1, Seed, FramesPerScene, ImageW, ImageH, *OutputDir);
}

UStaticMesh* ASyntheticDataGenerator::GetMesh(const FString& Path)
{
	if (TObjectPtr<UStaticMesh>* Found = MeshCache.Find(Path))
	{
		return *Found;
	}
	UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *Path);
	if (!Mesh)
	{
		UE_LOG(LogSynthData, Warning, TEXT("[DataGen] could not load mesh %s"), *Path);
	}
	MeshCache.Add(Path, Mesh);
	return Mesh;
}

bool ASyntheticDataGenerator::LoadCatalog()
{
	const FString Path = FPaths::ProjectContentDir() / TEXT("Pallets/catalog.json");
	FString Text;
	TSharedPtr<FJsonObject> Root;
	if (!FFileHelper::LoadFileToString(Text, *Path) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) || !Root)
	{
		UE_LOG(LogSynthData, Error, TEXT("[DataGen] cannot read %s - run Scripts/bake_pallets.py and import_pallets.py"), *Path);
		return false;
	}

	const auto Parse = [&Root](const TCHAR* Field, TArray<FAssetDef>& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* List = nullptr;
		if (!Root->TryGetArrayField(Field, List))
		{
			return;
		}
		for (const TSharedPtr<FJsonValue>& V : *List)
		{
			const TSharedPtr<FJsonObject> O = V->AsObject();
			FAssetDef D;
			D.Name = O->GetStringField(TEXT("name"));
			D.MeshPath = O->GetStringField(TEXT("mesh"));
			const FString Cls = O->GetStringField(TEXT("cls"));
			D.ClassId = Cls == TEXT("loaded_pallet") ? ForkliftClass::LoadedPallet : (Cls == TEXT("tote") ? ForkliftClass::Tote : ForkliftClass::Pallet);
			O->TryGetStringField(TEXT("standard"), D.Standard);
			const TSharedPtr<FJsonObject>* Box = nullptr;
			if (O->TryGetObjectField(TEXT("bbox_cm"), Box))
			{
				D.BBoxLo = JsonVec(*Box, TEXT("lo"));
				D.BBoxHi = JsonVec(*Box, TEXT("hi"));
			}
			D.BaseLo = D.BBoxLo;
			D.BaseHi = D.BBoxHi;
			if (O->TryGetObjectField(TEXT("base_bbox_cm"), Box))
			{
				D.BaseLo = JsonVec(*Box, TEXT("lo"));
				D.BaseHi = JsonVec(*Box, TEXT("hi"));
			}
			const TArray<TSharedPtr<FJsonValue>>* Pockets = nullptr;
			if (O->TryGetArrayField(TEXT("pockets"), Pockets))
			{
				for (const TSharedPtr<FJsonValue>& PV : *Pockets)
				{
					const TSharedPtr<FJsonObject> P = PV->AsObject();
					FPocketDef Pk;
					Pk.Entry = P->GetStringField(TEXT("entry_ue"));
					Pk.Center = JsonVec(P, TEXT("center_cm"));
					Pk.Size = JsonVec(P, TEXT("size_cm"));
					Pk.Normal = JsonVec(P, TEXT("normal"));
					Pk.DepthCm = static_cast<float>(P->GetNumberField(TEXT("depth_cm")));
					D.Pockets.Add(Pk);
				}
			}
			const TArray<TSharedPtr<FJsonValue>>* Slots = nullptr;
			if (O->TryGetArrayField(TEXT("material_slots"), Slots))
			{
				for (const TSharedPtr<FJsonValue>& SV : *Slots)
				{
					if (const TSharedPtr<FJsonObject> S = SV->AsObject())
					{
						D.Slots.Add(TPair<int32, FString>(static_cast<int32>(S->GetNumberField(TEXT("index"))), S->GetStringField(TEXT("key"))));
					}
				}
			}
			Out.Add(MoveTemp(D));
		}
	};
	Parse(TEXT("assets"), Pallets);
	Parse(TEXT("props"), Totes);
	UE_LOG(LogSynthData, Log, TEXT("[DataGen] catalogue: %d pallets / loaded pallets, %d totes"), Pallets.Num(), Totes.Num());
	return Pallets.Num() > 0;
}

void ASyntheticDataGenerator::PrepareWorld()
{
	UWorld* World = GetWorld();

	// The truck is not part of a data frame (and its own sensors would keep rendering): remove it.
	TArray<APawn*> Pawns;
	for (TActorIterator<APawn> It(World); It; ++It)
	{
		Pawns.Add(*It);
	}
	for (APawn* Pawn : Pawns)
	{
		Pawn->Destroy();
	}

	for (TActorIterator<AWarehouse> It(World); It; ++It)
	{
		Warehouse = *It;
		break;
	}
	if (Warehouse)
	{
		Warehouse->bSpawnPhysicsTestBox = false;
		Warehouse->Rebuild();
		TArray<UStaticMeshComponent*> Comps;
		Warehouse->GetComponents(Comps);
		for (UStaticMeshComponent* C : Comps)
		{
			const FString Name = C->GetName();
			if (Name.StartsWith(TEXT("LoadedPallet_")) || Name.StartsWith(TEXT("WoodPallet_")) || Name.StartsWith(TEXT("Bin_")))
			{
				// Fixed props that would otherwise appear in frames unlabelled.
				C->SetVisibility(false);
				C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			}
			else if (Name == TEXT("Floor") || Name.EndsWith(TEXT("Wall")))
			{
				if (UMaterialInstanceDynamic* Mid = Cast<UMaterialInstanceDynamic>(C->GetMaterial(0)))
				{
					EnvMats.Add(Name, Mid);
				}
			}
			else if (Name.StartsWith(TEXT("Rack_")) && Name.EndsWith(TEXT("_Visual")) && C->GetStaticMesh())
			{
				FPlacedProp Rack;
				Rack.ClassId = ForkliftClass::Rack;
				Rack.Asset = TEXT("Rack_A01");
				Rack.Comp = C;
				Rack.Xf = C->GetComponentTransform();
				const FBox Box = C->GetStaticMesh()->GetBoundingBox();
				Rack.Lo = Box.Min;
				Rack.Hi = Box.Max;
				FixedRacks.Add(Rack);
			}
			else if ((Name.StartsWith(TEXT("Rack_")) && Name.EndsWith(TEXT("_Collider"))) || Name.StartsWith(TEXT("GuardRail_")))
			{
				// Keep placements off the rack colliders and the guard rails (cube scale = size in metres, cube is 1 m).
				const FVector S = C->GetComponentScale() * 50.f;
				FRect2D R;
				R.C = FVector2D(C->GetComponentLocation());
				R.Ax = FVector2D(1, 0);
				R.Ay = FVector2D(0, 1);
				R.Half = Name.StartsWith(TEXT("GuardRail_")) ? FVector2D(120.f, 25.f) : FVector2D(S.X, S.Y);
				FixedRects.Add(R);
			}
		}
	}
	else
	{
		UE_LOG(LogSynthData, Warning, TEXT("[DataGen] no AWarehouse in the level"));
	}

	CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	RackMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/Warehouse/Meshes/SM_Rack_A01.SM_Rack_A01"));
	FlatMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Pallets/Materials/M_PalletFlat.M_PalletFlat"));
	UE_LOG(LogSynthData, Log, TEXT("[DataGen] hall: %d fixed racks labelled, %d keep-out rects, rack mesh %s, flat material %s"),
		FixedRacks.Num(), FixedRects.Num(), RackMesh ? TEXT("ok") : TEXT("MISSING"), FlatMaterial ? TEXT("ok") : TEXT("MISSING"));
}

void ASyntheticDataGenerator::CreateCamera()
{
	Camera = NewObject<UForkliftCameraComponent>(this, TEXT("DataCamera"));
	Camera->Width = ImageW;
	Camera->Height = ImageH;
	Camera->FOVAngle = 90.f;
	Camera->bCaptureDepth = true;
	Camera->bAutoCapture = false;
	Camera->bAlwaysPersistRenderingState = bPersistRendering;
	Camera->SetupAttachment(GetRootComponent());
	Camera->RegisterComponent();
}

// ---- Generator: scene --------------------------------------------------------------------------------------------

UStaticMeshComponent* ASyntheticDataGenerator::AddMeshComp(UStaticMesh* Mesh, const FTransform& Xf)
{
	UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(this, NAME_None, RF_Transient);
	Comp->SetMobility(EComponentMobility::Movable);
	Comp->SetStaticMesh(Mesh);
	Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Comp->SetupAttachment(GetRootComponent());
	Comp->SetRelativeTransform(Xf);
	Comp->RegisterComponent();
	SceneComps.Add(Comp);
	return Comp;
}

UStaticMeshComponent* ASyntheticDataGenerator::AddCube(const FVector& Loc, float YawDeg, const FVector& SizeCm, const FLinearColor& Color, bool bTransient)
{
	// /Engine/BasicShapes/Cube is 100 cm with its pivot at the centre; raise it so it stands on Loc.Z.
	UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(this, NAME_None, RF_Transient);
	Comp->SetMobility(EComponentMobility::Movable);
	Comp->SetStaticMesh(CubeMesh);
	Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Comp->SetupAttachment(GetRootComponent());
	Comp->SetRelativeTransform(FTransform(FRotator(0.f, YawDeg, 0.f), Loc + FVector(0.f, 0.f, SizeCm.Z * 0.5f), SizeCm / 100.f));
	if (FlatMaterial)
	{
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(FlatMaterial, Comp);
		Mid->SetVectorParameterValue(TEXT("Color"), Color);
		Mid->SetScalarParameterValue(TEXT("Roughness"), 0.85f);
		Comp->SetMaterial(0, Mid);
	}
	Comp->RegisterComponent();
	if (!bTransient)
	{
		SceneComps.Add(Comp);
	}
	return Comp;
}

void ASyntheticDataGenerator::RandomiseMaterials(UStaticMeshComponent* Comp, const FAssetDef* Def)
{
	static const FLinearColor PlasticColors[] = {
		FLinearColor(0.05f, 0.05f, 0.055f), FLinearColor(0.02f, 0.06f, 0.30f), FLinearColor(0.35f, 0.02f, 0.02f), FLinearColor(0.02f, 0.20f, 0.05f),
		FLinearColor(0.60f, 0.40f, 0.02f), FLinearColor(0.35f, 0.36f, 0.38f), FLinearColor(0.65f, 0.65f, 0.63f), FLinearColor(0.70f, 0.25f, 0.02f),
	};
	for (const TPair<int32, FString>& Slot : Def->Slots)
	{
		UMaterialInterface* Base = Comp->GetMaterial(Slot.Key);
		if (!Base)
		{
			continue;
		}
		const bool bPlastic = Slot.Value.StartsWith(TEXT("Plastic"));
		const bool bWood = Slot.Value.StartsWith(TEXT("Wood")) || Slot.Value.StartsWith(TEXT("M_BlockPallet"));
		const bool bCardboard = Slot.Value.StartsWith(TEXT("Pallet_Assembly_Tile"));
		if (!bPlastic && !bWood && !bCardboard)
		{
			continue;  // metals, wrap: leave as authored
		}
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Base, Comp);
		FLinearColor Color = Mid->K2_GetVectorParameterValue(TEXT("Color"));
		if (bPlastic && Rand.FRand() < 0.75f)
		{
			Color = PlasticColors[Rand.RandRange(0, static_cast<int32>(UE_ARRAY_COUNT(PlasticColors)) - 1)];
		}
		else
		{
			const float Gain = Rand.FRandRange(0.65f, 1.3f);
			Color = FLinearColor(Color.R * Gain * Rand.FRandRange(0.9f, 1.1f), Color.G * Gain, Color.B * Gain * Rand.FRandRange(0.85f, 1.1f));
		}
		Mid->SetVectorParameterValue(TEXT("Color"), Color);
		Comp->SetMaterial(Slot.Key, Mid);
	}
}

void ASyntheticDataGenerator::ClearScene()
{
	for (UStaticMeshComponent* C : SceneComps)
	{
		if (C)
		{
			C->DestroyComponent();
		}
	}
	SceneComps.Reset();
	if (Occluder)
	{
		Occluder->DestroyComponent();
		Occluder = nullptr;
	}
	PlacedPallets.Reset();
	PlacedProps.Reset();
	Targets.Reset();
	Occupied.Reset();
}

void ASyntheticDataGenerator::RandomiseEnvironment()
{
	LightingMeta = MakeShared<FJsonObject>();
	UWorld* World = GetWorld();

	for (TActorIterator<ADirectionalLight> It(World); It; ++It)
	{
		const float Pitch = Rand.FRandRange(-75.f, -18.f);
		const float Yaw = Rand.FRandRange(0.f, 360.f);
		const float Intensity = Rand.FRandRange(3.5f, 10.f);
		It->SetActorRotation(FRotator(Pitch, Yaw, 0.f));
		if (ULightComponent* Light = It->GetLightComponent())
		{
			Light->SetIntensity(Intensity);
		}
		LightingMeta->SetNumberField(TEXT("sun_pitch_deg"), Pitch);
		LightingMeta->SetNumberField(TEXT("sun_yaw_deg"), Yaw);
		LightingMeta->SetNumberField(TEXT("sun_intensity"), Intensity);
		break;
	}
	for (TActorIterator<ASkyLight> It(World); It; ++It)
	{
		const float Intensity = Rand.FRandRange(1.0f, 3.0f);  // fill for the shadows: with too little, backlit objects go pure black
		if (USkyLightComponent* Sky = It->GetLightComponent())
		{
			Sky->SetIntensity(Intensity);
		}
		LightingMeta->SetNumberField(TEXT("sky_intensity"), Intensity);
		break;
	}

	// Floor: mostly concrete greys, sometimes a painted / epoxy colour. Walls: light neutrals with a tint.
	if (TObjectPtr<UMaterialInstanceDynamic>* Floor = EnvMats.Find(TEXT("Floor")))
	{
		FLinearColor C;
		if (Rand.FRand() < 0.18f)
		{
			static const FLinearColor Epoxy[] = { FLinearColor(0.03f, 0.10f, 0.22f), FLinearColor(0.03f, 0.16f, 0.06f), FLinearColor(0.30f, 0.05f, 0.04f), FLinearColor(0.30f, 0.27f, 0.10f) };
			C = Epoxy[Rand.RandRange(0, static_cast<int32>(UE_ARRAY_COUNT(Epoxy)) - 1)];
		}
		else
		{
			const float G = Rand.FRandRange(0.05f, 0.45f);
			C = FLinearColor(G * Rand.FRandRange(0.95f, 1.08f), G, G * Rand.FRandRange(0.92f, 1.06f));
		}
		(*Floor)->SetVectorParameterValue(TEXT("Color"), C);
	}
	for (const TCHAR* Wall : { TEXT("NorthWall"), TEXT("SouthWall"), TEXT("EastWall"), TEXT("WestWall") })
	{
		if (TObjectPtr<UMaterialInstanceDynamic>* Mat = EnvMats.Find(Wall))
		{
			const float L = Rand.FRandRange(0.15f, 0.75f);
			(*Mat)->SetVectorParameterValue(TEXT("Color"), FLinearColor(L * Rand.FRandRange(0.85f, 1.1f), L * Rand.FRandRange(0.9f, 1.05f), L * Rand.FRandRange(0.85f, 1.1f)));
		}
	}
}

static bool RectsOverlap(const FVector2D& CA, const FVector2D& AxA, const FVector2D& AyA, const FVector2D& HA,
                         const FVector2D& CB, const FVector2D& AxB, const FVector2D& AyB, const FVector2D& HB, float Margin)
{
	const FVector2D Axes[4] = { AxA, AyA, AxB, AyB };
	const FVector2D D = CB - CA;
	for (const FVector2D& Axis : Axes)
	{
		const float RA = HA.X * FMath::Abs(FVector2D::DotProduct(AxA, Axis)) + HA.Y * FMath::Abs(FVector2D::DotProduct(AyA, Axis));
		const float RB = HB.X * FMath::Abs(FVector2D::DotProduct(AxB, Axis)) + HB.Y * FMath::Abs(FVector2D::DotProduct(AyB, Axis));
		if (FMath::Abs(FVector2D::DotProduct(D, Axis)) > RA + RB + Margin)
		{
			return false;  // a separating axis
		}
	}
	return true;
}

bool ASyntheticDataGenerator::AreaFree(const FVector2D& C, float YawDeg, const FVector& Lo, const FVector& Hi, float Margin) const
{
	const float S = FMath::Sin(FMath::DegreesToRadians(YawDeg)), Co = FMath::Cos(FMath::DegreesToRadians(YawDeg));
	const FVector2D Ax(Co, S), Ay(-S, Co);
	const FVector2D LocalC(0.5f * (Lo.X + Hi.X), 0.5f * (Lo.Y + Hi.Y));
	const FVector2D Center = C + Ax * LocalC.X + Ay * LocalC.Y;
	const FVector2D Half(0.5f * (Hi.X - Lo.X), 0.5f * (Hi.Y - Lo.Y));
	// Inside the hall (rotated extent).
	const float ExtX = FMath::Abs(Ax.X) * Half.X + FMath::Abs(Ay.X) * Half.Y;
	const float ExtY = FMath::Abs(Ax.Y) * Half.X + FMath::Abs(Ay.Y) * Half.Y;
	if (FMath::Abs(Center.X) + ExtX > HallX || FMath::Abs(Center.Y) + ExtY > HallY)
	{
		return false;
	}
	for (const FRect2D& R : Occupied)
	{
		if (RectsOverlap(Center, Ax, Ay, Half, R.C, R.Ax, R.Ay, R.Half, Margin))
		{
			return false;
		}
	}
	return true;
}

void ASyntheticDataGenerator::Occupy(const FVector2D& C, float YawDeg, const FVector& Lo, const FVector& Hi)
{
	const float S = FMath::Sin(FMath::DegreesToRadians(YawDeg)), Co = FMath::Cos(FMath::DegreesToRadians(YawDeg));
	FRect2D R;
	R.Ax = FVector2D(Co, S);
	R.Ay = FVector2D(-S, Co);
	R.C = C + R.Ax * (0.5f * (Lo.X + Hi.X)) + R.Ay * (0.5f * (Lo.Y + Hi.Y));
	R.Half = FVector2D(0.5f * (Hi.X - Lo.X), 0.5f * (Hi.Y - Lo.Y));
	Occupied.Add(R);
}

const FAssetDef* ASyntheticDataGenerator::PickPallet()
{
	// Asset-loaded pallets are ~30% of picks; bare pallets pick up totes / boxes later, so loaded ends up around half.
	const int32 WantClass = Rand.FRand() < 0.30f ? ForkliftClass::LoadedPallet : ForkliftClass::Pallet;
	// Standard mix: ~50% the 1200x1000 / 48x40 family (the most numerous), ~20% EUR 1200x800, rest anything else.
	const float R = Rand.FRand();
	const auto Matches = [&](const FAssetDef& D, int32 Bucket)
	{
		if (D.ClassId != WantClass)
		{
			return false;
		}
		return Bucket == 0 ? D.Standard == TEXT("1200x1000") : (Bucket == 1 ? D.Standard == TEXT("1200x800") : (D.Standard != TEXT("1200x1000") && D.Standard != TEXT("1200x800")));
	};
	int32 Bucket = R < 0.5f ? 0 : (R < 0.7f ? 1 : 2);
	TArray<const FAssetDef*> Pool;
	for (int32 Attempt = 0; Attempt < 3 && Pool.Num() == 0; ++Attempt)
	{
		for (const FAssetDef& D : Pallets)
		{
			if (Matches(D, (Bucket + Attempt) % 3))
			{
				Pool.Add(&D);
			}
		}
	}
	if (Pool.Num() == 0)
	{
		for (const FAssetDef& D : Pallets)
		{
			Pool.Add(&D);
		}
	}
	return Pool[Rand.RandRange(0, Pool.Num() - 1)];
}

bool ASyntheticDataGenerator::TryPlacePallet(const FVector2D& Center, float Radius)
{
	const FAssetDef* Def = PickPallet();
	UStaticMesh* Mesh = GetMesh(Def->MeshPath);
	if (!Mesh)
	{
		return false;
	}
	for (int32 Try = 0; Try < 40; ++Try)
	{
		const float Ang = Rand.FRandRange(0.f, 2.f * PI);
		const FVector2D P = Center + FVector2D(FMath::Cos(Ang), FMath::Sin(Ang)) * (Radius * FMath::Sqrt(Rand.FRand()));
		const float Yaw = SnapYaw(Rand);
		if (!AreaFree(P, Yaw, Def->BaseLo, Def->BaseHi, 25.f))
		{
			continue;
		}
		FPlacedPallet Pl;
		Pl.Def = Def;
		Pl.YawDeg = Yaw;
		Pl.Xf = FTransform(FRotator(0.f, Yaw, 0.f), FVector(P.X, P.Y, 0.f));
		Pl.Comp = AddMeshComp(Mesh, Pl.Xf);
		RandomiseMaterials(Pl.Comp, Def);
		Pl.ClassId = Def->ClassId;
		Pl.Lo = Def->BBoxLo;
		Pl.Hi = Def->BBoxHi;
		Pl.Load = Def->ClassId == ForkliftClass::LoadedPallet ? TEXT("asset") : TEXT("none");
		Occupy(P, Yaw, Def->BaseLo, Def->BaseHi);
		const int32 Index = PlacedPallets.Add(Pl);
		if (Def->ClassId == ForkliftClass::Pallet)
		{
			const float R = Rand.FRand();
			if (R < 0.16f)
			{
				PlaceToteLoad(PlacedPallets[Index]);
			}
			else if (R < 0.27f)
			{
				PlaceBoxLoad(PlacedPallets[Index]);
			}
		}
		return true;
	}
	return false;
}

void ASyntheticDataGenerator::PlaceToteLoad(FPlacedPallet& P)
{
	if (Totes.Num() == 0)
	{
		return;
	}
	const FAssetDef& T = Totes[Rand.RandRange(0, Totes.Num() - 1)];
	UStaticMesh* Mesh = GetMesh(T.MeshPath);
	const FVector Sz = T.BBoxHi - T.BBoxLo;
	const bool bRot = Rand.FRand() < 0.5f;
	const float Dx = bRot ? Sz.Y : Sz.X, Dy = bRot ? Sz.X : Sz.Y;
	const FVector& L = P.Def->BaseLo;
	const FVector& H = P.Def->BaseHi;
	const int32 Nx = FMath::Min(FMath::FloorToInt((H.X - L.X) / Dx), 4);
	const int32 Ny = FMath::Min(FMath::FloorToInt((H.Y - L.Y) / Dy), 4);
	if (!Mesh || Nx < 1 || Ny < 1)
	{
		return;  // the tote does not fit this pallet; leave it bare
	}
	const int32 StackLayers = FMath::Clamp(Rand.RandRange(1, 3), 1, FMath::Max(1, FMath::FloorToInt(140.f / Sz.Z)));
	const float X0 = 0.5f * (L.X + H.X) - 0.5f * Nx * Dx;
	const float Y0 = 0.5f * (L.Y + H.Y) - 0.5f * Ny * Dy;
	for (int32 Ix = 0; Ix < Nx; ++Ix)
	{
		for (int32 Iy = 0; Iy < Ny; ++Iy)
		{
			for (int32 Layer = 0; Layer < StackLayers; ++Layer)
			{
				const FVector Local(X0 + (Ix + 0.5f) * Dx, Y0 + (Iy + 0.5f) * Dy, H.Z + Layer * Sz.Z);
				const FTransform Xf = FTransform(FRotator(0.f, bRot ? 90.f : 0.f, 0.f), Local) * P.Xf;
				FPlacedProp Tote;
				Tote.ClassId = ForkliftClass::Tote;
				Tote.Asset = T.Name;
				Tote.Xf = Xf;
				Tote.Lo = T.BBoxLo;
				Tote.Hi = T.BBoxHi;
				Tote.Comp = AddMeshComp(Mesh, Xf);
				RandomiseMaterials(Tote.Comp, &T);
				PlacedProps.Add(Tote);
			}
		}
	}
	P.Hi.Z = H.Z + StackLayers * Sz.Z;
	P.ClassId = ForkliftClass::LoadedPallet;
	P.Load = TEXT("totes");
}

void ASyntheticDataGenerator::PlaceBoxLoad(FPlacedPallet& P)
{
	const FVector& L = P.Def->BaseLo;
	const FVector& H = P.Def->BaseHi;
	const float Sx = Rand.FRandRange(35.f, 60.f), Sy = Rand.FRandRange(30.f, 55.f), Sz = Rand.FRandRange(25.f, 45.f);
	const int32 Nx = FMath::Max(1, FMath::FloorToInt((H.X - L.X) / Sx));
	const int32 Ny = FMath::Max(1, FMath::FloorToInt((H.Y - L.Y) / Sy));
	const int32 StackLayers = Rand.RandRange(1, 3);
	const float X0 = 0.5f * (L.X + H.X) - 0.5f * Nx * Sx;
	const float Y0 = 0.5f * (L.Y + H.Y) - 0.5f * Ny * Sy;
	const FLinearColor Cardboard(Rand.FRandRange(0.28f, 0.42f), Rand.FRandRange(0.18f, 0.30f), Rand.FRandRange(0.08f, 0.16f));
	for (int32 Ix = 0; Ix < Nx; ++Ix)
	{
		for (int32 Iy = 0; Iy < Ny; ++Iy)
		{
			for (int32 Layer = 0; Layer < StackLayers; ++Layer)
			{
				const FVector Local(X0 + (Ix + 0.5f) * Sx, Y0 + (Iy + 0.5f) * Sy, H.Z + Layer * Sz);
				const FVector World = P.Xf.TransformPosition(Local);
				AddCube(World, P.YawDeg, FVector(Sx * 0.97f, Sy * 0.97f, Sz * 0.97f), Cardboard * Rand.FRandRange(0.85f, 1.15f));
			}
		}
	}
	P.Hi.Z = H.Z + StackLayers * Sz;
	P.ClassId = ForkliftClass::LoadedPallet;
	P.Load = TEXT("boxes");
}

void ASyntheticDataGenerator::PlaceLooseTotes(const FVector2D& Center, float Radius)
{
	if (Totes.Num() == 0)
	{
		return;
	}
	const int32 Stacks = Rand.RandRange(1, 3);
	for (int32 S = 0; S < Stacks; ++S)
	{
		const FAssetDef& T = Totes[Rand.RandRange(0, Totes.Num() - 1)];
		UStaticMesh* Mesh = GetMesh(T.MeshPath);
		if (!Mesh)
		{
			continue;
		}
		const FVector Sz = T.BBoxHi - T.BBoxLo;
		const int32 Nx = Rand.RandRange(1, 2), Ny = Rand.RandRange(1, 2);
		const int32 StackLayers = FMath::Clamp(Rand.RandRange(1, 4), 1, FMath::Max(1, FMath::FloorToInt(140.f / Sz.Z)));
		const FVector Lo(-0.5f * Nx * Sz.X, -0.5f * Ny * Sz.Y, 0.f), Hi(0.5f * Nx * Sz.X, 0.5f * Ny * Sz.Y, 0.f);
		for (int32 Try = 0; Try < 25; ++Try)
		{
			const float Ang = Rand.FRandRange(0.f, 2.f * PI);
			const FVector2D P = Center + FVector2D(FMath::Cos(Ang), FMath::Sin(Ang)) * (Radius * FMath::Sqrt(Rand.FRand()));
			const float Yaw = SnapYaw(Rand);
			if (!AreaFree(P, Yaw, Lo, Hi, 20.f))
			{
				continue;
			}
			const FTransform Base(FRotator(0.f, Yaw, 0.f), FVector(P.X, P.Y, 0.f));
			for (int32 Ix = 0; Ix < Nx; ++Ix)
			{
				for (int32 Iy = 0; Iy < Ny; ++Iy)
				{
					for (int32 Layer = 0; Layer < StackLayers; ++Layer)
					{
						const FVector Local(Lo.X + (Ix + 0.5f) * Sz.X + Rand.FRandRange(-1.f, 1.f), Lo.Y + (Iy + 0.5f) * Sz.Y + Rand.FRandRange(-1.f, 1.f), Layer * Sz.Z);
						FPlacedProp Tote;
						Tote.ClassId = ForkliftClass::Tote;
						Tote.Asset = T.Name;
						Tote.Xf = FTransform(FRotator(0.f, Rand.FRandRange(-2.f, 2.f), 0.f), Local) * Base;
						Tote.Lo = T.BBoxLo;
						Tote.Hi = T.BBoxHi;
						Tote.Comp = AddMeshComp(Mesh, Tote.Xf);
						RandomiseMaterials(Tote.Comp, &T);
						PlacedProps.Add(Tote);
					}
				}
			}
			Occupy(P, Yaw, Lo, Hi);
			break;
		}
	}
}

void ASyntheticDataGenerator::PlaceRack(const FVector2D& Center, float Radius)
{
	if (!RackMesh)
	{
		return;
	}
	const FBox Box = RackMesh->GetBoundingBox();
	for (int32 Try = 0; Try < 30; ++Try)
	{
		const float Ang = Rand.FRandRange(0.f, 2.f * PI);
		const FVector2D P = Center + FVector2D(FMath::Cos(Ang), FMath::Sin(Ang)) * Rand.FRandRange(Radius * 0.5f, Radius);
		const float Yaw = 90.f * Rand.RandRange(0, 3) + Rand.FRandRange(-4.f, 4.f);
		if (!AreaFree(P, Yaw, Box.Min, Box.Max, 40.f))
		{
			continue;
		}
		FPlacedProp Rack;
		Rack.ClassId = ForkliftClass::Rack;
		Rack.Asset = TEXT("Rack_A01");
		Rack.Xf = FTransform(FRotator(0.f, Yaw, 0.f), FVector(P.X, P.Y, 0.f));
		Rack.Lo = Box.Min;
		Rack.Hi = Box.Max;
		Rack.Comp = AddMeshComp(RackMesh, Rack.Xf);
		Occupy(P, Yaw, Box.Min, Box.Max);
		PlacedProps.Add(Rack);
		return;
	}
}

void ASyntheticDataGenerator::AddClutter(const FVector2D& Center, float Radius)
{
	const int32 Count = Rand.RandRange(2, 7);
	for (int32 I = 0; I < Count; ++I)
	{
		const bool bTall = Rand.FRand() < 0.2f;
		const FVector Size(Rand.FRandRange(30.f, 90.f), Rand.FRandRange(30.f, 90.f), bTall ? Rand.FRandRange(90.f, 180.f) : Rand.FRandRange(25.f, 100.f));
		const FVector Lo(-Size.X * 0.5f, -Size.Y * 0.5f, 0.f), Hi(Size.X * 0.5f, Size.Y * 0.5f, 0.f);
		for (int32 Try = 0; Try < 20; ++Try)
		{
			const float Ang = Rand.FRandRange(0.f, 2.f * PI);
			const FVector2D P = Center + FVector2D(FMath::Cos(Ang), FMath::Sin(Ang)) * (Radius * FMath::Sqrt(Rand.FRand()));
			const float Yaw = Rand.FRandRange(0.f, 360.f);
			if (!AreaFree(P, Yaw, Lo, Hi, 15.f))
			{
				continue;
			}
			const float Pick = Rand.FRand();
			const FLinearColor Color = Pick < 0.6f ? FLinearColor(Rand.FRandRange(0.28f, 0.42f), Rand.FRandRange(0.18f, 0.30f), Rand.FRandRange(0.08f, 0.16f))
				: (Pick < 0.85f ? FLinearColor(Rand.FRandRange(0.1f, 0.5f), Rand.FRandRange(0.1f, 0.5f), Rand.FRandRange(0.1f, 0.5f)) : FLinearColor(0.55f, 0.5f, 0.45f));
			AddCube(FVector(P.X, P.Y, 0.f), Yaw, Size, Color);
			Occupy(P, Yaw, Lo, Hi);
			break;
		}
	}
}

void ASyntheticDataGenerator::BuildScene(int32 SceneIndex)
{
	ClearScene();
	CurrentScene = SceneIndex;
	Rand.Initialize(Seed * 1000003 + SceneIndex * 7919 + SceneRetries * 104729);
	Occupied = FixedRects;
	RandomiseEnvironment();

	// ~12% of scenes have no pallet at all: racks / totes / boxes only, as hard negatives.
	const bool bNoPallets = Rand.FRand() < 0.12f;
	FVector2D Center;
	if (FixedRacks.Num() > 0 && Rand.FRand() < 0.3f)
	{
		// In front of one of the hall's racks, so racks are a partial backdrop / occluder.
		const FPlacedProp& R = FixedRacks[Rand.RandRange(0, FixedRacks.Num() - 1)];
		const FVector Loc = R.Xf.GetLocation();
		Center = FVector2D(Loc.X + Rand.FRandRange(-200.f, 200.f), Loc.Y + (Rand.FRand() < 0.5f ? -1.f : 1.f) * Rand.FRandRange(200.f, 320.f));
		Center.X = FMath::Clamp(Center.X, -HallX + 150.f, HallX - 150.f);
		Center.Y = FMath::Clamp(Center.Y, -HallY + 150.f, HallY - 150.f);
	}
	else
	{
		Center = FVector2D(Rand.FRandRange(-HallX + 250.f, HallX - 250.f), Rand.FRandRange(-HallY + 250.f, HallY - 250.f));
	}
	SceneCenter = Center;

	if (!bNoPallets)
	{
		const float R = Rand.FRand();
		const int32 Count = R < 0.30f ? 1 : (R < 0.60f ? 2 : (R < 0.85f ? 3 : 4));
		for (int32 I = 0; I < Count; ++I)
		{
			TryPlacePallet(Center, 320.f);
		}
	}
	if (bNoPallets || Rand.FRand() < 0.45f)
	{
		PlaceLooseTotes(Center, 400.f);
	}
	if (bNoPallets || Rand.FRand() < 0.5f)
	{
		PlaceRack(Center, 650.f);
	}
	AddClutter(Center, 550.f);

	// What the camera may aim at: pallets first; a hard-negative scene aims at totes / racks.
	for (const FPlacedPallet& P : PlacedPallets)
	{
		FTarget T;
		T.Pos = P.Xf.GetLocation();
		T.MidZ = 0.5f * (P.Lo.Z + P.Hi.Z);
		for (const FPocketDef& Pk : P.Def->Pockets)
		{
			const FVector N = P.Xf.TransformVectorNoScale(Pk.Normal);
			const FVector2D Dir = FVector2D(N.X, N.Y).GetSafeNormal();
			bool bKnown = false;
			for (const FVector2D& D : T.FrontDirs)
			{
				bKnown |= FVector2D::DotProduct(D, Dir) > 0.98f;
			}
			if (!bKnown)
			{
				T.FrontDirs.Add(Dir);
			}
		}
		Targets.Add(T);
	}
	if (Targets.Num() == 0)
	{
		const auto AddProp = [this](const FPlacedProp& P, bool bRack)
		{
			FTarget T;
			T.Pos = P.Xf.GetLocation();
			T.MidZ = bRack ? 100.f : 20.f;
			if (bRack)
			{
				const FVector Y = P.Xf.GetUnitAxis(EAxis::Y);
				T.FrontDirs = { FVector2D(Y.X, Y.Y), -FVector2D(Y.X, Y.Y) };
			}
			Targets.Add(T);
		};
		for (const FPlacedProp& P : PlacedProps)
		{
			AddProp(P, P.ClassId == ForkliftClass::Rack);
		}
		if (Targets.Num() == 0)
		{
			for (const FPlacedProp& R : FixedRacks)
			{
				AddProp(R, true);
			}
		}
	}
}

// ---- Generator: views --------------------------------------------------------------------------------------------

bool ASyntheticDataGenerator::SampleView(FVector& OutPos, FRotator& OutRot, int32& OutTarget)
{
	if (Targets.Num() == 0)
	{
		return false;
	}
	OutTarget = Rand.RandRange(0, Targets.Num() - 1);
	const FTarget& T = Targets[OutTarget];

	// Look at the target's front face most of the time (so pockets are visible), otherwise from anywhere.
	float Bearing = Rand.FRandRange(0.f, 360.f);
	if (T.FrontDirs.Num() > 0 && Rand.FRand() < 0.7f)
	{
		const FVector2D D = T.FrontDirs[Rand.RandRange(0, T.FrontDirs.Num() - 1)];
		Bearing = FMath::RadiansToDegrees(FMath::Atan2(D.Y, D.X)) + Rand.FRandRange(-35.f, 35.f);
	}
	const float Dist = 150.f + 450.f * FMath::Pow(Rand.FRand(), 1.3f);   // 1.5 - 6 m, biased close
	const float BRad = FMath::DegreesToRadians(Bearing);
	const FVector2D Away(FMath::Cos(BRad), FMath::Sin(BRad));
	const FVector2D Side(-Away.Y, Away.X);
	FVector2D P2 = FVector2D(T.Pos.X, T.Pos.Y) + Away * Dist + Side * Rand.FRandRange(-80.f, 80.f);

	const float Height = Rand.FRand() < 0.4f ? Rand.FRandRange(20.f, 90.f) : Rand.FRandRange(20.f, 250.f);
	if (FMath::Abs(P2.X) > HallX + 40.f || FMath::Abs(P2.Y) > HallY + 40.f)
	{
		return false;
	}
	for (const FRect2D& R : Occupied)
	{
		if (RectsOverlap(P2, FVector2D(1, 0), FVector2D(0, 1), FVector2D(20, 20), R.C, R.Ax, R.Ay, R.Half, 10.f))
		{
			return false;  // the camera would be inside something
		}
	}

	const FVector2D ToT = FVector2D(T.Pos.X, T.Pos.Y) - P2;
	const float YawToTarget = FMath::RadiansToDegrees(FMath::Atan2(ToT.Y, ToT.X));
	const float Yaw = YawToTarget + (Rand.FRand() < 0.5f ? Rand.FRandRange(-8.f, 8.f) : Rand.FRandRange(-30.f, 30.f));
	const float PitchLook = FMath::RadiansToDegrees(FMath::Atan2(T.MidZ - Height, FMath::Max(ToT.Size(), 1.f)));
	const float Pitch = FMath::Clamp(PitchLook + Rand.FRandRange(-10.f, 10.f), -30.f, 0.f);
	OutPos = FVector(P2.X, P2.Y, Height);
	OutRot = FRotator(Pitch, Yaw, Rand.FRandRange(-2.f, 2.f));
	return true;
}

void ASyntheticDataGenerator::PlaceOccluder(const FVector& CamPos, const FTarget& Target)
{
	if (Occluder)
	{
		Occluder->DestroyComponent();
		Occluder = nullptr;
	}
	if (Rand.FRand() > 0.35f || !CubeMesh)
	{
		return;
	}
	// Something between the camera and the target, roughly on the line of sight: partial occlusion.
	const FVector2D A(CamPos.X, CamPos.Y), B(Target.Pos.X, Target.Pos.Y);
	const FVector2D Dir = (B - A).GetSafeNormal();
	const FVector2D P = A + (B - A) * Rand.FRandRange(0.35f, 0.7f) + FVector2D(-Dir.Y, Dir.X) * Rand.FRandRange(-45.f, 45.f);
	const FVector Size(Rand.FRandRange(30.f, 70.f), Rand.FRandRange(30.f, 70.f), Rand.FRandRange(40.f, 130.f));
	const FVector Lo(-Size.X * 0.5f, -Size.Y * 0.5f, 0.f), Hi(Size.X * 0.5f, Size.Y * 0.5f, 0.f);
	const float Yaw = Rand.FRandRange(0.f, 360.f);
	if (!AreaFree(P, Yaw, Lo, Hi, 5.f))
	{
		return;
	}
	Occluder = AddCube(FVector(P.X, P.Y, 0.f), Yaw, Size, FLinearColor(Rand.FRandRange(0.1f, 0.5f), Rand.FRandRange(0.1f, 0.4f), Rand.FRandRange(0.05f, 0.4f)), true);
}

// ---- Generator: labels ---------------------------------------------------------------------------------------------

bool ASyntheticDataGenerator::ProjectPoint(const FTransform& Cam, const FVector& World, FVector2D& OutPx, float& OutZ) const
{
	const FVector L = Cam.InverseTransformPositionNoScale(World);  // X forward, Y right, Z up
	OutZ = L.X;
	if (L.X < 10.f)
	{
		return false;  // behind (or within 10 cm of) the camera
	}
	const float F = (ImageW * 0.5f) / FMath::Tan(FMath::DegreesToRadians(Camera->FOVAngle * 0.5f));
	OutPx.X = ImageW * 0.5f + F * L.Y / L.X;
	OutPx.Y = ImageH * 0.5f - F * L.Z / L.X;
	return true;
}

void ASyntheticDataGenerator::SamplePoints(const FTransform& Cam, const TArray<float>& Depth, const TArray<FVector>& Points, int32& OutTotal, int32& OutInFrame, int32& OutUnoccluded) const
{
	OutTotal = Points.Num();
	OutInFrame = 0;
	OutUnoccluded = 0;
	for (const FVector& P : Points)
	{
		FVector2D Px;
		float Z;
		if (!ProjectPoint(Cam, P, Px, Z) || Px.X < 0.f || Px.Y < 0.f || Px.X >= ImageW || Px.Y >= ImageH)
		{
			continue;
		}
		++OutInFrame;
		// Something nearer than the point along the same ray hides it. The depth buffer is the first opaque surface.
		const float D = Depth[FMath::FloorToInt(Px.Y) * ImageW + FMath::FloorToInt(Px.X)];
		if (D >= Z - FMath::Max(5.f, 0.015f * Z))
		{
			++OutUnoccluded;
		}
	}
}

bool ASyntheticDataGenerator::LabelBox(const FTransform& Cam, const TArray<float>& Depth, const FTransform& ObjXf, const FVector& Lo, const FVector& Hi,
                                       float MinPx, FLabelRow& Out, float& OutInFrame, float& OutUnocc) const
{
	// Visibility: a grid of points on each box face that points at the camera. Built first because the label box
	// itself is now taken from these (see below), not from the object's full 3D extent.
	TArray<FVector> Points;
	const FVector CamPos = Cam.GetLocation();
	const FVector C = (Lo + Hi) * 0.5f, E = (Hi - Lo) * 0.5f;
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		for (const float Sign : { -1.f, 1.f })
		{
			FVector N = FVector::ZeroVector;
			N[Axis] = Sign;
			const FVector FaceC = C + N * E[Axis];
			if (FVector::DotProduct(ObjXf.TransformVectorNoScale(N), CamPos - ObjXf.TransformPosition(FaceC)) <= 0.f)
			{
				continue;
			}
			const int32 A1 = (Axis + 1) % 3, A2 = (Axis + 2) % 3;
			for (int32 I = 0; I < 5; ++I)
			{
				for (int32 J = 0; J < 5; ++J)
				{
					FVector P = FaceC;
					P[A1] += ((I + 0.5f) / 5.f * 2.f - 1.f) * E[A1] * 0.96f;
					P[A2] += ((J + 0.5f) / 5.f * 2.f - 1.f) * E[A2] * 0.96f;
					Points.Add(ObjXf.TransformPosition(P));
				}
			}
		}
	}
	int32 Total, InFrame, Unocc;
	SamplePoints(Cam, Depth, Points, Total, InFrame, Unocc);
	OutInFrame = Total > 0 ? static_cast<float>(InFrame) / Total : 0.f;
	OutUnocc = InFrame > 0 ? static_cast<float>(Unocc) / InFrame : 0.f;

	// The label box: tight around the points above that are actually visible (in frame, not hidden behind something
	// nearer - including the object's own far side), rather than the projected extent of the full 3D box. Projecting
	// all 8 corners made the box balloon whenever the object sat at an oblique yaw to the camera, since it then
	// includes corners a viewer can never see; hugging only the visible points tracks the true silhouette instead.
	FVector2D Min(1e9, 1e9), Max(-1e9, -1e9);
	int32 VisibleCount = 0;
	for (const FVector& P : Points)
	{
		FVector2D Px;
		float Z;
		if (!ProjectPoint(Cam, P, Px, Z) || Px.X < 0.f || Px.Y < 0.f || Px.X >= ImageW || Px.Y >= ImageH)
		{
			continue;
		}
		const float D = Depth[FMath::FloorToInt(Px.Y) * ImageW + FMath::FloorToInt(Px.X)];
		if (D < Z - FMath::Max(5.f, 0.015f * Z))
		{
			continue;  // hidden behind something nearer (another object, or the far side of this one)
		}
		Min.X = FMath::Min(Min.X, Px.X);
		Min.Y = FMath::Min(Min.Y, Px.Y);
		Max.X = FMath::Max(Max.X, Px.X);
		Max.Y = FMath::Max(Max.Y, Px.Y);
		++VisibleCount;
	}
	if (VisibleCount < 4)
	{
		// Too little of the object visible to trust a silhouette box (Keep()'s visibility-ratio thresholds reject
		// objects this occluded before the box is ever written) - fall back to the full projected extent so the
		// caller still gets a box to test and reject, rather than none at all.
		for (int32 I = 0; I < 8; ++I)
		{
			const FVector Local((I & 1) ? Hi.X : Lo.X, (I & 2) ? Hi.Y : Lo.Y, (I & 4) ? Hi.Z : Lo.Z);
			FVector2D Px;
			float Z;
			if (!ProjectPoint(Cam, ObjXf.TransformPosition(Local), Px, Z))
			{
				return false;
			}
			Min.X = FMath::Min(Min.X, Px.X);
			Min.Y = FMath::Min(Min.Y, Px.Y);
			Max.X = FMath::Max(Max.X, Px.X);
			Max.Y = FMath::Max(Max.Y, Px.Y);
		}
	}
	if (Max.X < 0.f || Max.Y < 0.f || Min.X > ImageW || Min.Y > ImageH)
	{
		return false;
	}
	Out.X0 = FMath::Clamp(Min.X, 0.f, static_cast<float>(ImageW));
	Out.Y0 = FMath::Clamp(Min.Y, 0.f, static_cast<float>(ImageH));
	Out.X1 = FMath::Clamp(Max.X, 0.f, static_cast<float>(ImageW));
	Out.Y1 = FMath::Clamp(Max.Y, 0.f, static_cast<float>(ImageH));
	if (Out.X1 - Out.X0 < MinPx || Out.Y1 - Out.Y0 < MinPx)
	{
		return false;
	}
	return true;
}

// ---- Generator: a frame --------------------------------------------------------------------------------------------

bool ASyntheticDataGenerator::RenderFrame(int32 Index)
{
	const FString Split = SplitOf(CurrentScene);
	for (int32 Try = 0; Try < 8; ++Try)
	{
		FVector Pos;
		FRotator Rot;
		int32 TargetIndex = 0;
		if (!SampleView(Pos, Rot, TargetIndex))
		{
			continue;
		}
		Camera->SetWorldLocationAndRotation(Pos, Rot);
		PlaceOccluder(Pos, Targets[TargetIndex]);
		const float EV = BaseEV + Rand.FRandRange(-0.8f, 0.8f);
		Camera->SetExposureEV(EV);
		Camera->Capture();

		TArray<FColor> Pixels;
		TArray<float> Depth;
		if (!Camera->ReadPixelsBgra(Pixels) || !Camera->ReadDepthCm(Depth) || Pixels.Num() != ImageW * ImageH || Depth.Num() != ImageW * ImageH)
		{
			UE_LOG(LogSynthData, Warning, TEXT("[DataGen] readback failed"));
			continue;
		}
		for (float& D : Depth)
		{
			D = FMath::IsFinite(D) ? FMath::Clamp(D, 0.f, 60000.f) : 60000.f;  // sky comes back huge; half floats top out at 65504
		}

		const FTransform Cam = Camera->GetComponentTransform();
		TArray<FLabelRow> Rows;
		TArray<TSharedPtr<FJsonValue>> ObjectsMeta;
		int32 PalletLabels = 0;

		const auto Keep = [](float InFrame, float Unocc) { return InFrame >= 0.35f && Unocc >= 0.30f && InFrame * Unocc >= 0.20f; };
		const auto AddBox = [&](int32 ClassId, const TCHAR* Asset, const FString& Standard, const FString& Load, const FTransform& Xf, const FVector& Lo, const FVector& Hi, float MinPx) -> TSharedPtr<FJsonObject>
		{
			FLabelRow Row;
			float InFrame = 0.f, Unocc = 0.f;
			if (!LabelBox(Cam, Depth, Xf, Lo, Hi, MinPx, Row, InFrame, Unocc))
			{
				return nullptr;
			}
			Row.ClassId = ClassId;
			// An object sitting in flat black shadow cannot be learned from: drop it (thin pockets are dark inside, so
			// only whole-object boxes are tested). Luma is sampled on a 10 x 10 grid over the clipped box.
			float MeanLuma = 0.f, StdLuma = 0.f;
			{
				double Sum = 0.0, Sum2 = 0.0;
				for (int32 I = 0; I < 10; ++I)
				{
					for (int32 J = 0; J < 10; ++J)
					{
						const int32 X = FMath::Clamp(FMath::FloorToInt(Row.X0 + (Row.X1 - Row.X0) * (I + 0.5f) / 10.f), 0, ImageW - 1);
						const int32 Y = FMath::Clamp(FMath::FloorToInt(Row.Y0 + (Row.Y1 - Row.Y0) * (J + 0.5f) / 10.f), 0, ImageH - 1);
						const FColor& C = Pixels[Y * ImageW + X];
						const double L = 0.2126 * C.R + 0.7152 * C.G + 0.0722 * C.B;
						Sum += L;
						Sum2 += L * L;
					}
				}
				MeanLuma = static_cast<float>(Sum / 100.0);
				StdLuma = static_cast<float>(FMath::Sqrt(FMath::Max(0.0, Sum2 / 100.0 - (Sum / 100.0) * (Sum / 100.0))));
			}
			const bool bDark = MeanLuma < 14.f && StdLuma < 8.f;
			const bool bKept = Keep(InFrame, Unocc) && !bDark;
			TSharedPtr<FJsonObject> M = MakeShared<FJsonObject>();
			M->SetStringField(TEXT("class"), ClassNames[ClassId]);
			M->SetNumberField(TEXT("class_id"), ClassId);
			M->SetStringField(TEXT("asset"), Asset);
			if (!Standard.IsEmpty())
			{
				M->SetStringField(TEXT("standard"), Standard);
			}
			if (!Load.IsEmpty())
			{
				M->SetStringField(TEXT("load"), Load);
			}
			M->SetArrayField(TEXT("box_px"), NumArray({ Row.X0, Row.Y0, Row.X1, Row.Y1 }));
			M->SetNumberField(TEXT("in_frame"), InFrame);
			M->SetNumberField(TEXT("unoccluded"), Unocc);
			M->SetNumberField(TEXT("mean_luma"), MeanLuma);
			M->SetBoolField(TEXT("too_dark"), bDark);
			M->SetBoolField(TEXT("kept"), bKept);
			M->SetArrayField(TEXT("center_cam_cm"), VecArray(Cam.InverseTransformPositionNoScale(Xf.TransformPosition((Lo + Hi) * 0.5f))));
			M->SetArrayField(TEXT("world_pos_cm"), VecArray(Xf.GetLocation()));
			M->SetNumberField(TEXT("yaw_deg"), Xf.Rotator().Yaw);
			if (bKept)
			{
				Row.Meta = M;
				Rows.Add(Row);
			}
			ObjectsMeta.Add(MakeShared<FJsonValueObject>(M));
			return M;
		};

		// Pallets and loaded pallets, each with the pockets of the faces that point at the camera.
		for (const FPlacedPallet& P : PlacedPallets)
		{
			TSharedPtr<FJsonObject> M = AddBox(P.ClassId, *P.Def->Name, P.Def->Standard, P.Load, P.Xf, P.Lo, P.Hi, 8.f);
			if (M && M->GetBoolField(TEXT("kept")))
			{
				++PalletLabels;
			}
			TArray<TSharedPtr<FJsonValue>> PocketsMeta;
			for (const FPocketDef& Pk : P.Def->Pockets)
			{
				const bool bAlongX = FMath::Abs(Pk.Normal.X) > 0.5f;
				const float Depth_ = bAlongX ? Pk.Size.X : Pk.Size.Y, Lat = bAlongX ? Pk.Size.Y : Pk.Size.X, Hgt = Pk.Size.Z;
				const FVector LatAxis = bAlongX ? FVector(0, 1, 0) : FVector(1, 0, 0);
				const FVector OpenC = Pk.Center + Pk.Normal * (Depth_ * 0.5f);  // centre of the opening in the entry plane
				const FVector NW = P.Xf.TransformVectorNoScale(Pk.Normal);
				const FVector OpenW = P.Xf.TransformPosition(OpenC);
				const FVector ToCam = (Cam.GetLocation() - OpenW).GetSafeNormal();
				if (FVector::DotProduct(NW, ToCam) < 0.12f)
				{
					continue;  // the face points away from (or almost edge-on to) the camera
				}
				FVector2D Min(1e9, 1e9), Max(-1e9, -1e9);
				bool bOk = true;
				for (int32 I = 0; I < 4 && bOk; ++I)
				{
					const FVector Corner = OpenC + LatAxis * ((I & 1) ? Lat : -Lat) * 0.5f + FVector(0, 0, ((I & 2) ? Hgt : -Hgt) * 0.5f);
					FVector2D Px;
					float Z;
					bOk = ProjectPoint(Cam, P.Xf.TransformPosition(Corner), Px, Z);
					Min = FVector2D(FMath::Min(Min.X, Px.X), FMath::Min(Min.Y, Px.Y));
					Max = FVector2D(FMath::Max(Max.X, Px.X), FMath::Max(Max.Y, Px.Y));
				}
				if (!bOk || Max.X < 0.f || Max.Y < 0.f || Min.X > ImageW || Min.Y > ImageH)
				{
					continue;
				}
				FLabelRow Row;
				Row.ClassId = ForkliftClass::PalletPocket;
				Row.X0 = FMath::Clamp(Min.X, 0.f, static_cast<float>(ImageW));
				Row.Y0 = FMath::Clamp(Min.Y, 0.f, static_cast<float>(ImageH));
				Row.X1 = FMath::Clamp(Max.X, 0.f, static_cast<float>(ImageW));
				Row.Y1 = FMath::Clamp(Max.Y, 0.f, static_cast<float>(ImageH));
				const float W = Row.X1 - Row.X0, H = Row.Y1 - Row.Y0;
				if (W < 3.f || H < 3.f)
				{
					continue;
				}
				// Visibility: points across the opening, nudged 1 cm outward so they sit in free space in front of it.
				TArray<FVector> Points;
				for (int32 I = 0; I < 5; ++I)
				{
					for (int32 J = 0; J < 3; ++J)
					{
						const FVector Q = OpenC + Pk.Normal * 1.f + LatAxis * (((I + 0.5f) / 5.f) * 2.f - 1.f) * Lat * 0.44f + FVector(0, 0, (((J + 0.5f) / 3.f) * 2.f - 1.f) * Hgt * 0.44f);
						Points.Add(P.Xf.TransformPosition(Q));
					}
				}
				int32 Total, InFrame, Unocc;
				SamplePoints(Cam, Depth, Points, Total, InFrame, Unocc);
				const float InF = Total > 0 ? static_cast<float>(InFrame) / Total : 0.f;
				const float Un = InFrame > 0 ? static_cast<float>(Unocc) / InFrame : 0.f;
				const bool bKept = InF >= 0.6f && Un >= 0.5f;

				TSharedPtr<FJsonObject> PM = MakeShared<FJsonObject>();
				PM->SetStringField(TEXT("entry"), Pk.Entry);
				PM->SetArrayField(TEXT("opening_center_cam_cm"), VecArray(Cam.InverseTransformPositionNoScale(OpenW)));
				PM->SetArrayField(TEXT("normal_cam"), VecArray(Cam.GetRotation().UnrotateVector(NW)));
				PM->SetNumberField(TEXT("width_cm"), Lat);
				PM->SetNumberField(TEXT("height_cm"), Hgt);
				PM->SetNumberField(TEXT("depth_cm"), Pk.DepthCm);
				PM->SetArrayField(TEXT("box_px"), NumArray({ Row.X0, Row.Y0, Row.X1, Row.Y1 }));
				PM->SetNumberField(TEXT("in_frame"), InF);
				PM->SetNumberField(TEXT("unoccluded"), Un);
				PM->SetBoolField(TEXT("kept"), bKept);
				PocketsMeta.Add(MakeShared<FJsonValueObject>(PM));
				if (bKept)
				{
					Row.Meta = PM;
					Rows.Add(Row);
				}
			}
			if (M)
			{
				M->SetArrayField(TEXT("pockets"), PocketsMeta);
			}
		}
		for (const FPlacedProp& P : PlacedProps)
		{
			AddBox(P.ClassId, *P.Asset, FString(), FString(), P.Xf, P.Lo, P.Hi, P.ClassId == ForkliftClass::Rack ? 16.f : 8.f);
		}
		for (const FPlacedProp& R : FixedRacks)
		{
			AddBox(ForkliftClass::Rack, *R.Asset, FString(), FString(), R.Xf, R.Lo, R.Hi, 16.f);
		}

		if (Rows.Num() == 0)
		{
			continue;  // nothing labelled in view; take another camera pose
		}

		// ---- Write ---------------------------------------------------------------------------------------------
		const FString Name = FString::Printf(TEXT("f%06d"), Index);
		FString LabelText;
		for (const FLabelRow& Row : Rows)
		{
			const float W = Row.X1 - Row.X0, H = Row.Y1 - Row.Y0;
			LabelText += FString::Printf(TEXT("%d %.6f %.6f %.6f %.6f\n"), Row.ClassId, (Row.X0 + W * 0.5f) / ImageW, (Row.Y0 + H * 0.5f) / ImageH, W / ImageW, H / ImageH);
			ClassCounts.FindOrAdd(Split + TEXT("/") + ClassNames[Row.ClassId]) += 1;
			if (Row.ClassId == ForkliftClass::PalletPocket)
			{
				++PocketBoxes;
				TinyPockets += FMath::Min(W, H) < 8.f ? 1 : 0;
			}
		}
		NegativeFrames += PalletLabels == 0 ? 1 : 0;
		SplitFrames.FindOrAdd(Split) += 1;
		FFileHelper::SaveStringToFile(LabelText, *(OutputDir / TEXT("labels") / Split / (Name + TEXT(".txt"))));

		TSharedPtr<FJsonObject> Meta = MakeShared<FJsonObject>();
		Meta->SetStringField(TEXT("frame"), Name);
		Meta->SetNumberField(TEXT("index"), Index);
		Meta->SetNumberField(TEXT("scene"), CurrentScene);
		Meta->SetNumberField(TEXT("view"), Index % FramesPerScene);
		Meta->SetStringField(TEXT("split"), Split);
		Meta->SetNumberField(TEXT("seed"), Seed);
		TSharedPtr<FJsonObject> Image = MakeShared<FJsonObject>();
		Image->SetNumberField(TEXT("w"), ImageW);
		Image->SetNumberField(TEXT("h"), ImageH);
		Image->SetNumberField(TEXT("fov_deg"), Camera->FOVAngle);
		Image->SetNumberField(TEXT("exposure_ev"), EV);
		Meta->SetObjectField(TEXT("image"), Image);
		TSharedPtr<FJsonObject> CamMeta = MakeShared<FJsonObject>();
		CamMeta->SetArrayField(TEXT("pos_cm"), VecArray(Pos));
		CamMeta->SetArrayField(TEXT("rot_pitch_yaw_roll_deg"), NumArray({ Rot.Pitch, Rot.Yaw, Rot.Roll }));
		Meta->SetObjectField(TEXT("camera"), CamMeta);
		if (LightingMeta.IsValid())
		{
			Meta->SetObjectField(TEXT("lighting"), LightingMeta);
		}
		Meta->SetBoolField(TEXT("occluder"), Occluder != nullptr);
		Meta->SetArrayField(TEXT("objects"), ObjectsMeta);
		FFileHelper::SaveStringToFile(ToJsonString(Meta), *(OutputDir / TEXT("meta") / Split / (Name + TEXT(".json"))));

		// PNG compression and the float16 depth file go to a worker so the game thread can render the next frame.
		const FString PngPath = OutputDir / TEXT("images") / Split / (Name + TEXT(".png"));
		const FString DepthPath = OutputDir / TEXT("depth") / Split / (Name + TEXT(".npy"));
		const int32 W = ImageW, H = ImageH;
		const bool bHalf = bDepthHalf;
		PendingWrites.Increment();
		Async(EAsyncExecution::ThreadPool, [this, Pixels = MoveTemp(Pixels), Depth = MoveTemp(Depth), PngPath, DepthPath, W, H, bHalf]() mutable
		{
			for (FColor& C : Pixels)
			{
				C.A = 255;
			}
			TArray64<uint8> Png;
			FImageUtils::PNGCompressImageArray(W, H, TArrayView64<const FColor>(Pixels.GetData(), Pixels.Num()), Png);
			FFileHelper::SaveArrayToFile(Png, *PngPath);
			if (bHalf)
			{
				ForkliftNpy::WriteF16(DepthPath, Depth.GetData(), H, W);
			}
			else
			{
				ForkliftNpy::WriteF32(DepthPath, Depth.GetData(), H, W);
			}
			PendingWrites.Decrement();
		});
		return true;
	}
	++Rejected;
	return false;
}

void ASyntheticDataGenerator::FinishRun()
{
	const double Elapsed = FPlatformTime::Seconds() - StartTime;
	const int32 Written = NextFrame - StartFrame;

	const FString Yaml = FString::Printf(
		TEXT("# Generated by ASyntheticDataGenerator (SyntheticDataGenerator.cpp). Boxes hug visible points, not the full 3D extent; see the class comment for the rules.\n")
		TEXT("path: %s\ntrain: images/train\nval: images/val\ntest: images/test\nnc: 5\nnames:\n  0: pallet\n  1: loaded_pallet\n  2: pallet_pocket\n  3: tote\n  4: rack\n"),
		*OutputDir.Replace(TEXT("\\"), TEXT("/")));
	FFileHelper::SaveStringToFile(Yaml, *(OutputDir / TEXT("data.yaml")));

	TSharedPtr<FJsonObject> S = MakeShared<FJsonObject>();
	S->SetStringField(TEXT("dataset"), DatasetName);
	S->SetNumberField(TEXT("seed"), Seed);
	S->SetNumberField(TEXT("first_frame"), StartFrame);
	S->SetNumberField(TEXT("frames_written"), Written);
	S->SetNumberField(TEXT("attempts_rejected"), Rejected);
	S->SetNumberField(TEXT("seconds"), Elapsed);
	S->SetNumberField(TEXT("images_per_second"), Elapsed > 0.0 ? Written / Elapsed : 0.0);
	S->SetNumberField(TEXT("frames_without_pallet_label"), NegativeFrames);
	S->SetNumberField(TEXT("pocket_boxes"), PocketBoxes);
	S->SetNumberField(TEXT("pocket_boxes_under_8px"), TinyPockets);
	TSharedPtr<FJsonObject> Splits = MakeShared<FJsonObject>();
	for (const TPair<FString, int32>& P : SplitFrames)
	{
		Splits->SetNumberField(P.Key, P.Value);
	}
	S->SetObjectField(TEXT("frames_per_split"), Splits);
	TSharedPtr<FJsonObject> Classes = MakeShared<FJsonObject>();
	for (const TPair<FString, int32>& P : ClassCounts)
	{
		Classes->SetNumberField(P.Key, P.Value);
	}
	S->SetObjectField(TEXT("boxes_per_split_class"), Classes);
	FFileHelper::SaveStringToFile(ToJsonString(S), *(OutputDir / FString::Printf(TEXT("summary_from_%06d.json"), StartFrame)));

	UE_LOG(LogSynthData, Log, TEXT("[DataGen] done: %d frames in %.1f s = %.2f images/s (%d attempts rejected), pockets %d (%d under 8 px), frames without a pallet label %d -> %s"),
		Written, Elapsed, Elapsed > 0.0 ? Written / Elapsed : 0.0, Rejected, PocketBoxes, TinyPockets, NegativeFrames, *OutputDir);
	FPlatformMisc::RequestExit(false);
}

// ---- Generator: main loop ---------------------------------------------------------------------------------------

void ASyntheticDataGenerator::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	switch (State)
	{
	case EState::WaitReady:
	{
		// Same readiness rule as photo mode: nothing renders correctly while shaders / meshes are still compiling.
		if ((GShaderCompilingManager && GShaderCompilingManager->IsCompiling()) || FAssetCompilingManager::Get().GetNumRemainingAssets() > 0)
		{
			WaitFrames = 0;
			return;
		}
		if (++WaitFrames < 30)
		{
			return;
		}
		if (!LoadCatalog())
		{
			FPlatformMisc::RequestExit(false);
			State = EState::Done;
			return;
		}
		PrepareWorld();
		CreateCamera();
		StartTime = FPlatformTime::Seconds();
		SettleTicks = 5;
		State = EState::Settle;
		break;
	}
	case EState::Settle:
		if (--SettleTicks <= 0)
		{
			State = EState::Running;
		}
		break;
	case EState::Running:
	{
		if (NextFrame >= StartFrame + NumFrames)
		{
			State = EState::Flushing;
			break;
		}
		const int32 Scene = NextFrame / FramesPerScene;
		if (Scene != CurrentScene)
		{
			BuildScene(Scene);
			SettleTicks = 5;  // let the new lights, sky capture and Nanite pages settle
			State = EState::Settle;
			break;
		}
		if (RenderFrame(NextFrame))
		{
			++NextFrame;
			SceneRetries = 0;
			if ((NextFrame - StartFrame) % 25 == 0)
			{
				const double Elapsed = FPlatformTime::Seconds() - StartTime;
				UE_LOG(LogSynthData, Log, TEXT("[DataGen] %d / %d frames, %.2f images/s"), NextFrame - StartFrame, NumFrames, (NextFrame - StartFrame) / Elapsed);
			}
		}
		else if (Rejected > 20 + NumFrames * 10)
		{
			UE_LOG(LogSynthData, Error, TEXT("[DataGen] giving up: %d rejected attempts for %d frames"), Rejected, NextFrame - StartFrame);
			State = EState::Flushing;
		}
		else if (++SceneRetries >= 3)
		{
			// The layout keeps producing empty views: draw a different layout for this scene index.
			CurrentScene = -1;
		}
		break;
	}
	case EState::Flushing:
		if (PendingWrites.GetValue() == 0)
		{
			State = EState::Done;
			FinishRun();
		}
		break;
	default:
		break;
	}
}
