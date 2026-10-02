#include "ForkliftVehicle.h"

#include "Camera/CameraComponent.h"
#include "CollisionQueryParams.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "ForkliftCameraComponent.h"
#include "ForkliftAutoPickupComponent.h"
#include "ForkliftNavComponent.h"
#include "ForkliftMissionComponent.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputCoreTypes.h"
#include "LidarSensorComponent.h"
#include "ForkliftPalletFinderComponent.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodyInstance.h"
#include "UObject/StrongObjectPtr.h"

// ---- Rig tables ---------------------------------------------------------------------------------------------
// Everything model specific lives here. Positions are forklift-local cm in UE axes (+X forward = where the forks
// point, +Y right, +Z up). The USDs are right-handed, so converting a baked-USD point negates Y (FromUsd).

namespace ForkliftLayout
{
	FVector FromUsd(float X, float Y, float Z) { return FVector(X, -Y, Z); }

	constexpr float GravityCmPerSec2 = 980.f;

	// Rotation of `Q` about a point `P`.
	FTransform RotateAbout(const FVector& P, const FQuat& Q) { return FTransform(Q, P - Q.RotateVector(P)); }
}

struct FForkliftWheelSpec
{
	int32 MeshSlot = 0;      // index into WheelMeshes
	int32 RotatorSlot = -1;  // index into RotatorMeshes that carries this wheel when it steers, or -1
	FVector Center = FVector::ZeroVector;
	FVector SteerPivot = FVector::ZeroVector;
	float Radius = 30.f;
	bool bDriven = false;
	bool bSteered = false;
	int32 Group = 0;         // 0 / 1: the two support "axles" the static load is split between
};

struct FForkliftBoxSpec
{
	FVector Center = FVector::ZeroVector;  // at fork rest pose, forklift-local cm
	FVector Extent = FVector::ZeroVector;  // half sizes, cm
};

struct FForkliftRigSpec
{
	FString MeshDir;
	FString MeshPrefix;
	FString BodyMesh, LiftMesh, GlassMesh;  // GlassMesh empty = the model has none
	TArray<FString> WheelMeshes;
	TArray<FString> RotatorMeshes;
	TArray<FForkliftWheelSpec> Wheels;

	float BodyMassKg = 3000.f;
	FVector CenterOfMassCm = FVector::ZeroVector;
	float MaxWheelSpeedDeg = 360.f;
	float MaxSteerDeg = 30.f;
	float LiftMinCm = 0.f;
	float LiftMaxCm = 200.f;

	FVector CameraPosCm = FVector::ZeroVector;
	FRotator CameraRot = FRotator::ZeroRotator;
	FVector TineCameraPosCm = FVector::ZeroVector;
	FRotator TineCameraRot = FRotator::ZeroRotator;
	FVector LidarPosCm = FVector::ZeroVector;
	FVector LowLidarPosCm = FVector(100.f, 0.f, 30.f);   // body frame; just ahead of the operator platform

	FVector ForkTipCm = FVector::ZeroVector;  // middle of the tine tips at fork rest pose
	float ForkStopXCm = 0.f;                  // a seated pallet's near edge sits here (platform front / backrest)
	TArray<FForkliftBoxSpec> LiftBoxes;       // empty = the Lift mesh's own collision is used
};

namespace
{
	using namespace ForkliftLayout;

	// forklift_c.usd. Wheel centres: x/z from the USD wheel-joint anchors, y from the imported wheel meshes' bounds;
	// radii from those same bounds (front 64.6 cm dia, rear 51.3 cm dia). Rear wheels are turned by the rotators.
	FForkliftRigSpec MakeRigC()
	{
		FForkliftRigSpec R;
		R.MeshDir = TEXT("/Game/Forklift/Meshes/");
		R.MeshPrefix = TEXT("SM_Forklift_");
		R.BodyMesh = TEXT("Body");
		R.LiftMesh = TEXT("Lift");
		R.GlassMesh = TEXT("Glass");
		R.WheelMeshes = { TEXT("FrontWheelL"), TEXT("FrontWheelR"), TEXT("RearWheelL"), TEXT("RearWheelR") };
		R.RotatorMeshes = { TEXT("RotatorL"), TEXT("RotatorR") };

		const FVector RotatorL = FromUsd(-138.591f, 40.613f, 21.023f);
		const FVector RotatorR = FromUsd(-138.591f, -40.613f, 21.023f);
		const auto Wheel = [](int32 Slot, int32 Rot, const FVector& C, const FVector& Pivot, float Radius, bool bDriven, bool bSteered, int32 Group)
		{
			FForkliftWheelSpec W;
			W.MeshSlot = Slot; W.RotatorSlot = Rot; W.Center = C; W.SteerPivot = Pivot; W.Radius = Radius;
			W.bDriven = bDriven; W.bSteered = bSteered; W.Group = Group;
			return W;
		};
		R.Wheels = {
			Wheel(0, -1, FromUsd(26.878f, 49.9f, 32.51f), FVector::ZeroVector, 32.3f, true, false, 0),
			Wheel(1, -1, FromUsd(26.878f, -49.9f, 32.51f), FVector::ZeroVector, 32.3f, true, false, 0),
			Wheel(2, 0, FromUsd(-138.325f, 56.8f, 25.67f), RotatorL, 25.7f, false, true, 1),
			Wheel(3, 1, FromUsd(-138.325f, -56.8f, 25.67f), RotatorR, 25.7f, false, true, 1),
		};

		R.BodyMassKg = 3000.f;
		R.CenterOfMassCm = FVector(-40.f, 0.f, 55.f);
		R.MaxWheelSpeedDeg = 360.f;
		R.MaxSteerDeg = 30.f;
		R.LiftMinCm = 0.f;
		R.LiftMaxCm = 200.f;

		// Fork camera: top of the carriage backrest (which rises to z=120 cm at lift 0), 20 deg down so the tine tips
		// and the floor ahead are in view. Lidar: 10 cm above the overhead guard (roof at z=212 cm), over the seat.
		R.CameraPosCm = FVector(90.f, 0.f, 128.f);
		R.CameraRot = FRotator(-20.f, 0.f, 0.f);
		R.TineCameraPosCm = FVector(86.f, 0.f, 24.f);
		R.TineCameraRot = FRotator(-5.f, 0.f, 0.f);
		R.LidarPosCm = FVector(-55.f, 0.f, 222.f);
		R.ForkTipCm = FVector(187.f, 0.f, 12.f);
		R.ForkStopXCm = 82.f;  // backrest face
		return R;
	}

	// forklift_b.usd, baked with a 180 deg turn about Z so the forks lead along +X (see bake_usd.py). Numbers below
	// are baked-USD metres x100 run through FromUsd. Layout: the drive wheel + swivel are at the -X end under the
	// battery; the four rollers ride on the outrigger legs at x=72 / 91; the tines run from the carriage plate
	// (x=35) to their tips (x=215); the operator platform sits above the tines from x=35 to 110.
	FForkliftRigSpec MakeRigB()
	{
		FForkliftRigSpec R;
		R.MeshDir = TEXT("/Game/ForkliftB/Meshes/");
		R.MeshPrefix = TEXT("SM_ForkliftB_");
		R.BodyMesh = TEXT("Body");
		R.LiftMesh = TEXT("Lift");
		R.WheelMeshes = { TEXT("DriveWheel"), TEXT("RollerA"), TEXT("RollerB"), TEXT("RollerC"), TEXT("RollerD") };
		R.RotatorMeshes = { TEXT("Swivel") };

		const auto Wheel = [](int32 Slot, int32 Rot, const FVector& C, const FVector& Pivot, float Radius, bool bDriven, bool bSteered, int32 Group)
		{
			FForkliftWheelSpec W;
			W.MeshSlot = Slot; W.RotatorSlot = Rot; W.Center = C; W.SteerPivot = Pivot; W.Radius = Radius;
			W.bDriven = bDriven; W.bSteered = bSteered; W.Group = Group;
			return W;
		};
		// back_wheel: centre (-58, 0.8, 15.8), r=15.8; back_wheel_swivel pivot = USD joint anchor (0.578, 0.079, 0.356).
		// Rollers r=7.8 at their meshes' centres (USD roller joint anchors).
		R.Wheels = {
			Wheel(0, 0, FromUsd(-58.f, -0.8f, 15.8f), FromUsd(-57.8f, -7.9f, 35.6f), 15.8f, true, true, 0),
			Wheel(1, -1, FromUsd(91.f, -49.9f, 7.8f), FVector::ZeroVector, 7.8f, false, false, 1),
			Wheel(2, -1, FromUsd(91.f, 49.4f, 7.8f), FVector::ZeroVector, 7.8f, false, false, 1),
			Wheel(3, -1, FromUsd(71.8f, -49.9f, 7.8f), FVector::ZeroVector, 7.8f, false, false, 1),
			Wheel(4, -1, FromUsd(71.8f, 49.4f, 7.8f), FVector::ZeroVector, 7.8f, false, false, 1),
		};

		// No mass in the asset. A stand-on stacker is ~2-3 t; the battery / drive unit over the drive wheel puts the
		// centre of mass towards the -X end (about 70% of the weight on the drive wheel, the rest on the rollers).
		R.BodyMassKg = 2600.f;
		R.CenterOfMassCm = FVector(-25.f, 0.f, 65.f);
		R.MaxWheelSpeedDeg = 600.f;  // 600 deg/s x 15.8 cm = 1.65 m/s, a stacker's travel speed
		R.MaxSteerDeg = 60.f;        // back_wheel_swivel limit in the USD
		R.LiftMinCm = -15.f;         // lift_joint lower limit: puts the tines at z=3.0..8.3 cm, inside a 2..10 cm pallet pocket (centred at -14.65)
		R.LiftMaxCm = 200.f;

		// Fork camera: just in front of the operator canopy's front edge (posts at x=113, canopy at z=245), looking
		// forward over the tines and 28 deg down. Lidar: on top of the chassis / battery cover behind the mast (fixed
		// to the body, not the rising cab).
		R.CameraPosCm = FVector(116.f, 0.f, 236.f);
		R.CameraRot = FRotator(-28.f, 0.f, 0.f);
		// Low camera just ahead of the operator platform, centred between the tines. At the pocket-entry lift
		// height its optical axis is roughly level with the openings; a small downward pitch keeps the tips visible.
		R.TineCameraPosCm = FVector(112.f, 0.f, 30.f);
		R.TineCameraRot = FRotator(-5.f, 0.f, 0.f);
		R.LidarPosCm = FVector(-50.f, 0.f, 116.f);
		R.LowLidarPosCm = FVector(115.f, 0.f, 32.f);   // platform front edge is x=110; tines/load are rays' blind spot, masked in features
		R.ForkTipCm = FVector(215.f, 0.f, 20.65f);
		R.ForkStopXCm = 110.f;  // front edge of the operator platform

		// The USD only collides the cab mesh (which contains the tines); the lift is one 270k-triangle mesh, so use
		// boxes. Each tine is 10.7 cm wide (y = +-30.3), x 35..215, top flat at z=23.3, but its underside is NOT flat:
		// it is 18.0 (5.3 thick) for most of the length, ramps up over the last 40 cm and is 21.3 (2.0 thick) for the
		// last 10 cm. Three steps approximate that, always taking the LOWER edge of a step, so the model is never
		// thinner than the asset. The operator platform is a box above the tines (x 35..110, z 22..36).
		const auto Tine = [](float Y)
		{
			return TArray<FForkliftBoxSpec>{
				{ FVector(112.5f, Y, 20.65f), FVector(77.5f, 5.35f, 2.65f) },   // x 35..190, z 18.0..23.3
				{ FVector(197.5f, Y, 21.45f), FVector(7.5f, 5.35f, 1.85f) },    // x 190..205, z 19.6..23.3
				{ FVector(210.f, Y, 22.3f), FVector(5.f, 5.35f, 1.0f) },        // x 205..215, z 21.3..23.3
			};
		};
		R.LiftBoxes = Tine(30.3f);
		R.LiftBoxes.Append(Tine(-30.3f));
		R.LiftBoxes.Add({ FVector(72.5f, 0.f, 29.f), FVector(37.5f, 55.f, 7.f) });
		return R;
	}

	const FForkliftRigSpec& RigFor(EForkliftModel Model)
	{
		static const FForkliftRigSpec C = MakeRigC();
		static const FForkliftRigSpec B = MakeRigB();
		return Model == EForkliftModel::ForkliftC ? C : B;
	}
}

// ---- Vehicle ------------------------------------------------------------------------------------------------

AForkliftVehicle::AForkliftVehicle()
{
	PrimaryActorTick.bCanEverTick = true;
	// Forces are applied after the physics step, against the freshest body pose; the kinematic lift
	// also follows the body from here so it never lags a frame behind.
	PrimaryActorTick.TickGroup = TG_PostPhysics;
	AutoPossessPlayer = EAutoReceiveInput::Player0;
	bReplicates = false;

	Body = MakePart(TEXT("Body"));
	SetRootComponent(Body);
	Lift = MakePart(TEXT("Lift"));
	Glass = MakePart(TEXT("Glass"));
	Glass->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Glass->SetCastShadow(false);

	// A pool big enough for either rig; SelectRig() fills the slots the active model uses. Wheels, rotators and
	// glass are visual only: the wheels are raycasts, and colliders that ride on the body would just be extra
	// shapes welded into it.
	for (int32 i = 0; i < 5; ++i)
	{
		UStaticMeshComponent* Wheel = MakePart(*FString::Printf(TEXT("Wheel%d"), i));
		Wheel->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		WheelMeshes.Add(Wheel);
	}
	for (int32 i = 0; i < 2; ++i)
	{
		UStaticMeshComponent* Rotator = MakePart(*FString::Printf(TEXT("Rotator%d"), i));
		Rotator->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		RotatorMeshes.Add(Rotator);
	}

	// Chase camera. The arm extends backwards along its own -X, so identity rotation = behind the truck.
	CameraArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraArm"));
	CameraArm->SetupAttachment(Body);
	CameraArm->TargetArmLength = 500.f;
	CameraArm->SetRelativeLocation(FVector(0.f, 0.f, 120.f));
	CameraArm->SetRelativeRotation(FRotator(-18.f, 0.f, 0.f));
	// Every vehicle part already ignores ECC_Camera (see MakePart), so this only pulls the camera in for
	// walls/racks/other world geometry, never the truck itself - safe to turn on now that the bigger
	// warehouse gives the chase cam more wall/rack surface to clip through near the aisles.
	CameraArm->bDoCollisionTest = true;
	CameraArm->bInheritPitch = false;
	CameraArm->bInheritRoll = false;
	// No rotation lag: UpdateMouseLook already sets the arm's rotation directly every tick, so the spring arm's own
	// smoothing on top of that just adds a laggy, sluggish feel to the free-look instead of a clean follow.
	CameraArm->bEnableCameraRotationLag = false;

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(CameraArm, USpringArmComponent::SocketName);

	// Fork camera rides on the lift (a separate kinematic body; children follow it). Lidar is fixed to the chassis.
	// Both mount transforms are set per model in ApplyMounts().
	ForkCamera = CreateDefaultSubobject<UForkliftCameraComponent>(TEXT("ForkCamera"));
	ForkCamera->SetupAttachment(Lift);
	// Needed for the HUD's depth-view toggle (L): metric depth aligned to this camera's RGB frame.
	ForkCamera->bCaptureDepth = true;
	// Required for AutoExposureBias/ExposureEV to have any effect at all - see
	// UForkliftCameraComponent::ApplyExposure()'s comment. Must be set before the component registers
	// (i.e. here in the constructor, not later in BeginPlay), matching how SyntheticDataGenerator's own
	// camera already does this successfully.
	ForkCamera->bAlwaysPersistRenderingState = true;
	// 6.7 measured ~160 mean luma / zero clipping for this specific camera (640x480, 90 deg FOV) against
	// the current warehouse lighting - close to the ~154 luma the detector's synthetic training data was
	// calibrated to (see ForkliftCameraComponent.h's ExposureEV comment). Not the same numeric value as
	// SyntheticDataGenerator's own BaseEV=9.5: that camera has a different FOV/resolution/typical
	// distance from the lights, so the same visual brightness needs a different EV even with the same
	// underlying exposure mechanism now working correctly for both.
	ForkCamera->ExposureEV = 6.7f;
	TineCamera = CreateDefaultSubobject<UForkliftCameraComponent>(TEXT("TineCamera"));
	TineCamera->SetupAttachment(Lift);
	TineCamera->FOVAngle = 78.f;
	TineCamera->CaptureRateHz = 10.f;
	TineCamera->bCaptureDepth = true;
	TineCamera->bHideOwner = true;
	TineCamera->bAlwaysPersistRenderingState = true;
	TineCamera->ExposureEV = 6.7f;   // measured ~160 mean luma / zero clipping, see ForkCamera's comment above
	Lidar = CreateDefaultSubobject<ULidarSensorComponent>(TEXT("Lidar"));
	Lidar->SetupAttachment(Body);
	// Roof lidar looks further down so floor-level objects start to appear at ~2.5 m (was 4.3 m at -15 deg).
	Lidar->VerticalMinDeg = -25.f;
	Lidar->VerticalMaxDeg = 10.f;
	LowLidar = CreateDefaultSubobject<ULidarSensorComponent>(TEXT("LowLidar"));
	LowLidar->SetupAttachment(Body);
	LowLidar->Channels = 6;
	LowLidar->HorizontalSteps = 240;
	LowLidar->HorizontalFovDeg = 240.f;
	LowLidar->VerticalMinDeg = -12.f;
	LowLidar->VerticalMaxDeg = 3.f;
	LowLidar->MinRangeCm = 30.f;
	LowLidar->MaxRangeCm = 1500.f;
	LowLidar->PrimaryComponentTick.bStartWithTickEnabled = false;   // scanned on demand (datasets / learned nav) - no idle cost
	AutoPickup = CreateDefaultSubobject<UForkliftAutoPickupComponent>(TEXT("AutoPickup"));
	Nav = CreateDefaultSubobject<UForkliftNavComponent>(TEXT("Nav"));
	Mission = CreateDefaultSubobject<UForkliftMissionComponent>(TEXT("Mission"));
	Finder = CreateDefaultSubobject<UForkliftPalletFinderComponent>(TEXT("PalletFinder"));
}

UStaticMeshComponent* AForkliftVehicle::MakePart(FName Name)
{
	UStaticMeshComponent* Part = CreateDefaultSubobject<UStaticMeshComponent>(Name);
	if (Part != RootComponent && GetRootComponent() != nullptr)
	{
		Part->SetupAttachment(GetRootComponent());
	}

	// UStaticMeshComponent defaults to Static mobility, which silently refuses to simulate physics.
	Part->SetMobility(EComponentMobility::Movable);

	// Every forklift part uses the Vehicle channel and ignores it, so the truck never collides with
	// itself (forks vs. body) but still collides with the world and props.
	Part->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	Part->SetCollisionObjectType(ECC_Vehicle);
	Part->SetCollisionResponseToChannel(ECC_Vehicle, ECR_Ignore);
	Part->SetCollisionResponseToChannel(ECC_Camera, ECR_Ignore);
	Part->SetSimulatePhysics(false);
	return Part;
}

UPhysicalMaterial* AForkliftVehicle::GetCarryGripMaterial()
{
	static TStrongObjectPtr<UPhysicalMaterial> Grip;
	if (!Grip.IsValid())
	{
		UPhysicalMaterial* Mat = NewObject<UPhysicalMaterial>(GetTransientPackage(), TEXT("ForkliftCarryGrip"));
		Mat->Friction = 1.0f;
		Mat->StaticFriction = 1.2f;
		Mat->bOverrideFrictionCombineMode = true;
		Mat->FrictionCombineMode = EFrictionCombineMode::Max;  // the grippier of the two surfaces wins
		Grip.Reset(Mat);
	}
	return Grip.Get();
}

void AForkliftVehicle::SelectRig()
{
	// -ForkliftModel=C|B wins over the property, so the same map can be run against either rig.
	FString Override;
	ActiveModel = Model;
	if (FParse::Value(FCommandLine::Get(), TEXT("ForkliftModel="), Override))
	{
		ActiveModel = Override.StartsWith(TEXT("C"), ESearchCase::IgnoreCase) ? EForkliftModel::ForkliftC : EForkliftModel::ForkliftB;
	}
	Rig = &RigFor(ActiveModel);
}

void AForkliftVehicle::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	SelectRig();
	AssignMeshes();
	ApplyMounts();
}

void AForkliftVehicle::AssignMeshes()
{
	const auto Load = [this](const FString& Name) -> UStaticMesh*
	{
		if (Name.IsEmpty())
		{
			return nullptr;
		}
		const FString Path = FString::Printf(TEXT("%s%s%s.%s%s"), *Rig->MeshDir, *Rig->MeshPrefix, *Name, *Rig->MeshPrefix, *Name);
		return TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(Path)).LoadSynchronous();
	};

	Body->SetStaticMesh(Load(Rig->BodyMesh));
	Lift->SetStaticMesh(Load(Rig->LiftMesh));
	Glass->SetStaticMesh(Load(Rig->GlassMesh));
	for (int32 i = 0; i < WheelMeshes.Num(); ++i)
	{
		WheelMeshes[i]->SetStaticMesh(Rig->WheelMeshes.IsValidIndex(i) ? Load(Rig->WheelMeshes[i]) : nullptr);
	}
	for (int32 i = 0; i < RotatorMeshes.Num(); ++i)
	{
		RotatorMeshes[i]->SetStaticMesh(Rig->RotatorMeshes.IsValidIndex(i) ? Load(Rig->RotatorMeshes[i]) : nullptr);
	}
}

void AForkliftVehicle::ApplyMounts()
{
	ForkCamera->SetRelativeLocationAndRotation(Rig->CameraPosCm, Rig->CameraRot);
	TineCamera->SetRelativeLocationAndRotation(Rig->TineCameraPosCm, Rig->TineCameraRot);
	Lidar->SetRelativeLocation(Rig->LidarPosCm);
	LowLidar->SetRelativeLocation(Rig->LowLidarPosCm);
}

void AForkliftVehicle::SetupWheels()
{
	// Static load per wheel from where the centre of mass sits between the two support groups (C: front / rear axle;
	// B: the drive wheel / the rollers). Each spring is then sized so every wheel compresses by the same
	// StaticCompressionCm, which keeps the truck level.
	float GroupX[2] = { 0.f, 0.f };
	int32 GroupCount[2] = { 0, 0 };
	for (const FForkliftWheelSpec& S : Rig->Wheels)
	{
		GroupX[S.Group] += S.Center.X;
		++GroupCount[S.Group];
	}
	for (int32 g = 0; g < 2; ++g)
	{
		GroupX[g] /= FMath::Max(GroupCount[g], 1);
	}
	const float Group0Share = FMath::Clamp((CenterOfMassCm.X - GroupX[1]) / (GroupX[0] - GroupX[1]), 0.1f, 0.9f);
	const float GroupShare[2] = { Group0Share, 1.f - Group0Share };

	Wheels.Reset();
	for (const FForkliftWheelSpec& S : Rig->Wheels)
	{
		FWheel W;
		W.Mesh = WheelMeshes[S.MeshSlot];
		W.Rotator = S.RotatorSlot >= 0 ? RotatorMeshes[S.RotatorSlot].Get() : nullptr;
		W.Center = S.Center;
		W.SteerPivot = S.SteerPivot;
		W.Radius = S.Radius;
		W.bDriven = S.bDriven;
		W.bSteered = S.bSteered;
		W.LoadKg = BodyMassKg * GroupShare[S.Group] / FMath::Max(GroupCount[S.Group], 1);
		W.Stiffness = W.LoadKg * ForkliftLayout::GravityCmPerSec2 / FMath::Max(StaticCompressionCm, 0.1f);
		W.Damping = 2.f * DampingRatio * FMath::Sqrt(W.Stiffness * W.LoadKg);
		Wheels.Add(W);
	}
}

void AForkliftVehicle::OnForkliftHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	if (LoggedHits >= 40 || !OtherComp || OtherComp->GetOwner() == this || (OtherComp->GetName().StartsWith(TEXT("Floor"))))
	{
		return;
	}
	++LoggedHits;
	UE_LOG(LogTemp, Log, TEXT("[ForkliftHit] %s -> %s at (%.1f %.1f %.1f) normal=(%.2f %.2f %.2f) impulse=%.0f lift=%.1f"),
		*HitComp->GetName(), *OtherComp->GetName(), Hit.ImpactPoint.X, Hit.ImpactPoint.Y, Hit.ImpactPoint.Z, Hit.ImpactNormal.X, Hit.ImpactNormal.Y, Hit.ImpactNormal.Z,
		NormalImpulse.Size(), LiftHeightCm);
}

void AForkliftVehicle::CreateLiftColliders()
{
	// The lift is a separate kinematic body (detached from the truck in BeginPlay); box colliders attached to it move
	// with it and push dynamic pallets exactly like a kinematic mesh would. Same channel setup as every other part.
	for (int32 i = 0; i < Rig->LiftBoxes.Num(); ++i)
	{
		UBoxComponent* Box = NewObject<UBoxComponent>(this, *FString::Printf(TEXT("LiftCollider%d"), i));
		Box->SetMobility(EComponentMobility::Movable);
		Box->SetBoxExtent(Rig->LiftBoxes[i].Extent, false);
		Box->AttachToComponent(Lift, FAttachmentTransformRules::KeepRelativeTransform);
		Box->SetRelativeLocation(Rig->LiftBoxes[i].Center);
		Box->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
		Box->SetCollisionObjectType(ECC_Vehicle);
		Box->SetCollisionResponseToChannel(ECC_Vehicle, ECR_Ignore);
		Box->SetCollisionResponseToChannel(ECC_Camera, ECR_Ignore);
		Box->SetSimulatePhysics(false);
		Box->SetPhysMaterialOverride(GetCarryGripMaterial());
		if (FParse::Param(FCommandLine::Get(), TEXT("ForkliftLogHits")))
		{
			Box->SetNotifyRigidBodyCollision(true);
			Box->OnComponentHit.AddDynamic(this, &AForkliftVehicle::OnForkliftHit);
		}
		Box->SetCanEverAffectNavigation(false);
		Box->RegisterComponent();
		LiftColliders.Add(Box);
	}
	if (!Rig->LiftBoxes.IsEmpty())
	{
		Lift->SetCollisionEnabled(ECollisionEnabled::NoCollision);  // the boxes replace the mesh's collision
	}
}

void AForkliftVehicle::BeginPlay()
{
	Super::BeginPlay();

	if (!Rig || !Body->GetStaticMesh())
	{
		SelectRig();
		AssignMeshes();  // e.g. a Play-In-Editor copy that skipped OnConstruction
		ApplyMounts();
	}
	if (!Body->GetStaticMesh() || !Lift->GetStaticMesh())
	{
		UE_LOG(LogTemp, Error, TEXT("[Forklift] Meshes missing - run Scripts/setup.py to generate the /Game/Forklift and /Game/ForkliftB meshes."));
		return;
	}

	if (bUseModelDefaults)
	{
		BodyMassKg = Rig->BodyMassKg;
		CenterOfMassCm = Rig->CenterOfMassCm;
		MaxWheelSpeedDeg = Rig->MaxWheelSpeedDeg;
		MaxSteerDeg = Rig->MaxSteerDeg;
		LiftMinCm = Rig->LiftMinCm;
		LiftMaxCm = Rig->LiftMaxCm;
	}
	LiftHeightCm = FMath::Clamp(0.f, LiftMinCm, LiftMaxCm);

	// The lift was attached to the body only so it renders in place in the editor. It is a separate
	// kinematic body, moved along the mast by UpdateLift.
	Lift->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
	Lift->SetSimulatePhysics(false);
	CreateLiftColliders();

	Body->SetSimulatePhysics(true);
	if (FParse::Param(FCommandLine::Get(), TEXT("ForkliftLogHits")))
	{
		Body->SetNotifyRigidBodyCollision(true);
		Body->OnComponentHit.AddDynamic(this, &AForkliftVehicle::OnForkliftHit);
	}
	Body->SetMassOverrideInKg(NAME_None, BodyMassKg, true);
	Body->SetLinearDamping(0.02f);
	Body->SetAngularDamping(0.4f);

	// Bring the centre of mass down and to the middle: the body mesh includes the tall overhead guard
	// and mast stages, which would otherwise put it near the roof line.
	const FVector CurrentLocalCom = Body->GetComponentTransform().InverseTransformPosition(Body->GetCenterOfMass());
	Body->SetCenterOfMass(CenterOfMassCm - CurrentLocalCom);

	SetupWheels();
	bSimulating = true;

	BaseCameraArmPitchDeg = CameraArm->GetRelativeRotation().Pitch;
}

void AForkliftVehicle::PossessedBy(AController* NewController)
{
	Super::PossessedBy(NewController);

	// Mouse-look needs the viewport to actually route mouse movement to the game (not to an OS cursor); without
	// this a fresh Play-In-Editor window ignores the mouse until the player clicks into it once.
	if (APlayerController* PC = Cast<APlayerController>(NewController))
	{
		PC->SetInputMode(FInputModeGameOnly());
		PC->SetShowMouseCursor(false);
	}
}

void AForkliftVehicle::SetDriveInput(float InThrottle, float InSteer, float InLift)
{
	ThrottleCmd = FMath::Clamp(InThrottle, -1.f, 1.f);
	SteerCmd = FMath::Clamp(InSteer, -1.f, 1.f);
	LiftCmd = FMath::Clamp(InLift, -1.f, 1.f);
}

void AForkliftVehicle::ReadKeyboard()
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC)
	{
		return;
	}
	const auto Down = [PC](const FKey& A, const FKey& B) { return PC->IsInputKeyDown(A) || PC->IsInputKeyDown(B); };

	const float Fwd = Down(EKeys::W, EKeys::Up) ? 1.f : 0.f;
	const float Back = Down(EKeys::S, EKeys::Down) ? 1.f : 0.f;
	const float Left = Down(EKeys::A, EKeys::Left) ? 1.f : 0.f;
	const float Right = Down(EKeys::D, EKeys::Right) ? 1.f : 0.f;
	const float Up = PC->IsInputKeyDown(EKeys::R) ? 1.f : 0.f;
	const float Dn = PC->IsInputKeyDown(EKeys::F) ? 1.f : 0.f;

	SetDriveInput(Fwd - Back, Left - Right, Up - Dn);
}

void AForkliftVehicle::UpdateMouseLook(float DeltaSeconds)
{
	if (!bMouseLook || !CameraArm)
	{
		return;
	}
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC)
	{
		return;
	}
	float DeltaX = 0.f, DeltaY = 0.f;
	PC->GetInputMouseDelta(DeltaX, DeltaY);
	if (DeltaX == 0.f && DeltaY == 0.f)
	{
		return;
	}
	// The spring arm still inherits the body's yaw (bInheritYaw defaults on), so this offset is purely "how far the
	// player has looked away from dead-ahead" - turning the truck carries the look direction with it, same as any
	// chase-cam free look.
	// Raw MouseY (unlike the "LookUp" axis mapping some other setups use) is positive when the mouse moves DOWN the
	// screen, with no inversion baked in - negate it so moving the mouse up looks up, as players expect.
	CameraLookYawDeg += DeltaX * MouseSensitivity;
	CameraLookPitchDeg = FMath::Clamp(CameraLookPitchDeg - DeltaY * MouseSensitivity, -MouseLookPitchRangeDeg, MouseLookPitchRangeDeg);
	CameraArm->SetRelativeRotation(FRotator(BaseCameraArmPitchDeg + CameraLookPitchDeg, CameraLookYawDeg, 0.f));
}

void AForkliftVehicle::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!bSimulating)
	{
		return;
	}

	// Use the real frame time so rates (steer, lift) are correct at any frame rate; only cap hitches.
	const float Dt = FMath::Clamp(DeltaSeconds, 1.e-4f, 1.f / 20.f);
	if (bKeyboardControl)
	{
		ReadKeyboard();
	}
	UpdateMouseLook(Dt);
	if (const APlayerController* PC = Cast<APlayerController>(GetController()); PC && AutoPickup && PC->WasInputKeyJustPressed(EKeys::P))
	{
		AutoPickup->TogglePickup();
	}
	if (const APlayerController* PC = Cast<APlayerController>(GetController()); PC && Mission && PC->WasInputKeyJustPressed(EKeys::M))
	{
		Mission->ToggleMission();
	}

	// Steering: integrate like the Kit prototype, but return to centre when the key is released
	// (a real truck's operator steers back; the prototype held the last angle).
	if (FMath::Abs(SteerCmd) > KINDA_SMALL_NUMBER)
	{
		SteerTargetDeg += SteerCmd * SteerRateDegPerSec * Dt;
	}
	else
	{
		const float Step = SteerRateDegPerSec * Dt;
		SteerTargetDeg = FMath::Abs(SteerTargetDeg) <= Step ? 0.f : SteerTargetDeg - FMath::Sign(SteerTargetDeg) * Step;
	}
	SteerTargetDeg = FMath::Clamp(SteerTargetDeg, -MaxSteerDeg, MaxSteerDeg);

	SimulateWheels(Dt);
	UpdateLift(Dt);
	UpdateVisuals();
}

void AForkliftVehicle::SimulateWheels(float Dt)
{
	const FTransform BodyXf = Body->GetComponentTransform();
	const FVector Up = BodyXf.GetUnitAxis(EAxis::Z);
	const FVector Fwd = BodyXf.GetUnitAxis(EAxis::X);

	FCollisionQueryParams Params(SCENE_QUERY_STAT(ForkliftWheel), false, this);
	const float MaxWheelSpeedRad = FMath::DegreesToRadians(MaxWheelSpeedDeg);
	const float RearYawRad = FMath::DegreesToRadians(SteerTargetDeg);

	int32 DrivenCount = 0;
	for (const FWheel& W : Wheels)
	{
		DrivenCount += W.bDriven ? 1 : 0;
	}

	GroundedWheels = 0;
	for (FWheel& W : Wheels)
	{
		const FVector CenterWorld = BodyXf.TransformPosition(W.Center);
		const FVector Start = CenterWorld + Up * TravelUpCm;
		const FVector End = CenterWorld - Up * (W.Radius + TravelDownCm);

		FHitResult Hit;
		if (!GetWorld()->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params))
		{
			W.bGrounded = false;
			W.ExtensionCm = FMath::FInterpTo(W.ExtensionCm, -TravelDownCm, Dt, 10.f);
			W.SpinRad += ThrottleCmd * (W.bDriven ? MaxWheelSpeedRad : 0.f) * Dt;
			continue;
		}

		// Distance of the wheel centre below Start, and how far the spring is squeezed relative to fully
		// extended. At the static pose the centre is exactly TravelUpCm below Start.
		const float CenterDist = Hit.Distance - W.Radius;
		const float Compression = StaticCompressionCm + (TravelUpCm - CenterDist);
		W.ExtensionCm = TravelUpCm - CenterDist;
		if (Compression <= 0.f)
		{
			W.bGrounded = false;
			continue;
		}
		W.bGrounded = true;
		++GroundedWheels;

		const FVector P = Hit.ImpactPoint;
		const FVector N = Hit.ImpactNormal;
		const FVector V = Body->GetPhysicsLinearVelocityAtPoint(P);

		// Spring + damper along the body's up axis (the suspension axis).
		const float CompressionSpeed = -FVector::DotProduct(V, Up);
		const float SpringForce = FMath::Max(0.f, W.Stiffness * Compression + W.Damping * CompressionSpeed);

		// Tyre frame on the ground plane. Steered wheels are turned by the steering rotator / swivel.
		FVector WheelFwd = Fwd;
		if (W.bSteered)
		{
			// UE yaw is clockwise-positive; a left turn on a rear-steer truck points the rear wheel right.
			WheelFwd = FQuat(Up, RearYawRad).RotateVector(Fwd);
		}
		const FVector Fw = (WheelFwd - N * FVector::DotProduct(WheelFwd, N)).GetSafeNormal();
		const FVector Side = FVector::CrossProduct(N, Fw);
		const float VLong = FVector::DotProduct(V, Fw);
		const float VLat = FVector::DotProduct(V, Side);
		const float Grip = TyreFriction * SpringForce;

		// Lateral: cancel a fraction of the sideways slip each step, limited by friction.
		const float Fy = FMath::Clamp(-LateralGrip * W.LoadKg * VLat / Dt, -Grip, Grip);

		float Fx;
		if (W.bDriven)
		{
			// Velocity drive toward the commanded rolling speed (Kit: joint targetVelocity), friction limited.
			// Zero throttle drives toward zero speed, i.e. the truck brakes and holds like the Kit's drive did.
			const float TargetSpeed = ThrottleCmd * MaxWheelSpeedRad * W.Radius;
			const float Desired = BodyMassKg * (TargetSpeed - VLong) / FMath::Max(DriveResponseSec, 0.05f) / FMath::Max(DrivenCount, 1);
			Fx = FMath::Clamp(Desired, -Grip, Grip);
		}
		else
		{
			Fx = -RollingResistance * SpringForce * FMath::Clamp(VLong / 10.f, -1.f, 1.f);
		}

		Body->AddForceAtLocation(Up * SpringForce + Fw * Fx + Side * Fy, P);
		W.SpinRad += (VLong / W.Radius) * Dt;
	}
}

void AForkliftVehicle::UpdateLift(float Dt)
{
	LiftHeightCm = FMath::Clamp(LiftHeightCm + LiftCmd * LiftRateCmPerSec * Dt, LiftMinCm, LiftMaxCm);
	const FTransform Local(FVector(0.f, 0.f, LiftHeightCm));
	Lift->SetWorldTransform(Local * Body->GetComponentTransform(), false, nullptr, ETeleportType::None);
}

void AForkliftVehicle::UpdateVisuals()
{
	using namespace ForkliftLayout;
	const FQuat SteerQ(FVector::UpVector, FMath::DegreesToRadians(SteerTargetDeg));

	for (const FWheel& W : Wheels)
	{
		const FTransform Susp(FVector(0.f, 0.f, W.ExtensionCm));
		const FTransform Steer = W.bSteered ? RotateAbout(W.SteerPivot, SteerQ) : FTransform::Identity;

		if (W.Rotator)
		{
			W.Rotator->SetRelativeTransform(Susp * Steer);
		}
		// Spin about the axle (lateral Y through the wheel centre), then ride the suspension, then steer.
		const FTransform Spin = RotateAbout(W.Center, FQuat(FVector::RightVector, W.SpinRad));
		W.Mesh->SetRelativeTransform(Spin * Susp * Steer);
	}
}

float AForkliftVehicle::GetForwardSpeedCmPerSec() const
{
	return FVector::DotProduct(Body->GetPhysicsLinearVelocity(), Body->GetForwardVector());
}

FVector AForkliftVehicle::GetForkTipWorld() const
{
	const FVector Tip = Rig ? Rig->ForkTipCm : FVector::ZeroVector;
	return Body->GetComponentTransform().TransformPosition(Tip + FVector(0.f, 0.f, LiftHeightCm));
}

float AForkliftVehicle::GetForkStopXCm() const
{
	return Rig ? Rig->ForkStopXCm : 0.f;
}

FString AForkliftVehicle::GetDebugSummary() const
{
	FString Ext;
	FString Loads;
	for (const FWheel& W : Wheels)
	{
		Ext += FString::Printf(TEXT("%s%.1f "), W.bGrounded ? TEXT("") : TEXT("!"), W.ExtensionCm);
		Loads += FString::Printf(TEXT("%.0f "), W.LoadKg);
	}
	const FVector Vel = Body->GetPhysicsLinearVelocity();
	return FString::Printf(TEXT("model=%s bodyZ=%.1f grounded=%d/%d springExt=%s| staticLoadKg=%s| lift=%.1f | vel=(%.0f %.0f %.0f) com(local)=%s"),
		ActiveModel == EForkliftModel::ForkliftB ? TEXT("B") : TEXT("C"), Body->GetComponentLocation().Z, GroundedWheels, Wheels.Num(), *Ext, *Loads, LiftHeightCm, Vel.X, Vel.Y, Vel.Z,
		*Body->GetComponentTransform().InverseTransformPosition(Body->GetCenterOfMass()).ToString());
}
