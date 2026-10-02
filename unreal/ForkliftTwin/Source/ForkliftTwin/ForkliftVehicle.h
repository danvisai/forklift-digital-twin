// Forklift digital twin - drivable forklift (NVIDIA Isaac ForkliftB by default, ForkliftC selectable).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "ForkliftVehicle.generated.h"

class UStaticMeshComponent;
class USpringArmComponent;
class UCameraComponent;
class UBoxComponent;
class UPhysicalMaterial;
class ULidarSensorComponent;
class UForkliftPalletFinderComponent;
class UForkliftCameraComponent;
class UForkliftAutoPickupComponent;
class UForkliftNavComponent;
class UForkliftMissionComponent;
struct FForkliftRigSpec;

UENUM(BlueprintType)
enum class EForkliftModel : uint8
{
	/** forklift_b.usd: stand-on stacker. One steered drive wheel at the back, four load rollers on the outrigger
	 *  legs, forks + operator platform + mast on one lift, tines 1.8 m long. Forks lead along +X (the USD's forks
	 *  point at -X; the bake turns it around). */
	ForkliftB,
	/** forklift_c.usd: counterbalance truck. Driven front wheels, rear-wheel steering, short 1.1 m tines. */
	ForkliftC,
};

/**
 * One drivable forklift class for both Isaac Sim rigs. The rig (meshes, wheel layout, tuning defaults, lift
 * range, sensor mounts, lift colliders) comes from a per-model table in ForkliftVehicle.cpp; the physics is
 * shared. Launch with -ForkliftModel=C to force ForkliftC, e.g. for the regression self-test.
 *
 * The USDs model the rig as PhysX joints (C: 7, B: 7 with 4 free rollers). Reproducing that as a chain of Chaos
 * constraints was unstable, so the physics here is:
 *
 *   - the truck is ONE rigid body (the body mesh with convex-decomposition collision),
 *   - each wheel is a raycast spring-damper with a friction-limited tyre model (drive, steer and free wheels
 *     are just flags on the wheel),
 *   - wheel / steering-swivel meshes are visual only, animated from that simulation,
 *   - the lift is a kinematic body driven along the mast. Its colliders are the mesh's own (C) or hand-placed
 *     boxes for the tines and operator platform (B, whose lift is one 270k-triangle mesh).
 *
 * Because the lift is kinematic, a carried pallet's weight is NOT transmitted to the truck: it can neither tip
 * nor load the wheels. Neither asset has a mast-tilt joint, so there is no tilt control.
 *
 * Forward is +X (the direction the forks point). Every mesh is authored with its pivot at the forklift origin,
 * so parts are placed in place and animated by rotating about their real axles.
 */
UCLASS()
class FORKLIFTTWIN_API AForkliftVehicle : public APawn
{
	GENERATED_BODY()

public:
	AForkliftVehicle();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void PossessedBy(AController* NewController) override;

	/**
	 * Programmatic control, the same interface a scripted/RL controller would use.
	 * All values are -1..1. Throttle +1 = forward. Steer +1 = turn left. Lift +1 = raise forks.
	 * Overwritten every frame by the keyboard when bKeyboardControl is on and a player possesses it.
	 */
	UFUNCTION(BlueprintCallable, Category = "Forklift")
	void SetDriveInput(float InThrottle, float InSteer, float InLift);

	/** Current fork height above its rest position, in cm (B can go below 0: its rest pose is not its lowest). */
	UFUNCTION(BlueprintPure, Category = "Forklift")
	float GetLiftHeightCm() const { return LiftHeightCm; }

	UFUNCTION(BlueprintPure, Category = "Forklift")
	float GetLiftMinCm() const { return LiftMinCm; }

	UFUNCTION(BlueprintPure, Category = "Forklift")
	float GetLiftMaxCm() const { return LiftMaxCm; }

	/** Signed speed along the forklift's forward axis, cm/s. */
	UFUNCTION(BlueprintPure, Category = "Forklift")
	float GetForwardSpeedCmPerSec() const;

	/** Rear-wheel steering angle currently commanded, in degrees (+ = turn left). */
	UFUNCTION(BlueprintPure, Category = "Forklift")
	float GetSteerTargetDeg() const { return SteerTargetDeg; }

	/** Which rig is active (after the -ForkliftModel= command-line override). */
	UFUNCTION(BlueprintPure, Category = "Forklift")
	EForkliftModel GetActiveModel() const { return ActiveModel; }

	/** World position of the middle of the fork tips (at the current fork height). */
	UFUNCTION(BlueprintPure, Category = "Forklift")
	FVector GetForkTipWorld() const;

	/** Where the forks hand over to the truck body along +X, forklift-local cm: a pallet can seat no further back. */
	UFUNCTION(BlueprintPure, Category = "Forklift")
	float GetForkStopXCm() const;

	/** One-line dump of ride state for debugging / the self-test. */
	FString GetDebugSummary() const;

	/**
	 * High-friction physical material used by the tines and by pallets picked up in the sim. The assets have no
	 * mast tilt (real trucks tilt the load back so it cannot creep off the forks), so the load is held by friction
	 * alone and default steel-on-wood friction lets a pallet creep toward the tips while the truck accelerates.
	 */
	static UPhysicalMaterial* GetCarryGripMaterial();

	/** RGB camera on the fork carriage: it rises and falls with the forks and looks along them. */
	UForkliftCameraComponent* GetForkCamera() const { return ForkCamera; }

	/** Low RGB-D camera between the tines, used for pocket alignment and insertion. */
	UForkliftCameraComponent* GetTineCamera() const { return TineCamera; }

	UForkliftAutoPickupComponent* GetAutoPickup() const { return AutoPickup; }

	/** Waypoint-graph planner + pure-pursuit follower for driving to a named pose across the warehouse. */
	UForkliftNavComponent* GetNav() const { return Nav; }

	/** Autonomous pickup -> carry -> place mission supervisor (M key, or -ForkliftMission). */
	UForkliftMissionComponent* GetMission() const { return Mission; }

	/** Finds floor pallets from fork-camera detections + depth (no positions read from the simulator). */
	UForkliftPalletFinderComponent* GetFinder() const { return Finder; }

	/** 360 degree lidar fixed to the chassis. */
	ULidarSensorComponent* GetLidar() const { return Lidar; }

	/** Low forward lidar on the front of the body, sees the floor-level objects the roof lidar cannot (< ~4 m). */
	ULidarSensorComponent* GetLowLidar() const { return LowLidar; }

	/** Last commanded inputs (-1..1), e.g. to log them next to sensor frames. */
	float GetThrottleCmd() const { return ThrottleCmd; }
	float GetSteerCmd() const { return SteerCmd; }
	float GetLiftCmd() const { return LiftCmd; }

	// ---- Rig ------------------------------------------------------------------------------------------

	/** Which Isaac rig to build. Overridden by -ForkliftModel=B|C on the command line. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift")
	EForkliftModel Model = EForkliftModel::ForkliftB;

	/**
	 * When on, BeginPlay replaces the tuning values below (mass, centre of mass, wheel speed, steering and lift range)
	 * with the selected model's table entries. Turn it off to tune them per instance in the Details panel.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift")
	bool bUseModelDefaults = true;

	// ---- Tuning (defaults below are ForkliftC's / the Kit prototype's) ---------------------------------

	/** Driven wheel spin speed at full throttle (deg/s). Kit: DRIVE_MAX_ANGULAR_VELOCITY_DEG. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Drive")
	float MaxWheelSpeedDeg = 360.f;

	/** Time to reach commanded speed, seconds. Smaller = punchier (limited by tyre grip anyway). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Drive")
	float DriveResponseSec = 0.6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Steering")
	float MaxSteerDeg = 30.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Steering")
	float SteerRateDegPerSec = 60.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Lift")
	float LiftMinCm = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Lift")
	float LiftMaxCm = 200.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Lift")
	float LiftRateCmPerSec = 40.f;

	/** Read W/S/A/D/R/F (and arrow keys) from the possessing player. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Input")
	bool bKeyboardControl = true;

	/** Mouse moves the chase camera (free-look, stacked on top of the truck's own heading) rather than steering. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Input")
	bool bMouseLook = true;

	/** Degrees the camera turns per pixel of mouse movement. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Input")
	float MouseSensitivity = 0.3f;

	/** How far up/down free-look can pitch the camera from its resting angle, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Forklift|Input")
	float MouseLookPitchRangeDeg = 60.f;

	// ---- Chassis / suspension / tyres --------------------------------------------------------------

	UPROPERTY(EditAnywhere, Category = "Forklift|Chassis")
	float BodyMassKg = 3000.f;

	/** Where the centre of mass sits, forklift-local cm. Low and central keeps it from tipping. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Chassis")
	FVector CenterOfMassCm = FVector(-40.f, 0.f, 55.f);

	/** How far each spring is compressed when the truck is standing still, cm. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Suspension")
	float StaticCompressionCm = 3.f;

	/** Extra upward travel beyond the static pose before the spring bottoms out, cm. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Suspension")
	float TravelUpCm = 10.f;

	/** Downward droop allowed before the wheel leaves the ground, cm. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Suspension")
	float TravelDownCm = 6.f;

	/** 1 = critically damped. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Suspension")
	float DampingRatio = 0.6f;

	UPROPERTY(EditAnywhere, Category = "Forklift|Tyres")
	float TyreFriction = 1.0f;

	/** Fraction of lateral slip the tyre tries to cancel each step (1 = fully, may oscillate). */
	UPROPERTY(EditAnywhere, Category = "Forklift|Tyres")
	float LateralGrip = 0.7f;

	/** Rolling resistance on the free wheels, as a fraction of their load. */
	UPROPERTY(EditAnywhere, Category = "Forklift|Tyres")
	float RollingResistance = 0.015f;

private:
	struct FWheel
	{
		UStaticMeshComponent* Mesh = nullptr;
		UStaticMeshComponent* Rotator = nullptr;  // steering swivel / rotator that carries this wheel, if any
		FVector Center = FVector::ZeroVector;     // wheel centre at the static pose, forklift-local cm
		FVector SteerPivot = FVector::ZeroVector; // vertical steering axis point, forklift-local cm
		float Radius = 30.f;
		bool bDriven = false;
		bool bSteered = false;

		float Stiffness = 0.f;   // per-wheel spring rate from its static load share
		float Damping = 0.f;
		float LoadKg = 0.f;
		float SpinRad = 0.f;     // accumulated rolling angle for the visual
		float ExtensionCm = 0.f; // spring extension for the visual (+ = wheel higher relative to body)
		bool bGrounded = false;
	};

	/** Debug: with -ForkliftLogHits, logs the first contacts between the truck's colliders and other bodies. */
	UFUNCTION()
	void OnForkliftHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

	int32 LoggedHits = 0;

	void SelectRig();
	void AssignMeshes();
	void ApplyMounts();
	void SetupWheels();
	void CreateLiftColliders();
	void SimulateWheels(float DeltaSeconds);
	void UpdateVisuals();
	void UpdateLift(float DeltaSeconds);
	void ReadKeyboard();
	void UpdateMouseLook(float DeltaSeconds);

	UStaticMeshComponent* MakePart(FName Name);

	UPROPERTY(VisibleAnywhere, Category = "Forklift|Parts")
	TObjectPtr<UStaticMeshComponent> Body;
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Parts")
	TObjectPtr<UStaticMeshComponent> Lift;
	/** Wheel / roller meshes, filled from the rig table; unused slots stay empty. */
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Parts")
	TArray<TObjectPtr<UStaticMeshComponent>> WheelMeshes;
	/** Steering rotators (C) / swivel (B). */
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Parts")
	TArray<TObjectPtr<UStaticMeshComponent>> RotatorMeshes;
	/** Cab glass: visual only, rides on the body. Kept as its own mesh so the body can be Nanite. C only. */
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Parts")
	TObjectPtr<UStaticMeshComponent> Glass;
	/** Box colliders riding on the lift (B: tines + operator platform). */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UBoxComponent>> LiftColliders;

	UPROPERTY(VisibleAnywhere, Category = "Forklift|Camera")
	TObjectPtr<USpringArmComponent> CameraArm;
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Camera")
	TObjectPtr<UCameraComponent> Camera;

	// Sensors for the learning pipeline. Their mount transforms come from the rig table and are (re)applied in
	// OnConstruction, so change them in ForkliftVehicle.cpp rather than per instance.
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Sensors")
	TObjectPtr<UForkliftCameraComponent> ForkCamera;
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Sensors")
	TObjectPtr<UForkliftCameraComponent> TineCamera;
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Sensors")
	TObjectPtr<ULidarSensorComponent> Lidar;

	UPROPERTY(VisibleAnywhere, Category = "Forklift|Sensors")
	TObjectPtr<ULidarSensorComponent> LowLidar;

	UPROPERTY(VisibleAnywhere, Category = "Forklift|Sensors")
	TObjectPtr<UForkliftPalletFinderComponent> Finder;
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Automation")
	TObjectPtr<UForkliftAutoPickupComponent> AutoPickup;
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Automation")
	TObjectPtr<UForkliftNavComponent> Nav;
	UPROPERTY(VisibleAnywhere, Category = "Forklift|Automation")
	TObjectPtr<UForkliftMissionComponent> Mission;

	TArray<FWheel> Wheels;
	EForkliftModel ActiveModel = EForkliftModel::ForkliftB;
	const FForkliftRigSpec* Rig = nullptr;

	// Command state.
	float ThrottleCmd = 0.f;
	float SteerCmd = 0.f;
	float LiftCmd = 0.f;
	float SteerTargetDeg = 0.f;
	float LiftHeightCm = 0.f;
	int32 GroundedWheels = 0;
	bool bSimulating = false;

	// Mouse free-look, additive on top of the spring arm's resting pose (CameraArm still inherits the body's yaw,
	// so this is "how far off dead-ahead the player has looked", not the camera's absolute rotation).
	float CameraLookYawDeg = 0.f;
	float CameraLookPitchDeg = 0.f;
	float BaseCameraArmPitchDeg = 0.f;
};
