// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Component/FlyingMovementInterface.h"
#include "NavCore/NavPathCorridor.h"
#include "PathfindingComponent.generated.h"

class USplineComponent;
class UPathfindingStrategy;
class UPathfindingSubsystem;
class UAvoidanceSubsystem;
class USphereComponent;
class AVoxelVolumeBase;
class AOctreeVoxelVolume;
struct FOctreePathfindingData;

UENUM(BlueprintType)
enum class EPathfindingState : uint8
{
	Idle,
	FollowPath,
	PathInvalidated
};

UENUM(BlueprintType)
enum class EPathMovementMode : uint8
{
	Linear,
	Spline
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnDestinationReached);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnMovementRequested, const FVector&, Velocity, float, DeltaTime, const FVector&, FacingDirection);

/** Debug visualization settings for the agent's path. */
USTRUCT(BlueprintType)
struct FPathDebugDrawSettings
{
	GENERATED_BODY()

	/** If true, the agent's calculated path will be visualized in the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug")
	bool bDrawDebugPath = true;

	/** How long (in seconds) the debug path visualization persists in the viewport. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta=(DisplayName = "Draw Path Duration (sec)"))
	float DrawDuration = 1.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta=(DisplayName = "Path Line Thickness"))
	float LineThickness = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta=(DisplayName = "Path Point Box Thickness"))
	float BoxThickness = 35.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta=(DisplayName = "Path Point Box Half Size"))
	FVector BoxHalfSize = FVector(2.5f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta=(DisplayName = "Path Line Color"))
	FLinearColor PathColor = FLinearColor(0.5f, 0.0f, 1.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug", meta=(DisplayName = "Path Point Color"))
	FLinearColor PathPointColor = FLinearColor(.5f, 0.5f, 1.0f);
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class SMARTFLYINGNAVIGATION_API UPathfindingComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPathfindingComponent();

protected:
#pragma region Engine Overrides

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
							   FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void OnRegister() override;

#pragma endregion

#pragma region Public API
public:
	UFUNCTION(BlueprintCallable, Category = "Pathfinding|Configuration")
	bool RequestPathToLocation(const FVector& TargetLocation);

	bool RequestLocalPath_Internal(const FVector& Start, const FVector& End, AOctreeVoxelVolume* Volume);

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	bool RequestPathToTargetActor();

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	void SetTargetActor(AActor* NewTarget);

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	AVoxelVolumeBase* GetCurrentVolume() const;

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	AActor* GetTargetActor() const { return TargetActor; }

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	void SetMovementMode(EPathMovementMode InMovementMode) { MovementMode = InMovementMode; }

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	EPathMovementMode GetMovementMode() const { return MovementMode; }

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	void SetPathfindingState(EPathfindingState InState) { CurrentState = InState; }

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	EPathfindingState GetPathfindingState() const { return CurrentState; }

	UPROPERTY(BlueprintAssignable, Category = "Pathfinding|Event")
	FOnDestinationReached OnDestinationReached;

	void CreateOctreeDataSnapshot(const AOctreeVoxelVolume* OctreeVolume, FOctreePathfindingData& SnapshotData) const;

	bool HasReachedDestination() const { return CurrentPathDestinationIndex >= FoundPath.Num(); }

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	void StopMovement();

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
#pragma endregion

#pragma region Avoidance Interface

	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	FVector GetPreferredVelocity() const;

	FVector GetCurrentVelocity() const { return CurrentVelocity; }

	void ApplyMovement(const FVector& FinalVelocity, float DeltaTime);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Movement")
	TScriptInterface<IFlyingMovementExecutor> MovementExecutor;

	UPROPERTY(BlueprintAssignable, Category = "Pathfinding|Movement")
	FOnMovementRequested OnMovementRequested;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Movement")
	bool bExecutorHandlesCollision = false;

	/** Cached owner's UFlyingNavMovementComponent; preferred movement route when present. */
	UPROPERTY(Transient)
	TWeakObjectPtr<class UFlyingNavMovementComponent> NavMovementComponent;

	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetAgentRadius() const { return AgentRadius; }

	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetAvoidanceRadius() const { return AvoidanceRadius; }

	void SetAvoidanceRadius(float InRadius) { AvoidanceRadius = InRadius; }

	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetMovementSpeed() const { return MovementSpeed; }

	UPROPERTY()
	FVector AppliedVelocity;

	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	FVector GetAppliedVelocity() const { return AppliedVelocity; }
	void SetAppliedVelocity(FVector InAppliedVelocity) { AppliedVelocity = InAppliedVelocity; }

	bool GetbDrawDebugPath() const { return DebugDraw.bDrawDebugPath; }
	void SetbDrawDebugPath(bool NewValue) { DebugDraw.bDrawDebugPath = NewValue; }

#pragma endregion

#pragma region Spline Following
	void UpdateSteering(float DeltaTime);
	void UpdatePreferredVelocity(float DeltaTime);
	void UpdateLinearPreferredVelocity(float DeltaTime);
	void UpdateSplinePreferredVelocity(float DeltaTime);

	/** Bends PreferredVelocity away from nearby walls using the baked distance field gradient. */
	void ApplyWallRepulsion();

	bool IsSplinePathSafe() const;
	bool IsOnSplineLine() const;

	void AdjustProgressToSpline(const FVector& MoveDelta);
	void ProgressNextPoint();
	void DetermineMovementModeForNextPath();

#pragma endregion

protected:
	FVector PreferredVelocity;

	UPROPERTY()
	USphereComponent* DebugSphereComponent;

	bool bPeriodicRepath = true;

#pragma region Internal Logic

	void ExecuteDefaultMovement(const FVector& Velocity, float DeltaTime);
	void HandleWaypointProgression();
	void PeriodicRepath();

	/** Lazily caches and returns the owner's UFlyingNavMovementComponent (nullptr when absent). */
	class UFlyingNavMovementComponent* ResolveNavMovementComponent();

	/** One-time warning flag for the deprecated AddActorWorldOffset fallback. */
	bool bWarnedDefaultMovement = false;

	UFUNCTION()
	void HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess);

	void PartialSplinePointUpdate(int32 StartIndex);
	void UpdateAgentRadiusFromScale(bool bForceUpdate = false);
	void InvalidatePathAndRequestRepath();
	void RebuildCurrentCorridorFromFoundPath(bool bDynamicValidated);

	bool IsPathObstructed(const FVector& Start, const FVector& End, bool bIsPortalApproach = false) const;

public:
#if ENABLE_DRAW_DEBUG
	/** Draws the current path (single frame). Works in any non-Shipping build, including on-device. */
	void DrawDebugPath();
#endif

protected:

#pragma endregion

#pragma region Configuration Properties
public:

	UPROPERTY()
	TSubclassOf<UPathfindingStrategy> PathfindingStrategyClass;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"), DisplayName = "Pathfinding Radius")
	float AgentRadius = 50.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (ClampMin = "0.0", DisplayName = "Base Radius"))
	float BaseAgentRadius = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"), DisplayName = "Avoidance Radius")
	float AvoidanceRadius = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
	float MovementSpeed = 500.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Configuration", meta = (AllowPrivateAccess = "true"))
	float RotationInterpSpeed = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Configuration", meta = (AllowPrivateAccess = "true"))
	float RepathCooldown = 1.f;

	/** When true (and a baked distance field exists), steering is pushed away from walls closer than AgentRadius * 1.5. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Configuration", meta = (AllowPrivateAccess = "true"))
	bool bUseWallRepulsion = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Dynamic Repath", meta = (AllowPrivateAccess = "true"))
	int32 PartialPointSplineNum = 5;

	UPROPERTY()
	float LastKnownBaseRadius;

	UPROPERTY()
	FVector LastKnownScale;

	UPROPERTY()
	bool bIsReachDestination = false;

	/** Debug path visualization settings. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Debug", meta = (AllowPrivateAccess = "true"))
	FPathDebugDrawSettings DebugDraw;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding")
	float AcceptanceRadius = 10.f;

#pragma endregion

#pragma region Internal State

	UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Pathfinding|Dynamic Repath", meta = (AllowPrivateAccess = "true"))
	EPathMovementMode MovementMode = EPathMovementMode::Spline;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
	EPathfindingState CurrentState = EPathfindingState::Idle;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
	FVector CurrentTargetLocation;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
	TArray<FVector> FoundPath;

	FNavPathCorridor CurrentPathCorridor;

	int32 CurrentPathDestinationIndex = -1;
	float DistanceAlongSpline = 0.f;
	FVector CurrentVelocity;
	int32 CurrentRequestID = 0;

#pragma endregion

#pragma region Dependencies & Handles

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess = "true", HiddenInDetails))
	TObjectPtr<AActor> TargetActor;

	UPROPERTY()
	TObjectPtr<UPathfindingStrategy> PathfindingStrategy;

	UPROPERTY()
	TObjectPtr<UPathfindingStrategy> FallbackPathfindingStrategy;

	/** Subsystems are singletons per world: fetched on demand instead of cached. */
	UPathfindingSubsystem* GetPathfindingSubsystem() const;
	UAvoidanceSubsystem* GetAvoidanceSubsystem() const;

	FTimerHandle RepathCooldownTimer;

private:
	UPROPERTY()
	TObjectPtr<USplineComponent> PathSplinePtr;

	FVector LastKnownTargetLocation;

#pragma endregion

	UFUNCTION()
	void OnVolumeBecameReadyForPendingRequest(AOctreeVoxelVolume* RegisteredVolume);

	bool bHasPendingPathRequest = false;
	bool bIsPathRequestInFlight = false;
	int32 ConsecutiveBlockedFrames = 0;

	UPROPERTY()
	TWeakObjectPtr<AOctreeVoxelVolume> ActiveVolume;
	FVector PendingTargetLocation;
};
