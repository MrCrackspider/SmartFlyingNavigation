// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
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

	UFUNCTION(BlueprintCallable, Category = "Pathfinding|Debug")
	void DEBUG_StartManualPathTest();
	
#pragma endregion

#pragma region Public API
public:
	/**
	* Starts a path search to the specified world location.
	* @param TargetLocation World search of the target target
	* @return If the search request is successful, returns true if started.
	*/
	UFUNCTION(BlueprintCallable, Category = "Pathfinding|Configuration")
	bool RequestPathToLocation(const FVector& TargetLocation);

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	bool RequestPathToTargetActor();

	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	void SetTargetActor(AActor* NewTarget);
	
	/** Returns the Voxel Volume that this component currently targets. */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	AVoxelVolumeBase* GetTargetVolume() const { return TargetVolume; }

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
	
	void SetbPeriodicRepath(bool NewValue) { bPeriodicRepath = NewValue; }

	bool HasReachedDestination() const { return CurrentPathDestinationIndex >= FoundPath.Num(); }

	void StopMovement();

#if WITH_EDITOR
	// A function called when a property is changed in the editor.
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	
#pragma endregion

#pragma region Avoidance Interface

	/** Returns the ideal speed the agent would like to go, which the avoidance system would use. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	FVector GetPreferredVelocity() const;

	/** Returns the actual velocity applied to the current frame. */
	FVector GetCurrentVelocity() const { return CurrentVelocity; }

	/** The final velocity calculated by the evasion system is then used to actually move the actor. */
	void ApplyMovement(const FVector& FinalVelocity, float DeltaTime);

	/** Agent radius getter for the evasion system. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetAgentRadius() const { return AgentRadius; }

	/** Agent radius getter for the avoidance system. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetAvoidanceRadius() const { return AvoidanceRadius; }

	void SetAvoidanceRadius(float InRadius) { AvoidanceRadius = InRadius; }

	/** Getter for the agent's maximum movement speed for the evasion system. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetMovementSpeed() const { return MovementSpeed; }

	UPROPERTY()
	FVector AppliedVelocity;

	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	FVector GetAppliedVelocity() const { return AppliedVelocity; }
	void SetAppliedVelocity(FVector InAppliedVelocity) { AppliedVelocity = InAppliedVelocity; }

	bool GetbDrawDebugPath() const { return bDrawDebugPath; }
	void SetbDrawDebugPath(bool NewValue) { bDrawDebugPath = NewValue; }
	
#pragma endregion

#pragma region Spline Following
	void UpdateSteering(float DeltaTime);
	void UpdatePreferredVelocity(float DeltaTime);
	void UpdateLinearPreferredVelocity(float DeltaTime);
	void UpdateSplinePreferredVelocity(float DeltaTime);

	bool IsSplinePathSafe() const;
	bool IsOnSplineLine() const;
	
	void AdjustProgressToSpline(const FVector& FinalVelocity);
	void ProgressNextPoint();
	void DetermineMovementModeForNextPath();

#pragma endregion

	
protected:
	FVector PreferredVelocity;

	UPROPERTY()
	USphereComponent* DebugSphereComponent;
	
#pragma region Internal Logic

	void PeriodicRepath();
	
	UFUNCTION()
	void HandleWorldPathResult(const TArray<FVector>& Path, bool bSuccess);

	void PartialSplinePointUpdate(int32 StartIndex);

	void CreateOctreeDataSnapshot(const AOctreeVoxelVolume* OctreeVolume, FOctreePathfindingData& SnapshotData) const;

	void UpdateAgentRadiusFromScale(bool bForceUpdate = false);
	
	/** When the path is invalid, the current movement is stopped and a retracing timer is started. */
	void InvalidatePathAndRequestRepath();

	bool IsPathObstructed(const FVector& Start, const FVector& End) const;
	
	void DrawDebugPath();

	
#pragma endregion

#pragma region Configuration Properties
public:

	UPROPERTY()
	TSubclassOf<UPathfindingStrategy> PathfindingStrategyClass;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding", meta = (AllowPrivateAccess = true))
	float AgentRadius = 50.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (ClampMin = "0.0", AllowPrivateAccess = true))
	float BaseAgentRadius = 50.0f;
	
	UPROPERTY(EditAnywhere,Category = "Pathfinding|Configuration", BlueprintReadWrite,
		meta = (AllowPrivateAccess = true))
	float AvoidanceRadius = 50.f;

	UPROPERTY(EditAnywhere, Category = "Pathfinding", BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	float MovementSpeed = 500.f;

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite,
		meta = (AllowPrivateAccess = true))
	float RotationInterpSpeed = 5.0f;

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite,
		meta = (AllowPrivateAccess = true))
	TArray<bool> RotationAxisLock = {false, false, false};

	// --- dynamic repath settings ---
	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite,
		meta = (AllowPrivateAccess = true))
	float RepathCooldown = 1.f;

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere,BlueprintReadWrite,
		meta = (AllowPrivateAccess = true))
	bool bPeriodicRepath = false;

	UPROPERTY(Category = "Pathfinding|Dynamic Repath",EditAnywhere, BlueprintReadWrite,
		meta = (AllowPrivateAccess = true))
	int32 PathValidationLookAhead = 3;
	
	UPROPERTY(Category = "Pathfinding|Dynamic Repath", EditAnywhere,BlueprintReadWrite,
		meta = (AllowPrivateAccess = true))
	int32 PartialPointSplineNum = 5;
	
	UPROPERTY()
	float LastKnownBaseRadius;
	
	UPROPERTY()
	FVector LastKnownScale;

	UPROPERTY()
	bool bIsReachDestination = false;

	UPROPERTY()
	bool bContinuosFollowingWhenReached;
	
	UPROPERTY(Category = "Pathfinding|Spline", EditAnywhere, BlueprintReadWrite,
		meta = (EditCondition = "MovementMode == EPathMovementMode::Spline", AllowPrivateAccess = true))
	float SafetyCheckStepSize = 50.f;
 
	UPROPERTY(EditAnywhere, Category = "Pathfinding", BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	bool bDrawDebugPath = true;
	
	UPROPERTY(Category = "Pathfinding|Dynamic Goal", EditAnywhere, BlueprintReadWrite,
		meta = (AllowPrivateAccess = "true"))
	float GoalUpdateThreshold = 100.f; // The target must move this distance (cm) or more to re-explore

	UPROPERTY(Category = "Pathfinding|Optimization", EditAnywhere, BlueprintReadWrite,
		meta = (AllowPrivateAccess = "true"))
	float PathOptimizationCheckInterval = 1.0f; // Shortcut check every second

	UPROPERTY(Category = "Pathfinding|Optimization", EditAnywhere, BlueprintReadWrite,
		meta = (AllowPrivateAccess = "true", ClampMin = "0.1", ClampMax = "1.0"))
	float OptimizationHeuristicFactor = 0.8f; // Re-search when the straight-line distance is shorter than 80% of the remaining path.

#pragma endregion

#pragma region Internal State

	UPROPERTY(Category = "Pathfinding|Dynamic Repath", EditAnywhere, BlueprintReadWrite,
		meta = (AllowPrivateAccess = true))
	EPathMovementMode MovementMode = EPathMovementMode::Spline;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
	EPathfindingState CurrentState = EPathfindingState::Idle;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
	FVector CurrentTargetLocation;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
	TArray<FVector> FoundPath;

	TArray<FIntVector> FoundPathIndices;
	int32 CurrentPathDestinationIndex = -1;
	float DistanceAlongSpline = 0.f;
	FVector CurrentVelocity;
	int CurrentRequestID;
#pragma endregion

#pragma region Dependencies & Handles

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess = "true", HiddenInDetails))
	TObjectPtr<AActor> TargetActor;
	
	UPROPERTY(EditAnywhere, Category = "Pathfinding", meta = (HiddenInDetails))
	TObjectPtr<AVoxelVolumeBase> TargetVolume;
	
	UPROPERTY()
	TObjectPtr<UPathfindingStrategy> PathfindingStrategy;

	UPROPERTY() 
	TObjectPtr<UPathfindingSubsystem> PathSubsystem;

	UPROPERTY()
	TObjectPtr<UAvoidanceSubsystem> AvoidanceSubsystem;
	
	FTimerHandle RepathCooldownTimer;
	
private:
	UPROPERTY()
	TObjectPtr<USplineComponent> PathSplinePtr; 
#pragma endregion
};
