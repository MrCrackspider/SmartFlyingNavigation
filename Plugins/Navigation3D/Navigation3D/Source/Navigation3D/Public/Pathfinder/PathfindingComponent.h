#pragma once

#include "CoreMinimal.h"
#include "FenwickVoxelVolume.h"
#include "Components/ActorComponent.h"
#include "PathfindingComponent.generated.h"

class USplineComponent;
class UPathfindingStrategy;
class UPathfindingSubsystem;
class UAvoidanceSubsystem;
class APathActor;
class AVoxelVolumeBase;
/**
 * 길찾기 컴포넌트의 현재 상태를 나타냅니다.
 */
UENUM(BlueprintType)
enum class EPathfindingState : uint8
{
	Idle,           // 대기 중
	FollowPath,     // 경로를 따라 이동 중 (스플라인/직선 통합)
	PathInvalidated // 경로가 막혀 재탐색 대기 중
};

/**
 * 경로를 따라 이동하는 방식을 결정합니다.
 */
UENUM(BlueprintType)
enum class EPathMovementMode  : uint8
{
	Linear, // 점프 포인트를 직선으로 따라감
	Spline  // 점프 포인트를 기반으로 부드러운 스플라인 경로를 생성하여 따라감
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnDestinationReached);

/**
 * 액터에게 비동기 길찾기, 경로 추종, 동적 장애물 회피 기능을 제공하는 컴포넌트입니다.
 * UPathfindingSubsystem과 UAvoidanceSubsystem에 의존하여 동작합니다.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class NAVIGATION3D_API
UPathfindingComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPathfindingComponent();

#pragma region Public API

	/**
	 * 지정된 월드 위치로의 경로 탐색을 시작합니다.
	 * @param TargetLocation 목표 지점의 월드 좌표
	 * @return 경로 탐색 요청이 성공적으로 시작되었으면 true를 반환합니다.
	 */
	UFUNCTION(BlueprintCallable, Category = "Pathfinding|Configuration")
	bool RequestPathToLocation(const FVector& TargetLocation);
	
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	bool RequestPathToTargetActor();
	
	UFUNCTION(BlueprintCallable, Category = "Pathfinding")
	void SetTargetActor(AActor* NewTarget);

	UPROPERTY(BlueprintAssignable, Category = "Pathfinding|Event")
	FOnDestinationReached OnDestinationReached;

#pragma endregion

#pragma region Avoidance Interface

	/** 회피 시스템이 사용할, 에이전트가 가고 싶어하는 이상적인 속도를 반환합니다. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	FVector GetPreferredVelocity() const;

	/** 현재 프레임에 적용되고 있는 실제 속도를 반환합니다. */
	FVector GetCurrentVelocity() const { return CurrentVelocity; }
	
	/** 회피 시스템이 계산한 최종 속도를 받아 액터를 실제로 움직입니다. */
	void ApplyMovement(const FVector& FinalVelocity, float DeltaTime);

	/** 회피 시스템을 위한 에이전트의 반경 Getter 입니다. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetAgentRadius() const { return AgentRadius; }

	/** 회피 시스템을 위한 에이전트의 반경 Getter 입니다. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetAvoidanceRadius() const { return AvoidanceRadius; }

	/** 현재 경로보다 더 효율적인 경로가 있는지 주기적으로 검사합니다. */
	void CheckForPathOptimization();

	/** 회피 시스템을 위한 에이전트의 최대 이동 속도 Getter 입니다. */
	UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
	float GetMovementSpeed() const { return MovementSpeed; }

	/** 이 컴포넌트가 현재 목표로 하는 Voxel Volume을 반환합니다. */
	AVoxelVolumeBase* GetTargetVolume() const { return TargetVolume; }
	
	AActor* GetTargetActor() const { return TargetActor; }

	void UpdateSteering(float DeltaTime);

#pragma endregion

protected:
	FVector PreferredVelocity;
	
protected:
#pragma region Engine Overrides

	virtual void BeginPlay() override;
	void PeriodicRepath();
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

#pragma endregion

#pragma region Internal Logic

	//subsystem에서 길찾기가 비동기로 완료 되었을 때 실행될 함수
	UFUNCTION()
	void HandlePathfindingResult(const TArray<FIntVector>& PathIndices, bool bSuccess);
	
	/** 비동기 길찾기 작업이 완료되었을 때 결과를 처리합니다. */
	void HandlePathfindingResult();

	/** 새로 찾은 경로를 기반으로 이동 상태를 초기화하고 이동 방식을 결정합니다. */
	void ProcessNewPath(const TArray<FIntVector>& PathIndices);

	/** 경로가 유효하지 않을 때, 현재 이동을 중지하고 재탐색 타이머를 시작합니다. */
	void InvalidatePathAndRequestRepath();

	/** 재탐색 타이머가 만료되었을 때 호출될 콜백 함수입니다. */
	void OnRepathTimerFired();

	/** 현재 이동 방식(직선/스플라인)에 따라 다음 목표 위치를 업데이트합니다. */
	void UpdatePreferredVelocity(float DeltaTime);
	void UpdateLinearPreferredVelocity(float DeltaTime);
	void UpdateSplinePreferredVelocity(float DeltaTime);

	/** 스플라인 경로가 물리적으로 안전한지 검사합니다. */
	bool IsSplinePathSafe() const;

	/** 직선 경로가 복셀 데이터상으로 안전한지 검사합니다. */
	bool IsPathAheadValid() const;

#pragma endregion

public:
	void AdjustProgressToSpline(FVector FinalVelocity);
	void ProgressNextPoint();
	bool IsOnSplineLine();
	void DetermineMovementModeForNextPath();

public:
#pragma region Configuration Properties

	// --- 기본 설정 ---
	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	TSubclassOf<UPathfindingStrategy> PathfindingStrategyClass;

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	float AgentRadius = 50.f;

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	float AvoidanceRadius = 150.f;

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	float MovementSpeed = 500.f;

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	float RotationInterpSpeed = 5.0f; 

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	TArray<bool> RotationAxisLock = {false, false, false};
	
	// --- 동적 재탐색 설정 ---
	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	float RepathCooldown = 1.f;

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	bool bPeriodicRepath = false;

	UPROPERTY(Category = "Pathfinding|Dynamic Repath", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	int32 PathValidationLookAhead = 3;

	UPROPERTY()
	bool bIsReachDestination = false;
	
	// --- 스플라인 안전성 검사 설정 ---
	UPROPERTY(Category = "Pathfinding|Spline", EditAnywhere, BlueprintReadWrite, meta = (EditCondition = "MovementMode == EPathMovementMode::Spline", AllowPrivateAccess = true))
	float SafetyCheckStepSize = 50.f;

	// --- 디버깅 ---
	UPROPERTY(Category = "Pathfinding|Debug", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
	bool bDrawDebugPath = true;

	// --- 동적 목표 설정 ---
	UPROPERTY(Category = "Pathfinding|Dynamic Goal", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
	float GoalUpdateThreshold = 100.f; // 목표가 이 거리(cm) 이상 움직여야 재탐색

	// --- 경로 최적화 설정 ---
	UPROPERTY(Category = "Pathfinding|Optimization", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
	float PathOptimizationCheckInterval = 1.0f; // 1초마다 지름길 검사

	UPROPERTY(Category = "Pathfinding|Optimization", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true", ClampMin = "0.1", ClampMax = "1.0"))
	float OptimizationHeuristicFactor = 0.8f; // 직선 거리가 남은 경로의 80%보다 짧아지면 재탐색

public:
	void SetMovementMode(EPathMovementMode InMovementMode) { MovementMode = InMovementMode; }
	EPathMovementMode GetMovementMode() const { return MovementMode; }

#pragma endregion

#pragma region Internal State

	// --- 이동 방식 설정 ---
	UPROPERTY(Category = "Pathfinding|Dynamic Repath", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = true))
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

	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
	TObjectPtr<AActor> TargetActor;
	
	UPROPERTY(Category = "Pathfinding|Configuration", EditAnywhere, BlueprintReadWrite, meta = (AllowPrivateAccess = "true"))
	TObjectPtr<AVoxelVolumeBase> TargetVolume;

	UPROPERTY()
	TObjectPtr<UPathfindingStrategy> PathfindingStrategy;

	UPROPERTY()
	TObjectPtr<UPathfindingSubsystem> PathSubsystem;

	UPROPERTY()
	TObjectPtr<UAvoidanceSubsystem> AvoidanceSubsystem;
	
	TWeakObjectPtr<USplineComponent> PathSplinePtr;

	TFuture<TArray<FIntVector>> PathfindingTaskFuture;
	FTimerHandle RepathCooldownTimer;
	
	/* 경로 최적화 검사를 위한 타이머 핸들 */
	FTimerHandle PathOptimizationTimer;

#pragma endregion
};
