#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PathfindingComponent.generated.h"

class USplineComponent;
// 순환 종속성 방지를 위한 전방 선언
class UPathfindingStrategy;
class UPathfindingSubsystem;
class UAvoidanceSubsystem; // UAgentAvoidanceSubsystem에서 이름 변경
class APathActor;
class AVoxelVolumeBase;

UENUM(BlueprintType)
enum class EPathfindingState : uint8
{
	Idle,			// 대기 중
	FollowSpline,			// 경로 따라 이동 중
	FollowLinear,
	RequestingPath, // 백그라운드 스레드에 경로를 요청하고 결과를 기다리는 중
	PathInvalidated	// 경로가 막혀 재탐색 대기 중
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class EPICPROJECT_3DNAVI_API UPathfindingComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UPathfindingComponent();
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UFUNCTION(BlueprintCallable, Category = "Pathfinding")
    bool FindPathToLocation(const FVector& TargetLocation);
  // --- UAvoidanceSubsystem과의 연동을 위한 새로운 인터페이스 함수들 ---

    /** 에이전트가 방해받지 않을 때 가고 싶어하는 이상적인 속도를 반환합니다. */
    UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
    FVector GetPreferredVelocity() const;

    /** 현재 프레임에 적용되고 있는 실제 속도를 반환합니다. */
    FVector GetCurrentVelocity() const { return CurrentVelocity; }

    /** UAvoidanceSubsystem이 계산한 최종 속도를 받아 액터를 실제로 움직입니다. */
    void ApplyMovement(const FVector& FinalVelocity, float DeltaTime);

    /** UAvoidanceSubsystem에서 에이전트의 크기 정보를 가져가기 위한 Getter 입니다. */
    UFUNCTION(BlueprintPure, Category = "Pathfinding|Avoidance")
    float GetAgentRadius() const { return Radius; }


    // --- 블루프린트 및 에디터 노출 속성 ---

    // 에이전트의 물리적 반경 (월드 유닛)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding")
    float Radius = 0.f;

    // 에이전트의 최대 이동 속도
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding")
    float MovementSpeed;
	//Tick에서 호출될 이동 함수
	void UpdateSplinePreferredLocation(float DeltaTime);

	void UpdateLinearPreferredLocation(float DeltTime);

	//길찾기 성공 후 호출되어 스플라인을 생성/업데이트하는 함수
	void CreateSplineFromPath();

	bool IsSplinePathSafe(); //Spline 안정성 검사 함수
	
	// 재탐색 타이머가 만료되었을 때 호출될 함수
	void OnRepathTimerFired();
	
    void UpdateNextLocation(float DeltaTime);

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pathfinding|Dynamic")
	FVector NextPreferredLocation;
	
    /** 이 에이전트가 현재 가지고 있는 실제 속도 벡터입니다. */
    FVector CurrentVelocity;

    // 경로가 막혔을 때 재탐색을 시도하기까지의 대기 시간
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Dynamic")
    float RepathCooldown = 0.5f;

    // 전방 경로의 유효성을 몇 개의 점프 포인트 앞까지 검사할지 결정합니다.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Dynamic")
    int32 PathValidationLookAhead = 3;

    //UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
    TObjectPtr<AVoxelVolumeBase> TargetVolume;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spline Safety")
	bool bEnableSplineSafetyCheck = true; //안정성 검사 활성화 여부

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spline Safety")
	float SafetyCheckStepSize = 50.f; //샘플링 간격
	
    // 디버깅용
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding|Debug")
    bool bDrawDebugPath = true;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    // 경로 요청 결과를 처리하는 함수
    void HandleRequestingPathState();

    // 전방 경로가 막혔는지 확인하고 상태를 변경하는 함수
    bool CheckAndHandlePathInvalidation();

    // 전방 경로가 실제로 유효한지 검사하는 함수
    bool IsPathAheadValid() const;

    // --- 내부 상태 및 데이터 ---

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
    EPathfindingState CurrentState;

    // 재탐색을 위해 목표 위치를 저장
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
    FVector CurrentTargetLocation;
	
    // 현재 이동 목표인 경로상의 인덱스 (-1은 이동 중이 아님을 의미)
    int32 CurrentPathIndex;

    // 길찾기 결과로 받은 월드 좌표 경로
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding|State", meta = (AllowPrivateAccess = "true"))
    TArray<FVector> FoundPath;

    // 길찾기 결과로 받은 복셀 인덱스 경로
    TArray<FIntVector> FoundPathIndices;

    // 비동기 길찾기 작업을 위한 Future
    TFuture<TArray<FIntVector>> PathfindingTaskFuture;

    // 재탐색 쿨다운 타이머 핸들
    FTimerHandle RepathCooldownTimer;

    // --- 길찾기 전략 및 서브시스템 포인터 ---

    UPROPERTY(EditAnywhere, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
    TSubclassOf<UPathfindingStrategy> PathfindingStrategyClass;

    UPROPERTY()
    TObjectPtr<UPathfindingStrategy> PathfindingStrategy;

protected:
    UPROPERTY()
    TObjectPtr<UPathfindingSubsystem> PathSubsystem;

    UPROPERTY()
    TObjectPtr<UAvoidanceSubsystem> AvoidanceSubsystem;

	UPROPERTY()
	APathActor* CurrentPathActor = nullptr; // 스플라인을 소유한 액터에 대한 포인터

	// USplineComponent* PathSpline; // 직접 소유하는 대신 포인터만 가짐
	UPROPERTY()
	TWeakObjectPtr<USplineComponent> PathSplinePtr = nullptr;
	
	//스플라인을 따라 이동한 총 거리
	float DistanceAlongSpline;
};