#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Async/AsyncWork.h"
#include "AvoidanceSubsystem.generated.h"

// 스레드 간 데이터 전달을 위한 구조체
struct FAgentThreadData
{
    uint32 AgentId;
    FVector Position;
    FVector CurrentVelocity;
    FVector PreferredVelocity;
    float AgentRadius;
    float MovementSpeed;
    // 대칭성 파괴 로직을 위해 UniqueID도 전달합니다.
    uint32 UniqueID;
};

// 스레드 계산 결과를 담을 구조체
struct FAvoidanceResult
{
    uint32 AgentId;
    FVector OptimalVelocity;
    // 디버깅을 위해 추가 정보 전달
    FVector PreferredVelocity;
    FVector ActualPreferredVelocity;
};

class UPathfindingComponent;
class FParallelAvoidanceTask; // Forward Declaration

UCLASS()
class NAVIGATION3D_API UAvoidanceSubsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void PostInitialize() override;
    virtual void Deinitialize() override;
    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;

    /** 에이전트를 회피 시스템에 등록합니다. */
    void RegisterAgent(UPathfindingComponent* AgentComponent);

    /** 에이전트를 회피 시스템에서 제거합니다. */
    void UnregisterAgent(UPathfindingComponent* AgentComponent);

    // 비동기 작업이 사용할 정적 함수
    static void RunAvoidanceCalculation(
        const TArray<FAgentThreadData>& AgentData,
        TMap<uint32, FAvoidanceResult>& OutResults,
        const float InTimeHorizon,
        const int32 InMaxNeighbors,
        const float InGridCellSize,
        const float InSymmetryBreakingRangeMultiplier,
        const int32 InSolverIterations,
        const float InSideBiasMagnitude
    );

protected:
    // ~ Begin Tuning Parameters
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float TimeHorizon;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    int32 MaxNeighbors;

    // RECOMMENDATION: VoxelExtent * 2
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float GridCellSize;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float SymmetryBreakingRangeMultiplier;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    int32 SolverIterations;

    // 대칭성 파괴 시 측면 힘의 강도 (0.0 ~ 1.0f)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float SideBiasMagnitude;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float InterpSpeed;
    // ~ End Tuning Parameters

private:
    /** 등록된 모든 회피 에이전트 목록 */
    UPROPERTY()
    TArray<TObjectPtr<UPathfindingComponent>> RegisteredAgentComponents;

    /** ID로 에이전트를 빠르게 찾기 위한 맵 */
    UPROPERTY()
    TMap<uint32, TObjectPtr<UPathfindingComponent>> AgentMap;

    /** 비동기 회피 계산 작업 */
    FAsyncTask<FParallelAvoidanceTask>* AsyncTask;

    /** 워커 스레드에서 계산된 결과 */
    TMap<uint32, FAvoidanceResult> AvoidanceResults;

    /** 결과 맵에 대한 스레드 안전 접근을 위한 크리티컬 섹션 */
    FCriticalSection ResultsCriticalSection;

    /** 작업이 현재 실행 중인지 여부 */
    bool bIsTaskRunning;

    /** 정적 복셀 장애물에 대한 ORCA 제약 조건을 생성합니다. (게임 스레드에서 호출) */
    void GenerateVoxelConstraints(UPathfindingComponent* CurrentAgent, TArray<FPlane>& OutConstraints);

    /** ORCA 제약 조건들을 기반으로 최종 최적 속도를 계산합니다. (게임 스레드에서 호출) */
    FVector SolveOptimalVelocity(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const;

    /** 회피 관련 속도 벡터들을 시각화하여 디버깅합니다. */
    void DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FAvoidanceResult& Result, const FVector& FinalOptimalVelocity) const;
};