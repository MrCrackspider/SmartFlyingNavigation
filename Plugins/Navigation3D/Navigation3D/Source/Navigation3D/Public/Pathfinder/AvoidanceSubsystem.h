// --- START OF FILE AvoidanceSubsystem.h ---

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
    float AvoidanceRadius;
    float MovementSpeed;
    uint32 UniqueID;
};

// 스레드 계산 결과를 담을 구조체
struct FThreadAvoidanceResult
{
    uint32 AgentId;
    TArray<FPlane> AgentConstraints;
    // --- ADDED ---: 스레드에서 계산된 Voxel 제약 조건을 저장할 배열
    TArray<FPlane> VoxelConstraints;
    FVector PreferredVelocity;
    FVector ActualPreferredVelocity;
};

class UPathfindingComponent;
class FParallelAvoidanceTask;

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

    void RegisterAgent(UPathfindingComponent* AgentComponent);
    void UnregisterAgent(UPathfindingComponent* AgentComponent);

    // --- CHANGED ---: 스레드에서 World에 접근하기 위해 UWorld 포인터를 받도록 수정
    static void RunAvoidanceCalculation(
        const UWorld* World,
        const TArray<FAgentThreadData>& AgentData,
        TMap<uint32, FThreadAvoidanceResult>& OutResults,
        const float InTimeHorizon,
        const int32 InMaxNeighbors,
        const float InGridCellSize,
        const float InSymmetryBreakingRangeMultiplier,
        const float InSideBiasMagnitude,
        const float InCollisionRadiusMultiplier
    );

protected:
    // ~ Begin Tuning Parameters
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float TimeHorizon;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    int32 MaxNeighbors;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float GridCellSize;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float SymmetryBreakingRangeMultiplier;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    int32 SolverIterations;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float SideBiasMagnitude;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float InterpSpeed;

    // --- ADDED ---: 하드코딩된 값을 UPROPERTY로 만들어 튜닝이 용이하게 함
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning", meta = (ClampMin = "1.0"))
    float CollisionRadiusMultiplier;
    // ~ End Tuning Parameters

private:
    UPROPERTY()
    TArray<TObjectPtr<UPathfindingComponent>> RegisteredAgentComponents;

    UPROPERTY()
    TMap<uint32, TObjectPtr<UPathfindingComponent>> AgentMap;

    FAsyncTask<FParallelAvoidanceTask>* AsyncTask;
    TMap<uint32, FThreadAvoidanceResult> AvoidanceResults;
    FCriticalSection ResultsCriticalSection;
    bool bIsTaskRunning;

    // --- REMOVED ---: 이 함수는 스레드 안전한 static 함수로 대체됩니다.
    // void GenerateVoxelConstraints(UPathfindingComponent* CurrentAgent, const FVector& DesiredVelocity, TArray<FPlane>& OutConstraints);

    FVector SolveOptimalVelocity(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const;
    void DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FThreadAvoidanceResult& Result, const FVector& FinalOptimalVelocity) const;
};