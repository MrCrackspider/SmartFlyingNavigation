#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "AvoidanceSubsystem.generated.h"

class UPathfindingComponent;
class AFenwickVoxelVolume;

UCLASS()
class EPICPROJECT_3DNAVI_API UAvoidanceSubsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    //~ Begin USubsystem Interface
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;
    //~ End USubsystem Interface

    //~ Begin FTickableGameObject Interface
    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;
    //~ End FTickableGameObject Interface

    /** 에이전트를 회피 시스템에 등록합니다. */
    void RegisterAgent(UPathfindingComponent* AgentComponent);

    /** 에이전트를 회피 시스템에서 제거합니다. */
    void UnregisterAgent(UPathfindingComponent* AgentComponent);

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
    TArray<TObjectPtr<UPathfindingComponent>> RegisteredAgents;

    /** 공간 분할을 위한 에이전트 그리드 */
    TMap<FIntVector, TArray<UPathfindingComponent*>> AgentGrid;

    /** 에이전트 그리드를 최신 상태로 업데이트합니다. */
    void UpdateAgentGrid();

    /** 지정된 에이전트의 주변 에이전트를 그리드에서 찾습니다. */
    TArray<UPathfindingComponent*> GetNearbyAgents(UPathfindingComponent* Agent);

    /** 다른 에이전트들에 대한 ORCA 제약 조건을 생성합니다. */
    void GenerateAgentConstraints(UPathfindingComponent* CurrentAgent, const TArray<UPathfindingComponent*>& NearbyAgents, TArray<FPlane>& OutConstraints);

    /** 정적 복셀 장애물에 대한 ORCA 제약 조건을 생성합니다. */
    void GenerateVoxelConstraints(UPathfindingComponent* CurrentAgent, TArray<FPlane>& OutConstraints);

    /** 대칭성 파괴 로직을 적용하여 수정된 선호 속도를 계산합니다. */
    FVector CalculateSymmetryBreakingVelocity(UPathfindingComponent* CurrentAgent, const FVector& PreferredVelocity, const TArray<UPathfindingComponent*>& NearbyAgents);

    /** ORCA 제약 조건들을 기반으로 최종 최적 속도를 계산합니다. */
    FVector SolveOptimalVelocity(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const;

    /** 회피 관련 속도 벡터들을 시각화하여 디버깅합니다. */
    void DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FVector& PreferredVelocity, const FVector& ActualPreferredVelocity, const FVector& OptimalVelocity) const;
};