#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "AvoidanceSubsystem.generated.h"

class UPathfindingComponent;

UCLASS()
class EPICPROJECT_3DNAVI_API UAvoidanceSubsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;
    void RegisterAgent(UPathfindingComponent* AgentComponent);
    void UnregisterAgent(UPathfindingComponent* AgentComponent);

protected:
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

    // 최적 속도 보간 속도
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float InterpSpeed = 10.0f;

private:
    UPROPERTY()
    TArray<TObjectPtr<UPathfindingComponent>> RegisteredAgents;

    TMap<FIntVector, TArray<UPathfindingComponent*>> AgentGrid;

    void UpdateAgentGrid();
    TArray<UPathfindingComponent*> GetNearbyAgents(UPathfindingComponent* Agent);

    FVector SolveOptimalVelocity_Final(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const;

};