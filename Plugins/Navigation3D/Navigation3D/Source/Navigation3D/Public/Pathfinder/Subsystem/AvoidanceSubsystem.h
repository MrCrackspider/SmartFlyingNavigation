// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Async/AsyncWork.h"
#include "AvoidanceSubsystem.generated.h"

struct FAgentThreadData
{
    uint32 AgentId;
    FVector Position;
    FVector UpVector;
    FVector CurrentVelocity;
    FVector PreferredVelocity;
    float AvoidanceRadius;
    float MovementSpeed;
};

struct FThreadAvoidanceResult
{
    uint32 AgentId;
    TArray<FPlane> AgentConstraints;
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

    UFUNCTION(BlueprintCallable, Category = "Avoidance Subsystem")
    void RegisterAgent(UPathfindingComponent* AgentComponent);
    UFUNCTION(BlueprintCallable, Category = "Avoidance Subsystem")
    void UnregisterAgent(UPathfindingComponent* AgentComponent);

    static void RunAvoidanceCalculation(
        const UWorld* World,
        const TArray<FAgentThreadData>& AgentData,
        TMap<uint32, FThreadAvoidanceResult>& OutResults,
        const float InTimeHorizon,
        const int32 InMaxNeighbors,
        const float InGridCellSize,
        const float InSymmetryBreakingRangeMultiplier,
        const float InSideBiasMagnitude
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

    FVector SolveOptimalVelocity(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const;

    bool bShowVelocityDebugLine;
    void DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FThreadAvoidanceResult& Result, const FVector& FinalOptimalVelocity) const;

};