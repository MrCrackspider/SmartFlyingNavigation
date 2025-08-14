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

UCLASS(Config = Game)
class SMARTFLYINGNAVIGATION_API UAvoidanceSubsystem : public UTickableWorldSubsystem
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

    void SetTimeHorizon(float InValue) { TimeHorizon = InValue; }
    void SetMaxNeighbors(int32 InValue) { MaxNeighbors = InValue; }
    void SetSymmetryBreakingRangeMultiplier(float InValue) { SymmetryBreakingRangeMultiplier = InValue; }
    void SetSolverIterations(int32 InValue) { SolverIterations = InValue; }
    void SetSideBiasMagnitude(float InValue) { SideBiasMagnitude = InValue; }
    void SetInterpSpeed(float InValue) { InterpSpeed = InValue; }
    
    bool GetbUseAvoidance() const { return bUseAvoidance; }
    void SetbUseAvoidance(bool IsUse) { bUseAvoidance = IsUse; };

    UFUNCTION(BlueprintPure, Category = "Avoidance|Tuning")
    float GetTimeHorizon() const { return TimeHorizon; }

    UFUNCTION(BlueprintPure, Category = "Avoidance|Tuning")
    int32 GetMaxNeighbors() const { return MaxNeighbors; }

    UFUNCTION(BlueprintPure, Category = "Avoidance|Tuning")
    float GetSymmetryBreakingRangeMultiplier() const { return SymmetryBreakingRangeMultiplier; }

    UFUNCTION(BlueprintPure, Category = "Avoidance|Tuning")
    int32 GetSolverIterations() const { return SolverIterations; }

    UFUNCTION(BlueprintPure, Category = "Avoidance|Tuning")
    float GetSideBiasMagnitude() const { return SideBiasMagnitude; }

    UFUNCTION(BlueprintPure, Category = "Avoidance|Tuning")
    float GetInterpSpeed() const { return InterpSpeed; }
    
    
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


    float GetGridCellSize() const {return GridCellSize;}
    void SetGridCellSize(float InGridCellSize) {GridCellSize = InGridCellSize;}
    
    bool GetbShowVelocityDebugLine() const{ return bShowVelocityDebugLine;}
    void SetbShowVelocityDebugLine(bool bShow) { bShowVelocityDebugLine = bShow;} 
    
protected:
    UPROPERTY(Config, EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float TimeHorizon = 1.f;

    UPROPERTY(Config, EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    int32 MaxNeighbors = 8;

    UPROPERTY(Config, EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float GridCellSize = 100.f;

    UPROPERTY(Config, EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float SymmetryBreakingRangeMultiplier = 8.f;

    UPROPERTY(Config, EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    int32 SolverIterations = 5;

    UPROPERTY(Config, EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float SideBiasMagnitude = 10.f;

    UPROPERTY(Config, EditAnywhere, BlueprintReadWrite, Category = "Avoidance|Tuning")
    float InterpSpeed = 1.f;

private:
    UPROPERTY()
    TArray<TObjectPtr<UPathfindingComponent>> RegisteredAgentComponents;

    UPROPERTY()
    TMap<uint32, TObjectPtr<UPathfindingComponent>> AgentMap;

    FAsyncTask<FParallelAvoidanceTask>* AsyncTask = nullptr;
    TMap<uint32, FThreadAvoidanceResult> AvoidanceResults;
    FCriticalSection ResultsCriticalSection;
    bool bIsTaskRunning = false;

    FVector SolveOptimalVelocity(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const;

    UPROPERTY(Config)
    bool bShowVelocityDebugLine;
    
    void DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FThreadAvoidanceResult& Result, const FVector& FinalOptimalVelocity) const;

    UPROPERTY(Config)
    bool bUseAvoidance;
};