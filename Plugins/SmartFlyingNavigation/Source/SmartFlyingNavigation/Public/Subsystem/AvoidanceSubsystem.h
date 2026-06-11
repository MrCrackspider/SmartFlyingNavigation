// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Async/AsyncWork.h"
#include "Templates/UniquePtr.h"
#include "Utility/Parallel/ParallelAvoidanceTask.h"
#include "AvoidanceSubsystem.generated.h"

class UPathfindingComponent;
class UDynamicObstacleComponent;

UENUM(BlueprintType)
enum class ESmartFlyingAvoidanceMode : uint8
{
    None UMETA(DisplayName = "None"),
    Simple UMETA(DisplayName = "Simple Mobile"),
    ORCA UMETA(DisplayName = "High ORCA")
};

UCLASS(Config = Game)
class SMARTFLYINGNAVIGATION_API UAvoidanceSubsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    /** Out-of-line so TUniquePtr<FAsyncTask<FParallelAvoidanceTask>> can destroy a complete type. */
    virtual ~UAvoidanceSubsystem() override;

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
    
    bool GetbUseAvoidance() const { return bUseAvoidance && AvoidanceMode != ESmartFlyingAvoidanceMode::None; }
    void SetbUseAvoidance(bool IsUse);

    UFUNCTION(BlueprintPure, Category = "Avoidance|Tuning")
    ESmartFlyingAvoidanceMode GetAvoidanceMode() const { return AvoidanceMode; }

    UFUNCTION(BlueprintCallable, Category = "Avoidance|Tuning")
    void SetAvoidanceMode(ESmartFlyingAvoidanceMode InMode) { AvoidanceMode = InMode; }

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
        const float InSideBiasMagnitude,
        const TArray<FDynamicObstacleThreadData>& DynamicObstacles = TArray<FDynamicObstacleThreadData>()
    );


    float GetGridCellSize() const {return GridCellSize;}
    void SetGridCellSize(float InGridCellSize) {GridCellSize = InGridCellSize;}
    
    bool GetbShowVelocityDebugLine() const{ return bShowVelocityDebugLine;}
    void SetbShowVelocityDebugLine(bool bShow) { bShowVelocityDebugLine = bShow;}

    /** Number of agents currently registered for avoidance (for stats/HUD). */
    int32 GetRegisteredAgentCount() const { return RegisteredAgentComponents.Num(); }

    /** Registered agents (for debug tooling). */
    const TArray<TObjectPtr<UPathfindingComponent>>& GetRegisteredAgents() const { return RegisteredAgentComponents; }
    
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
    float InterpSpeed = 10.f;

private:
    UPROPERTY()
    TArray<TObjectPtr<UPathfindingComponent>> RegisteredAgentComponents;

    UPROPERTY()
    TMap<uint32, TObjectPtr<UPathfindingComponent>> AgentMap;

    TUniquePtr<FAsyncTask<FParallelAvoidanceTask>> AsyncTask;
    TMap<uint32, FThreadAvoidanceResult> AvoidanceResults;
    FCriticalSection ResultsCriticalSection;
    bool bIsTaskRunning = false;

    FVector SolveOptimalVelocity(const TArray<FPlane>& Constraints, const FVector& PreferredVelocity) const;
    void TickSimpleAvoidance(float DeltaTime);

    /** Builds the per-solve dynamic obstacle snapshot from the world obstacle registry. */
    void GatherDynamicObstacleData(TArray<FDynamicObstacleThreadData>& OutObstacles) const;

private:
    UPROPERTY(Config)
    bool bShowVelocityDebugLine = false;
    
    void DebugVisualizeVelocities(const UPathfindingComponent* Agent, const FThreadAvoidanceResult& Result, const FVector& FinalOptimalVelocity) const;

    UPROPERTY(Config)
    bool bUseAvoidance = true;

    UPROPERTY(Config, EditAnywhere, Category = "Avoidance|Tuning")
    ESmartFlyingAvoidanceMode AvoidanceMode = ESmartFlyingAvoidanceMode::ORCA;

    /** When true, dynamic obstacles (DynamicObstacleComponent) generate ORCA constraints for local avoidance. */
    UPROPERTY(Config, EditAnywhere, Category = "Avoidance|Tuning", meta=(AllowPrivateAcess))
    bool bAvoidDynamicObstacles = true;

    /** Applies conservative Android defaults for low-memory devices. */
    UPROPERTY(Config, EditAnywhere, Category = "Avoidance|Mobile")
    bool bUseMobileLowSpecSettings = true;

    /** Minimum interval between full ORCA avoidance solves on low-spec mobile. */
    UPROPERTY(Config, EditAnywhere, Category = "Avoidance|Mobile", meta=(ClampMin="0.0"))
    float MobileAvoidanceUpdateInterval = 0.1f;

    UPROPERTY(Config, EditAnywhere, Category = "Avoidance|Mobile", meta=(ClampMin="0"))
    int32 MobileMaxNeighbors = 3;

    UPROPERTY(Config, EditAnywhere, Category = "Avoidance|Mobile")
    bool bMobileDisableDynamicObstacleAvoidance = true;

    float AvoidanceUpdateAccumulator = 0.f;

    void ApplyMobileLowSpecSettings();
};
