#pragma once

#include "CoreMinimal.h"
#include "Async/AsyncWork.h"
#include "Pathfinder/Subsystem/AvoidanceSubsystem.h" 

class FParallelAvoidanceTask : public FNonAbandonableTask
{
public:
    FParallelAvoidanceTask(
        UWorld* InWorld,
        TArray<FAgentThreadData>&& InAgentData,
        TMap<uint32, FThreadAvoidanceResult>& InResults,
        FCriticalSection& InResultsLock,
        const float InTimeHorizon,
        const int32 InMaxNeighbors,
        const float InGridCellSize,
        const float InSymmetryBreakingRangeMultiplier,
        const float InSideBiasMagnitude
    );

    void DoWork();

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(FParallelAvoidanceTask, STATGROUP_ThreadPoolAsyncTasks);
    }

private:
    UWorld* World;

    TArray<FAgentThreadData> AgentData;
    TMap<uint32, FThreadAvoidanceResult>& Results;
    FCriticalSection& ResultsLock;

    const float TimeHorizon;
    const int32 MaxNeighbors;
    const float GridCellSize;
    const float SymmetryBreakingRangeMultiplier;
    const float SideBiasMagnitude;
};