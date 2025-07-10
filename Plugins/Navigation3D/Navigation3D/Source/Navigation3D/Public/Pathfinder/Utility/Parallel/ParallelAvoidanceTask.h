// --- START OF FILE ParallelAvoidanceTask.h ---

#pragma once

#include "CoreMinimal.h"
#include "Async/AsyncWork.h"
#include "Pathfinder/AvoidanceSubsystem.h" 

class FParallelAvoidanceTask : public FNonAbandonableTask
{
public:
    // --- CHANGED ---: 생성자 시그니처 변경
    FParallelAvoidanceTask(
        UWorld* InWorld,
        TArray<FAgentThreadData>&& InAgentData,
        TMap<uint32, FThreadAvoidanceResult>& InResults,
        FCriticalSection& InResultsLock,
        const float InTimeHorizon,
        const int32 InMaxNeighbors,
        const float InGridCellSize,
        const float InSymmetryBreakingRangeMultiplier,
        const float InSideBiasMagnitude,
        const float InCollisionRadiusMultiplier
    );

    void DoWork();

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(FParallelAvoidanceTask, STATGROUP_ThreadPoolAsyncTasks);
    }

private:
    // --- ADDED ---: 스레드에서 사용할 World 포인터
    UWorld* World;

    TArray<FAgentThreadData> AgentData;
    TMap<uint32, FThreadAvoidanceResult>& Results;
    FCriticalSection& ResultsLock;

    const float TimeHorizon;
    const int32 MaxNeighbors;
    const float GridCellSize;
    const float SymmetryBreakingRangeMultiplier;
    const float SideBiasMagnitude;
    // --- ADDED ---
    const float CollisionRadiusMultiplier;
};