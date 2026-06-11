// Copyright © 2025 TeamHamDev. All Rights Reserved.


#include "Utility/Parallel/ParallelAvoidanceTask.h"
#include "Subsystem/AvoidanceSubsystem.h"
#include "NavCore/SmartFlyingNavStats.h"

FParallelAvoidanceTask::FParallelAvoidanceTask(
    UWorld* InWorld,
    TArray<FAgentThreadData>&& InAgentData,
    TMap<uint32, FThreadAvoidanceResult>& InResults,
    FCriticalSection& InResultsLock,
    const float InTimeHorizon,
    const int32 InMaxNeighbors,
    const float InGridCellSize,
    const float InSymmetryBreakingRangeMultiplier,
    const float InSideBiasMagnitude,
    TArray<FDynamicObstacleThreadData>&& InDynamicObstacles)
    : World(InWorld)
    , AgentData(MoveTemp(InAgentData))
    , Results(InResults)
    , ResultsLock(InResultsLock)
    , TimeHorizon(InTimeHorizon)
    , MaxNeighbors(InMaxNeighbors)
    , GridCellSize(InGridCellSize)
    , SymmetryBreakingRangeMultiplier(InSymmetryBreakingRangeMultiplier)
    , SideBiasMagnitude(InSideBiasMagnitude)
    , DynamicObstacles(MoveTemp(InDynamicObstacles))
{
}

void FParallelAvoidanceTask::DoWork()
{
    SCOPE_CYCLE_COUNTER(STAT_SFN_AvoidanceSolve);

    TMap<uint32, FThreadAvoidanceResult> TempResults;

    UAvoidanceSubsystem::RunAvoidanceCalculation(
        World,
        AgentData,
        TempResults,
        TimeHorizon,
        MaxNeighbors,
        GridCellSize,
        SymmetryBreakingRangeMultiplier,
        SideBiasMagnitude,
        DynamicObstacles
    );

    FScopeLock Lock(&ResultsLock);
    Results = MoveTemp(TempResults);
}