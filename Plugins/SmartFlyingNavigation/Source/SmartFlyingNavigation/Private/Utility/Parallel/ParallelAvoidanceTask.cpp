#include "Utility/Parallel/ParallelAvoidanceTask.h"

FParallelAvoidanceTask::FParallelAvoidanceTask(
    UWorld* InWorld,
    TArray<FAgentThreadData>&& InAgentData,
    TMap<uint32, FThreadAvoidanceResult>& InResults,
    FCriticalSection& InResultsLock,
    const float InTimeHorizon,
    const int32 InMaxNeighbors,
    const float InGridCellSize,
    const float InSymmetryBreakingRangeMultiplier,
    const float InSideBiasMagnitude)
    : World(InWorld)
    , AgentData(MoveTemp(InAgentData))
    , Results(InResults)
    , ResultsLock(InResultsLock)
    , TimeHorizon(InTimeHorizon)
    , MaxNeighbors(InMaxNeighbors)
    , GridCellSize(InGridCellSize)
    , SymmetryBreakingRangeMultiplier(InSymmetryBreakingRangeMultiplier)
    , SideBiasMagnitude(InSideBiasMagnitude)
{
}

void FParallelAvoidanceTask::DoWork()
{
    TMap<uint32, FThreadAvoidanceResult> TempResults;

    UAvoidanceSubsystem::RunAvoidanceCalculation(
        World,
        AgentData,
        TempResults,
        TimeHorizon,
        MaxNeighbors,
        GridCellSize,
        SymmetryBreakingRangeMultiplier,
        SideBiasMagnitude
    );

    FScopeLock Lock(&ResultsLock);
    Results = MoveTemp(TempResults);
}