// --- START OF FILE ParallelAvoidanceTask.cpp ---

#include "Pathfinder/Utility/Parallel/ParallelAvoidanceTask.h"

// --- CHANGED ---: 생성자 구현 변경
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
    const float InCollisionRadiusMultiplier)
    : World(InWorld)
    , AgentData(MoveTemp(InAgentData))
    , Results(InResults)
    , ResultsLock(InResultsLock)
    , TimeHorizon(InTimeHorizon)
    , MaxNeighbors(InMaxNeighbors)
    , GridCellSize(InGridCellSize)
    , SymmetryBreakingRangeMultiplier(InSymmetryBreakingRangeMultiplier)
    , SideBiasMagnitude(InSideBiasMagnitude)
    , CollisionRadiusMultiplier(InCollisionRadiusMultiplier)
{
}

void FParallelAvoidanceTask::DoWork()
{
    TMap<uint32, FThreadAvoidanceResult> TempResults;

    // --- CHANGED ---: World와 CollisionRadiusMultiplier를 계산 함수에 전달
    UAvoidanceSubsystem::RunAvoidanceCalculation(
        World,
        AgentData,
        TempResults,
        TimeHorizon,
        MaxNeighbors,
        GridCellSize,
        SymmetryBreakingRangeMultiplier,
        SideBiasMagnitude,
        CollisionRadiusMultiplier
    );

    FScopeLock Lock(&ResultsLock);
    Results = MoveTemp(TempResults);
}