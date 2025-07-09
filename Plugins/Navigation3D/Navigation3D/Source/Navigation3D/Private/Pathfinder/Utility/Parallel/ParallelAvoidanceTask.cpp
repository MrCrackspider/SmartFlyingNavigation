#include "Pathfinder/Utility/Parallel/ParallelAvoidanceTask.h"

FParallelAvoidanceTask::FParallelAvoidanceTask(
    TArray<FAgentThreadData>&& InAgentData,
    TMap<uint32, FThreadAvoidanceResult>& InResults,
    FCriticalSection& InResultsLock,
    const float InTimeHorizon,
    const int32 InMaxNeighbors,
    const float InGridCellSize,
    const float InSymmetryBreakingRangeMultiplier,
    const float InSideBiasMagnitude)
    : AgentData(MoveTemp(InAgentData))
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
        AgentData,
        TempResults,
        TimeHorizon,
        MaxNeighbors,
        GridCellSize,
        SymmetryBreakingRangeMultiplier,
        SideBiasMagnitude
    );

    // 계산이 모두 끝난 후, 크리티컬 섹션으로 잠그고 메인 결과 맵에 한 번에 복사합니다.
    FScopeLock Lock(&ResultsLock);
    Results = MoveTemp(TempResults);
}