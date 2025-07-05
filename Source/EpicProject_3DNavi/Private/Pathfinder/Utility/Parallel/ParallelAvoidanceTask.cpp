// ParallelAvoidanceTask.cpp

#include "Pathfinder/Utility/Parallel/ParallelAvoidanceTask.h"

FParallelAvoidanceTask::FParallelAvoidanceTask(TArray<FAgentThreadData>&& InAgentData, TMap<uint32, FAvoidanceResult>& InResults, FCriticalSection& InResultsLock, const float InTimeHorizon, const int32 InMaxNeighbors, const float InGridCellSize, const float InSymmetryBreakingRangeMultiplier, const int32 InSolverIterations, const float InSideBiasMagnitude)
    : AgentData(MoveTemp(InAgentData))
    , Results(InResults)
    , ResultsLock(InResultsLock)
    , TimeHorizon(InTimeHorizon)
    , MaxNeighbors(InMaxNeighbors)
    , GridCellSize(InGridCellSize)
    , SymmetryBreakingRangeMultiplier(InSymmetryBreakingRangeMultiplier)
    , SolverIterations(InSolverIterations)
    , SideBiasMagnitude(InSideBiasMagnitude)
{
}

void FParallelAvoidanceTask::DoWork()
{
    // 임시 결과 맵을 만들어 계산을 수행합니다.
    TMap<uint32, FAvoidanceResult> TempResults;
    UAvoidanceSubsystem::RunAvoidanceCalculation(AgentData, TempResults, TimeHorizon, MaxNeighbors, GridCellSize, SymmetryBreakingRangeMultiplier, SolverIterations, SideBiasMagnitude);

    // 계산이 모두 끝난 후, 크리티컬 섹션으로 잠그고 메인 결과 맵에 한 번에 복사합니다.
    // 이렇게 하면 락을 최소한의 시간만 유지할 수 있습니다.
    FScopeLock Lock(&ResultsLock);
    Results = MoveTemp(TempResults);
}