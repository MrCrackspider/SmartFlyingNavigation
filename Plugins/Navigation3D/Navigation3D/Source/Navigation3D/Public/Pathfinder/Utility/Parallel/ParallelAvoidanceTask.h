#pragma once

#include "CoreMinimal.h"
#include "Async/AsyncWork.h"
#include "Pathfinder/AvoidanceSubsystem.h" 

class FParallelAvoidanceTask : public FNonAbandonableTask
{
public:
    FParallelAvoidanceTask(
        TArray<FAgentThreadData>&& InAgentData,
        TMap<uint32, FThreadAvoidanceResult>& InResults,
        FCriticalSection& InResultsLock,
        const float InTimeHorizon,
        const int32 InMaxNeighbors,
        const float InGridCellSize,
        const float InSymmetryBreakingRangeMultiplier,
        const float InSideBiasMagnitude
    );

    // 워커 스레드에서 실제 작업을 수행할 함수
    void DoWork();

    // 통계 시스템에서 사용할 이름
    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(FParallelAvoidanceTask, STATGROUP_ThreadPoolAsyncTasks);
    }

private:
    // 스레드에서 사용할 데이터
    TArray<FAgentThreadData> AgentData;
    TMap<uint32, FThreadAvoidanceResult>& Results;
    FCriticalSection& ResultsLock;

    // 서브시스템의 튜닝 파라미터 복사본
    const float TimeHorizon;
    const int32 MaxNeighbors;
    const float GridCellSize;
    const float SymmetryBreakingRangeMultiplier;
    const float SideBiasMagnitude;
};