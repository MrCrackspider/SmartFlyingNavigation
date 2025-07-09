// PathfindingAsyncTask.h
#pragma once

#include "CoreMinimal.h"
#include "Async/AsyncWork.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "Pathfinder/PathfindingRequest.h"

class FPathfindingAsyncTask : public FNonAbandonableTask
{
public:
	/**
	 * 생성자: 길찾기 요청 데이터를 복사하여 작업을 준비합니다.
	 * @param InRequest 길찾기에 필요한 모든 정보가 담긴 요청 구조체
	 * @param InOnTaskCompleted 작업 완료 시 서브시스템에 알리기 위한 콜백
	 */
	FPathfindingAsyncTask(const FPathfindingRequest& InRequest, FSimpleDelegate InOnTaskCompleted)
		: Request(InRequest)
		, OnTaskCompleted(InOnTaskCompleted)
		, bSuccess(false)
	{
	}

	/** 백그라운드 스레드에서 실제 길찾기 계산을 수행합니다. */
	void DoWork()
	{
		// Strategy가 유효한지 확인
		if (Request.Strategy.IsValid())
		{
			// 실제 계산 로직 호출
			bSuccess = Request.Strategy->FindPathWithStrategy(Request, PathIndices);
		}

		AsyncTask(ENamedThreads::GameThread, [RequestCopy = Request, PathCopy = PathIndices, bCopy = bSuccess, Completion = OnTaskCompleted]()
		{
			if (RequestCopy.RequestingObject.IsValid())
			{
				RequestCopy.OnPathfindingComplete.ExecuteIfBound(PathCopy, bCopy);
			}
		
		});
	}

	/** 언리얼 프로파일러(Insights)에서 통계를 추적하기 위한 ID를 반환합니다. */
	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FPathfindingAsyncTask, STATGROUP_ThreadPoolAsyncTasks);
	}

	/** 복사된 길찾기 요청 데이터 */
	FPathfindingRequest Request;

	/** 작업 완료 콜백 */
	FSimpleDelegate OnTaskCompleted;

	// --- 작업 결과 ---
	TArray<FIntVector> PathIndices;
	bool bSuccess;
private:
};