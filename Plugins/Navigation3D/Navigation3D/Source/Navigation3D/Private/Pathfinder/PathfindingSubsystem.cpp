// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/PathfindingRequest.h"

// --- 1. 서브시스템 초기화 ---
// 서브시스템이 생성될 때 단 한 번 호출됩니다.
void UPathfindingSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// NextRequestID 카운터를 0으로 초기화합니다.
	NextRequestID.Set(0);
}

// --- 2. 서브시스템 소멸 ---
// 서브시스템이 파괴될 때 (예: 맵 전환, 게임 종료) 단 한 번 호출됩니다.
void UPathfindingSubsystem::Deinitialize()
{
	// Ensure all background tasks are completed and cleaned up before this subsystem is destroyed.
	// This will block the game thread until tasks are finished, which can cause a hitch
	// during level transitions, but it prevents crashes and memory leaks.
    
	UE_LOG(LogTemp, Log, TEXT("PathfindingSubsystem is being destroyed. Cleaning up %d active tasks."), ActiveTasks.Num());

	// Lock to safely access the ActiveTasks array.
	FScopeLock Lock(&TasksCriticalSection);

	for (FAsyncTask<FPathfindingAsyncTask>* Task : ActiveTasks)
	{
		if (Task)
		{
			// Wait for the task to finish its work.
			Task->EnsureCompletion(); 
			// Now that it's complete, we can safely delete it.
			delete Task;
		}
	}
	// Clear the array.
	ActiveTasks.Empty();

	Super::Deinitialize();
}

// --- 3. 길찾기 요청 접수 (Public API) ---
// PathfindingComponent에서 이 함수를 호출합니다.
int32 UPathfindingSubsystem::RequestPath(FPathfindingRequest& Request)
{
	// 요청 유효성 검사
	if (!Request.Strategy.IsValid() || !Request.RequestingObject.IsValid() || !Request.OnPathfindingComplete.IsBound())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding request is invalid. (Strategy, RequestingObject, or OnPathfindingComplete is missing)"));
		return -1;
	}

	// 고유 ID 할당
	Request.RequestID = NextRequestID.Increment();

	// 1. 새로운 비동기 작업을 생성합니다.
	//    작업 완료 시 호출될 람다를 함께 전달합니다.
	FAsyncTask<FPathfindingAsyncTask>* NewTask = new FAsyncTask<FPathfindingAsyncTask>(Request,
		FSimpleDelegate::CreateLambda([this, RequestID = Request.RequestID, WeakThis = TWeakObjectPtr<UPathfindingSubsystem>(this)]()
		{
			// 이 람다는 작업이 '완전히' 끝났을 때 게임 스레드에서 호출됩니다.
			if (!WeakThis.IsValid()) return;

			// ActiveTasks 배열에서 완료된 작업을 찾아 제거합니다.
			// 이 부분은 성능에 민감하다면 TMap으로 바꾸는 것을 고려할 수 있습니다.
			FScopeLock Lock(&TasksCriticalSection);
			for (int32 i = ActiveTasks.Num() - 1; i >= 0; --i)
			{
				// RequestID를 비교하여 정확한 작업을 찾습니다. (더 안전한 방법)
				if (ActiveTasks[i] && ActiveTasks[i]->GetTask().Request.RequestID == RequestID)
				{
					delete ActiveTasks[i];
					ActiveTasks.RemoveAt(i);
					break;
				}
			}
		})
	);

	// 2. 생성된 작업을 추적 목록에 추가합니다. (스레드 안전하게)
	{
		FScopeLock Lock(&TasksCriticalSection);
		ActiveTasks.Add(NewTask);
	}

	// 3. 엔진의 스레드 풀에서 작업을 시작시킵니다.
	NewTask->StartBackgroundTask();

	UE_LOG(LogTemp, Log, TEXT("Pathfinding request #%d dispatched."), Request.RequestID);

	return Request.RequestID;
}
