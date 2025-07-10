// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"

void FPathfindingAsyncTasks::DoThreadedWork()
{
	// 실제 길찾기 로직 (백그라운드 스레드에서 실행)
	TArray<FIntVector> PathIndices;
	bool bSuccess = false;

	if (Request.Strategy.IsValid())
	{
		bSuccess = Request.Strategy->FindPathWithStrategy(Request, PathIndices);
	}

	// 결과 처리를 게임 스레드로 넘김
	if (OwnerSubsystem.IsValid())
	{
		// AsyncTask를 사용해 게임 스레드에서 OnPathfindingCompleted를 호출하도록 예약
		AsyncTask(ENamedThreads::GameThread, [Owner = OwnerSubsystem, Req = Request, Path = PathIndices, bOk = bSuccess]()
		{
			if (Owner.IsValid())
			{
				Owner->OnPathfindingCompleted(Req, Path, bOk);
			}
		});
	}
}

void FPathfindingAsyncTasks::Abandon()
{
	// 스레드 풀이 파괴될 때 아직 처리되지 않은 작업은 이 함수가 호출됨
	// 필요하다면 요청자에게 실패를 알릴 수 있음
	if (OwnerSubsystem.IsValid())
	{
		AsyncTask(ENamedThreads::GameThread, [Owner = OwnerSubsystem, Req = Request]()
		{
			if (Owner.IsValid())
			{
				// 빈 경로와 실패 플래그로 완료 콜백 호출
				Owner->OnPathfindingCompleted(Req, {}, false);
			}
		});
	}
}


// --- 1. Subsystem Initialization ---
// This is called once when the subsystem is created.
void UPathfindingSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// 1. Get the number of physical CPU cores on this machine (e.g., 8 for an 8-core CPU).
	const int32 NumPhysicalCores = FPlatformMisc::NumberOfCores();

	// 2. Calculate the 'ideal maximum' number of threads to allocate for pathfinding.
	//    - We reserve at least 2 cores for other critical tasks like the game's main thread and rendering thread.
	//    - However, we ensure at least 1 thread is always available, even on CPUs with 1 or 2 cores.
	const int32 IdealMaxThreads = FMath::Max(1, NumPhysicalCores - 2);

	// 3. Take the user-configured 'NumWorkerThreads' (from .ini files, etc.) and clamp it
	//    to the ideal maximum calculated above.
	//    - This forces the value to be within the range of [1, IdealMaxThreads].
	const int32 FinalNumThreads = FMath::Clamp(NumWorkerThreads, 1, IdealMaxThreads);

	// 4. If the user's requested value was too high and had to be clamped, log a warning for the developer.
	if (FinalNumThreads != NumWorkerThreads)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Requested %d worker threads, but clamping to %d to leave cores for other tasks. (Physical Cores: %d)"),
			NumWorkerThreads, FinalNumThreads, NumPhysicalCores);
	}

	// 5. Create the thread pool using the final, safe number of threads.
	PathfindingThreadPool = FQueuedThreadPool::Allocate();
	if (PathfindingThreadPool)
	{
		PathfindingThreadPool->Create(FinalNumThreads, 32 * 1024, TPri_Normal, TEXT("PathfindingThreadPool"));
		UE_LOG(LogTemp, Log, TEXT("Pathfinding Thread Pool created with %d threads."), FinalNumThreads);
	}
}

// --- 2. 서브시스템 소멸 ---
// 서브시스템이 파괴될 때 (예: 맵 전환, 게임 종료) 단 한 번 호출됩니다.
void UPathfindingSubsystem::Deinitialize()
{
	if (PathfindingThreadPool)
	{
		PathfindingThreadPool->Destroy();
		PathfindingThreadPool = nullptr;
		UE_LOG(LogTemp, Log, TEXT("Pathfinding Thread Pool destroyed."));
	}

	Super::Deinitialize();
}

// --- 3. 길찾기 요청 접수 (Public API) ---
// PathfindingComponent에서 이 함수를 호출합니다.
int32 UPathfindingSubsystem::RequestPath(FPathfindingRequest& Request)
{
	if (!PathfindingThreadPool)
	{
		UE_LOG(LogTemp, Error, TEXT("Pathfinding thread pool is not initialized."));
		return -1;
	}

	if (!Request.Strategy.IsValid() || !Request.RequestingObject.IsValid() || !Request.OnPathfindingComplete.IsBound())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding request is invalid."));
		return -1;
	}

	Request.RequestID = NextRequestID.Increment();

	// IQueuedWork를 구현한 작업 객체를 생성하여 스레드 풀에 추가
	PathfindingThreadPool->AddQueuedWork(new FPathfindingAsyncTasks(Request, this));

	UE_LOG(LogTemp, Log, TEXT("Pathfinding request #%d queued to dedicated pool."), Request.RequestID);
	return Request.RequestID;
}

void UPathfindingSubsystem::OnPathfindingCompleted(FPathfindingRequest Result, TArray<FIntVector> Path, bool bSuccess)
{
	// 이 함수는 항상 게임 스레드에서 실행됨
	if (Result.RequestingObject.IsValid())
	{
		Result.OnPathfindingComplete.ExecuteIfBound(Path, bSuccess);
	}
	UE_LOG(LogTemp, Log, TEXT("Pathfinding request #%d completed."), Result.RequestID);
}