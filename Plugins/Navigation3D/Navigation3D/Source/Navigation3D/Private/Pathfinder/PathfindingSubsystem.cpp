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

	if (PathfindingThreadPool) return;

	// 1. 멤버 변수인 NumWorkerThreads 값을 직접 사용합니다.
	// 이 값은 서브시스템이 생성될 때 .ini 파일에서 자동으로 로드됩니다.
	const int32 RequestedNumThreads = this->CurrentNumWorkerThreads;

	// 2. 안전한 값으로 클램핑하는 로직은 그대로 유지합니다.
	const int32 NumPhysicalCores = FPlatformMisc::NumberOfCores();
	const int32 IdealMaxThreads = FMath::Max(1, NumPhysicalCores - 2);
	const int32 FinalNumThreads = FMath::Clamp(RequestedNumThreads, 1, IdealMaxThreads);

	if (FinalNumThreads != RequestedNumThreads)
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Configured %d worker threads, but clamping to %d."), RequestedNumThreads, FinalNumThreads);
	}

	// 3. 최종 값으로 스레드 풀을 생성합니다.
	PathfindingThreadPool = FQueuedThreadPool::Allocate();
	if (PathfindingThreadPool)
	{
		PathfindingThreadPool->Create(FinalNumThreads, 32 * 1024, TPri_Normal, TEXT("PathfindingThreadPool"));
		// 실제 적용된 값을 다시 멤버 변수에 써줍니다. (클램핑된 경우를 위해)
		this->CurrentNumWorkerThreads = FinalNumThreads;
		UE_LOG(LogTemp, Log, TEXT("Pathfinding Thread Pool created with %d threads."), this->CurrentNumWorkerThreads);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Pathfinding: Failed to allocate new thread pool."));
		this->CurrentNumWorkerThreads = 0;
	}

	NextRequestID.Set(0);
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

int32 UPathfindingSubsystem::GetNumWorkerThreads() const
{
	return CurrentNumWorkerThreads;
}

void UPathfindingSubsystem::SetNumWorkerThreads(int32 NewNumThreads)
{
	   // 1. 입력 값 유효성 검사
    if (NewNumThreads <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Requested %d worker threads, which is invalid. Must be > 0."), NewNumThreads);
        return;
    }

    // 2. 물리적 CPU 코어 수를 기반으로 이상적인 최대 스레드 수 계산
    const int32 NumPhysicalCores = FPlatformMisc::NumberOfCores();
    // 게임/렌더링 스레드를 위해 최소 2개의 코어를 남겨두되, 최소 1개의 스레드는 보장
    const int32 IdealMaxThreads = FMath::Max(1, NumPhysicalCores - 2);

    // 3. 요청된 스레드 수를 안전한 범위 [1, IdealMaxThreads] 내로 클램핑
    const int32 FinalNumThreads = FMath::Clamp(NewNumThreads, 1, IdealMaxThreads);

    // 4. 현재 스레드 수와 동일하면 변경할 필요 없음
    if (FinalNumThreads == CurrentNumWorkerThreads)
    {
        // 사용자가 요청한 값이 현재 값과 달랐지만, 클램핑 결과 같아진 경우 로그를 남길 수 있음
        if (NewNumThreads != FinalNumThreads)
        {
             UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Requested %d worker threads, but this would be clamped to %d, which is the current count. No change needed."), NewNumThreads, FinalNumThreads);
        }
        return;
    }

    // 5. 사용자가 요청한 값이 너무 높아서 클램핑되었다면 경고 로그 출력
    if (FinalNumThreads != NewNumThreads)
    {
        UE_LOG(LogTemp, Warning, TEXT("Pathfinding: Requested %d worker threads, but clamping to %d to leave cores for other tasks. (Physical Cores: %d)"),
            NewNumThreads, FinalNumThreads, NumPhysicalCores);
    }

    UE_LOG(LogTemp, Log, TEXT("PathfindingSubsystem: Attempting to change thread count from %d to %d."), CurrentNumWorkerThreads, FinalNumThreads);

    // 6. 기존 스레드 풀 파괴
    if (PathfindingThreadPool)
    {
        PathfindingThreadPool->Destroy();
        PathfindingThreadPool = nullptr;
    }

    // 7. 최종적으로 계산된 안전한 스레드 수로 새 스레드 풀 생성
    PathfindingThreadPool = FQueuedThreadPool::Allocate();
    if (PathfindingThreadPool)
    {
        PathfindingThreadPool->Create(FinalNumThreads, 32 * 1024, TPri_Normal, TEXT("PathfindingThreadPool"));
        CurrentNumWorkerThreads = FinalNumThreads; // 실제 적용된 스레드 수로 업데이트
        UE_LOG(LogTemp, Log, TEXT("PathfindingSubsystem: Thread pool restarted with %d worker threads."), CurrentNumWorkerThreads);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Pathfinding: Failed to allocate new thread pool."));
        CurrentNumWorkerThreads = 0;
    }
}

void UPathfindingSubsystem::Tick(float DeltaTime)
{
	// 1. PathfindingThreadPool 포인터가 유효한지 먼저 확인합니다.
	if (PathfindingThreadPool)
	{
		// 2. GetNumThreads() 함수를 호출하여 실제 스레드 수를 가져옵니다.
		const int32 ActualThreadCount = PathfindingThreadPool->GetNumThreads();
        
		// 3. 화면에 디버그 메시지를 출력합니다.
		if (GEngine)
		{
			const FString DebugMessage = FString::Printf(
				TEXT("Pathfinding Threads (Actual/Current): %d / %d"), 
				ActualThreadCount, 
				CurrentNumWorkerThreads
			);
			// -1 키는 같은 메시지가 계속 덮어쓰여지도록 합니다.
			GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Cyan, DebugMessage);
		}
	}
	else
	{
		// 스레드 풀이 없는 경우
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Red, TEXT("Pathfinding Threads: POOL NOT INITIALIZED"));
		}
	}
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