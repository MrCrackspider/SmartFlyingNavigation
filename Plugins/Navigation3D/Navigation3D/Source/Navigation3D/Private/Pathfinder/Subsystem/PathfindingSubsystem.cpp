// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/Subsystem/PathfindingSubsystem.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "Pathfinder/Utility/StrategyUtils.h"
#include "Pathfinder/Utility/Parallel/MemoryPool.h"
#include "Pathfinder/Utility/AStarStrategyUtils.h"

void FPathfindingAsyncTasks::DoThreadedWork()
{
	// 1. 결과 구조체 생성
	FPathfindingResult Result;
	Result.Request = Request; // 요청서 복사
	Result.bSuccess = false;

	if (Request.Strategy.IsValid())
	{
		// 2. 요청 타입에 따라 올바른 전략 함수를 호출하고,
		//    그에 맞는 경로 변수에 결과를 저장합니다.
		if (Request.NodeType == EPathfindingNodeType::Grid)
		{
			// VoxelPath가 채워지고, WorldPath는 비어있게 됩니다.
			Result.bSuccess = Request.Strategy->FindPathWithStrategy(Request, Result.VoxelPath);
		}
		else if (Request.NodeType == EPathfindingNodeType::Octree)
		{
			// WorldPath가 채워지고, VoxelPath는 비어있게 됩니다.
			Result.bSuccess = Request.Strategy->FindPathWithStrategy(Request, Result.WorldPath);
		}
	}

	// 3. 결과 처리를 위해 서브시스템의 큐에 추가합니다.
	if (OwnerSubsystem.IsValid())
	{
		OwnerSubsystem->QueueCompletedResult(Result);
	}

	// 4. 풀 반납 및 자기 파괴
	if (OwnerSubsystem.IsValid() && Request.MemoryPool)
	{
		OwnerSubsystem->ReturnPool(Request.MemoryPool);
	}
    
	delete this;
}

void FPathfindingAsyncTasks::Abandon()
{
	// // 스레드 풀이 파괴될 때 아직 처리되지 않은 작업은 이 함수가 호출됨
	// // 필요하다면 요청자에게 실패를 알릴 수 있음
	// if (OwnerSubsystem.IsValid())
	// {
	// 	AsyncTask(ENamedThreads::GameThread, [Owner = OwnerSubsystem, Req = Request]()
	// 	{
	// 		if (Owner.IsValid())
	// 		{
	// 			// 빈 경로와 실패 플래그로 완료 콜백 호출
	// 			Owner->OnPathfindingCompleted(Req, {}, false);
	// 		}
	// 	});
	// }

	if (OwnerSubsystem.IsValid())
	{
		FPathfindingResult Result;
		Result.Request = Request;
		Result.bSuccess = false; // 실패로 처리
		OwnerSubsystem->QueueCompletedResult(Result);
	}

	if (OwnerSubsystem.IsValid() && Request.MemoryPool)
	{
		OwnerSubsystem->ReturnPool(Request.MemoryPool);
	}
	
	delete this;
}


FPathNodeMemoryPoolBase* UPathfindingSubsystem::LeasePool(EPathfindingNodeType NodeType)
{
	FScopeLock Lock(&PoolLock);
    
	// 요청된 타입에 맞는 유휴 풀 목록을 선택합니다.
	TArray<FPathNodeMemoryPoolBase*>& TargetIdlePools = (NodeType == EPathfindingNodeType::Grid) ? IdleGridPools : IdleOctreePools;
	
	if (TargetIdlePools.Num() > 0)
	{
		return TargetIdlePools.Pop();
	}

	switch (NodeType)
	{
	case EPathfindingNodeType::Grid:
		return new TNodeMemoryPool<FPathNode>(1024);
	case EPathfindingNodeType::Octree:
		return new TNodeMemoryPool<FAstarPathNode>(1024);
	default:
		return nullptr;
	}
}
void UPathfindingSubsystem::ReturnPool(FPathNodeMemoryPoolBase* PoolToReturn)
{
	if (!PoolToReturn) return;
	FScopeLock Lock(&PoolLock);
	PoolToReturn->Reset();
    
	// dynamic_cast도 TNodeMemoryPool 이름을 사용합니다.
	if (PoolToReturn->GetNodeType() == EPathfindingNodeType::Grid)
	{
		IdleGridPools.Add(PoolToReturn);
	}
	else if (PoolToReturn->GetNodeType() == EPathfindingNodeType::Octree)
	{
		IdleOctreePools.Add(PoolToReturn);
	}
	else
	{
		delete PoolToReturn;
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
void UPathfindingSubsystem::Deinitialize()
{
	UE_LOG(LogTemp, Log, TEXT("PathfindingSubsystem: Deinitializing..."));
	
	// 1. 스레드 풀을 먼저 안전하게 파괴합니다.
	if (PathfindingThreadPool)
	{
		PathfindingThreadPool->Destroy();
		PathfindingThreadPool = nullptr;
		UE_LOG(LogTemp, Log, TEXT("Pathfinding Thread Pool destroyed."));
	}
	
	// 2. 락을 걸고 모든 유휴 메모리 풀을 정리합니다.
	{
		FScopeLock Lock(&PoolLock);

		UE_LOG(LogTemp, Log, TEXT("Cleaning up %d idle Grid pools and %d idle Octree pools..."), IdleGridPools.Num(), IdleOctreePools.Num());

		// Grid 풀 정리
		for (FPathNodeMemoryPoolBase* Pool : IdleGridPools)
		{
			delete Pool;
		}
		IdleGridPools.Empty();

		// Octree 풀 정리
		for (FPathNodeMemoryPoolBase* Pool : IdleOctreePools)
		{
			delete Pool;
		}
		IdleOctreePools.Empty();
		
		UE_LOG(LogTemp, Log, TEXT("All idle pools cleaned up."));
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
	// 디버그 메시지
	if (GEngine && PathfindingThreadPool)
	{
		// const FString DebugMessage = FString::Printf(TEXT("Pathfinding Threads: %d"), CurrentNumWorkerThreads);
		// GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Cyan, DebugMessage);
	}

	// 처리할 작업들을 담을 임시 배열
	TArray<FPathfindingResult> TasksToProcessThisFrame;
    
	// 큐에서 N개만큼만 꺼내옵니다.
	{
		FScopeLock Lock(&QueueLock);
		for (int32 i = 0; i < MaxCallbacksPerFrame && !CompletedTasksQueue.IsEmpty(); ++i)
		{
			FPathfindingResult TempResult;
			if (CompletedTasksQueue.Dequeue(TempResult))
			{
				TasksToProcessThisFrame.Add(TempResult);
			}
		}
	}

	for (const FPathfindingResult& Result : TasksToProcessThisFrame)
	{
		if (Result.Request.NodeType == EPathfindingNodeType::Grid)
		{
			OnPathfindingCompleted(Result.Request, Result.VoxelPath, Result.bSuccess);
		}
		else if (Result.Request.NodeType == EPathfindingNodeType::Octree)
		{
			OnPathfindingCompleted(Result.Request, Result.WorldPath, Result.bSuccess);
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

	if (!Request.Strategy.IsValid() || !Request.RequestingObject.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding request is invalid."));
		return -1;
	}

	Request.RequestID = NextRequestID.Increment();
	Request.MemoryPool = LeasePool(Request.NodeType); 
	if (!Request.MemoryPool) return -1;
	
	// IQueuedWork를 구현한 작업 객체를 생성하여 스레드 풀에 추가
	PathfindingThreadPool->AddQueuedWork(new FPathfindingAsyncTasks(Request, this));

	UE_LOG(LogTemp, Log, TEXT("Pathfinding request #%d queued to dedicated pool."), Request.RequestID);
	return Request.RequestID;
}

void UPathfindingSubsystem::OnPathfindingCompleted(const FPathfindingRequest& Request, const TArray<FIntVector>& VoxelPath, bool bSuccess)
{
	if (Request.RequestingObject.IsValid())
	{
		// OnVoxelPathComplete 델리게이트를 호출합니다.
		Request.OnVoxelPathComplete.ExecuteIfBound(VoxelPath, bSuccess);
	}
	UE_LOG(LogTemp, Log, TEXT("Pathfinding request #%d (Grid) completed."), Request.RequestID);
}

void UPathfindingSubsystem::OnPathfindingCompleted(const FPathfindingRequest& Request, const TArray<FVector>& WorldPath, bool bSuccess)
{ if (Request.RequestingObject.IsValid())
{
	// OnWorldPathComplete 델리게이트를 호출합니다.
	Request.OnWorldPathComplete.ExecuteIfBound(WorldPath, bSuccess);
}
	UE_LOG(LogTemp, Log, TEXT("Pathfinding request #%d (Octree) completed."), Request.RequestID);
}

void UPathfindingSubsystem::QueueCompletedResult(const FPathfindingResult& Result)
{
	FScopeLock Lock(&QueueLock);
	CompletedTasksQueue.Enqueue(Result);
}
