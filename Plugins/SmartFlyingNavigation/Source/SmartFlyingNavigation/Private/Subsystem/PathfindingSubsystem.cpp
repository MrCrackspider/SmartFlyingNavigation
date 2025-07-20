// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Subsystem/PathfindingSubsystem.h"
#include "Strategy/PathfindingStrategy.h"
#include "Utility/Parallel/MemoryPool.h"
#include "Utility/AStarStrategyUtils.h"


void FPathfindingAsyncTasks::DoThreadedWork()
{
	FPathfindingResult Result;
	Result.Request = Request;
	Result.bSuccess = false;

	if (Request.Strategy.IsValid())
	{
		if (Request.NodeType == EPathfindingNodeType::Octree)
		{
			Result.bSuccess = Request.Strategy->FindPathWithStrategy(Request, Result.WorldPath);
		}
	}

	if (OwnerSubsystem.IsValid())
	{
		OwnerSubsystem->QueueCompletedResult(Result);
	}

	if (OwnerSubsystem.IsValid() && Request.MemoryPool)
	{
		OwnerSubsystem->ReturnPool(Request.MemoryPool);
	}

	delete this;
}

void FPathfindingAsyncTasks::Abandon()
{
	if (OwnerSubsystem.IsValid())
	{
		FPathfindingResult Result;
		Result.Request = Request;
		Result.bSuccess = false;
		OwnerSubsystem->QueueCompletedResult(Result);
	}

	if (OwnerSubsystem.IsValid() && Request.MemoryPool)
	{
		OwnerSubsystem->ReturnPool(Request.MemoryPool);
	}

	delete this;
}

//~=============================================================================
// Subsystem Lifecycle
//~=============================================================================

void UPathfindingSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	if (PathfindingThreadPool)
	{
		return;
	}
	
	const int32 RequestedNumThreads = this->CurrentNumWorkerThreads;
	const int32 NumPhysicalCores = FPlatformMisc::NumberOfCores();
	const int32 IdealMaxThreads = FMath::Max(1, NumPhysicalCores - 2); // Leave cores for game and render threads
	const int32 FinalNumThreads = FMath::Clamp(RequestedNumThreads, 1, IdealMaxThreads);
	

	PathfindingThreadPool = FQueuedThreadPool::Allocate();
	if (PathfindingThreadPool)
	{
		PathfindingThreadPool->Create(FinalNumThreads, 32 * 1024, TPri_Normal, TEXT("PathfindingThreadPool"));
		this->CurrentNumWorkerThreads = FinalNumThreads;
	}
	else
	{
		if(PathfindingThreadPool)
		{
			PathfindingThreadPool->Destroy();
			PathfindingThreadPool = nullptr;
		}
		this->CurrentNumWorkerThreads = 0;
	}

	NextRequestID.Set(0);
}


void UPathfindingSubsystem::Deinitialize()
{
	if (PathfindingThreadPool)
	{
		PathfindingThreadPool->Destroy();
		PathfindingThreadPool = nullptr;
	}

	{
		FScopeLock Lock(&PoolLock);

		for (FPathNodeMemoryPoolBase* Pool : IdleGridPools)
		{
			delete Pool;
		}
		IdleGridPools.Empty();

		for (FPathNodeMemoryPoolBase* Pool : IdleOctreePools)
		{
			delete Pool;
		}
		IdleOctreePools.Empty();
	}

	Super::Deinitialize();
}

//~=============================================================================
// Public API
//~=============================================================================


int32 UPathfindingSubsystem::RequestPath(FPathfindingRequest& Request)
{
	if (!PathfindingThreadPool)
	{
		return -1;
	}

	if (!Request.Strategy.IsValid() || !Request.RequestingObject.IsValid())
	{
		return -1;
	}

	// This subsystem now only handles Octree pathfinding.
	if (Request.NodeType != EPathfindingNodeType::Octree)
	{
		return -1;
	}
	
	Request.RequestID = NextRequestID.Increment();
	Request.MemoryPool = LeasePool(Request.NodeType);
	if (!Request.MemoryPool)
	{
		return -1;
	}

	// Create a worker task and add it to the thread pool.
	PathfindingThreadPool->AddQueuedWork(new FPathfindingAsyncTasks(Request, this));
	return Request.RequestID;

}


void UPathfindingSubsystem::Tick(float DeltaTime)
{
	TArray<FPathfindingResult> TasksToProcessThisFrame;
	TasksToProcessThisFrame.Reserve(MaxCallbacksPerFrame);
	
	for (int32 i = 0; i < MaxCallbacksPerFrame; ++i)
	{
		FPathfindingResult TempResult;
		if (CompletedTasksQueue.Dequeue(TempResult))
		{
			TasksToProcessThisFrame.Add(MoveTemp(TempResult));
		}
		else
		{
			break;
		}
	}
	

	for (const FPathfindingResult& Result : TasksToProcessThisFrame)
	{
		OnPathfindingCompleted(Result.Request, Result.WorldPath, Result.bSuccess);
	}
}


//~=============================================================================
// Debug & Configuration
//~=============================================================================

int32 UPathfindingSubsystem::GetNumWorkerThreads() const
{
	return CurrentNumWorkerThreads;
}

void UPathfindingSubsystem::SetNumWorkerThreads(int32 NewNumThreads)
{
	if (NewNumThreads <= 0)
	{
		return;
	}

	const int32 NumPhysicalCores = FPlatformMisc::NumberOfCores();
	const int32 IdealMaxThreads = FMath::Max(1, NumPhysicalCores - 2);
	const int32 FinalNumThreads = FMath::Clamp(NewNumThreads, 1, IdealMaxThreads);

	if (FinalNumThreads == CurrentNumWorkerThreads)
	{
		return;
	}
	
	if (PathfindingThreadPool)
	{
		PathfindingThreadPool->Destroy();
		PathfindingThreadPool = nullptr;
	}

	PathfindingThreadPool = FQueuedThreadPool::Allocate();
	PathfindingThreadPool = FQueuedThreadPool::Allocate();
	if (PathfindingThreadPool && PathfindingThreadPool->Create(FinalNumThreads, 32 * 1024, TPri_Normal, TEXT("PathfindingThreadPool")))
	{
		CurrentNumWorkerThreads = FinalNumThreads;
	}
	else
	{
		if(PathfindingThreadPool)
		{
			PathfindingThreadPool->Destroy();
			PathfindingThreadPool = nullptr;
		}
		CurrentNumWorkerThreads = 0;
	}
}

//~=============================================================================
// Internal Management
//~=============================================================================

void UPathfindingSubsystem::QueueCompletedResult(const FPathfindingResult& Result)
{
	CompletedTasksQueue.Enqueue(Result);
}

FPathNodeMemoryPoolBase* UPathfindingSubsystem::LeasePool(EPathfindingNodeType NodeType)
{
	FScopeLock Lock(&PoolLock);

	// Note: This logic supports both Grid and Octree pools, though the rest of the
	// subsystem is currently only using Octree. This allows for future expansion.

	TArray<FPathNodeMemoryPoolBase*>& TargetIdlePools = (NodeType == EPathfindingNodeType::Grid) ? IdleGridPools : IdleOctreePools;
	
	if (TargetIdlePools.Num() > 0)
	{
		return TargetIdlePools.Pop();
	}

	// If no pools are available, create a new one.
	switch (NodeType)
	{
	case EPathfindingNodeType::Octree:
		return new TNodeMemoryPool<FAstarPathNode>(1024);
	default:
		return nullptr;
	}
}


void UPathfindingSubsystem::ReturnPool(FPathNodeMemoryPoolBase* PoolToReturn)
{
	if (!PoolToReturn)
	{
		return;
	}

	FScopeLock Lock(&PoolLock);
	PoolToReturn->Reset();

	// This logic currently re-uses Octree pools but deletes Grid pools.
	// This might be intentional. If Grid pools should also be reused,
	// they should be added to the IdleGridPools array here.
	
	if (PoolToReturn->GetNodeType() == EPathfindingNodeType::Octree)
	{
		IdleOctreePools.Add(PoolToReturn);
	}
	else
	{
		delete PoolToReturn;
	}
}

//~=============================================================================
// Path Completion Callbacks
//~=============================================================================

void UPathfindingSubsystem::OnPathfindingCompleted(const FPathfindingRequest& Request, const TArray<FVector>& WorldPath,
                                                   bool bSuccess)
{
	if (Request.RequestingObject.IsValid())
	{
		Request.OnWorldPathComplete.ExecuteIfBound(WorldPath, bSuccess);
	}
}

