// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Subsystem/PathfindingSubsystem.h"
#include "Strategy/PathfindingStrategy.h"
#include "Utility/Parallel/MemoryPool.h"
#include "Utility/AStarStrategyUtils.h"
#include "Component/PathfindingComponent.h"
#include "HAL/PlatformProcess.h"
#include "HAL/ThreadManager.h"
#include "NavData/WorldNavigationDataAsset.h"
#include "Utility/PathfindingStreamingProbe.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Misc/Paths.h"

#include "WorldPartition/WorldPartitionSubsystem.h"

FORCEINLINE uint32 GetTypeHash(const TSoftObjectPtr<UOctreeDataAsset>& Ptr)
{
	return GetTypeHash(Ptr.ToSoftObjectPath());
}

static void ReconstructDataAssetPath(
	TMap<TSoftObjectPtr<UOctreeDataAsset>, TPair<TSoftObjectPtr<UOctreeDataAsset>, int32>> CameFrom,
	TSoftObjectPtr<UOctreeDataAsset> CurrentDataAsset,
	TArray<TPair<TSoftObjectPtr<UOctreeDataAsset>, int32>>& OutDataAssetPath)
{
	TArray<TPair<TSoftObjectPtr<UOctreeDataAsset>, int32>> TotalPath;
	TotalPath.Add({ CurrentDataAsset, -1 });

	while (CameFrom.Contains(CurrentDataAsset))
	{
		TPair<TSoftObjectPtr<UOctreeDataAsset>, int32> ParentInfo = CameFrom[CurrentDataAsset];
		CurrentDataAsset = ParentInfo.Key;
		if (CurrentDataAsset.IsNull()) break;
		TotalPath.Add(ParentInfo);
	}
	Algo::Reverse(TotalPath);

	OutDataAssetPath = TotalPath;
}

void FPathfindingAsyncTasks::DoThreadedWork()
{
	if (!OwnerSubsystem.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding Task [%d] is aborting because the subsystem is no longer valid."), Request.RequestID);

		if (Request.MemoryPool)
		{
			delete Request.MemoryPool;
			Request.MemoryPool = nullptr;
		}
		
		delete this;
		return;
	}

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

		if (Request.MemoryPool)
		{
			OwnerSubsystem->ReturnPool(Request.MemoryPool);
		}
	}
	delete this;
}

//~=============================================================================
// Subsystem Lifecycle
//~=============================================================================

void UPathfindingSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	UWorld* World = GetWorld();
	UE_LOG(LogTemp, Log, TEXT("PathfindingSubsystem::Initialize for world '%s'."), *World->GetMapName());

	if (PathfindingThreadPool)
	{
		PathfindingThreadPool->Destroy();
		PathfindingThreadPool = nullptr;
	}

	const int32 RequestedNumThreads = this->CurrentNumWorkerThreads;
	const int32 NumPhysicalCores = FPlatformMisc::NumberOfCores();
	const int32 IdealMaxThreads = FMath::Max(1, NumPhysicalCores - 2); 
	const int32 FinalNumThreads = FMath::Clamp(RequestedNumThreads, 1, IdealMaxThreads);

	PathfindingThreadPool = FQueuedThreadPool::Allocate();
	if (PathfindingThreadPool)
	{
		if (PathfindingThreadPool->Create(FinalNumThreads, 32 * 1024, TPri_Normal, TEXT("PathfindingThreadPool")))
		{
			this->CurrentNumWorkerThreads = FinalNumThreads;
			UE_LOG(LogTemp, Log, TEXT("PathfindingThreadPool created with %d threads."), FinalNumThreads);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create PathfindingThreadPool."));
			PathfindingThreadPool->Destroy();
			PathfindingThreadPool = nullptr;
			this->CurrentNumWorkerThreads = 0;
		}
	}
	
#if WITH_EDITOR
	const FString MapName = UWorld::RemovePIEPrefix(World->GetMapName());
	// FString MapName = GetWorld()->GetMapName();
	// MapName.RemoveFromStart(GetWorld()->StreamingLevelsPrefix);
	const FString ManifestAssetName = TEXT("DA_WorldNav_") + MapName;
	const FString ManifestAssetPath = FPaths::Combine(TEXT("/Game/Data/Navigation/"), ManifestAssetName);

	CurrentWorldNavData = LoadObject<UWorldNavigationDataAsset>(nullptr, *ManifestAssetPath);

	if (CurrentWorldNavData)
	{
		UE_LOG(LogTemp, Log, TEXT("World Navigation Data Manifest '%s' loaded successfully."), *ManifestAssetPath);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("World Navigation Data Manifest '%s' could not be found. Long-range hierarchical pathfinding may be disabled."), *ManifestAssetPath);
	}
#endif
	
	NextRequestID.Set(0);
}

void UPathfindingSubsystem::Deinitialize()
{
	UE_LOG(LogTemp, Log, TEXT("PathfindingSubsystem::Deinitialize for world '%s'."), *GetWorld()->GetMapName());

	for (auto& ProbePair : ActiveStreamingProbes)
	{
		if (IsValid(ProbePair.Value))
		{
			ProbePair.Value->Destroy();
		}
	}
	ActiveStreamingProbes.Empty();

	if (PathfindingThreadPool)
	{
		UE_LOG(LogTemp, Log, TEXT("Destroying PathfindingThreadPool."));
		
		PathfindingThreadPool->Destroy();
		PathfindingThreadPool = nullptr;
	}

	CompletedTasksQueue.Empty();

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


int32 UPathfindingSubsystem::RequestPath(const UObject* WorldContextObject, FPathfindingRequest& Request)
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

// int32 UPathfindingSubsystem::RequestHierarchicalPath(const UObject* WorldContextObject, FHierarchicalPathRequest& Request)
// {
// 	if (!PathfindingThreadPool || !Request.RequestingComponent.IsValid() || !Request.Strategy.IsValid())
// 	{
// 		return -1;
// 	}
//
// 	Request.RequestID = NextRequestID.Increment();
// 	
// 	FHierarchicalPathfindingTask NewTask(Request);
// 	
// 	if (!FindHighLevelPath(WorldContextObject, Request, NewTask.HighLevelDataAssetPath))
// 	{
// 		UE_LOG(LogTemp, Warning, TEXT("Hierarchical Pathfinding Failed for RequestID %d: Could not find a high-level path of data assets."), Request.RequestID);
// 		Request.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
// 		return -1;
// 	}
// 	
// 	NewTask.State = EHierarchicalTaskState::ReadyToFindLocalPath;
// 	ActiveHierarchicalTasks.Add(Request.RequestID, NewTask);
// 	
// 	ProcessNextHierarchicalSegment(WorldContextObject, ActiveHierarchicalTasks.FindChecked(Request.RequestID));
//
// 	return Request.RequestID;
// }

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
		bool bIsHierarchicalSegment = false;
		for (auto& Elem : ActiveHierarchicalTasks)
		{
			if (Elem.Value.LastLocalRequestID == Result.Request.RequestID)
			{
				//OnHierarchicalSegmentCompleted(Elem.Key, Result.WorldPath, Result.bSuccess);
				bIsHierarchicalSegment = true;
				break;
			}
		}
		
		if (!bIsHierarchicalSegment)
		{
			OnPathfindingCompleted(Result.Request, Result.WorldPath, Result.bSuccess);
		}

		// Use this until develop hierachy path finding
		// OnPathfindingCompleted(Result.Request, Result.WorldPath, Result.bSuccess);
	}

	UOctreeNavSubsystem* NavSubsystem = GetWorld()->GetSubsystem<UOctreeNavSubsystem>();
	if (!NavSubsystem) return;

	for (auto& TaskPair : ActiveHierarchicalTasks)
	{
		FHierarchicalPathfindingTask& Task = TaskPair.Value;

		if (Task.State == EHierarchicalTaskState::WaitingForVolumeToLoad)
		{
			const TSoftObjectPtr<UOctreeDataAsset> WaitingForDataAsset = Task.HighLevelDataAssetPath[Task.CurrentSegmentIndex].Key;

			AOctreeVoxelVolume* LoadedVolume = NavSubsystem->GetVolumeByDataAsset(WaitingForDataAsset);
			if (LoadedVolume && LoadedVolume->IsReadyForNavigation())
			{
				UE_LOG(LogTemp, Log, TEXT("Segment %d for RequestID %d: Volume with data '%s' has been successfully loaded! Resuming task."), Task.CurrentSegmentIndex, Task.OriginalRequest.RequestID, *WaitingForDataAsset.ToString());
				Task.State = EHierarchicalTaskState::ReadyToFindLocalPath;

				if (UPathfindingComponent* RequestingComp = Task.OriginalRequest.RequestingComponent.Get())
				{
					//ProcessNextHierarchicalSegment(RequestingComp, Task);
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("Hierarchical path failed for RequestID %d: Requesting component is no longer valid."), Task.OriginalRequest.RequestID);
					Task.State = EHierarchicalTaskState::Failed;
					//DestroyStreamingProbeForRequest(nullptr, Task.OriginalRequest.RequestID);
					Task.OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
					ActiveHierarchicalTasks.Remove(Task.OriginalRequest.RequestID);
				}
			}
		}
	}
}

TStatId UPathfindingSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UPathfindingSubsystem, STATGROUP_Tickables);
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
		//TasksToProcessThisFrame에서 remove해주기
	}
}
