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
	// 서브시스템이 Deinitialize되면 IsValid()가 false를 반환하여 스레드가 안전하게 중단됩니다.
	if (!OwnerSubsystem.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("Pathfinding Task [%d] is aborting because the subsystem is no longer valid."), Request.RequestID);
		// 메모리 풀 반환 로직은 Abandon()에서 처리되므로 여기서 직접 호출할 필요가 없을 수 있지만,
		// 안전을 위해 유지하거나 Abandon()으로 통일할 수 있습니다.
		if (Request.MemoryPool)
		{
			OwnerSubsystem->ReturnPool(Request.MemoryPool);
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

	const FString OriginalMapName = UWorld::RemovePIEPrefix(World->GetMapName());
	const FString ManifestAssetName = TEXT("DA_WorldNav_") + OriginalMapName;
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

int32 UPathfindingSubsystem::RequestHierarchicalPath(const UObject* WorldContextObject, FHierarchicalPathRequest& Request)
{
	if (!PathfindingThreadPool || !Request.RequestingComponent.IsValid() || !Request.Strategy.IsValid())
	{
		return -1;
	}

	Request.RequestID = NextRequestID.Increment();

	// 태스크 객체를 생성
	FHierarchicalPathfindingTask NewTask(Request);

	// 데이터 기반의 상위 레벨 경로를 탐색
	if (!FindHighLevelPath(WorldContextObject, Request, NewTask.HighLevelDataAssetPath))
	{
		UE_LOG(LogTemp, Warning, TEXT("Hierarchical Pathfinding Failed for RequestID %d: Could not find a high-level path of data assets."), Request.RequestID);
		Request.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
		return -1;
	}

	// 태스크를 활성 목록에 추가하고, 첫 번째 세그먼트 처리를 위해 상태를 변경
	// 첫 번째 볼륨은 Agent가 이미 그 안에 있으므로 항상 로드되어 있다고 가정
	NewTask.State = EHierarchicalTaskState::ReadyToFindLocalPath;
	ActiveHierarchicalTasks.Add(Request.RequestID, NewTask);

	// 4. 상태 머신의 첫 번째 턴을 시작합니다.
	ProcessNextHierarchicalSegment(WorldContextObject, ActiveHierarchicalTasks.FindChecked(Request.RequestID));

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
		bool bIsHierarchicalSegment = false;
		for (auto& Elem : ActiveHierarchicalTasks)
		{
			if (Elem.Value.LastLocalRequestID == Result.Request.RequestID)
			{
				OnHierarchicalSegmentCompleted(Elem.Key, Result.WorldPath, Result.bSuccess);
				bIsHierarchicalSegment = true;
				break;
			}
		}

		if (!bIsHierarchicalSegment)
		{
			OnPathfindingCompleted(Result.Request, Result.WorldPath, Result.bSuccess);
		}
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
					ProcessNextHierarchicalSegment(RequestingComp, Task);
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("Hierarchical path failed for RequestID %d: Requesting component is no longer valid."), Task.OriginalRequest.RequestID);
					Task.State = EHierarchicalTaskState::Failed;
					DestroyStreamingProbeForRequest(nullptr, Task.OriginalRequest.RequestID);
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
	}
}

void UPathfindingSubsystem::OnHierarchicalSegmentCompleted(int32 HierarchicalRequestID, const TArray<FVector>& SegmentPath, bool bSuccess)
{
	FHierarchicalPathfindingTask* Task = ActiveHierarchicalTasks.Find(HierarchicalRequestID);
	if (!Task) return;

	const UObject* WorldContextObject = Task->OriginalRequest.RequestingComponent.Get();
	if (!bSuccess)
	{
		UE_LOG(LogTemp, Error, TEXT("Hierarchical Pathfinding Failed for RequestID %d: Segment %d failed."), HierarchicalRequestID, Task->CurrentSegmentIndex);
		Task->OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound({}, false);

		DestroyStreamingProbeForRequest(WorldContextObject, HierarchicalRequestID);
		ActiveHierarchicalTasks.Remove(HierarchicalRequestID);
		return;
	}

	if (Task->CombinedWorldPath.Num() > 0 && SegmentPath.Num() > 0)
	{
		Task->CombinedWorldPath.Append(SegmentPath.GetData() + 1, SegmentPath.Num() - 1);
	}
	else
	{
		Task->CombinedWorldPath.Append(SegmentPath);
	}

	Task->CurrentSegmentIndex++;

	if (Task->CurrentSegmentIndex >= Task->HighLevelDataAssetPath.Num())
	{
		Task->State = EHierarchicalTaskState::Complete;
		Task->OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound(Task->CombinedWorldPath, true);

		DestroyStreamingProbeForRequest(WorldContextObject, HierarchicalRequestID);
		ActiveHierarchicalTasks.Remove(HierarchicalRequestID);
	}
	else
	{
		Task->State = EHierarchicalTaskState::ReadyToFindLocalPath;
		ProcessNextHierarchicalSegment(WorldContextObject, *Task);
	}
}

void UPathfindingSubsystem::CreateStreamingProbeForRequest(const UObject* WorldContextObject, int32 RequestID, const FVector& Location)
{
	DestroyStreamingProbeForRequest(WorldContextObject, RequestID);

	UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	if (World)
	{
		APathfindingStreamingProbe* NewProbe = World->SpawnActor<APathfindingStreamingProbe>();
		if (NewProbe)
		{
			NewProbe->ActivateProbe(Location);
			ActiveStreamingProbes.Add(RequestID, NewProbe);
		}
	}
}

void UPathfindingSubsystem::DestroyStreamingProbeForRequest(const UObject* WorldContextObject, int32 RequestID)
{
	if (TObjectPtr<APathfindingStreamingProbe>* FoundProbe = ActiveStreamingProbes.Find(RequestID))
	{
		if (IsValid(*FoundProbe))
		{
			(*FoundProbe)->Destroy();
		}
		ActiveStreamingProbes.Remove(RequestID);
	}
}

//~=============================================================================
// Hierarchical Pathfinding Internals
//~=============================================================================
bool UPathfindingSubsystem::FindHighLevelPath(const UObject* WorldContextObject, const FHierarchicalPathRequest& Request, TArray<TPair<TSoftObjectPtr<UOctreeDataAsset>, int32>>& OutDataAssetPath)
{
	if (!CurrentWorldNavData)
	{
		UE_LOG(LogTemp, Error, TEXT("FindHighLevelPath failed: World Navigation Data Manifest (CurrentWorldNavData) is not loaded."));
		return false;
	}

	TSoftObjectPtr<UOctreeDataAsset> StartDataAsset, EndDataAsset;
	FBox StartVolumeBounds, EndVolumeBounds;

	for (const FVolumeSpatialInfo& Info : CurrentWorldNavData->VolumeInfos)
	{
		if (Info.Bounds.IsInside(Request.StartLocation))
		{
			StartDataAsset = Info.NavDataAsset;
			StartVolumeBounds = Info.Bounds;
		}
		if (Info.Bounds.IsInside(Request.EndLocation))
		{
			EndDataAsset = Info.NavDataAsset;
			EndVolumeBounds = Info.Bounds;
		}
	}

	if (StartDataAsset.IsNull() )
	{
		UE_LOG(LogTemp, Warning, TEXT("FindHighLevelPath Failed: Start location is not inside any navigation volume defined in the World Manifest. Start: %s"), *StartDataAsset.ToString());
		return false;
	}
	if (EndDataAsset.IsNull())
	{
		UE_LOG(LogTemp, Warning, TEXT("FindHighLevelPath Failed: End location is not inside any navigation volume defined in the World Manifest. End: %s"), *EndDataAsset.ToString());
		return false;
	}

	if (StartDataAsset == EndDataAsset)
	{
		OutDataAssetPath.Add({ StartDataAsset, -1 });
		return true;
	}

	UE_LOG(LogTemp, Log, TEXT("--- Starting Data-Driven High-Level A* Search from '%s' to '%s' ---"), *StartDataAsset.ToString(), *EndDataAsset.ToString());

	struct FDataAssetNode
	{
		TSoftObjectPtr<UOctreeDataAsset> DataAsset;
		FBox VolumeBounds;
		float GCost = 0;
		float HCost = 0;
		float GetFCost() const { return GCost + HCost; }
		bool operator>(const FDataAssetNode& Other) const { return GetFCost() > Other.GetFCost(); }
		bool operator<(const FDataAssetNode& Other) const { return GetFCost() < Other.GetFCost(); }
	};

	TArray<FDataAssetNode> OpenSet;
	TMap<TSoftObjectPtr<UOctreeDataAsset>, TPair<TSoftObjectPtr<UOctreeDataAsset>, int32>> CameFrom;
	TMap<TSoftObjectPtr<UOctreeDataAsset>, float> GCostMap;

	const float HCost = FVector::Dist(StartVolumeBounds.GetCenter(), EndVolumeBounds.GetCenter());
	FDataAssetNode StartNode = { StartDataAsset, StartVolumeBounds, 0.f, HCost };
	OpenSet.HeapPush(StartNode, TGreater<FDataAssetNode>());
	GCostMap.Add(StartDataAsset, 0.f);

	while (OpenSet.Num() > 0)
	{
		FDataAssetNode CurrentNode;
		OpenSet.HeapPop(CurrentNode, TGreater<FDataAssetNode>());

		if (CurrentNode.DataAsset == EndDataAsset)
		{
			UE_LOG(LogTemp, Log, TEXT("--- Data-Driven High-Level A* Search SUCCEEDED! Reconstructing path... ---"));
			ReconstructDataAssetPath(CameFrom, CurrentNode.DataAsset, OutDataAssetPath);
			return true;
		}

		UOctreeDataAsset* CurrentData = CurrentNode.DataAsset.LoadSynchronous();
		if (!CurrentData)
		{
			UE_LOG(LogTemp, Warning, TEXT("A* Search: Failed to load data asset '%s'."), *CurrentNode.DataAsset.ToString());
			continue;
		}
		CurrentData->DecompressAndInitialize();

		for (const auto& PortalPair : CurrentData->PortalNodes)
		{
			const FVolumePortalLink& PortalLink = PortalPair.Value;
			TSoftObjectPtr<UOctreeDataAsset> NeighborDataAsset = PortalLink.TargetVolume;
			FBox NeighborBounds;
			bool bFoundNeighborInfo = false;
			for (const FVolumeSpatialInfo& Info : CurrentWorldNavData->VolumeInfos)
			{
				if (Info.NavDataAsset == NeighborDataAsset)
				{
					NeighborBounds = Info.Bounds;
					bFoundNeighborInfo = true;
					break;
				}
			}
			if (!bFoundNeighborInfo) continue;

			const float NewGCost = CurrentNode.GCost + FVector::Dist(CurrentNode.VolumeBounds.GetCenter(), NeighborBounds.GetCenter());
			if (!GCostMap.Contains(NeighborDataAsset) || NewGCost < GCostMap[NeighborDataAsset])
			{
				GCostMap.Add(NeighborDataAsset, NewGCost);
				const float NeighborHCost = FVector::Dist(NeighborBounds.GetCenter(), EndVolumeBounds.GetCenter());
				FDataAssetNode NeighborNode = { NeighborDataAsset, NeighborBounds, NewGCost, NeighborHCost };
				OpenSet.HeapPush(NeighborNode, TGreater<FDataAssetNode>());
				CameFrom.Add(NeighborDataAsset, { CurrentNode.DataAsset, PortalPair.Key });
			}
		}
	}

	UE_LOG(LogTemp, Error, TEXT("--- Data-Driven High-Level A* Search FAILED! OpenSet is empty but goal was not reached. ---"));
	return false;
}

void UPathfindingSubsystem::ProcessNextHierarchicalSegment(const UObject* WorldContextObject, FHierarchicalPathfindingTask& Task)
{
	if (!IsValid(WorldContextObject))
	{
		UE_LOG(LogTemp, Error, TEXT("Hierarchical path failed for RequestID %d: WorldContextObject is invalid."), Task.OriginalRequest.RequestID);
		Task.State = EHierarchicalTaskState::Failed;
		DestroyStreamingProbeForRequest(WorldContextObject, Task.OriginalRequest.RequestID);
		Task.OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
		ActiveHierarchicalTasks.Remove(Task.OriginalRequest.RequestID);
		return;
	}

	UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("Hierarchical path failed for RequestID %d: Could not get World from WorldContextObject."), Task.OriginalRequest.RequestID);
		Task.State = EHierarchicalTaskState::Failed;
		DestroyStreamingProbeForRequest(WorldContextObject, Task.OriginalRequest.RequestID);
		Task.OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
		ActiveHierarchicalTasks.Remove(Task.OriginalRequest.RequestID);
		return;
	}

	if (!Task.HighLevelDataAssetPath.IsValidIndex(Task.CurrentSegmentIndex))
	{
		// OnHierarchicalSegmentCompleted에서 모든 세그먼트가 완료되었을 때 처리
		Task.State = EHierarchicalTaskState::Complete;
		return;
	}

	const TSoftObjectPtr<UOctreeDataAsset> CurrentDataAsset = Task.HighLevelDataAssetPath[Task.CurrentSegmentIndex].Key;

	if (Task.State == EHierarchicalTaskState::ReadyToFindLocalPath)
	{
		UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>();
		AOctreeVoxelVolume* TargetVolume = NavSubsystem ? NavSubsystem->GetVolumeByDataAsset(CurrentDataAsset) : nullptr;

		if (TargetVolume && TargetVolume->IsReadyForNavigation())
		{
			UE_LOG(LogTemp, Log, TEXT("Segment %d for RequestID %d: Volume is ready. Finding local path in '%s'."), Task.CurrentSegmentIndex, Task.OriginalRequest.RequestID, *TargetVolume->GetName());

			// 이 볼륨을 로드하기 위해 생성했을 수 있는 프로브는 이제 임무를 완수했으므로 파괴
			DestroyStreamingProbeForRequest(WorldContextObject, Task.OriginalRequest.RequestID);

			FVector StartLocation, EndLocation;
			bool bLocationsValid = true;

			if (Task.CurrentSegmentIndex == 0)
			{
				StartLocation = Task.OriginalRequest.StartLocation;
			}
			else
			{
				const TSoftObjectPtr<UOctreeDataAsset> PreviousDataAsset = Task.HighLevelDataAssetPath[Task.CurrentSegmentIndex - 1].Key;
				const int32 ExitPortalNodeIndexInPrevVolume = Task.HighLevelDataAssetPath[Task.CurrentSegmentIndex - 1].Value;

				if (UOctreeDataAsset* PreviousData = PreviousDataAsset.LoadSynchronous())
				{
					PreviousData->DecompressAndInitialize();
					if (const FVolumePortalLink* PortalLink = PreviousData->PortalNodes.Find(ExitPortalNodeIndexInPrevVolume))
					{
						if (UOctreeDataAsset* CurrentData = CurrentDataAsset.LoadSynchronous())
						{
							CurrentData->DecompressAndInitialize();
							if (CurrentData->NodePool.IsValidIndex(PortalLink->TargetNodeIndex))
							{
								StartLocation = CurrentData->NodePool[PortalLink->TargetNodeIndex].Bounds.GetCenter();
							}
							else { bLocationsValid = false; }
						}
						else { bLocationsValid = false; }
					}
					else { bLocationsValid = false; }
				}
				else { bLocationsValid = false; }
			}

			const bool bIsLastSegment = (Task.CurrentSegmentIndex == Task.HighLevelDataAssetPath.Num() - 1);
			if (bIsLastSegment)
			{
				EndLocation = Task.OriginalRequest.EndLocation;
			}
			else
			{
				const int32 ExitPortalNodeIndex = Task.HighLevelDataAssetPath[Task.CurrentSegmentIndex].Value;
				if (UOctreeDataAsset* CurrentData = CurrentDataAsset.LoadSynchronous())
				{
					CurrentData->DecompressAndInitialize();
					if (CurrentData->NodePool.IsValidIndex(ExitPortalNodeIndex))
					{
						EndLocation = CurrentData->NodePool[ExitPortalNodeIndex].Bounds.GetCenter();
					}
					else { bLocationsValid = false; }
				}
				else { bLocationsValid = false; }
			}

			if (!bLocationsValid || StartLocation.IsZero() || EndLocation.IsZero())
			{
				UE_LOG(LogTemp, Error, TEXT("Hierarchical path failed for RequestID %d: Could not calculate Start or End location for segment %d."), Task.OriginalRequest.RequestID, Task.CurrentSegmentIndex);
				Task.State = EHierarchicalTaskState::Failed;
				DestroyStreamingProbeForRequest(WorldContextObject, Task.OriginalRequest.RequestID);
				Task.OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
				ActiveHierarchicalTasks.Remove(Task.OriginalRequest.RequestID);
				return;
			}

			FPathfindingRequest LocalRequest;
			LocalRequest.RequestingObject = Task.OriginalRequest.RequestingComponent.Get();
			LocalRequest.StartLocation = StartLocation;
			LocalRequest.EndLocation = EndLocation;
			LocalRequest.AgentRadius = Task.OriginalRequest.AgentRadius;
			LocalRequest.Strategy = Task.OriginalRequest.Strategy;
			LocalRequest.NodeType = EPathfindingNodeType::Octree;

			if (UPathfindingComponent* Comp = Task.OriginalRequest.RequestingComponent.Get())
			{
				Comp->CreateOctreeDataSnapshot(TargetVolume, LocalRequest.NavigationData);
			}

			Task.LastLocalRequestID = RequestPath(WorldContextObject, LocalRequest);
			if (Task.LastLocalRequestID != -1)
			{
				Task.State = EHierarchicalTaskState::FindingLocalPath;
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Hierarchical path failed for RequestID %d: Failed to submit local path request for segment %d."), Task.OriginalRequest.RequestID, Task.CurrentSegmentIndex);
				Task.State = EHierarchicalTaskState::Failed;
				DestroyStreamingProbeForRequest(WorldContextObject, Task.OriginalRequest.RequestID);
				Task.OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
				ActiveHierarchicalTasks.Remove(Task.OriginalRequest.RequestID);
			}
		}
		// 다음 세그먼트의 볼륨이 로드되지 않은 경우, 프로브 활성화
		else
		{
			UE_LOG(LogTemp, Log, TEXT("Segment %d for RequestID %d: Volume with data '%s' is not loaded. Activating streaming probe."), Task.CurrentSegmentIndex, Task.OriginalRequest.RequestID, *CurrentDataAsset.ToString());

			bool bFoundVolumeInfo = false;
			if (CurrentWorldNavData)
			{
				for (const FVolumeSpatialInfo& Info : CurrentWorldNavData->VolumeInfos)
				{
					if (Info.NavDataAsset == CurrentDataAsset)
					{
						DestroyStreamingProbeForRequest(WorldContextObject, Task.OriginalRequest.RequestID);

						const FVector SpawnLocation = Info.Bounds.GetCenter();
						const FTransform SpawnTransform(FRotator::ZeroRotator, SpawnLocation);

						FActorSpawnParameters SpawnParams;
						SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

						if (const AActor* ContextActor = Cast<const AActor>(WorldContextObject))
						{
							SpawnParams.OverrideLevel = ContextActor->GetLevel();
						}
						else if (const UActorComponent* ContextComponent = Cast<const UActorComponent>(WorldContextObject))
						{
							if (ContextComponent->GetOwner())
							{
								SpawnParams.OverrideLevel = ContextComponent->GetOwner()->GetLevel();
							}
						}

						UE_LOG(LogTemp, Warning, TEXT("[Probe Spawner] Attempting to spawn probe in Level: %s"),
							SpawnParams.OverrideLevel ? *SpawnParams.OverrideLevel->GetPathName() : TEXT("NULL (This will likely fail)"));
						UE_LOG(LogTemp, Warning, TEXT("[Probe Spawner] Spawning at Transform: %s"), *SpawnTransform.ToString());

						if (APathfindingStreamingProbe* NewProbe = World->SpawnActor<APathfindingStreamingProbe>(APathfindingStreamingProbe::StaticClass(), SpawnTransform, SpawnParams))
						{
							UE_LOG(LogTemp, Warning, TEXT("[Probe Spawner] Spawn Succeeded. Probe's location post-spawn: %s"), *NewProbe->GetActorLocation().ToString());

							const float RequiredRadius = Info.Bounds.GetExtent().Size();
							NewProbe->SetupStreamingSource(RequiredRadius);
							NewProbe->ActivateProbe(SpawnLocation);

							ActiveStreamingProbes.Add(Task.OriginalRequest.RequestID, NewProbe);
						}
						else
						{
							UE_LOG(LogTemp, Error, TEXT("[Probe Spawner] Spawn FAILED. World->SpawnActor returned nullptr."));
						}

						bFoundVolumeInfo = true;
						break;
					}
				}
			}

			if (bFoundVolumeInfo)
			{
				Task.State = EHierarchicalTaskState::WaitingForVolumeToLoad;
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("Hierarchical path failed for RequestID %d: Could not find spatial info for data asset '%s' in the world manifest."), Task.OriginalRequest.RequestID, *CurrentDataAsset.ToString());
				Task.State = EHierarchicalTaskState::Failed;
				DestroyStreamingProbeForRequest(WorldContextObject, Task.OriginalRequest.RequestID);
				Task.OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
				ActiveHierarchicalTasks.Remove(Task.OriginalRequest.RequestID);
			}
		}
	}
}