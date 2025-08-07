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

int32 UPathfindingSubsystem::RequestHierarchicalPath(const UObject* WorldContextObject, FHierarchicalPathRequest& Request)
{
	if (!PathfindingThreadPool || !Request.RequestingComponent.IsValid() || !Request.Strategy.IsValid())
	{
		return -1;
	}

	Request.RequestID = NextRequestID.Increment();
	
	FHierarchicalPathfindingTask NewTask(Request);
	
	if (!FindHighLevelPath(WorldContextObject, Request, NewTask.HighLevelDataAssetPath))
	{
		UE_LOG(LogTemp, Warning, TEXT("Hierarchical Pathfinding Failed for RequestID %d: Could not find a high-level path of data assets."), Request.RequestID);
		Request.OnHierarchicalPathComplete.ExecuteIfBound({}, false);
		return -1;
	}
	
	NewTask.State = EHierarchicalTaskState::ReadyToFindLocalPath;
	ActiveHierarchicalTasks.Add(Request.RequestID, NewTask);
	
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
		// bool bIsHierarchicalSegment = false;
		// for (auto& Elem : ActiveHierarchicalTasks)
		// {
		// 	if (Elem.Value.LastLocalRequestID == Result.Request.RequestID)
		// 	{
		// 		OnHierarchicalSegmentCompleted(Elem.Key, Result.WorldPath, Result.bSuccess);
		// 		bIsHierarchicalSegment = true;
		// 		break;
		// 	}
		// }
		//
		// if (!bIsHierarchicalSegment)
		// {
		// 	OnPathfindingCompleted(Result.Request, Result.WorldPath, Result.bSuccess);
		// }

		// Use this until develop hierachy path finding
		OnPathfindingCompleted(Result.Request, Result.WorldPath, Result.bSuccess);
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

		// [수정] 실패 시 프로브를 정리하고 태스크를 종료합니다.
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
		// [수정] 전체 경로가 성공적으로 완료되었습니다.
		Task->State = EHierarchicalTaskState::Complete;
		Task->OriginalRequest.OnHierarchicalPathComplete.ExecuteIfBound(Task->CombinedWorldPath, true);

		// 프로브를 정리하고 태스크를 종료합니다.
		DestroyStreamingProbeForRequest(WorldContextObject, HierarchicalRequestID);
		ActiveHierarchicalTasks.Remove(HierarchicalRequestID);
	}
	else
	{
		// 다음 세그먼트를 처리합니다. 프로브는 계속 유지됩니다.
		Task->State = EHierarchicalTaskState::ReadyToFindLocalPath;
		ProcessNextHierarchicalSegment(WorldContextObject, *Task);
	}
}

void UPathfindingSubsystem::CreateStreamingProbeForRequest(const UObject* WorldContextObject, int32 RequestID, const FVector& Location, float Radius)
{
	UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("CreateStreamingProbeForRequest failed: Could not get World."));
		return;
	}

	// [수정된 로직] 이미 해당 요청 ID로 활성화된 프로브가 있는지 확인합니다.
	if (TObjectPtr<APathfindingStreamingProbe>* FoundProbePtr = ActiveStreamingProbes.Find(RequestID))
	{
		APathfindingStreamingProbe* ExistingProbe = *FoundProbePtr;
		if (IsValid(ExistingProbe))
		{
			// 기존 프로브가 있다면, 새로 생성하는 대신 위치와 반경을 업데이트하여 재활용합니다.
			UE_LOG(LogTemp, Log, TEXT("Moving existing probe for RequestID %d to new location: %s with radius %.f"), RequestID, *Location.ToString(), Radius);
			ExistingProbe->SetupStreamingSource(Radius);
			ExistingProbe->ActivateProbe(Location);
			return; // 함수 종료
		}
		else
		{
			// 맵에 있지만 유효하지 않은 경우, 제거하고 아래에서 새로 생성하도록 합니다.
			ActiveStreamingProbes.Remove(RequestID);
		}
	}

	// [기존 생성 로직] 기존 프로브가 없으면 새로 생성합니다.
	UE_LOG(LogTemp, Log, TEXT("Creating new probe for RequestID %d at location: %s with radius %.f"), RequestID, *Location.ToString(), Radius);

	const FTransform SpawnTransform(FRotator::ZeroRotator, Location);
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	// 월드 파티션 환경에서는 올바른 레벨에 스폰하는 것이 중요합니다.
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

	if (APathfindingStreamingProbe* NewProbe = World->SpawnActor<APathfindingStreamingProbe>(APathfindingStreamingProbe::StaticClass(), SpawnTransform, SpawnParams))
	{
		NewProbe->SetupStreamingSource(Radius);
		NewProbe->ActivateProbe(Location); // 스폰 후 위치를 다시 한번 설정해줍니다.
		ActiveStreamingProbes.Add(RequestID, NewProbe);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Probe spawn FAILED for RequestID %d. World->SpawnActor returned nullptr."), RequestID);
	}
}

void UPathfindingSubsystem::DestroyStreamingProbeForRequest(const UObject* WorldContextObject, int32 RequestID)
{
	if (TObjectPtr<APathfindingStreamingProbe>* FoundProbe = ActiveStreamingProbes.Find(RequestID))
	{
		if (IsValid(*FoundProbe))
		{
			UE_LOG(LogTemp, Log, TEXT("Destroying streaming probe for RequestID %d."), RequestID);
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
	// --- 1. 사전 조건 확인: 월드 내비게이션 명세(Manifest)가 로드되었는지 확인 ---
	if (!CurrentWorldNavData)
	{
		UE_LOG(LogTemp, Error, TEXT("FindHighLevelPath failed: World Navigation Data Manifest (CurrentWorldNavData) is not loaded."));
		return false;
	}

	// --- 2. 시작점과 도착점이 속한 볼륨의 데이터 에셋 찾기 ---
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

	// --- 3. 시작/도착 데이터 에셋 유효성 검사 ---
	if (StartDataAsset.IsNull())
	{
		UE_LOG(LogTemp, Warning, TEXT("FindHighLevelPath Failed: Start location is not inside any navigation volume defined in the World Manifest. Start: %s"), *Request.StartLocation.ToString());
		return false;
	}
	if (EndDataAsset.IsNull())
	{
		UE_LOG(LogTemp, Warning, TEXT("FindHighLevelPath Failed: End location is not inside any navigation volume defined in the World Manifest. End: %s"), *Request.EndLocation.ToString());
		return false;
	}

	// --- 4. 시작과 도착이 같은 볼륨에 있는 경우 (가장 간단한 케이스) ---
	if (StartDataAsset == EndDataAsset)
	{
		UE_LOG(LogTemp, Log, TEXT("FindHighLevelPath: Start and End are in the same volume. No high-level path needed."));
		OutDataAssetPath.Add({ StartDataAsset, -1 });
		return true;
	}

	// --- 5. 상위 레벨 A* 탐색 시작 ---
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
	OpenSet.HeapPush(StartNode, TLess<FDataAssetNode>());
	GCostMap.Add(StartDataAsset, 0.f);

	while (OpenSet.Num() > 0)
	{
		FDataAssetNode CurrentNode;
		OpenSet.HeapPop(CurrentNode, TLess<FDataAssetNode>());

		if (CurrentNode.DataAsset == EndDataAsset)
		{
			UE_LOG(LogTemp, Log, TEXT("--- Data-Driven High-Level A* Search SUCCEEDED! Reconstructing path... ---"));
			ReconstructDataAssetPath(CameFrom, CurrentNode.DataAsset, OutDataAssetPath);
			return true;
		}

		UE_LOG(LogTemp, Log, TEXT("A* Search: Popped '%s' from OpenSet. Loading its data..."), *CurrentNode.DataAsset.ToString());
		UOctreeDataAsset* CurrentData = CurrentNode.DataAsset.LoadSynchronous();

		if (!CurrentData)
		{
			UE_LOG(LogTemp, Error, TEXT(">>> RUNTIME FAILURE: Failed to load data asset '%s'. The asset might be corrupted or path is wrong."), *CurrentNode.DataAsset.ToString());
			continue;
		}

		CurrentData->DecompressAndInitialize();

		const int32 PortalNodeCount = CurrentData->PortalNodes.Num();
		UE_LOG(LogTemp, Log, TEXT(">>> RUNTIME DATA CHECK: '%s' loaded. It contains [%d] nodes with portals."), *CurrentData->GetName(), PortalNodeCount);

		if (PortalNodeCount == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT(">>> RUNTIME WARNING: This node has no portals. A* search cannot expand from here."));
		}

		// ==================================================================
		// <<< [수정된 핵심 로직] 중첩 루프를 사용하여 포털을 순회합니다. >>>
		// ==================================================================

		// 1. 바깥쪽 루프: PortalNodes 맵을 순회합니다.
		//    PortalNodePair.Key는 포털을 가진 노드의 인덱스입니다.
		//    PortalNodePair.Value는 해당 노드가 가진 모든 포털 링크를 담은 FVolumePortalLinks 구조체입니다.
		for (const auto& PortalNodePair : CurrentData->PortalNodes)
		{
			const FVolumePortalLinks& LinksContainer = PortalNodePair.Value;

			// 2. 안쪽 루프: FVolumePortalLinks 구조체 안의 Links 배열을 순회하여
			//    개별 FVolumePortalLink에 접근합니다.
			for (const FVolumePortalLink& PortalLink : LinksContainer.Links)
			{
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
					OpenSet.HeapPush(NeighborNode, TLess<FDataAssetNode>());

					// 경로 추적 정보(CameFrom)를 기록합니다.
					// NeighborDataAsset으로 가기 위해서는,
					// CurrentNode.DataAsset을 PortalNodePair.Key 인덱스의 노드를 통해 빠져나왔다는 의미입니다.
					CameFrom.Add(NeighborDataAsset, { CurrentNode.DataAsset, PortalNodePair.Key });
				}
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

			FVector StartLocation, EndLocation;
			bool bLocationsValid = true;

			// 첫 번째 세그먼트의 시작점은 에이전트의 현재 위치입니다.
			if (Task.CurrentSegmentIndex == 0)
			{
				StartLocation = Task.OriginalRequest.StartLocation;
			}
			else // 두 번째 이후 세그먼트의 시작점은 이전 볼륨의 포털에서 결정됩니다.
			{
				const TSoftObjectPtr<UOctreeDataAsset> PreviousDataAsset = Task.HighLevelDataAssetPath[Task.CurrentSegmentIndex - 1].Key;
				const int32 ExitPortalNodeIndexInPrevVolume = Task.HighLevelDataAssetPath[Task.CurrentSegmentIndex - 1].Value;

				if (UOctreeDataAsset* PreviousData = PreviousDataAsset.LoadSynchronous())
				{
					PreviousData->DecompressAndInitialize();

					// [수정된 핵심 로직]
					// 1. 이전 볼륨의 출구 노드에 연결된 모든 포털 링크 컨테이너를 찾습니다.
					if (const FVolumePortalLinks* LinksContainer = PreviousData->PortalNodes.Find(ExitPortalNodeIndexInPrevVolume))
					{
						const FVolumePortalLink* CorrectPortalLink = nullptr;
						// 2. 그 컨테이너 안의 모든 링크를 순회하며, 현재 목표 볼륨(CurrentDataAsset)으로 향하는 링크를 찾습니다.
						for (const FVolumePortalLink& CandidateLink : LinksContainer->Links)
						{
							if (CandidateLink.TargetVolume == CurrentDataAsset)
							{
								CorrectPortalLink = &CandidateLink;
								break; // 올바른 링크를 찾았으므로 루프를 종료합니다.
							}
						}

						// 3. 올바른 포털 링크를 찾았다면, 그 링크가 가리키는 현재 볼륨의 노드를 시작점으로 사용합니다.
						if (CorrectPortalLink)
						{
							if (UOctreeDataAsset* CurrentData = CurrentDataAsset.LoadSynchronous())
							{
								CurrentData->DecompressAndInitialize();
								if (CurrentData->NodePool.IsValidIndex(CorrectPortalLink->TargetNodeIndex))
								{
									StartLocation = CurrentData->NodePool[CorrectPortalLink->TargetNodeIndex].Bounds.GetCenter();
								}
								else { bLocationsValid = false; }
							}
							else { bLocationsValid = false; }
						}
						else { bLocationsValid = false; } // 현재 볼륨으로 향하는 포털 링크를 찾지 못함
					}
					else { bLocationsValid = false; } // 이전 볼륨의 출구 노드 정보를 찾지 못함
				}
				else { bLocationsValid = false; }
			}

			// 세그먼트의 끝점 계산 로직은 기존과 동일합니다.
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

			// 로컬 길찾기 요청 생성 및 전송 로직은 기존과 동일합니다.
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
		else // 볼륨이 로드되지 않은 경우의 처리 로직은 기존과 동일합니다.
		{
			UE_LOG(LogTemp, Log, TEXT("Segment %d for RequestID %d: Volume with data '%s' is not loaded. Activating/Moving streaming probe."), Task.CurrentSegmentIndex, Task.OriginalRequest.RequestID, *CurrentDataAsset.ToString());

			bool bFoundVolumeInfo = false;
			if (CurrentWorldNavData)
			{
				for (const FVolumeSpatialInfo& Info : CurrentWorldNavData->VolumeInfos)
				{
					if (Info.NavDataAsset == CurrentDataAsset)
					{
						const FVector ProbeLocation = Info.Bounds.GetCenter();
						const float RequiredRadius = Info.Bounds.GetExtent().Size();
						CreateStreamingProbeForRequest(WorldContextObject, Task.OriginalRequest.RequestID, ProbeLocation, RequiredRadius);

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