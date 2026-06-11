// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Subsystem/PathfindingSubsystem.h"
#include "Strategy/PathfindingStrategy.h"
#include "Utility/Parallel/MemoryPool.h"
#include "Utility/AStarStrategyUtils.h"
#include "Component/PathfindingComponent.h"
#include "Subsystem/OctreeNavSubsystem.h"
#include "Subsystem/AvoidanceSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "NavData/WorldNavigationDataAsset.h"
#include "NavCore/SmartFlyingNavStats.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMemory.h"

#if !UE_BUILD_SHIPPING
#include "Debug/SNavDebugPanel.h"
#include "Engine/GameViewportClient.h"
#include "GameFramework/PlayerController.h"
#include "Widgets/Layout/SBox.h"
#endif
#include "HAL/PlatformProcess.h"
#include "HAL/ThreadManager.h"
#include "Misc/QueuedThreadPool.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Misc/Paths.h"
#include "Algo/Reverse.h"

void FQueuedThreadPoolDeleter::operator()(FQueuedThreadPool* Pool) const
{
	if (Pool)
	{
		Pool->Destroy();
	}
}

/**
 * Always-on nav benchmark overlay. Enable with `sfn.ShowNavHUD 1` (console) or add
 * `ExecCmds="sfn.ShowNavHUD 1"` to the launch command line / DefaultEngine.ini
 * [SystemSettings] for devices without console access.
 */
static TAutoConsoleVariable<int32> CVarShowNavHUD(
	TEXT("sfn.ShowNavHUD"),
	0,
	TEXT("When 1, draws an on-screen overlay with frame time, process memory, nav chunk memory, agent and obstacle counts."),
	ECVF_Default);

#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarShowDebugPanel(
	TEXT("sfn.ShowDebugPanel"),
	0,
	TEXT("When 1, shows the on-device touch debug panel (pause/resume, path line toggle, stat/dump buttons)."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarStartPaused(
	TEXT("sfn.StartPaused"),
	0,
	TEXT("When 1, the game starts paused so debug options can be configured before agents begin moving. Resume from the debug panel."),
	ECVF_Default);
#endif

void FPathfindingAsyncTasks::DoThreadedWork()
{
	SCOPE_CYCLE_COUNTER(STAT_SFN_FindPath);

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

	PathfindingThreadPool.Reset();

	int32 RequestedNumThreads = this->CurrentNumWorkerThreads;
#if PLATFORM_ANDROID
	RequestedNumThreads = 1;
	MaxCallbacksPerFrame = FMath::Min(MaxCallbacksPerFrame, 2);
#endif
	const int32 NumPhysicalCores = FPlatformMisc::NumberOfCores();
	const int32 IdealMaxThreads = FMath::Max(1, NumPhysicalCores - 2);
	const int32 FinalNumThreads = FMath::Clamp(RequestedNumThreads, 1, IdealMaxThreads);

	PathfindingThreadPool.Reset(FQueuedThreadPool::Allocate());
	if (PathfindingThreadPool)
	{
		if (PathfindingThreadPool->Create(FinalNumThreads, 32 * 1024, TPri_Normal, TEXT("PathfindingThreadPool")))
		{
			this->CurrentNumWorkerThreads = FinalNumThreads;
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to create PathfindingThreadPool."));
			PathfindingThreadPool.Reset();
			this->CurrentNumWorkerThreads = 0;
		}
	}
	if (World)
	{
		const FString MapName = UWorld::RemovePIEPrefix(World->GetMapName());
		const FString ManifestAssetName = TEXT("DA_WorldNav_") + MapName;
		const FString ManifestAssetPath = FPaths::Combine(TEXT("/Game/Data/Navigation/"), ManifestAssetName);

		CurrentWorldNavData = LoadObject<UWorldNavigationDataAsset>(nullptr, *ManifestAssetPath);
		if (CurrentWorldNavData && !CurrentWorldNavData->IsFormatSupported())
		{
			CurrentWorldNavData = nullptr;
		}

		if (CurrentWorldNavData)
		{
			UE_LOG(LogTemp, Log, TEXT("Loaded world navigation manifest '%s' with %d chunks and %d portals."),
				*ManifestAssetPath,
				CurrentWorldNavData->Chunks.Num(),
				CurrentWorldNavData->Portals.Num());

			if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
			{
				NavSubsystem->SetWorldNavigationData(CurrentWorldNavData);
			}
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("World navigation manifest not found at '%s'. Bake the navigation data for this level."), *ManifestAssetPath);
		}
	}

	NextRequestID.Set(0);
}

void UPathfindingSubsystem::Deinitialize()
{
#if !UE_BUILD_SHIPPING
	if (DebugPanelWidget.IsValid() && GEngine && GEngine->GameViewport)
	{
		GEngine->GameViewport->RemoveViewportWidgetContent(DebugPanelWidget.ToSharedRef());
	}
	DebugPanelWidget.Reset();
#endif

	CurrentWorldNavData = nullptr;

	PathfindingThreadPool.Reset();

	CompletedTasksQueue.Empty();

	{
		FScopeLock Lock(&PoolLock);
		IdleGridPools.Empty();
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

	INC_DWORD_STAT(STAT_SFN_PathRequests);

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

	DrawNavHUD(DeltaTime);

#if !UE_BUILD_SHIPPING
	TickDebugTools();
#endif
}

#if !UE_BUILD_SHIPPING
void UPathfindingSubsystem::TickDebugTools()
{
	UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld() || !GEngine || !GEngine->GameViewport)
	{
		return;
	}

	// On-device debug panel (touch buttons), top-right.
	if (CVarShowDebugPanel.GetValueOnGameThread() != 0 && !DebugPanelWidget.IsValid())
	{
		TSharedRef<SWidget> Panel =
			SNew(SBox)
			.HAlign(HAlign_Right)
			.VAlign(VAlign_Top)
			.Padding(FMargin(0.f, 60.f, 16.f, 0.f))
			[
				SNew(SNavDebugPanel).World(World)
			];

		GEngine->GameViewport->AddViewportWidgetContent(Panel, 1000);
		DebugPanelWidget = Panel;
	}

	// Start paused: lets you configure debug options before BeginPlay-driven agents start moving.
	if (!bStartPausedHandled && CVarStartPaused.GetValueOnGameThread() != 0)
	{
		if (APlayerController* PC = World->GetFirstPlayerController())
		{
			bStartPausedHandled = true;
			PC->SetPause(true);
		}
	}

#if ENABLE_DRAW_DEBUG
	// While paused, agent components stop ticking — keep their cached paths visible from here.
	if (World->IsPaused())
	{
		static IConsoleVariable* DrawCVar = IConsoleManager::Get().FindConsoleVariable(TEXT("sfn.DrawPathDebug"));
		if (DrawCVar && DrawCVar->GetInt() == 1)
		{
			if (const UAvoidanceSubsystem* Avoidance = World->GetSubsystem<UAvoidanceSubsystem>())
			{
				for (UPathfindingComponent* Agent : Avoidance->GetRegisteredAgents())
				{
					if (IsValid(Agent))
					{
						Agent->DrawDebugPath();
					}
				}
			}
		}
	}
#endif
}
#endif // !UE_BUILD_SHIPPING

void UPathfindingSubsystem::DrawNavHUD(float DeltaTime) const
{
	if (CVarShowNavHUD.GetValueOnGameThread() == 0 || !GEngine)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld())
	{
		return;
	}

	int32 LoadedChunks = 0;
	int64 ChunkBytes = 0;
	int32 ObstacleCount = 0;
	if (const UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
	{
		for (const AOctreeVoxelVolume* Volume : NavSubsystem->GetRegisteredVolumes())
		{
			if (IsValid(Volume) && Volume->IsReadyForNavigation())
			{
				LoadedChunks++;
				ChunkBytes += Volume->GetLoadedChunkBytes();
			}
		}
		ObstacleCount = NavSubsystem->GetObstacleRegistry().Num();
	}

	int32 AgentCount = 0;
	if (const UAvoidanceSubsystem* Avoidance = World->GetSubsystem<UAvoidanceSubsystem>())
	{
		AgentCount = Avoidance->GetRegisteredAgentCount();
	}

	const FPlatformMemoryStats MemStats = FPlatformMemory::GetStats();
	const float FrameMs = DeltaTime * 1000.f;
	const float Fps = DeltaTime > KINDA_SMALL_NUMBER ? 1.f / DeltaTime : 0.f;

	const FString HudText = FString::Printf(
		TEXT("[SFN] %.1f FPS (%.2f ms) | Proc %.0f MB | DevFree %.0f/%.0f MB | NavChunks %d (%.2f MB) | Agents %d | DynObstacles %d | Threads %d"),
		Fps, FrameMs,
		MemStats.UsedPhysical / (1024.0 * 1024.0),
		MemStats.AvailablePhysical / (1024.0 * 1024.0),
		MemStats.TotalPhysical / (1024.0 * 1024.0),
		LoadedChunks,
		ChunkBytes / (1024.0 * 1024.0),
		AgentCount,
		ObstacleCount,
		CurrentNumWorkerThreads);

	constexpr uint64 HudKey = 0x53464E48; // 'SFNH' — stable key so the line updates in place.
	GEngine->AddOnScreenDebugMessage(HudKey, 0.f, FColor::Cyan, HudText);
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

	PathfindingThreadPool.Reset(FQueuedThreadPool::Allocate());
	if (PathfindingThreadPool && PathfindingThreadPool->Create(FinalNumThreads, 32 * 1024, TPri_Normal, TEXT("PathfindingThreadPool")))
	{
		CurrentNumWorkerThreads = FinalNumThreads;
	}
	else
	{
		PathfindingThreadPool.Reset();
		CurrentNumWorkerThreads = 0;
	}
}

bool UPathfindingSubsystem::TryGetNavigationChunkForLocation(const FVector& WorldLocation, FNavigationChunkRecord& OutChunk) const
{
	if (!CurrentWorldNavData)
	{
		return false;
	}

	for (const FNavigationChunkRecord& Chunk : CurrentWorldNavData->Chunks)
	{
		if (Chunk.Bounds.IsInsideOrOn(WorldLocation))
		{
			OutChunk = Chunk;
			return true;
		}
	}

	return false;
}

bool UPathfindingSubsystem::GetChunkPortals(int32 ChunkId, TArray<FNavigationChunkPortal>& OutPortals) const
{
	OutPortals.Reset();
	if (!CurrentWorldNavData)
	{
		return false;
	}

	const FNavigationChunkRecord* FoundChunk = CurrentWorldNavData->Chunks.FindByPredicate(
		[ChunkId](const FNavigationChunkRecord& Chunk)
		{
			return Chunk.ChunkId == ChunkId;
		});

	if (!FoundChunk)
	{
		return false;
	}

	for (int32 PortalIndex = 0; PortalIndex < FoundChunk->PortalCount; ++PortalIndex)
	{
		const int32 SourceIndex = FoundChunk->PortalStartIndex + PortalIndex;
		if (CurrentWorldNavData->Portals.IsValidIndex(SourceIndex))
		{
			OutPortals.Add(CurrentWorldNavData->Portals[SourceIndex]);
		}
	}

	return true;
}

bool UPathfindingSubsystem::FindHighLevelChunkPath(const FVector& StartLocation, const FVector& EndLocation, TArray<int32>& OutChunkPath) const
{
	OutChunkPath.Reset();
	if (!CurrentWorldNavData)
	{
		return false;
	}

	FNavigationChunkRecord StartChunk;
	FNavigationChunkRecord EndChunk;
	if (!TryGetNavigationChunkForLocation(StartLocation, StartChunk) ||
		!TryGetNavigationChunkForLocation(EndLocation, EndChunk))
	{
		return false;
	}

	if (StartChunk.ChunkId == EndChunk.ChunkId)
	{
		OutChunkPath.Add(StartChunk.ChunkId);
		return true;
	}

	TQueue<int32> Open;
	TSet<int32> Visited;
	TMap<int32, int32> CameFrom;

	Open.Enqueue(StartChunk.ChunkId);
	Visited.Add(StartChunk.ChunkId);

	while (!Open.IsEmpty())
	{
		int32 CurrentChunkId = INDEX_NONE;
		Open.Dequeue(CurrentChunkId);

		TArray<FNavigationChunkPortal> Portals;
		if (!GetChunkPortals(CurrentChunkId, Portals))
		{
			continue;
		}

		for (const FNavigationChunkPortal& Portal : Portals)
		{
			const int32 NextChunkId = Portal.TargetChunkId;
			if (NextChunkId == INDEX_NONE || Visited.Contains(NextChunkId))
			{
				continue;
			}

			Visited.Add(NextChunkId);
			CameFrom.Add(NextChunkId, CurrentChunkId);

			if (NextChunkId == EndChunk.ChunkId)
			{
				TArray<int32> ReversedPath;
				int32 WalkChunkId = EndChunk.ChunkId;
				ReversedPath.Add(WalkChunkId);

				while (CameFrom.Contains(WalkChunkId))
				{
					WalkChunkId = CameFrom[WalkChunkId];
					ReversedPath.Add(WalkChunkId);
				}

				Algo::Reverse(ReversedPath);
				OutChunkPath = MoveTemp(ReversedPath);
				return true;
			}

			Open.Enqueue(NextChunkId);
		}
	}

	return false;
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

	TArray<TUniquePtr<FPathNodeMemoryPoolBase>>& TargetIdlePools = (NodeType == EPathfindingNodeType::Grid) ? IdleGridPools : IdleOctreePools;

	if (TargetIdlePools.Num() > 0)
	{
		// Leased pools are owned by the request until returned via ReturnPool.
		return TargetIdlePools.Pop().Release();
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

	if (PoolToReturn->GetNodeType() == EPathfindingNodeType::Octree)
	{
		IdleOctreePools.Add(TUniquePtr<FPathNodeMemoryPoolBase>(PoolToReturn));
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
