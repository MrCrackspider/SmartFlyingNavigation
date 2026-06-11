// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "NavVolume/OctreeVoxelVolume.h"

// Engine Headers
#include "Engine/World.h"
#include "TimerManager.h"
#include "Components/BoxComponent.h"
#include "Async/Async.h"

// Pathfinder Core Headers
#include "NavCore/OctreePathfindingData.h"
#include "NavCore/SmartFlyingNavStats.h"

#if WITH_EDITOR
// Editor-specific Headers
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#endif
#include "DrawDebugHelpers.h"
#include "HAL/IConsoleManager.h"
#include "Subsystem/OctreeNavSubsystem.h"

static TAutoConsoleVariable<int32> CVarPathUseSDF(
	TEXT("sfn.Path.UseSDF"),
	1,
	TEXT("When 1, static clearance queries use the baked distance field (when present). When 0, fall back to octree capsule tests."),
	ECVF_Default);

AOctreeVoxelVolume::AOctreeVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = false;

#if WITH_EDITOR
	SetIsSpatiallyLoaded(true);
#endif
}

//~======================================================================
// AActor Overrides
//~======================================================================

void AOctreeVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	{
		const FVector TotalSize = GetVolumeBounds().GetSize();
		const FIntVector CalculatedMapSize(
			FMath::RoundToInt(TotalSize.X / VoxelExtent),
			FMath::RoundToInt(TotalSize.Y / VoxelExtent),
			FMath::RoundToInt(TotalSize.Z / VoxelExtent)
		);

		if (MapSize != CalculatedMapSize)
		{
			UE_LOG(LogTemp, Warning, TEXT("AOctreeVoxelVolume '%s': Baked MapSize %s differs from current volume size %s. Overwriting with current size."),
				*GetName(), *MapSize.ToString(), *CalculatedMapSize.ToString());
			MapSize = CalculatedMapSize;
		}
	}
}

void AOctreeVoxelVolume::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UWorld* World = GetWorld())
	{
		if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
		{
			NavSubsystem->UnregisterVolumeChunk(this);
		}
	}

	{
		FScopeLock Lock(&ChunkDataCS);
		ChunkData.Reset();
	}
	DEC_MEMORY_STAT_BY(STAT_SFN_ChunkMemory, LoadedChunkBytes);
	LoadedChunkBytes = 0;
	RootNodeIndex = -1;
	LoadedChunkId = INDEX_NONE;
	LoadState = EVolumeLoadState::Unloaded;

	Super::EndPlay(EndPlayReason);
}

void AOctreeVoxelVolume::PostInitializeComponents()
{
	Super::PostInitializeComponents();

	if (UWorld* World = GetWorld())
	{
		if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
		{
			NavSubsystem->RegisterVolumeChunk(this);
		}
	}
}


bool AOctreeVoxelVolume::IsLocationInBlockedVoxel(const FVector& WorldLocation) const
{
	if (LoadState != EVolumeLoadState::Loaded)
	{
		return true;
	}
	if (!GetVolumeBounds().IsInsideOrOn(WorldLocation))
	{
		return true;
	}

	// Dynamic obstacles: exact primitive query against the world registry.
	if (const UWorld* World = GetWorld())
	{
		if (const UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
		{
			if (NavSubsystem->GetObstacleRegistry().IsPointBlocked(WorldLocation))
			{
				return true;
			}
		}
	}

	const TArray<FOctreeNode>* NodePool = GetNodePoolPtr();
	if (!NodePool || RootNodeIndex == -1)
	{
		return false;
	}

	int32 CurrentNodeIndex = RootNodeIndex;
	int32 CurrentDepth = 0;

	while (NodePool->IsValidIndex(CurrentNodeIndex))
	{
		const FOctreeNode& Node = (*NodePool)[CurrentNodeIndex];

		if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= GetMaxOctreeDepth())
		{
			return Node.State == EOctreeNodeState::Solid;
		}

		const int32 ChildOctant = GetOctantContainingVoxel(Node.Bounds, WorldLocation);
		CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
		CurrentDepth++;

		if (CurrentNodeIndex == -1)
		{
			return false;
		}
	}

	return false;
}

void AOctreeVoxelVolume::CreatePathfindingDataView(FOctreePathfindingData& OutData, bool bSnapshotDynamicObstacles) const
{
	OutData = FOctreePathfindingData();
	OutData.RootNodeIndex = RootNodeIndex;
	OutData.MaxOctreeDepth = GetMaxOctreeDepth();
	OutData.VolumeBounds = GetVolumeBounds();
	OutData.MapSize = GetMapSize();
	OutData.VoxelSize = VoxelExtent;

	// Hold a shared reference so worker tasks keep the payload alive even if this
	// volume streams out or re-initializes while the task is in flight.
	{
		FScopeLock Lock(&ChunkDataCS);
		OutData.ChunkDataRef = ChunkData;
	}

	if (OutData.ChunkDataRef)
	{
		const FLoadedNavChunkData& Data = *OutData.ChunkDataRef;
		OutData.StaticNodePoolPtr = Data.NodePool.Num() > 0 ? &Data.NodePool : nullptr;
		OutData.AirwayGraphPtr = Data.AirwayGraph.Num() > 0 ? &Data.AirwayGraph : nullptr;

		if (Data.DistanceField.IsValid())
		{
			OutData.DistanceFieldPtr = &Data.DistanceField;
		}
	}

	// Snapshot dynamic obstacles relevant to this volume (owned copy: safe on worker
	// threads). Allocates — skipped for cheap game-thread probes.
	if (bSnapshotDynamicObstacles)
	{
		if (const UWorld* World = GetWorld())
		{
			if (const UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
			{
				const FBox RelevantBounds = GetVolumeBounds().ExpandBy(VoxelExtent * 2.f);
				NavSubsystem->GetObstacleRegistry().Snapshot(OutData.DynamicObstacles, &RelevantBounds);
			}
		}
	}

	OutData.bUseDistanceField = CVarPathUseSDF.GetValueOnGameThread() != 0;
}

bool AOctreeVoxelVolume::IsPointBlockedForAgent(const FVector& Location, float AgentRadius, bool bCheckStatic,
                                                bool bCheckDynamic) const
{
	if (bCheckStatic)
	{
		FOctreePathfindingData DataView;
		CreatePathfindingDataView(DataView, /*bSnapshotDynamicObstacles=*/false);
		if (DataView.IsPointBlockedForAgent(Location, AgentRadius, true, false))
		{
			return true;
		}
	}

	if (bCheckDynamic)
	{
		if (const UWorld* World = GetWorld())
		{
			if (const UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
			{
				if (NavSubsystem->GetObstacleRegistry().IsPointBlocked(Location, AgentRadius))
				{
					return true;
				}
			}
		}
	}

	return false;
}

bool AOctreeVoxelVolume::IsSegmentBlockedForAgent(const FVector& Start, const FVector& End, float AgentRadius,
                                                  bool bCheckStatic, bool bCheckDynamic) const
{
	if (bCheckStatic)
	{
		FOctreePathfindingData DataView;
		CreatePathfindingDataView(DataView, /*bSnapshotDynamicObstacles=*/false);
		if (DataView.IsSegmentBlockedForAgent(Start, End, AgentRadius, true, false))
		{
			return true;
		}
	}

	if (bCheckDynamic)
	{
		if (const UWorld* World = GetWorld())
		{
			if (const UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
			{
				if (NavSubsystem->GetObstacleRegistry().IsSegmentBlocked(Start, End, AgentRadius))
				{
					return true;
				}
			}
		}
	}

	return false;
}
//~======================================================================
// Query Helpers
//~======================================================================

bool AOctreeVoxelVolume::IsRegionEmpty(const FBox& QueryBox) const
{
	// If data is not ready, assume the region is empty to avoid false positives.
	if (LoadState != EVolumeLoadState::Loaded || RootNodeIndex == -1 || !GetNodePoolPtr())
	{
		return true;
	}

	const FBox VolumeBounds = GetVolumeBounds();
	return IsRegionEmpty_Internal(RootNodeIndex, VolumeBounds, QueryBox);
}

bool AOctreeVoxelVolume::IsRegionEmpty_Internal(int32 NodeIndex, const FBox& NodeBounds, const FBox& QueryBox) const
{
	// If the query box doesn't intersect this node's bounds, there's no collision here.
	if (!NodeBounds.Intersect(QueryBox))
	{
		return true;
	}

	const TArray<FOctreeNode>* NodePool = GetNodePoolPtr();
	if (!NodePool || !NodePool->IsValidIndex(NodeIndex))
	{
		return true;
	}

	const FOctreeNode& Node = (*NodePool)[NodeIndex];

	// Terminal conditions for the recursion.
	if (Node.State == EOctreeNodeState::Solid)
	{
		return false; // Collision detected.
	}
	if (Node.State == EOctreeNodeState::Empty)
	{
		return true; // No collision in this branch.
	}

	// If the node is mixed, we must check its children.
	for (int i = 0; i < 8; ++i)
	{
		if (Node.ChildrenIndices[i] != -1)
		{
			const FBox ChildBounds = CalculateChildBounds(NodeBounds, i);
			if (!IsRegionEmpty_Internal(Node.ChildrenIndices[i], ChildBounds, QueryBox))
			{
				// If any child reports a collision, the query fails.
				return false;
			}
		}
	}

	// If all children were checked and none reported a collision, the region is clear.
	return true;
}

#if WITH_EDITOR

#if WITH_EDITORONLY_DATA
void AOctreeVoxelVolume::UpdateDebugObstacleVisualization()
{
	Super::UpdateDebugObstacleVisualization();

	if (!VolumeBox || bIsUpdatingVisualization)
	{
		return;
	}

	if (LoadState != EVolumeLoadState::Loaded || RootNodeIndex == -1 || !GetNodePoolPtr())
	{
		if (DebugSolidNodesComponent) DebugSolidNodesComponent->ClearInstances();
		if (DebugMixedNodesComponent) DebugMixedNodesComponent->ClearInstances();
		return;
	}

	bIsUpdatingVisualization = true;
	TWeakObjectPtr<AOctreeVoxelVolume> WeakThis(this);
	const FBox InitialBounds = VolumeBox->Bounds.GetBox();

	TArray<FOctreeNode> NodePoolCopy;
	{
		FScopeLock Lock(&ChunkDataCS);
		if (ChunkData)
		{
			NodePoolCopy = ChunkData->NodePool;
		}
	}
	const int32 RootIdxCopy = RootNodeIndex;
	const int32 MaxDepthCopy = GetMaxOctreeDepth();


	Async(EAsyncExecution::ThreadPool, [WeakThis, InitialBounds, NodePoolCopy, RootIdxCopy, MaxDepthCopy]()
		{
			if (!WeakThis.IsValid())
			{
				return;
			}

			TArray<FDebugNodeInfo> NodesToDraw;
			TFunction<void(int32, const FBox&, int32)> GetDebugInfoRecursive_Safe;
			GetDebugInfoRecursive_Safe =
				[&](int32 NodeIndex, const FBox& NodeBounds, int32 CurrentDepth)
				{
					constexpr int32 ABSOLUTE_MAX_DEPTH = 32;
					if (CurrentDepth > ABSOLUTE_MAX_DEPTH)
					{
						UE_LOG(LogTemp, Error, TEXT("GetDebugInfoRecursive_Safe: Exceeded absolute max depth! Possible infinite recursion detected."));
						return;
					}

					if (!NodePoolCopy.IsValidIndex(NodeIndex) || CurrentDepth > MaxDepthCopy)
					{
						return;
					}

					const FOctreeNode& Node = NodePoolCopy[NodeIndex];
					if (Node.State != EOctreeNodeState::Mixed || CurrentDepth == MaxDepthCopy)
					{
						NodesToDraw.Add({ NodeIndex, NodeBounds, Node.State });
					}
					else
					{
						for (int i = 0; i < 8; ++i)
						{
							if (Node.ChildrenIndices[i] != -1)
							{
								GetDebugInfoRecursive_Safe(Node.ChildrenIndices[i], WeakThis->CalculateChildBounds(NodeBounds, i), CurrentDepth + 1);
							}
						}
					}
				};

			GetDebugInfoRecursive_Safe(RootIdxCopy, InitialBounds, 0);

			AsyncTask(ENamedThreads::GameThread, [WeakThis, NodesToDraw = MoveTemp(NodesToDraw)]()
				{
					AOctreeVoxelVolume* StrongThis = WeakThis.Get();
					if (!StrongThis) return;

					if (StrongThis->DebugSolidNodesComponent) StrongThis->DebugSolidNodesComponent->ClearInstances();
					if (StrongThis->DebugMixedNodesComponent) StrongThis->DebugMixedNodesComponent->ClearInstances();

					if (NodesToDraw.IsEmpty())
					{
						StrongThis->bIsUpdatingVisualization = false;
						return;
					}

					TArray<FTransform> SolidTransforms, MixedTransforms;
					for (const FDebugNodeInfo& NodeInfo : NodesToDraw)
					{
						const FVector Scale = NodeInfo.Bounds.GetSize() / 100.0f;
						const FTransform NodeTransform(FRotator::ZeroRotator, NodeInfo.Bounds.GetCenter(), Scale);

						if (NodeInfo.State == EOctreeNodeState::Solid && StrongThis->DebugSolidNodesComponent)
						{
							SolidTransforms.Add(NodeTransform);
						}
						else if (NodeInfo.State == EOctreeNodeState::Mixed && StrongThis->DebugMixedNodesComponent)
						{
							MixedTransforms.Add(NodeTransform);
						}
					}

					if (StrongThis->DebugSolidNodesComponent && !SolidTransforms.IsEmpty())
					{
						StrongThis->DebugSolidNodesComponent->AddInstances(SolidTransforms, false, true);
					}
					if (StrongThis->DebugMixedNodesComponent && !MixedTransforms.IsEmpty())
					{
						StrongThis->DebugMixedNodesComponent->AddInstances(MixedTransforms, false, true);
					}

					StrongThis->bIsUpdatingVisualization = false;
				});
		});
}
#endif

#endif // WITH_EDITOR

FVector AOctreeVoxelVolume::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
{
	const FVector VoxelSize(VoxelExtent);
	return GetVolumeBounds().Min + (FVector(VoxelIndex) * VoxelExtent) + (VoxelSize * 0.5f);
}


// ======================================================================
// Query Helpers - Implementation
// ======================================================================

FIntVector AOctreeVoxelVolume::WorldToVoxelIndex(const FVector& WorldLocation) const
{
	if (VoxelExtent <= 0.f) return FIntVector::ZeroValue;

	const FVector LocalPosition = WorldLocation - GetVolumeBounds().Min;
	return FIntVector(
		FMath::FloorToInt(LocalPosition.X / VoxelExtent),
		FMath::FloorToInt(LocalPosition.Y / VoxelExtent),
		FMath::FloorToInt(LocalPosition.Z / VoxelExtent)
	);
}

bool AOctreeVoxelVolume::IsIndexValid(const FIntVector& Index) const
{
	return Index.X >= 0 && Index.X < MapSize.X &&
		   Index.Y >= 0 && Index.Y < MapSize.Y &&
		   Index.Z >= 0 && Index.Z < MapSize.Z;
}


bool AOctreeVoxelVolume::ClampVoxelIndicesToVolume(const FIntVector& InMin, const FIntVector& InMax, FIntVector& OutClampedMin, FIntVector& OutClampedMax) const
{
	OutClampedMin.X = FMath::Max(InMin.X, 0);
	OutClampedMin.Y = FMath::Max(InMin.Y, 0);
	OutClampedMin.Z = FMath::Max(InMin.Z, 0);

	OutClampedMax.X = FMath::Min(InMax.X, MapSize.X - 1);
	OutClampedMax.Y = FMath::Min(InMax.Y, MapSize.Y - 1);
	OutClampedMax.Z = FMath::Min(InMax.Z, MapSize.Z - 1);

	return OutClampedMin.X <= OutClampedMax.X &&
		   OutClampedMin.Y <= OutClampedMax.Y &&
		   OutClampedMin.Z <= OutClampedMax.Z;
}


int32 AOctreeVoxelVolume::FindLeafNodeAt(const FVector& WorldPos) const
{
	const TArray<FOctreeNode>* NodePool = GetNodePoolPtr();
	if (LoadState != EVolumeLoadState::Loaded || RootNodeIndex == -1 || !NodePool)
	{
		return -1;
	}

	int32 CurrentNodeIndex = RootNodeIndex;
	int32 CurrentDepth = 0;

	while (NodePool->IsValidIndex(CurrentNodeIndex))
	{
		const FOctreeNode& Node = (*NodePool)[CurrentNodeIndex];

		if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= GetMaxOctreeDepth())
		{
			return CurrentNodeIndex;
		}

		const int32 ChildOctant = GetOctantContainingVoxel(Node.Bounds, WorldPos);
		CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
		CurrentDepth++;
	}

	return -1;
}


FBox AOctreeVoxelVolume::CalculateChildBounds(const FBox& ParentBounds, int32 ChildIndex) const
{
	const FVector Center = ParentBounds.GetCenter();
	const FVector Min = ParentBounds.Min;
	const FVector Max = ParentBounds.Max;

	// Determine the min/max corners for the child octant based on its index.
	switch (ChildIndex)
	{
	case 0: return FBox(Min, Center); // ---
	case 1: return FBox(FVector(Center.X, Min.Y, Min.Z), FVector(Max.X, Center.Y, Center.Z)); // +--
	case 2: return FBox(FVector(Min.X, Center.Y, Min.Z), FVector(Center.X, Max.Y, Center.Z)); // -+-
	case 3: return FBox(FVector(Center.X, Center.Y, Min.Z), FVector(Max.X, Max.Y, Center.Z)); // ++-
	case 4: return FBox(FVector(Min.X, Min.Y, Center.Z), FVector(Center.X, Center.Y, Max.Z)); // --+
	case 5: return FBox(FVector(Center.X, Min.Y, Center.Z), FVector(Max.X, Center.Y, Max.Z)); // +-+
	case 6: return FBox(FVector(Min.X, Center.Y, Center.Z), FVector(Center.X, Max.Y, Max.Z)); // -++
	case 7: return FBox(Center, Max); // +++
	default: return FBox(ForceInit);
	}
}


bool AOctreeVoxelVolume::InitializeFromManifestPayload(int32 ChunkId, TArray<FOctreeNode>&& InNodePool,
                                                       FAirwayGraph&& InAirwayGraph,
                                                       FNavDistanceField&& InDistanceField)
{
	if (InNodePool.IsEmpty() && InAirwayGraph.IsEmpty())
	{
		return false;
	}

	TSharedPtr<FLoadedNavChunkData, ESPMode::ThreadSafe> NewChunkData = MakeShared<FLoadedNavChunkData, ESPMode::ThreadSafe>();
	NewChunkData->NodePool = MoveTemp(InNodePool);
	NewChunkData->AirwayGraph = MoveTemp(InAirwayGraph);
	NewChunkData->DistanceField = MoveTemp(InDistanceField);

#if PLATFORM_ANDROID
	// Mobile Lite mode: when an airway graph exists, the octree node pool is not needed at runtime.
	if (!NewChunkData->AirwayGraph.IsEmpty())
	{
		NewChunkData->NodePool.Empty();
	}
#endif

	const bool bHasNodePool = NewChunkData->NodePool.Num() > 0;
	const bool bHasAirwayGraph = !NewChunkData->AirwayGraph.IsEmpty();

	if (!bHasNodePool && !bHasAirwayGraph)
	{
		LoadState = EVolumeLoadState::Failed;
		return false;
	}

	{
		FScopeLock Lock(&ChunkDataCS);
		ChunkData = MoveTemp(NewChunkData);

		DEC_MEMORY_STAT_BY(STAT_SFN_ChunkMemory, LoadedChunkBytes);
		LoadedChunkBytes = ChunkData->NodePool.GetAllocatedSize()
			+ ChunkData->AirwayGraph.GetAllocatedBytes()
			+ ChunkData->DistanceField.GetAllocatedBytes();
		INC_MEMORY_STAT_BY(STAT_SFN_ChunkMemory, LoadedChunkBytes);
	}

	LoadedChunkId = ChunkId;
	RootNodeIndex = bHasNodePool ? 0 : -1;
	LoadState = EVolumeLoadState::Loaded;
	return true;
}

FBox AOctreeVoxelVolume::GetVolumeBounds() const
{
	return VolumeBox ? VolumeBox->Bounds.GetBox() : FBox(ForceInit);
}


int32 AOctreeVoxelVolume::GetOctantContainingVoxel(const FBox& NodeBounds, const FVector& VoxelWorldPos) const
{
	int32 Octant = 0;
	const FVector Center = NodeBounds.GetCenter();
	if (VoxelWorldPos.X >= Center.X) Octant |= 1; // Right half
	if (VoxelWorldPos.Y >= Center.Y) Octant |= 2; // Front half
	if (VoxelWorldPos.Z >= Center.Z) Octant |= 4; // Top half
	return Octant;
}

