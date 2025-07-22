// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "NavVolume/OctreeVoxelVolume.h"

// Engine Headers
#include "Engine/World.h"
#include "TimerManager.h"
#include "Components/BoxComponent.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Async/Async.h"

// Pathfinder Core Headers
#include "NavData/VoxelObstacleProfile.h"
#include "Component/DynamicObstacleComponent.h"

// SIMD Headers for Dynamic Grid Query
#include "emmintrin.h"
#include "smmintrin.h"

#if WITH_EDITOR
// Editor-specific Headers
#include "FileHelpers.h"
#include "EditorAssetLibrary.h"
#include "ObjectTools.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Misc/ScopedSlowTask.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#endif

AOctreeVoxelVolume::AOctreeVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;

	// Default value for octree depth. For a 128^3 volume, a depth of 7 provides voxel-level precision (2^7 = 128).
	MaxOctreeDepth = 7;
}

//~======================================================================
// AActor Overrides
//~======================================================================

void AOctreeVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	// 1. Load the pre-baked static octree data from the assigned data asset.
	if (BakedOctreeData)
	{
		UE_LOG(LogTemp, Log, TEXT("Step 1: BakedOctreeData is VALID. Asset Name: %s"), *BakedOctreeData->GetName());

		BakedOctreeData->DecompressAndInitialize();
		if (BakedOctreeData->NodePool.Num() > 0)
		{
			UE_LOG(LogTemp, Log, TEXT("Step 2: Decompression SUCCESS. NodePool has %d nodes."), BakedOctreeData->NodePool.Num());
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Step 2: Decompression FAILED or data is empty. NodePool is EMPTY."));
		}

		StaticNodePoolPtr = &BakedOctreeData->NodePool;

		if (!StaticNodePoolPtr->IsEmpty())
		{
			// The root node is always assumed to be at the first index.
			RootNodeIndex = 0;
		}
		if (StaticNodePoolPtr)
		{
			UE_LOG(LogTemp, Error, TEXT("Step 3. Complete"));
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Step 3: Decompression FAILED or StaticNodePoolPtr is empty. StaticNodePoolPtr is EMPTY."));
		}
	}

	// 2. Initialize the dynamic obstacle layer and start a timer for periodic updates.
	InitializeDynamicLayer();
	GetWorldTimerManager().SetTimer(DynamicObstacleTimer, this, &AOctreeVoxelVolume::UpdateDynamicObstacles, 0.1f, true, 0.1f);
}

void AOctreeVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	// Main logic is handled by BeginPlay initialization and the DynamicObstacleTimer.
}

//~======================================================================
// Core Public API
//~======================================================================

void AOctreeVoxelVolume::RegisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent)
{
	if (IsValid(ObstacleComponent))
	{
		RegisteredObstacles.Add(ObstacleComponent);
	}
}

void AOctreeVoxelVolume::UnregisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent)
{
	if (IsValid(ObstacleComponent))
	{
		// Clear the voxels that the obstacle last occupied.
		UpdateVoxelStateForBounds(ObstacleComponent->GetLastKnownBounds(), false);
		RegisteredObstacles.Remove(ObstacleComponent);
	}
}

//~======================================================================
// Debugging
//~======================================================================

void AOctreeVoxelVolume::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
	OutObstacleIndices.Empty();

	// 1. Collect obstacle voxels from the dynamic grid by iterating through the bitmask.
	if (!DynamicObstacleBitmask.IsEmpty())
	{
		const int32 ChunksPerRow = MapSize.X / 128;
		for (int32 z = 0; z < MapSize.Z; ++z)
		{
			for (int32 y = 0; y < MapSize.Y; ++y)
			{
				for (int32 chunkX = 0; chunkX < ChunksPerRow; ++chunkX)
				{
					const int32 ChunkIndex = (z * MapSize.Y + y) * ChunksPerRow + chunkX;
					const F128BitChunk& DynamicChunk = DynamicObstacleBitmask[ChunkIndex];

					// Skip empty chunks for performance.
					if (DynamicChunk.Words[0] == 0 && DynamicChunk.Words[1] == 0) continue;

					for (int i = 0; i < 2; ++i)
					{
						if (DynamicChunk.Words[i] != 0)
						{
							for (int bit = 0; bit < 64; ++bit)
							{
								if ((DynamicChunk.Words[i] >> bit) & 1)
								{
									OutObstacleIndices.Emplace(chunkX * 128 + i * 64 + bit, y, z);
								}
							}
						}
					}
				}
			}
		}
	}

	// 2. Collect static obstacle voxels by traversing the octree.
	// NOTE: This requires a full traversal to find all solid leaf nodes and can be very slow.
	// It is intended for debugging purposes only. Implementation is omitted here for brevity
	// but would involve a recursive traversal similar to GetDebugNodeInfoRecursive.
	if (RootNodeIndex != -1)
	{
		// TSet<FIntVector> StaticObstacles;
		// ... recursive traversal to find solid leaf nodes and add their voxels to the set ...
		// OutObstacleIndices.Append(StaticObstacles.Array());
	}
}

//~======================================================================
// Dynamic Obstacle Internals
//~======================================================================

void AOctreeVoxelVolume::InitializeDynamicLayer()
{
	// Ensure map dimensions are valid for the bitmask implementation.
	check(MapSize.X > 0 && MapSize.X % 128 == 0);
	check(MapSize.Y > 0);
	check(MapSize.Z > 0);

	const int32 NumChunks = (MapSize.X / 128) * MapSize.Y * MapSize.Z;
	DynamicObstacleBitmask.Empty(NumChunks);
	DynamicObstacleBitmask.AddZeroed(NumChunks);
}

void AOctreeVoxelVolume::UpdateDynamicObstacles()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AOctreeVoxelVolume::UpdateDynamicObstacles);

	for (UDynamicObstacleComponent* Comp : RegisteredObstacles)
	{
		if (!IsValid(Comp)) continue;

		// Only update if the component has moved (is "dirty").
		if (Comp->IsDirty())
		{
			// 1. Clear the voxels at the obstacle's previous location.
			UpdateVoxelStateForBounds(Comp->GetLastKnownBounds(), false);

			// 2. Voxelize the obstacle at its new location.
			const FBox CurrentBounds = Comp->GetCurrentBounds();
			UpdateVoxelStateForBounds(CurrentBounds, true);

			// 3. Update the component's state for the next frame.
			Comp->UpdateLastKnownBounds();
			Comp->ClearDirtyFlag();
		}
	}
}

void AOctreeVoxelVolume::UpdateVoxelStateForBounds(const FBox& InBounds, bool bAsObstacle)
{
	if (!InBounds.IsValid) return;

	// Convert world bounds to voxel indices.
	const FIntVector MinVoxelIndex = WorldToVoxelIndex(InBounds.Min);
	const FIntVector MaxVoxelIndex = WorldToVoxelIndex(InBounds.Max);

	// Clamp indices to ensure they are within the volume's grid.
	FIntVector ClampedMin, ClampedMax;
	if (!ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex, ClampedMin, ClampedMax)) return;

	// Iterate through the clamped voxel range and update each voxel's state.
	for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
	{
		for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
		{
			for (int32 x = ClampedMin.X; x <= ClampedMax.X; ++x)
			{
				UpdateDynamicVoxelState({ x, y, z }, bAsObstacle);
			}
		}
	}
}

void AOctreeVoxelVolume::UpdateDynamicVoxelState(const FIntVector& Index, bool bIsObstacle)
{
	if (!IsIndexValid(Index)) return;

	// Calculate the memory location of the target bit.
	const int32 ChunksPerRow = MapSize.X / 128;
	const int32 ChunkIndex = (Index.Z * MapSize.Y + Index.Y) * ChunksPerRow + (Index.X / 128);
	const int32 BitIndex = Index.X % 128;

	if (!DynamicObstacleBitmask.IsValidIndex(ChunkIndex)) return;

	F128BitChunk& TargetChunk = DynamicObstacleBitmask[ChunkIndex];
	const int32 WordIndex = BitIndex / 64;      // 0 for bits 0-63, 1 for bits 64-127
	const uint64 BitMask = 1ULL << (BitIndex % 64); // The specific bit to flip

	// Set or clear the bit.
	if (bIsObstacle)
	{
		TargetChunk.Words[WordIndex] |= BitMask;
	}
	else
	{
		TargetChunk.Words[WordIndex] &= ~BitMask;
	}
}

//~======================================================================
// Query Helpers
//~======================================================================

bool AOctreeVoxelVolume::IsRegionEmpty(const FBox& QueryBox) const
{
	// If there's no octree data, the region is considered empty.
	if (RootNodeIndex == -1 || !StaticNodePoolPtr || StaticNodePoolPtr->IsEmpty())
	{
		return true;
	}
	
	const FBox VolumeBounds = GetComponentsBoundingBox(true, true);
	return IsRegionEmpty_Internal(RootNodeIndex, VolumeBounds, QueryBox);
}

bool AOctreeVoxelVolume::IsRegionEmpty_Internal(int32 NodeIndex, const FBox& NodeBounds, const FBox& QueryBox) const
{
	// If the query box doesn't intersect this node's bounds, there's no collision here.
	if (!NodeBounds.Intersect(QueryBox))
	{
		return true;
	}

	if (!StaticNodePoolPtr || !StaticNodePoolPtr->IsValidIndex(NodeIndex))
	{
		return true;
	}
	
	const FOctreeNode& Node = (*StaticNodePoolPtr)[NodeIndex];

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

//~======================================================================
// Editor & Baking
//~======================================================================

void AOctreeVoxelVolume::BakeStaticData()
{
	const FBox VolumeBounds = VolumeBox->Bounds.GetBox();
	if (!VolumeBounds.IsValid)
	{
		return;
	}

	// --- 1. Scan for all static obstacles within the volume ---
	TSet<FIntVector> AllStaticObstacles;
	ScanForAllStaticObstacles(AllStaticObstacles);

	// --- 2. Set up a progress dialog for the editor UI ---
	const double TotalWorkAmount = static_cast<double>(AllStaticObstacles.Num()) + 3.0;
	FScopedSlowTask BakeTask(TotalWorkAmount, FText::FromString(TEXT("Baking Octree Data...")));
	BakeTask.MakeDialog();

	
	// --- 3. Build the initial octree structure ---
	BakeTask.EnterProgressFrame(0.f, FText::FromString(TEXT("Building Octree... (1/4)")));

	StaticNodePool.Empty();

	RootNodeIndex = StaticNodePool.AddDefaulted();
	
	if (StaticNodePool.IsValidIndex(RootNodeIndex))
	{
		StaticNodePool[RootNodeIndex].Bounds = VolumeBounds;
	}
	else
	{
		UE_LOG(LogTemp, Fatal, TEXT("Failed to create root node for Octree."));
		return;
	}
	
	for (const FIntVector& ObstacleVoxel : AllStaticObstacles)
	{
		BakeTask.EnterProgressFrame(1.f);
		if (BakeTask.ShouldCancel()) return;
		InsertObstacleRecursive(RootNodeIndex, ObstacleVoxel, 0);
	}

	// --- 4. Optimize the tree by collapsing uniform nodes ---
	BakeTask.EnterProgressFrame(1.f, FText::FromString(TEXT("Optimizing Octree... (2/4)")));
	if (BakeTask.ShouldCancel()) return;
	OptimizeAndCollapse(RootNodeIndex, VolumeBounds);

	// --- 5. Precompute neighbors for leaf nodes to accelerate pathfinding ---
	BakeTask.EnterProgressFrame(1.f, FText::FromString(TEXT("Precomputing Neighbors... (3/4)")));
	if (BakeTask.ShouldCancel()) return;
	PrecomputeNeighbors();

	// --- 6. Save the generated octree data to a UOctreeDataAsset ---
	BakeTask.EnterProgressFrame(1.f, FText::FromString(TEXT("Saving Data Asset... (4/4)")));
	if (BakeTask.ShouldCancel()) return;
	SaveOctreeToAsset();

}

void AOctreeVoxelVolume::ScanForAllStaticObstacles(TSet<FIntVector>& OutStaticObstacles) const
{
	OutStaticObstacles.Empty();

	// 1. Determine which object types to scan based on the filter settings.
	TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypesToQuery;
	if (StaticObjectProfile)
	{
		for (const TEnumAsByte<ECollisionChannel>& Channel : StaticObjectProfile->CollisionChannels)
		{
			ObjectTypesToQuery.Add(UEngineTypes::ConvertToObjectType(Channel));
		}
		if (ObjectTypesToQuery.IsEmpty())
		{
			return;
		}
	}
	else
	{
		ObjectTypesToQuery.Add(UEngineTypes::ConvertToObjectType(ECollisionChannel::ECC_WorldStatic));
	}

	// 2. Perform a broad-phase overlap check to find all potentially relevant actors.
	const FBox VolumeWorldBounds = GetComponentsBoundingBox(true, true);
	TArray<AActor*> OverlappingActors;
	UKismetSystemLibrary::BoxOverlapActors(GetWorld(), VolumeWorldBounds.GetCenter(), VolumeWorldBounds.GetExtent(),
		ObjectTypesToQuery, AActor::StaticClass(), BakeExcludeActors, OverlappingActors);

	if (OverlappingActors.IsEmpty())
	{
		return;
	}

	// 3. Perform a narrow-phase check: iterate through components, apply the filter, and voxelize.
	for (AActor* Actor : OverlappingActors)
	{
		if (!IsValid(Actor)) continue;

		TArray<UPrimitiveComponent*> PrimitiveComponents;
		Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);

		for (UPrimitiveComponent* Comp : PrimitiveComponents)
		{
			if (!IsValid(Comp) || !Comp->IsQueryCollisionEnabled()) continue;

			// Use the filter to decide if this specific component should be baked.
			const bool bShouldBake = StaticObjectProfile ? StaticObjectProfile->DoesActorMatch(Actor, Comp) : true;

			if (bShouldBake)
			{
				const FBox ComponentBounds = Comp->Bounds.GetBox();
				FIntVector MinVoxel, MaxVoxel;
				if (!ClampVoxelIndicesToVolume(WorldToVoxelIndex(ComponentBounds.Min), WorldToVoxelIndex(ComponentBounds.Max), MinVoxel, MaxVoxel)) continue;

				// Voxelize the component's bounding box.
				for (int32 z = MinVoxel.Z; z <= MaxVoxel.Z; ++z)
				{
					for (int32 y = MinVoxel.Y; y <= MaxVoxel.Y; ++y)
					{
						for (int32 x = MinVoxel.X; x <= MaxVoxel.X; ++x)
						{
							const FIntVector VoxelIndex(x, y, z);
							if (OutStaticObstacles.Contains(VoxelIndex)) continue;

							// Final check to ensure the voxel actually overlaps the component geometry.
							if (IsVoxelOverlapping(VoxelIndex, Comp))
							{
								OutStaticObstacles.Add(VoxelIndex);
							}
						}
					}
				}
			}
		}
	}
}

void AOctreeVoxelVolume::InsertObstacleRecursive(int32 NodeIndex, const FIntVector& ObstacleVoxel, int32 CurrentDepth)
{
	if (!StaticNodePool.IsValidIndex(NodeIndex))
	{
		return;
	}

	// If the node is already solid, this obstacle is already accounted for.
	if (StaticNodePool[NodeIndex].State == EOctreeNodeState::Solid)
	{
		return;
	}

	// If we've reached the maximum depth, this leaf node must become solid.
	if (CurrentDepth == MaxOctreeDepth)
	{
		StaticNodePool[NodeIndex].State = EOctreeNodeState::Solid;
		return;
	}

	// If this node is currently empty, it must be subdivided to contain the obstacle.
	// It becomes a 'Mixed' node, and its children are created.
	if (StaticNodePool[NodeIndex].State == EOctreeNodeState::Empty)
	{
		StaticNodePool[NodeIndex].State = EOctreeNodeState::Mixed;
		const FBox CurrentNodeBounds = StaticNodePool[NodeIndex].Bounds;

		for (int i = 0; i < 8; ++i)
		{
			const int32 NewChildIndex = StaticNodePool.Emplace(FOctreeNode());
			// It's crucial to re-acquire the node reference after Emplace, as the array might have been reallocated.
			StaticNodePool[NewChildIndex].Bounds = CalculateChildBounds(CurrentNodeBounds, i);
			StaticNodePool[NodeIndex].ChildrenIndices[i] = NewChildIndex;
		}
	}

	// Recurse into the correct child octant.
	const FVector VoxelWorldPos = VoxelIndexToWorld(ObstacleVoxel);
	const int32 ChildOctant = GetOctantContainingVoxel(StaticNodePool[NodeIndex].Bounds, VoxelWorldPos);
	const int32 ChildNodeIndex = StaticNodePool[NodeIndex].ChildrenIndices[ChildOctant];

	if (ChildNodeIndex != -1)
	{
		InsertObstacleRecursive(ChildNodeIndex, ObstacleVoxel, CurrentDepth + 1);
	}
}

int32 AOctreeVoxelVolume::FindLeafNodeAt_BakeTime(const FVector& WorldPos) const
{
	if (!VolumeBox->Bounds.GetBox().IsInside(WorldPos) || RootNodeIndex == -1)
	{
		return -1;
	}

	int32 CurrentNodeIndex = RootNodeIndex;
	int32 CurrentDepth = 0;

	while (StaticNodePool.IsValidIndex(CurrentNodeIndex))
	{
		const FOctreeNode& Node = StaticNodePool[CurrentNodeIndex];

		// A leaf node is found if it's not mixed or if max depth is reached.
		if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= MaxOctreeDepth)
		{
			return CurrentNodeIndex;
		}

		// Descend to the appropriate child.
		const int32 ChildOctant = GetOctantContainingVoxel(Node.Bounds, WorldPos);
		CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
		CurrentDepth++;
	}
	return -1; // Should not be reached if logic is correct.
}

void AOctreeVoxelVolume::PrecomputeNeighbors()
{
	FScopedSlowTask NeighborTask(StaticNodePool.Num(), FText::FromString(TEXT("Precomputing Neighbors...")));
	NeighborTask.MakeDialog();

	const FBox TotalVolumeBounds = VolumeBox->Bounds.GetBox();
	constexpr float Epsilon = 1.0f; // Small offset to push test point into the neighboring space.

	for (int32 i = 0; i < StaticNodePool.Num(); ++i)
	{
		NeighborTask.EnterProgressFrame(1);
		if (NeighborTask.ShouldCancel()) return;

		// Neighbors are only relevant for leaf nodes (Solid or Empty).
		if (StaticNodePool[i].State == EOctreeNodeState::Mixed)
		{
			continue;
		}

		// Also pre-calculate the max agent radius this node can accommodate.
		StaticNodePool[i].MaxAllowedRadius = StaticNodePool[i].Bounds.GetExtent().GetMin();

		const FBox NodeBounds = StaticNodePool[i].Bounds;
		for (int32 DirIndex = 0; DirIndex < static_cast<int32>(ENeighborDirection::MAX); ++DirIndex)
		{
			FVector TestPoint = NodeBounds.GetCenter();
			const ENeighborDirection Dir = static_cast<ENeighborDirection>(DirIndex);

			// Position the test point just outside the current node's face in the desired direction.
			switch (Dir)
			{
			case ENeighborDirection::Right: TestPoint.X = NodeBounds.Max.X + Epsilon; break;
			case ENeighborDirection::Left:  TestPoint.X = NodeBounds.Min.X - Epsilon; break;
			case ENeighborDirection::Front: TestPoint.Y = NodeBounds.Max.Y + Epsilon; break;
			case ENeighborDirection::Back:  TestPoint.Y = NodeBounds.Min.Y - Epsilon; break;
			case ENeighborDirection::Up:    TestPoint.Z = NodeBounds.Max.Z + Epsilon; break;
			case ENeighborDirection::Down:  TestPoint.Z = NodeBounds.Min.Z - Epsilon; break;
			default: break;
			}

			// Find the leaf node at the test point and store it as the neighbor.
			if (TotalVolumeBounds.IsInside(TestPoint))
			{
				const int32 NeighborNodeIndex = FindLeafNodeAt_BakeTime(TestPoint);
				StaticNodePool[i].Neighbors[DirIndex] = NeighborNodeIndex;
			}
			else
			{
				StaticNodePool[i].Neighbors[DirIndex] = -1; // Neighbor is outside the volume.
			}
		}
	}
}

void AOctreeVoxelVolume::OptimizeAndCollapse(int32 NodeIndex, const FBox& NodeBounds)
{
	if (!StaticNodePool.IsValidIndex(NodeIndex)) return;

	FOctreeNode& Node = StaticNodePool[NodeIndex];
	if (Node.State != EOctreeNodeState::Mixed)
	{
		return; // This is a leaf node, nothing to collapse below it.
	}

	// Recursively collapse children first (bottom-up approach).
	for (int i = 0; i < 8; ++i)
	{
		if (Node.ChildrenIndices[i] != -1)
		{
			OptimizeAndCollapse(Node.ChildrenIndices[i], CalculateChildBounds(NodeBounds, i));
		}
	}

	// Check if all children are now leaf nodes of the same state.
	const int32 FirstChildIndex = Node.ChildrenIndices[0];
	if (FirstChildIndex == -1) return; // Should not happen in a mixed node.

	const EOctreeNodeState FirstChildState = StaticNodePool[FirstChildIndex].State;
	if (FirstChildState == EOctreeNodeState::Mixed) return; // Cannot collapse if a child is still mixed.

	bool bAllChildrenAreSame = true;
	for (int i = 1; i < 8; ++i)
	{
		const int32 ChildNodeIndex = Node.ChildrenIndices[i];
		if (ChildNodeIndex == -1 || StaticNodePool[ChildNodeIndex].State != FirstChildState)
		{
			bAllChildrenAreSame = false;
			break;
		}
	}

	// If all children are identical leaves, collapse them by making the parent a leaf of that state.
	if (bAllChildrenAreSame)
	{
		Node.State = FirstChildState;
		// The children nodes are now orphaned but remain in the pool. A final compaction step could remove them.
		for (int i = 0; i < 8; ++i)
		{
			Node.ChildrenIndices[i] = -1;
		}
	}
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

int32 AOctreeVoxelVolume::GetOctantContainingVoxel(const FBox& NodeBounds, const FVector& VoxelWorldPos) const
{
	int32 Octant = 0;
	const FVector Center = NodeBounds.GetCenter();
	if (VoxelWorldPos.X >= Center.X) Octant |= 1; // Right half
	if (VoxelWorldPos.Y >= Center.Y) Octant |= 2; // Front half
	if (VoxelWorldPos.Z >= Center.Z) Octant |= 4; // Top half
	return Octant;
}

bool AOctreeVoxelVolume::GetBakedAssetPath(FString& OutPackagePath, FString& OutAssetName) const
{
	if (BakedDataSaveDirectory.Path.IsEmpty())
	{
		return false;
	}
	
	UWorld* World = GetWorld();
	if (!World)
	{
		return false;
	}

	const FString RawLevelName = World->GetMapName();

	const FString CleanLevelName = UWorld::RemovePIEPrefix(RawLevelName);

	OutAssetName = FString::Printf(TEXT("DA_%s_%s"), *CleanLevelName, *GetActorLabel());

	const FString PackageDirectory = FPaths::Combine(TEXT("/Game"), BakedDataSaveDirectory.Path);

	OutPackagePath = FPaths::Combine(PackageDirectory, OutAssetName);

	return true;
}

void AOctreeVoxelVolume::SaveOctreeToAsset()
{
	FString PackageName, AssetName;
	if (!GetBakedAssetPath(PackageName, AssetName))
	{
		UE_LOG(LogTemp, Error, TEXT("Could not determine a valid path for the Data Asset."));
		return;
	}

	// Ensure the target directory exists.
	const FString DirectoryPath = FPaths::GetPath(PackageName);
	UEditorAssetLibrary::MakeDirectory(DirectoryPath);

	// Check for and delete any existing asset at the target path to prevent conflicts.
	FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
	const FAssetData ExistingAssetData = AssetRegistryModule.Get().GetAssetByObjectPath(FSoftObjectPath(PackageName));
	if (ExistingAssetData.IsValid())
	{
		ObjectTools::DeleteAssets({ ExistingAssetData }, false); // false = do not show confirmation dialog
	}

	// Create a new package and data asset object.
	UPackage* Package = CreatePackage(*PackageName);
	Package->FullyLoad();

	UOctreeDataAsset* NewOctreeData = NewObject<UOctreeDataAsset>(Package, FName(*AssetName), RF_Public | RF_Standalone);
	if (!NewOctreeData)
	{
		return;
	}

	// --- KEY ADDITIONS ---
	// 1. Notify the engine that we are about to modify this object. This is important for the transaction buffer (Undo/Redo).
	NewOctreeData->Modify();

	// 2. Call your function to populate and compress the data.
	NewOctreeData->SetAndCompressBakedData(StaticNodePool, this->MapSize, this->MaxOctreeDepth);
  
	// 3. Mark the package as dirty. This is the flag that tells the engine "this file needs to be saved".
	Package->MarkPackageDirty();

	// Notify the asset registry that a new asset has been created. This helps the Content Browser update immediately.
	FAssetRegistryModule::AssetCreated(NewOctreeData);
	
	TArray<UPackage*> PackagesToSave;
	PackagesToSave.Add(Package);

	if (UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true))
	{
		// Assign the newly created asset to the volume property for immediate use in the editor.
		this->BakedOctreeData = NewOctreeData;
		UE_LOG(LogTemp, Log, TEXT("Successfully baked and saved Octree data to: %s"), *PackageName);
	}
}

void AOctreeVoxelVolume::UpdateDebugObstacleVisualization()
{
	Super::UpdateDebugObstacleVisualization();
#if WITH_EDITORONLY_DATA
	if (!VolumeBox || bIsUpdatingVisualization)
	{
		return;
	}

	bIsUpdatingVisualization = true;
	TWeakObjectPtr<AOctreeVoxelVolume> WeakThis(this);
	const FBox InitialBounds = VolumeBox->Bounds.GetBox();

	// 1. Collect node information asynchronously to avoid blocking the game thread.
	Async(EAsyncExecution::ThreadPool, [WeakThis, InitialBounds]()
		{
			if (!WeakThis.IsValid() || WeakThis->RootNodeIndex == -1)
			{
				if (WeakThis.IsValid()) WeakThis->bIsUpdatingVisualization = false;
				return;
			}

			TArray<FDebugNodeInfo> NodesToDraw;
			WeakThis->GetDebugNodeInfoRecursive(NodesToDraw, WeakThis->RootNodeIndex, InitialBounds, 0);

			// 2. Schedule the instance update back on the game thread with the collected data.
			AsyncTask(ENamedThreads::GameThread, [WeakThis, NodesToDraw = MoveTemp(NodesToDraw)]()
				{
					AOctreeVoxelVolume* StrongThis = WeakThis.Get();
					if (!StrongThis) return;

					// Clear all existing debug instances.
					if (StrongThis->DebugSolidNodesComponent) StrongThis->DebugSolidNodesComponent->ClearInstances();
					if (StrongThis->DebugMixedNodesComponent) StrongThis->DebugMixedNodesComponent->ClearInstances();

					if (NodesToDraw.IsEmpty())
					{
						StrongThis->bIsUpdatingVisualization = false;
						return;
					}

					// Collect transforms for each node type.
					TArray<FTransform> SolidTransforms, MixedTransforms;
					for (const FDebugNodeInfo& NodeInfo : NodesToDraw)
					{
						const FVector Scale = NodeInfo.Bounds.GetSize() / 100.0f; // Assuming 100-unit debug mesh.
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

					// Batch-add the new instances. Using bWorldSpace=true is simpler here.
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
#endif
}

void AOctreeVoxelVolume::GetDebugNodeInfoRecursive(TArray<FDebugNodeInfo>& OutNodes, int32 NodeIndex, const FBox& NodeBounds, int32 CurrentDepth) const
{
	if (!StaticNodePoolPtr || StaticNodePoolPtr->IsEmpty()) return;

	if (!StaticNodePoolPtr->IsValidIndex(NodeIndex) || CurrentDepth > MaxOctreeDepth)
	{
		return;
	}
	const FOctreeNode& Node = (*StaticNodePoolPtr)[NodeIndex];

	// Add node info if it's a leaf or if we've reached the desired debug draw depth.
	if (Node.State != EOctreeNodeState::Mixed || CurrentDepth == MaxOctreeDepth)
	{
		OutNodes.Add({ NodeIndex, NodeBounds, Node.State });
	}
	else // If mixed, recurse into children.
	{
		for (int i = 0; i < 8; ++i)
		{
			if (Node.ChildrenIndices[i] != -1)
			{
				GetDebugNodeInfoRecursive(OutNodes, Node.ChildrenIndices[i], CalculateChildBounds(NodeBounds, i), CurrentDepth + 1);
			}
		}
	}
}

#endif // WITH_EDITOR