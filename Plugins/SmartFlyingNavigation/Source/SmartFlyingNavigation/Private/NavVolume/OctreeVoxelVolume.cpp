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

#include "Subsystem/OctreeNavSubsystem.h"
#include "Engine/StreamableManager.h"
#include "Engine/AssetManager.h"
// #include "LandscapeComponent.h" 
// #include "Landscape.h"
// #include "LandscapeStreamingProxy.h"
// #include "LandscapeSubsystem.h"
// #include "EngineUtils.h"
// #include "LandscapeInfo.h"

AOctreeVoxelVolume::AOctreeVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;

#if WITH_EDITOR
	SetIsSpatiallyLoaded(true);
#endif
	
	// Default value for octree depth. For a 128^3 volume, a depth of 7 provides voxel-level precision (2^7 = 128
}

//~======================================================================
// AActor Overrides
//~======================================================================

void AOctreeVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	//if (UWorld* World = GetWorld())
	//{
	//	if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
	//	{
	//		NavSubsystem->RegisterVolumeChunk(this);
	//	}
	//}

	//RequestAsyncLoad();
	
	{
		const FVector TotalSize = GetVolumeBounds().GetSize();
		const FIntVector CalculatedMapSize(
			FMath::RoundToInt(TotalSize.X / VoxelExtent),
			FMath::RoundToInt(TotalSize.Y / VoxelExtent),
			FMath::RoundToInt(TotalSize.Z / VoxelExtent)
		);

		if (MapSize != CalculatedMapSize)
		{
			UE_LOG(LogTemp, Warning, TEXT("AOctreeVoxelVolume '%s': Baked MapSize %s differs from current volume size %s. Overwriting with current size to ensure correct dynamic obstacle indexing."),
				*GetName(), *MapSize.ToString(), *CalculatedMapSize.ToString());
			MapSize = CalculatedMapSize;
		}
	}
	
	InitializeDynamicLayer();
	GetWorldTimerManager().SetTimer(DynamicObstacleTimer, this, &AOctreeVoxelVolume::UpdateDynamicObstacles, 0.01f, true, 0.1f);
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

	if (AsyncLoadHandle.IsValid())
	{
		AsyncLoadHandle->CancelHandle();
		AsyncLoadHandle.Reset();
	}

	StaticNodePoolPtr = nullptr;
	LoadedDataAsset = nullptr;
	LoadState = EVolumeLoadState::Unloaded;

	Super::EndPlay(EndPlayReason);

}

void AOctreeVoxelVolume::PostInitializeComponents()
{
	Super::PostInitializeComponents();

	RequestAsyncLoad();

	if (UWorld* World = GetWorld())
	{
		if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
		{
			NavSubsystem->RegisterVolumeChunk(this);
		}
	}
}

void AOctreeVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	//DrawDebugDynamicObstaclesWithBoxes();
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

	const FVector LocalPos = WorldLocation - GetVolumeBounds().Min;
	const int32 VoxelIndexX = FMath::FloorToInt(LocalPos.X / VoxelExtent);
	const int32 VoxelIndexY = FMath::FloorToInt(LocalPos.Y / VoxelExtent);
	const int32 VoxelIndexZ = FMath::FloorToInt(LocalPos.Z / VoxelExtent);
	
	if (IsIndexValid({VoxelIndexX, VoxelIndexY, VoxelIndexZ}))
	{
		const int64 LinearIndex = (static_cast<int64>(VoxelIndexZ) * MapSize.X * MapSize.Y) + 
								  (static_cast<int64>(VoxelIndexY) * MapSize.X) + VoxelIndexX;
		const int32 ChunkIndex = LinearIndex / 128;
		const int32 BitIndex = LinearIndex % 128;

		if (DynamicObstacleBitmask.IsValidIndex(ChunkIndex) && DynamicObstacleBitmask[ChunkIndex].GetBit(BitIndex))
		{
			return true;
		}
	}

	if (!StaticNodePoolPtr || RootNodeIndex == -1)
	{
		return false;
	}

	int32 CurrentNodeIndex = RootNodeIndex;
	int32 CurrentDepth = 0;

	while (StaticNodePoolPtr->IsValidIndex(CurrentNodeIndex))
	{
		const FOctreeNode& Node = (*StaticNodePoolPtr)[CurrentNodeIndex];

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
		if (!IsValid(Comp))
		{
			continue;
		}
		
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
	// If data is not ready, assume the region is empty to avoid false positives.
	if (LoadState != EVolumeLoadState::Loaded || RootNodeIndex == -1 || !StaticNodePoolPtr)
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
// 
// NOTE: The baking process should ideally be moved to a separate Editor Utility class/widget
// that orchestrates baking for the entire world, not just one volume.
// The functions here are modified to use a temporary, local node pool for baking.
//~======================================================================

bool AOctreeVoxelVolume::BakeStaticData(UWorld* World, TArray<FOctreeNode>& OutNodePool)
{
	const FBox VolumeBounds = GetVolumeBounds();
	if (!VolumeBounds.IsValid || !World) return false;

	TSet<FIntVector> AllStaticObstacles;
	ScanForAllStaticObstacles(AllStaticObstacles, World, VolumeBounds);

	const double TotalWorkAmount = static_cast<double>(AllStaticObstacles.Num()) + 2.0;
	FScopedSlowTask BakeTask(TotalWorkAmount, FText::FromString(TEXT("Baking Octree Data for ") + GetName() + "...")); 
	BakeTask.MakeDialog();

	TArray<FOctreeNode> BakeTimeNodePool;

	BakeTask.EnterProgressFrame(0.f, FText::FromString(TEXT("Building Octree... (1/3)")));
	const int32 BakeRootIndex = BakeTimeNodePool.AddDefaulted();
	BakeTimeNodePool[BakeRootIndex].Bounds = VolumeBounds;

	auto a = MapSize;
	
	for (const FIntVector& ObstacleVoxel : AllStaticObstacles)
	{
		BakeTask.EnterProgressFrame(1.f);
		if (BakeTask.ShouldCancel()) return false;
		InsertObstacleRecursive_Internal(BakeTimeNodePool, BakeRootIndex, ObstacleVoxel, 0, GetMaxOctreeDepth());
	}

	BakeTask.EnterProgressFrame(1.f, FText::FromString(TEXT("Optimizing Octree... (2/3)")));
	if (BakeTask.ShouldCancel()) return false;
	OptimizeAndCollapse_Internal(BakeTimeNodePool, BakeRootIndex);

	BakeTask.EnterProgressFrame(1.f, FText::FromString(TEXT("Precomputing Neighbors... (3/3)")));
	if (BakeTask.ShouldCancel()) return false;
	PrecomputeNeighbors_Internal(BakeTimeNodePool, VolumeBounds, GetMaxOctreeDepth());

	OutNodePool = BakeTimeNodePool;

	return true;
}

void AOctreeVoxelVolume::SaveOctreeToAsset(const TArray<FOctreeNode>& NodePoolToSave, const TMap<int32, FVolumePortalLinks>& PortalsToSave)
{
#if WITH_EDITOR
	FString PackageName, AssetName;
	if (!GetBakedAssetPath(PackageName, AssetName))
	{
		UE_LOG(LogTemp, Error, TEXT("SaveOctreeToAsset failed for '%s': Could not determine the data asset path."), *GetActorLabel());
		return;
	}

	UOctreeDataAsset* DataAssetToSave = GetBakedOctreeData().Get();
	if (!DataAssetToSave)
	{
		UE_LOG(LogTemp, Error, TEXT("SaveOctreeToAsset failed for '%s': The assigned data asset is missing."), *GetActorLabel());
		return;
	}
	
	int32 TotalPortalLinkCount = 0;
	if (PortalsToSave.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT(">>> VALIDATION WARNING for '%s': Saving asset with ZERO portals."), *GetActorLabel());
	}
	else
	{
		for (const auto& PortalNodePair : PortalsToSave)
		{
			const int32 MyNodeIndex = PortalNodePair.Key;
			const FVolumePortalLinks& LinksContainer = PortalNodePair.Value;
			TotalPortalLinkCount += LinksContainer.Links.Num();

			for (const FVolumePortalLink& PortalLink : LinksContainer.Links)
			{
				if (PortalLink.TargetVolume.IsNull())
				{
					UE_LOG(LogTemp, Error, TEXT(">>> VALIDATION FAILED for '%s': Portal for Node %d has a NULL TargetVolume!"), *GetActorLabel(), MyNodeIndex);
				}
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Saving data for '%s': %d nodes, %d nodes with portals, %d total portal links."), *GetActorLabel(), NodePoolToSave.Num(), PortalsToSave.Num(), TotalPortalLinkCount);

	DataAssetToSave->Modify();

	DataAssetToSave->PortalNodes = PortalsToSave;

	DataAssetToSave->SetAndCompressBakedData(NodePoolToSave, this->MapSize, this->GetMaxOctreeDepth());

	TArray<UObject*> AssetsToSave;
	AssetsToSave.Add(DataAssetToSave);

	const bool bSaved = UEditorAssetLibrary::SaveLoadedAssets(AssetsToSave, true);

	if (!bSaved)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to save UOctreeDataAsset '%s' to disk."), *AssetName);
	}
#endif
}

void AOctreeVoxelVolume::VoxelizeComponent(const FBox& ComponentBounds, const FCollisionObjectQueryParams& ObjectQueryParams, TSet<FIntVector>& OutStaticObstacles) const
{
	if (!ComponentBounds.IsValid) return;

	const FIntVector MinVoxel = WorldToVoxelIndex(ComponentBounds.Min);
	const FIntVector MaxVoxel = WorldToVoxelIndex(ComponentBounds.Max);

	FIntVector ClampedMin, ClampedMax;
	if (!ClampVoxelIndicesToVolume(MinVoxel, MaxVoxel, ClampedMin, ClampedMax))
	{
		return;
	}

	for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
	{
		for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
		{
			for (int32 x = ClampedMin.X; x <= ClampedMax.X; ++x)
			{
				const FIntVector VoxelIndex(x, y, z);
				if (OutStaticObstacles.Contains(VoxelIndex)) continue;

				if (IsVoxelOverlapping(VoxelIndex, ObjectQueryParams))
				{
					OutStaticObstacles.Add(VoxelIndex);
				}
			}
		}
	}
}

void AOctreeVoxelVolume::ScanForAllStaticObstacles(TSet<FIntVector>& OutStaticObstacles, UObject* WorldContextObject, const FBox& ScanWorldBounds) const
{
	OutStaticObstacles.Empty();
	UWorld* World = GEngine->GetWorldFromContextObject(WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	if (!World) return;

	FCollisionObjectQueryParams ObjectQueryParams;
	if (StaticObjectProfile && StaticObjectProfile->CollisionChannels.Num() > 0)
	{
		for (const TEnumAsByte<ECollisionChannel>& Channel : StaticObjectProfile->CollisionChannels)
		{
			ObjectQueryParams.AddObjectTypesToQuery(Channel);
		}
	}
	else
	{
		ObjectQueryParams.AddObjectTypesToQuery(ECollisionChannel::ECC_WorldStatic);
	}

	FCollisionQueryParams CollisionQueryParams(SCENE_QUERY_STAT(VoxelBakeOverlap), false);
	CollisionQueryParams.AddIgnoredActor(this);

	TArray<FOverlapResult> OverlapResults;
	World->OverlapMultiByObjectType(
		OverlapResults,
		ScanWorldBounds.GetCenter(),
		FQuat::Identity,
		ObjectQueryParams,
		FCollisionShape::MakeBox(ScanWorldBounds.GetExtent()),
		CollisionQueryParams
	);
	
	for (const FOverlapResult& Result : OverlapResults)
	{
		UPrimitiveComponent* PrimitiveComp = Result.GetComponent();

		if (IsValid(PrimitiveComp) && PrimitiveComp->IsQueryCollisionEnabled())
		{
			const bool bShouldBake = StaticObjectProfile ? StaticObjectProfile->DoesActorMatch(PrimitiveComp->GetOwner(), PrimitiveComp) : true;
			if (bShouldBake)
			{
				VoxelizeComponent(PrimitiveComp->Bounds.GetBox(), ObjectQueryParams, OutStaticObstacles);
			}
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

#if WITH_EDITORONLY_DATA
void AOctreeVoxelVolume::UpdateDebugObstacleVisualization()
{
	Super::UpdateDebugObstacleVisualization();

	if (!VolumeBox || bIsUpdatingVisualization)
	{
		return;
	}

	if (LoadState != EVolumeLoadState::Loaded || RootNodeIndex == -1 || !StaticNodePoolPtr || StaticNodePoolPtr->IsEmpty())
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
		FScopeLock Lock(&NodePoolCS);
		NodePoolCopy = *StaticNodePoolPtr;
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

					// 배치(Batch) 추가로 성능을 최적화합니다.
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

FVector AOctreeVoxelVolume::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
{
	const FVector VoxelSize(VoxelExtent);
	return GetVolumeBounds().Min + (FVector(VoxelIndex) * VoxelExtent) + (VoxelSize * 0.5f);
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
	if (LoadState != EVolumeLoadState::Loaded || RootNodeIndex == -1 || !StaticNodePoolPtr)
	{
		return -1;
	}
	
	int32 CurrentNodeIndex = RootNodeIndex;
	int32 CurrentDepth = 0;

	while (StaticNodePoolPtr->IsValidIndex(CurrentNodeIndex))
	{
		const FOctreeNode& Node = (*StaticNodePoolPtr)[CurrentNodeIndex];

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

void AOctreeVoxelVolume::DrawDebugDynamicObstaclesWithBoxes()
{
    if (DynamicObstacleBitmask.IsEmpty() || !GetWorld())
    {
        return;
    }

    const UWorld* World = GetWorld();
    const FBox VolumeWorldBounds = GetVolumeBounds();
    const FVector TotalSize = VolumeWorldBounds.GetSize();
    const float MinNodeSize = TotalSize.X / FMath::Pow(2.f, GetMaxOctreeDepth());
    const FVector VoxelSize(100);
	
    const FIntVector CalculatedMapSize(
        FMath::RoundToInt(TotalSize.X / VoxelSize.X),
        FMath::RoundToInt(TotalSize.Y / VoxelSize.Y),
        FMath::RoundToInt(TotalSize.Z / VoxelSize.Z)
    );

    if (CalculatedMapSize.X == 0 || CalculatedMapSize.Y == 0 || CalculatedMapSize.Z == 0) return;

    constexpr int32 BITS_PER_CHUNK = 128;

    for (int32 ChunkIndex = 0; ChunkIndex < DynamicObstacleBitmask.Num(); ++ChunkIndex)
    {
        const F128BitChunk& Chunk = DynamicObstacleBitmask[ChunkIndex];

        for (int32 BitInChunkIndex = 0; BitInChunkIndex < BITS_PER_CHUNK; ++BitInChunkIndex)
        {
            if (Chunk.GetBit(BitInChunkIndex))
            {
                const int32 FlatIndex = (ChunkIndex * BITS_PER_CHUNK) + BitInChunkIndex;
                
                const FIntVector VoxelIndex(
                    FlatIndex % CalculatedMapSize.X,
                    (FlatIndex / CalculatedMapSize.X) % CalculatedMapSize.Y,
                    FlatIndex / (CalculatedMapSize.X * CalculatedMapSize.Y)
                );

                const FVector VoxelWorldCenter = VolumeWorldBounds.Min + 
                                                 (FVector(VoxelIndex) * VoxelSize) + 
                                                 (VoxelSize * 0.5f);

                DrawDebugBox(
                    World,
                    VoxelWorldCenter,
                    VoxelSize * 0.5f,
                    FColor::Yellow,
                    false,
                    0.0f,
                    0,
                    2.0f
                );
            }
        }
    }
}

void AOctreeVoxelVolume::GetDebugNodeInfoRecursive(TArray<FDebugNodeInfo>& OutNodes, int32 NodeIndex, const FBox& NodeBounds, int32 CurrentDepth) const
{
	if (!StaticNodePoolPtr || StaticNodePoolPtr->IsEmpty()) return;

	if (!StaticNodePoolPtr->IsValidIndex(NodeIndex) || CurrentDepth > GetMaxOctreeDepth())
	{
		return;
	}

	{
		FScopeLock Lock(&NodePoolCS);
	
		const FOctreeNode& Node = (*StaticNodePoolPtr)[NodeIndex];

		// Add node info if it's a leaf or if we've reached the desired debug draw depth.
		if (Node.State != EOctreeNodeState::Mixed || CurrentDepth == GetMaxOctreeDepth())
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
}

#endif // WITH_EDITOR

void AOctreeVoxelVolume::OnDataAssetLoaded()
{
	if (AsyncLoadHandle.IsValid())
	{
		AsyncLoadHandle.Reset();
	}

	LoadedDataAsset = BakedOctreeData.Get();

	if (IsValid(LoadedDataAsset))
	{
		LoadedDataAsset->DecompressAndInitialize();
		if (LoadedDataAsset->NodePool.Num() > 0)
		{
			StaticNodePoolPtr = &LoadedDataAsset->NodePool;
			RootNodeIndex = 0;
			LoadState = EVolumeLoadState::Loaded;

			if (UWorld* World = GetWorld())
			{
				if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
				{
					NavSubsystem->UpdateVolumeDataAssetMapping(this);
					NavSubsystem->OnVolumeRegistered.Broadcast(this);
				}
			}
		}
		else
		{
			LoadState = EVolumeLoadState::Failed;
			UE_LOG(LogTemp, Error, TEXT("Octree Data Asset '%s' is empty or failed to decompress."), *LoadedDataAsset->GetName());
		}
	}
	else
	{
		LoadState = EVolumeLoadState::Failed;
		UE_LOG(LogTemp, Error, TEXT("Failed to load Octree Data Asset from path: %s"), *BakedOctreeData.ToString());
	}
}

void AOctreeVoxelVolume::RequestAsyncLoad()
{
	// Request loading only if a valid asset path is set and it's not already loaded/loading.
	if (BakedOctreeData.IsPending() && LoadState == EVolumeLoadState::Unloaded)
	{
		LoadState = EVolumeLoadState::Loading;

		FStreamableManager& StreamableManager = UAssetManager::Get().GetStreamableManager();
		AsyncLoadHandle = StreamableManager.RequestAsyncLoad(
			BakedOctreeData.ToSoftObjectPath(),
			FStreamableDelegate::CreateUObject(this, &AOctreeVoxelVolume::OnDataAssetLoaded)
		);
	}
	else if (BakedOctreeData.Get()) // Or if the asset is already loaded synchronously (e.g., in PIE)
	{
		OnDataAssetLoaded();
	}
}

FBox AOctreeVoxelVolume::GetVolumeBounds() const
{
	return VolumeBox ? VolumeBox->Bounds.GetBox() : FBox(ForceInit);
}

void AOctreeVoxelVolume::InsertObstacleRecursive_Internal(TArray<FOctreeNode>& InOutNodePool, int32 NodeIndex, const FIntVector& ObstacleVoxel, int32 CurrentDepth, int32 MaxDepth)
{
 	if (!InOutNodePool.IsValidIndex(NodeIndex))
	{
		return;
	}

	if (InOutNodePool[NodeIndex].State == EOctreeNodeState::Solid)
	{
		return;
	}

	if (CurrentDepth == MaxDepth)
	{
		InOutNodePool[NodeIndex].State = EOctreeNodeState::Solid;
		return;
	}

	if (InOutNodePool[NodeIndex].State == EOctreeNodeState::Empty)
	{
		InOutNodePool[NodeIndex].State = EOctreeNodeState::Mixed;
		const FBox CurrentNodeBounds = InOutNodePool[NodeIndex].Bounds;

		for (int i = 0; i < 8; ++i)
		{
			const int32 NewChildIndex = InOutNodePool.Emplace(FOctreeNode());
			InOutNodePool[NewChildIndex].Bounds = CalculateChildBounds(CurrentNodeBounds, i);
			InOutNodePool[NodeIndex].ChildrenIndices[i] = NewChildIndex;
		}
	}

	const FVector VoxelWorldPos = VoxelIndexToWorld(ObstacleVoxel);
	const int32 ChildOctant = GetOctantContainingVoxel(InOutNodePool[NodeIndex].Bounds, VoxelWorldPos);
	const int32 ChildNodeIndex = InOutNodePool[NodeIndex].ChildrenIndices[ChildOctant];

	if (ChildNodeIndex != -1)
	{
		InsertObstacleRecursive_Internal(InOutNodePool, ChildNodeIndex, ObstacleVoxel, CurrentDepth + 1, MaxDepth);
	}
}

int32 AOctreeVoxelVolume::FindLeafNode_Internal(const TArray<FOctreeNode>& InNodePool, int32 CurrentNodeIndex, const FVector& WorldPos, int32 MaxDepth, int32 CurrentDepth)
{
	while (InNodePool.IsValidIndex(CurrentNodeIndex))
	{
		const FOctreeNode& Node = InNodePool[CurrentNodeIndex];

		if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= MaxDepth)
		{
			return CurrentNodeIndex;
		}

		const int32 ChildOctant = GetOctantContainingVoxel(Node.Bounds, WorldPos);
		CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
		CurrentDepth++;
	}
	return -1;
}

void AOctreeVoxelVolume::OptimizeAndCollapse_Internal(TArray<FOctreeNode>& InOutNodePool, int32 NodeIndex)
{
	if (!InOutNodePool.IsValidIndex(NodeIndex)) return;

	FOctreeNode& Node = InOutNodePool[NodeIndex];
	if (Node.State != EOctreeNodeState::Mixed)
	{
		return;
	}

	for (int i = 0; i < 8; ++i)
	{
		if (Node.ChildrenIndices[i] != -1)
		{
			OptimizeAndCollapse_Internal(InOutNodePool, Node.ChildrenIndices[i]);
		}
	}

	const int32 FirstChildIndex = Node.ChildrenIndices[0];
	if (FirstChildIndex == -1 || !InOutNodePool.IsValidIndex(FirstChildIndex)) return;

	const EOctreeNodeState FirstChildState = InOutNodePool[FirstChildIndex].State;
	if (FirstChildState == EOctreeNodeState::Mixed)
	{
		return;
	}

	bool bAllChildrenAreSame = true;
	for (int i = 1; i < 8; ++i)
	{
		const int32 ChildNodeIndex = Node.ChildrenIndices[i];
		if (ChildNodeIndex == -1 || !InOutNodePool.IsValidIndex(ChildNodeIndex) || InOutNodePool[ChildNodeIndex].State != FirstChildState)
		{
			bAllChildrenAreSame = false;
			break;
		}
	}

	if (bAllChildrenAreSame)
	{
		FOctreeNode& ParentNodeToCollapse = InOutNodePool[NodeIndex];
		ParentNodeToCollapse.State = FirstChildState;

		for (int i = 0; i < 8; ++i)
		{
			ParentNodeToCollapse.ChildrenIndices[i] = -1;
		}
	}
}

#if WITH_EDITOR
void AOctreeVoxelVolume::PrecomputeNeighbors_Internal(TArray<FOctreeNode>& InOutNodePool, const FBox& TotalVolumeBounds, int32 MaxDepth)
{
	FScopedSlowTask NeighborTask(InOutNodePool.Num(), FText::FromString(TEXT("Precomputing Neighbors...")));
	NeighborTask.MakeDialog();

	constexpr float Epsilon = 1.0f;

	const int32 NumNodes = InOutNodePool.Num();
	for (int32 i = 0; i < NumNodes; ++i)
	{
		NeighborTask.EnterProgressFrame(1);
		if (NeighborTask.ShouldCancel()) return;

		if (InOutNodePool[i].State == EOctreeNodeState::Mixed)
		{
			continue;
		}

		InOutNodePool[i].MaxAllowedRadius = InOutNodePool[i].Bounds.GetExtent().GetMin();

		const FBox NodeBounds = InOutNodePool[i].Bounds;
		for (int32 DirIndex = 0; DirIndex < static_cast<int32>(ENeighborDirection::MAX); ++DirIndex)
		{
			FVector TestPoint = NodeBounds.GetCenter();
			const ENeighborDirection Dir = static_cast<ENeighborDirection>(DirIndex);

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

			if (TotalVolumeBounds.IsInside(TestPoint))
			{
				const int32 NeighborNodeIndex = FindLeafNode_Internal(InOutNodePool, 0, TestPoint, MaxDepth);
				InOutNodePool[i].Neighbors[DirIndex] = NeighborNodeIndex;
			}
			else
			{
				// TO-DO: Hierarchical Pathfinding should be implemented.
				InOutNodePool[i].Neighbors[DirIndex] = -1;
			}
		}
	}
}

void AOctreeVoxelVolume::ConnectPortals_Bake(
	TArray<FOctreeNode>& InOutMyNodePool,
	TMap<int32, FVolumePortalLinks>& OutPortalNodes,
	const TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>>& AllVolumeData,
	const TArray<AOctreeVoxelVolume*>& AllCreatedVolumes)
{
#if WITH_EDITOR
	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("ConnectPortals_Bake: Debug drawing failed because World is null."));
		return;
	}

	// Epsilon: A small value to step just outside the boundary.
	constexpr float Epsilon = 1.0f;
	int32 BoundaryNodeCount = 0;
	int32 CreatedPortalCount = 0;

	// Iterate through all nodes in the current volume.
	for (int32 MyNodeIndex = 0; MyNodeIndex < InOutMyNodePool.Num(); ++MyNodeIndex)
	{
		FOctreeNode& MyNode = InOutMyNodePool[MyNodeIndex];

		// Portals can only be created from empty nodes.
		if (MyNode.State != EOctreeNodeState::Empty)
		{
			continue;
		}

		// Check all 6 directions.
		for (int32 DirIndex = 0; DirIndex < static_cast<int32>(ENeighborDirection::MAX); ++DirIndex)
		{
			// Skip if there's already an internal neighbor (we only care about boundary nodes).
			if (MyNode.Neighbors[DirIndex] != -1)
			{
				continue;
			}

			BoundaryNodeCount++;
			const ENeighborDirection Direction = static_cast<ENeighborDirection>(DirIndex);

			FVector TestPoint = MyNode.Bounds.GetCenter();
			switch (Direction)
			{
			case ENeighborDirection::Right: TestPoint.X = MyNode.Bounds.Max.X + Epsilon; break;
			case ENeighborDirection::Left:  TestPoint.X = MyNode.Bounds.Min.X - Epsilon; break;
			case ENeighborDirection::Front: TestPoint.Y = MyNode.Bounds.Max.Y + Epsilon; break;
			case ENeighborDirection::Back:  TestPoint.Y = MyNode.Bounds.Min.Y - Epsilon; break;
			case ENeighborDirection::Up:    TestPoint.Z = MyNode.Bounds.Max.Z + Epsilon; break;
			case ENeighborDirection::Down:  TestPoint.Z = MyNode.Bounds.Min.Z - Epsilon; break;
			}

			// Find the neighboring volume by iterating through all created volumes.
			AOctreeVoxelVolume* NeighborVolume = nullptr;
			float MinDistanceSq = TNumericLimits<float>::Max();

			// Iterate through all created volumes to find the most suitable neighbor.
			for (AOctreeVoxelVolume* PotentialNeighbor : AllCreatedVolumes)
			{
				if (!PotentialNeighbor || PotentialNeighbor == this) continue;

				const FBox NeighborBounds = PotentialNeighbor->GetVolumeBounds();

				// 1. First, check if the test point is inside the potential neighbor's bounds.
				if (NeighborBounds.IsInsideOrOn(TestPoint))
				{
					// 2. Calculate the distance between the test point and the center of the potential neighbor.
					const float DistSq = FVector::DistSquared(TestPoint, NeighborBounds.GetCenter());

					// 3. Choose the volume with the closest center as the neighbor.
					//    This is a robust method for grid-aligned volumes of the same size.
					if (DistSq < MinDistanceSq)
					{
						MinDistanceSq = DistSq;
						NeighborVolume = PotentialNeighbor;
					}
				}
			}

			// If the found neighbor is too far away, ignore it.
			// This is an important defensive check to prevent incorrect neighbor selection
			// when volumes overlap or are arranged complexly.
			if (NeighborVolume)
			{
				// Assuming the volume is cubic, get the extent of one axis.
				const float VolumeHalfExtent = NeighborVolume->GetVolumeBounds().GetExtent().X;
				// A reasonable distance threshold (squared distance of 1.5 times the half-extent).
				// Since TestPoint is near the boundary, the distance to the center should be similar to the half-extent.
				const float DistanceThresholdSq = FMath::Square(VolumeHalfExtent * 1.5f);

				if (MinDistanceSq > DistanceThresholdSq)
				{
					// The found neighbor is too far away, so we consider it invalid and abandon the connection.
					UE_LOG(LogTemp, Warning, TEXT("Neighbor '%s' for '%s' was found, but it's too far (DistSq: %.f, ThresholdSq: %.f). Discarding connection."),
						*NeighborVolume->GetName(), *GetName(), MinDistanceSq, DistanceThresholdSq);
					NeighborVolume = nullptr;
				}
			}

			// If a suitable and valid neighbor volume was found, attempt to create a portal.
			if (NeighborVolume)
			{
				TSoftObjectPtr<UOctreeDataAsset> NeighborDataAssetPtr = NeighborVolume->GetBakedOctreeData();
				if (NeighborDataAssetPtr.IsNull())
				{
					UE_LOG(LogTemp, Error, TEXT("ConnectPortals_Bake CRITICAL: Neighbor volume '%s' does not have a valid BakedOctreeData asset!"), *NeighborVolume->GetName());
					continue;
				}

				const TArray<FOctreeNode>* NeighborNodePoolPtr = AllVolumeData.Find(NeighborVolume);
				if (!NeighborNodePoolPtr)
				{
					UE_LOG(LogTemp, Warning, TEXT("ConnectPortals_Bake: Found neighbor volume '%s', but its node data is not in the AllVolumeData map."), *NeighborVolume->GetName());
					continue;
				}

				// Find the corresponding leaf node in the neighbor volume at the test point.
				const int32 NeighborNodeIndex = NeighborVolume->FindLeafNode_Internal(*NeighborNodePoolPtr, 0, TestPoint, NeighborVolume->GetMaxOctreeDepth());

				if (NeighborNodePoolPtr->IsValidIndex(NeighborNodeIndex))
				{
					const FOctreeNode& NeighborNode = (*NeighborNodePoolPtr)[NeighborNodeIndex];

					// A portal can only be created if the neighbor node is also empty.
					if (NeighborNode.State == EOctreeNodeState::Empty)
					{
						const float PortalClearance = FMath::Min(MyNode.MaxAllowedRadius, NeighborNode.MaxAllowedRadius);

						FVolumePortalLink PortalLink;
						PortalLink.TargetVolume = NeighborDataAssetPtr;
						PortalLink.TargetNodeIndex = NeighborNodeIndex;
						PortalLink.MaxAllowedRadius = PortalClearance;
						FVolumePortalLinks& LinksContainer = OutPortalNodes.FindOrAdd(MyNodeIndex);
						LinksContainer.Links.Add(PortalLink);

						CreatedPortalCount++;
					}
					else
					{
						FString StateString = (NeighborNode.State == EOctreeNodeState::Solid) ? TEXT("Solid") : TEXT("Mixed");
						UE_LOG(LogTemp, Warning, TEXT(">>> FAILED: Portal creation for '%s' (Node %d) failed. Neighbor node %d in '%s' is not empty (State: %s)."),
							*GetName(), MyNodeIndex, *NeighborVolume->GetName(), NeighborNodeIndex, *StateString);

#if WITH_EDITOR
						DrawDebugBox(World, NeighborNode.Bounds.GetCenter(), NeighborNode.Bounds.GetExtent(), FColor::Red, false, 30.0f, 0, 10.0f);
#endif
					}
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT(">>> FAILED: Could not find a valid leaf node in neighbor volume '%s' at TestPoint(%s)."), *GetActorLabel(), *TestPoint.ToString());
				}
			}
		}
	}

	if (CreatedPortalCount > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("ConnectPortals_Bake SUCCESS for '%s': Found %d boundary nodes and created %d portals."), *GetActorLabel(), BoundaryNodeCount, CreatedPortalCount);
	}
#endif
}
#endif
