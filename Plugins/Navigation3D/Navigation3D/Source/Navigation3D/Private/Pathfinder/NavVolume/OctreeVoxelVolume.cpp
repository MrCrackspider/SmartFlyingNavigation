#include "Pathfinder/NavVolume/OctreeVoxelVolume.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "Misc/ScopedSlowTask.h"
#include "EditorAssetLibrary.h" 

// 베이킹에 필요한 헤더
#if WITH_EDITOR
#include "FileHelpers.h"
#endif

// SIMD 헤더 (동적 그리드용)
#include "emmintrin.h"
#include "ObjectTools.h"
#include "smmintrin.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Components/BoxComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Pathfinder/NavData/VoxelObstacleFilter.h"

AOctreeVoxelVolume::AOctreeVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;
	// Set default values
	MaxOctreeDepth = 7; // e.g., 128x128x128 volume -> 2^7=128
}

void AOctreeVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	// 1. Load baked static octree data
	if (BakedOctreeData)
	{
		StaticNodePool = BakedOctreeData->NodePool;
		if (StaticNodePool.Num() > 0)
		{
			RootNodeIndex = 0; // Assume root node is always at index 0
			UE_LOG(LogTemp, Log, TEXT("OctreeVoxelVolume: Loaded static octree with %d nodes."), StaticNodePool.Num());
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning,
		       TEXT("OctreeVoxelVolume: No BakedOctreeData asset assigned. Static layer will be empty."));
	}

	// 2. Initialize dynamic obstacle layer and start timer
	InitializeDynamicLayer();
	GetWorldTimerManager().SetTimer(DynamicObstacleTimer, this, &AOctreeVoxelVolume::UpdateDynamicObstacles, 2.f, true,
	                                0.1f);
}

void AOctreeVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

//======================================================================
// --- Core Interface Implementation ---
//======================================================================

bool AOctreeVoxelVolume::IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const
{
	const FIntVector AgentRadius(AgentRadiusInVoxels);
	const FIntVector MinCorner = VoxelIndex - AgentRadius;
	const FIntVector MaxCorner = VoxelIndex + AgentRadius;

	// Calculate query box in world coordinates
	const FVector QueryMinWorld = VoxelIndexToWorld(MinCorner);
	
	// Max corner should include the end point of the voxel, so use world position of index + 1
	const FVector QueryMaxWorld = VoxelIndexToWorld(MaxCorner + FIntVector(1, 1, 1));
	const FBox QueryBox(QueryMinWorld, QueryMaxWorld);

	// 1. Check static octree layer
	if (!IsRegionEmpty_StaticOctree(QueryBox))
	{
		return false; // 정적 지형에 막힘
	}

	// 2. Check dynamic grid layer
	if (!IsRegionEmpty_DynamicGrid(MinCorner, MaxCorner))
	{
		return false; // Blocked by dynamic obstacles
	}

	// If both layers are clear, it's walkable
	return true;
}

void AOctreeVoxelVolume::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
	OutObstacleIndices.Empty();

	// 1. Collect dynamic obstacle voxels
	if (DynamicObstacleBitmask.Num() > 0)
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

	// 2. Collect static obstacle voxels (octree traversal)
	if (RootNodeIndex != -1)
	{
		TSet<FIntVector> StaticObstacles; // 중복 방지
		// Recursively find Solid leaf nodes
		// ... (This part can be very slow, use only for debugging)
	}
}

bool AOctreeVoxelVolume::IsVoxelWalkable(const FIntVector& Index) const
{
	return IsRegionWalkable(Index, 0);
}

void AOctreeVoxelVolume::RegisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent)
{
	if (ObstacleComponent)
	{
		RegisteredObstacles.Add(ObstacleComponent);
	}
}

void AOctreeVoxelVolume::UnregisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent)
{
	if (ObstacleComponent)
	{
		UpdateVoxelStateForBounds(ObstacleComponent->GetLastKnownBounds(), false);
		RegisteredObstacles.Remove(ObstacleComponent);
	}
}

//======================================================================
// --- Baking Implementation ---
//======================================================================

// Helper function implementation
void AOctreeVoxelVolume::SafeRemoveInstances(UInstancedStaticMeshComponent* Component,
                                             TArray<int32>& InstanceIndicesToRemove)
{
	if (!Component || InstanceIndicesToRemove.Num() == 0)
	{
		return;
	}
	
	// Remove instances from largest index to smallest to keep other indices valid.
	InstanceIndicesToRemove.Sort([](const int32& A, const int32& B)
	{
		return A > B;
	});
	Component->RemoveInstances(InstanceIndicesToRemove);
}

void AOctreeVoxelVolume::UpdateDebugObstacleVisualization()
{
	Super::UpdateDebugObstacleVisualization();

	if (!VolumeBox)
	{
		return;
	}

	if (bIsUpdatingVisualization)
	{
		return;
	}

	bIsUpdatingVisualization = true;
	TWeakObjectPtr<AOctreeVoxelVolume> WeakThis(this);
	const FBox InitialBounds = VolumeBox->Bounds.GetBox();

	// 1. Collect node information to draw in a background thread
	Async(EAsyncExecution::ThreadPool, [WeakThis, InitialBounds]()
	{
		if (!WeakThis.IsValid() || WeakThis->RootNodeIndex == -1)
		{
			if (WeakThis.IsValid()) WeakThis->bIsUpdatingVisualization = false;
			return;
		}

		TArray<FDebugNodeInfo> NodesToDraw;
		WeakThis->GetDebugNodeInfoRecursive(NodesToDraw, WeakThis->RootNodeIndex, InitialBounds, 0);

		// 2. Update instances on the game thread
		AsyncTask(ENamedThreads::GameThread, [WeakThis, NodesToDraw = MoveTemp(NodesToDraw)]()
		{
			AOctreeVoxelVolume* StrongThis = WeakThis.Get();
			if (!StrongThis) return;

			// --- Clear instances from all components ---
			if (StrongThis->DebugSolidNodesComponent) StrongThis->DebugSolidNodesComponent->ClearInstances();
			if (StrongThis->DebugEmptyNodesComponent) StrongThis->DebugEmptyNodesComponent->ClearInstances();
			if (StrongThis->DebugMixedNodesComponent) StrongThis->DebugMixedNodesComponent->ClearInstances();

			if (NodesToDraw.Num() == 0)
			{
				StrongThis->bIsUpdatingVisualization = false;
				return;
			}

			// --- Collect "local" Transforms for nodes to draw ---
			TArray<FTransform> SolidTransforms, EmptyTransforms, MixedTransforms;

			for (const auto& NodeInfo : NodesToDraw)
			{
				const FVector Scale = NodeInfo.Bounds.GetSize() / 100.0f;
				const FVector WorldCenter = NodeInfo.Bounds.GetCenter();

				if (NodeInfo.State == EOctreeNodeState::Solid && StrongThis->DebugSolidNodesComponent)
				{
					// Calculate local coordinates relative to the "Solid" component's Transform
					const FTransform CompXform = StrongThis->DebugSolidNodesComponent->GetComponentTransform();
					const FVector LocalCenter = CompXform.InverseTransformPosition(WorldCenter);
					SolidTransforms.Emplace(FRotator::ZeroRotator, LocalCenter, Scale);
				}
				else if (NodeInfo.State == EOctreeNodeState::Empty && StrongThis->DebugEmptyNodesComponent)
				{
					// Calculate local coordinates relative to the "Empty" component's Transform
					const FTransform CompXform = StrongThis->DebugEmptyNodesComponent->GetComponentTransform();
					const FVector LocalCenter = CompXform.InverseTransformPosition(WorldCenter);
					EmptyTransforms.Emplace(FRotator::ZeroRotator, LocalCenter, Scale);
				}
				else if (NodeInfo.State == EOctreeNodeState::Mixed && StrongThis->DebugMixedNodesComponent)
				{
					// Calculate local coordinates relative to the "Mixed" component's Transform
					const FTransform CompXform = StrongThis->DebugMixedNodesComponent->GetComponentTransform();
					const FVector LocalCenter = CompXform.InverseTransformPosition(WorldCenter);
					MixedTransforms.Emplace(FRotator::ZeroRotator, LocalCenter, Scale);
				}
			}

			// --- Batch add instances by state (bWorldSpace = false) ---
			if (StrongThis->DebugSolidNodesComponent && SolidTransforms.Num() > 0)
			{
				StrongThis->DebugSolidNodesComponent->AddInstances(SolidTransforms, false, false);
			}
			if (StrongThis->DebugEmptyNodesComponent && EmptyTransforms.Num() > 0)
			{
				StrongThis->DebugEmptyNodesComponent->AddInstances(EmptyTransforms, false, false);
			}
			if (StrongThis->DebugMixedNodesComponent && MixedTransforms.Num() > 0)
			{
				StrongThis->DebugMixedNodesComponent->AddInstances(MixedTransforms, false, false);
			}

			StrongThis->bIsUpdatingVisualization = false;
		});
	});
}

void AOctreeVoxelVolume::GetDebugNodeInfoRecursive(TArray<FDebugNodeInfo>& OutNodes, int32 NodeIndex,
                                                   const FBox& NodeBounds, int32 CurrentDepth) const
{
	if (!StaticNodePool.IsValidIndex(NodeIndex) || CurrentDepth > MaxOctreeDepth) return;

	const FOctreeNode& Node = StaticNodePool[NodeIndex];

	// Add node info if it's a leaf node or at debug max depth
	if (Node.State != EOctreeNodeState::Mixed || CurrentDepth == MaxOctreeDepth)
	{
		OutNodes.Add({NodeIndex, NodeBounds, Node.State});
	}
	else // If Mixed state and can go deeper, traverse children
	{
		for (int i = 0; i < 8; ++i)
		{
			if (Node.ChildrenIndices[i] != -1)
			{
				GetDebugNodeInfoRecursive(OutNodes, Node.ChildrenIndices[i], CalculateChildBounds(NodeBounds, i),
				                          CurrentDepth + 1);
			}
		}
	}
} 

#if WITH_EDITOR
void AOctreeVoxelVolume::BakeStaticData()
{
	Super::BakeStaticData();

	// --- 1. Prepare and scan for obstacles ---
	TSet<FIntVector> AllStaticObstacles;
	ScanForAllStaticObstacles(AllStaticObstacles);

	const FBox VolumeBounds = VolumeBox->Bounds.GetBox();
	if (!VolumeBounds.IsValid)
	{
		UE_LOG(LogTemp, Error, TEXT("Baking failed: Volume has invalid bounds."));
		return;
	}

	// --- 2. Set up UI progress bar ---
	const double TotalWorkAmount = static_cast<double>(AllStaticObstacles.Num()) + 3.0; 
	FScopedSlowTask BakeTask(TotalWorkAmount, FText::FromString(TEXT("Baking Octree Data...")));
	BakeTask.MakeDialog(); // UI 대화상자 표시

	// --- 3. Build Octree ---
	BakeTask.EnterProgressFrame(0.f, FText::FromString(TEXT("Building Octree... (1/3)")));

	StaticNodePool.Empty();
	RootNodeIndex = StaticNodePool.Emplace(FOctreeNode()); // 루트 노드 생성

	StaticNodePool[RootNodeIndex].Bounds = VolumeBounds;
	
	for (const FIntVector& ObstacleVoxel : AllStaticObstacles)
	{
		BakeTask.EnterProgressFrame(1.f);
		if (BakeTask.ShouldCancel()) return;

		InsertObstacleRecursive(RootNodeIndex, ObstacleVoxel, 0); // <-- 이렇게 수정합니다.
	}

	// --- 4. Optimize Octree ---
	BakeTask.EnterProgressFrame(1.f, FText::FromString(TEXT("Optimizing Octree...(2/3)")));
	if (BakeTask.ShouldCancel()) return;

	OptimizeAndCollapse(RootNodeIndex, VolumeBounds);

	BakeTask.EnterProgressFrame(1.f, FText::FromString(TEXT("Precomputing Neighbors...(2.5/3)")));
	if (BakeTask.ShouldCancel()) return;
	PrecomputeNeighbors();
	
	// --- 5. Save to asset file ---
	BakeTask.EnterProgressFrame(1.f, FText::FromString(TEXT("Saving data asset...(3/3)")));
	if (BakeTask.ShouldCancel()) return;

	SaveOctreeToAsset(); // 헬퍼 함수 호출로 깔끔하게 처리

	UE_LOG(LogTemp, Log, TEXT("Baking complete! Final Octree has %d nodes."), StaticNodePool.Num());
}

void AOctreeVoxelVolume::ScanForAllStaticObstacles(TSet<FIntVector>& OutStaticObstacles) const
{
	OutStaticObstacles.Empty();

	// 1. [Scan Target Setup] Determine object types to scan based on StaticObjectFilter.
	TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypesToQuery;
	if (StaticObjectFilter)
	{
		// If filter is set: use channels defined in the filter.
		UE_LOG(LogTemp, Log, TEXT("Using custom 'Static Object Filter' for baking."));
		for (const TEnumAsByte<ECollisionChannel>& Channel : StaticObjectFilter->CollisionChannels)
		{
			ObjectTypesToQuery.Add(UEngineTypes::ConvertToObjectType(Channel));
		}
		
		if (ObjectTypesToQuery.Num() == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("ScanForAllStaticObstacles: The assigned 'Static Object Filter' has no Collision Channels specified. No objects were scanned."));
			return;
		}
	}
	else
	{
		// If no filter: use default WorldStatic.
		UE_LOG(LogTemp, Log, TEXT("'Static Object Filter' not set. Falling back to default: scanning 'WorldStatic' objects."));
		ObjectTypesToQuery.Add(UEngineTypes::ConvertToObjectType(ECollisionChannel::ECC_WorldStatic));
	}
	
	// 2. [Object Scan] Find all actors of determined types within the volume.
	const FBox VolumeWorldBounds = GetComponentsBoundingBox(true, true);
	TArray<AActor*> OverlappingActors;
	UKismetSystemLibrary::BoxOverlapActors(GetWorld(), VolumeWorldBounds.GetCenter(), VolumeWorldBounds.GetExtent(),
	                                       ObjectTypesToQuery, AActor::StaticClass(), BakeExcludeActors, OverlappingActors);

	if (OverlappingActors.Num() == 0)
	{
		UE_LOG(LogTemp, Log, TEXT("ScanForAllStaticObstacles: No overlapping actors found for the specified query."));
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("ScanForAllStaticObstacles: Found %d potential actors. Applying filter and checking voxels..."), OverlappingActors.Num());

	// 3. [Filtering and Voxelization] Final check and voxelization for each actor.
	for (AActor* Actor : OverlappingActors)
	{
		if (!Actor) continue;
		
		TArray<UPrimitiveComponent*> PrimitiveComponents;
		Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);

		for (UPrimitiveComponent* Comp : PrimitiveComponents)
		{
			if (!Comp || !Comp->IsQueryCollisionEnabled()) continue;
			
			bool bShouldBake;
			if (StaticObjectFilter)
			{
				// If filter is present, bake only if it matches the filter.
				bShouldBake = StaticObjectFilter->DoesActorMatch(Actor, Comp);
			}
			else
			{
				bShouldBake = true;
			}

			if (bShouldBake)
			{
				// --- Proceed with voxelization for confirmed bake targets ---
				const FBox ComponentBounds = Comp->Bounds.GetBox();
				FIntVector MinVoxel, MaxVoxel;
				FIntVector MinVoxelUnclamped = WorldToVoxelIndex(ComponentBounds.Min);
				FIntVector MaxVoxelUnclamped = WorldToVoxelIndex(ComponentBounds.Max);
				if (!ClampVoxelIndicesToVolume(MinVoxelUnclamped, MaxVoxelUnclamped, MinVoxel, MaxVoxel)) continue;

				for (int32 z = MinVoxel.Z; z <= MaxVoxel.Z; ++z)
				{
					for (int32 y = MinVoxel.Y; y <= MaxVoxel.Y; ++y)
					{
						for (int32 x = MinVoxel.X; x <= MaxVoxel.X; ++x)
						{
							const FIntVector VoxelIndex(x, y, z);
							if (OutStaticObstacles.Contains(VoxelIndex)) continue;
							if (CheckVoxelOverlap(VoxelIndex, Comp))
							{
								OutStaticObstacles.Add(VoxelIndex);
							}
						}
					}
				}
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Scan complete. Baked %d unique static obstacle voxels."), OutStaticObstacles.Num());
}

void AOctreeVoxelVolume::InsertObstacleRecursive(int32 NodeIndex, // <-- const FBox& NodeBounds 인자를 제거합니다!
												 const FIntVector& ObstacleVoxel, int32 CurrentDepth)
{
	
	// 1. Validate NodeIndex (should always be valid, but defensive coding)
    if (!StaticNodePool.IsValidIndex(NodeIndex))
    {
        UE_LOG(LogTemp, Error, TEXT("InsertObstacleRecursive: Invalid NodeIndex %d"), NodeIndex);
        return;
    }

	// 2. Get a reference to the current node.
	//    Note: This reference might become invalid if StaticNodePool.Emplace() causes a reallocation.
	//    We will re-acquire it safely after potential reallocations..
    FOctreeNode& CurrentNodeRef = StaticNodePool[NodeIndex]; 
    const FBox CurrentNodeBounds = CurrentNodeRef.Bounds; // <-- 노드에 저장된 Bounds를 사용합니다.

	// 3. If node state is Solid, no further action needed.
    if (CurrentNodeRef.State == EOctreeNodeState::Solid)
    {
        return;
    }

	// 4. If max depth reached, mark current node as Solid.
    if (CurrentDepth == MaxOctreeDepth)
    {
        CurrentNodeRef.State = EOctreeNodeState::Solid;
        return;
    }

	// 5. If the current node is in the Empty state, change it to Mixed and create child nodes.
	//    This block may trigger reallocation of StaticNodePool,
	//    which can invalidate the previously obtained CurrentNodeRef reference.
	//    Therefore, access StaticNodePool[NodeIndex] directly to ensure safety.
    if (CurrentNodeRef.State == EOctreeNodeState::Empty)
    {
        StaticNodePool[NodeIndex].State = EOctreeNodeState::Mixed; 

       
        for (int i = 0; i < 8; ++i)
        {
            const int32 NewChildIndex = StaticNodePool.Emplace(FOctreeNode());
            StaticNodePool[NewChildIndex].Bounds = CalculateChildBounds(CurrentNodeBounds, i);
            StaticNodePool[NodeIndex].ChildrenIndices[i] = NewChildIndex;
        }
    }
	
    FOctreeNode& Node = StaticNodePool[NodeIndex]; 

    const FVector VoxelWorldPos = VoxelIndexToWorld(ObstacleVoxel);
    const int32 ChildOctant = GetOctantContainingVoxel(Node.Bounds, VoxelWorldPos); 

    const int32 ChildNodeIndex = Node.ChildrenIndices[ChildOctant];
    if (ChildNodeIndex != -1)
    {
        InsertObstacleRecursive(ChildNodeIndex, ObstacleVoxel, CurrentDepth + 1); 
    }
    else
    {
        UE_LOG(LogTemp, Error,
               TEXT(
                   "InsertObstacleRecursive: Invalid child node index (-1) encountered. This should not happen after the fix."
               ));
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
		if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= MaxOctreeDepth)
		{
			return CurrentNodeIndex;
		}

		const FVector VoxelWorldPos = WorldPos; // Already in world coordinates
		const int32 ChildOctant = GetOctantContainingVoxel(Node.Bounds, VoxelWorldPos);
		CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
		CurrentDepth++;
	}
	return -1;
}

void AOctreeVoxelVolume::PrecomputeNeighbors()
{
	FScopedSlowTask NeighborTask(StaticNodePool.Num(), FText::FromString(TEXT("Precomputing Neighbors...")));
	NeighborTask.MakeDialog();

	const FBox TotalVolumeBounds = VolumeBox->Bounds.GetBox();
	const float Epsilon = 1.0f;

	for (int32 i = 0; i < StaticNodePool.Num(); ++i)
	{
		NeighborTask.EnterProgressFrame(1);
		if (NeighborTask.ShouldCancel()) return;

		// 리프 노드가 아닌 경우(Mixed)는 이웃을 계산할 필요가 없습니다.
		if (StaticNodePool[i].State == EOctreeNodeState::Mixed)
		{
			continue;
		}
		StaticNodePool[i].MaxAllowedRadius = StaticNodePool[i].Bounds.GetExtent().GetMin();
		
		const FBox NodeBounds = StaticNodePool[i].Bounds;

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
			}

			if (TotalVolumeBounds.IsInside(TestPoint))
			{
				const int32 NeighborNodeIndex = FindLeafNodeAt_BakeTime(TestPoint);
				StaticNodePool[i].Neighbors[DirIndex] = NeighborNodeIndex;
			}
			else
			{
				StaticNodePool[i].Neighbors[DirIndex] = -1; // 볼륨 밖은 이웃이 없음
			}
		}
	}
}

void AOctreeVoxelVolume::OptimizeAndCollapse(int32 NodeIndex, const FBox& NodeBounds)
{
	if (!StaticNodePool.IsValidIndex(NodeIndex))
	{
		return;
	}

	FOctreeNode& Node = StaticNodePool[NodeIndex];
	if (Node.State != EOctreeNodeState::Mixed)
	{
		return; 
	}
	
	for (int i = 0; i < 8; ++i)
	{
		const int32 ChildNodeIndex = Node.ChildrenIndices[i];
		if (ChildNodeIndex != -1)
		{
			OptimizeAndCollapse(ChildNodeIndex, CalculateChildBounds(NodeBounds, i));
		}
	}

	const int32 FirstChildIndex = Node.ChildrenIndices[0];
	if (FirstChildIndex == -1)
	{
		return;
	}

	
	const EOctreeNodeState FirstChildState = StaticNodePool[Node.ChildrenIndices[0]].State;
	if (FirstChildState == EOctreeNodeState::Mixed) return;

	bool bAllChildrenAreSame = true;
	for (int i = 1; i < 8; ++i)
	{
		const int32 ChildNodeIndex = Node.ChildrenIndices[i];
		if (ChildNodeIndex == -1 || StaticNodePool[ChildNodeIndex].State != FirstChildState || StaticNodePool[
			ChildNodeIndex].State == EOctreeNodeState::Mixed)
		{
			bAllChildrenAreSame = false;
			break;
		}
	}
	
	if (bAllChildrenAreSame)
	{
		Node.State = FirstChildState;
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
	default: return FBox{};
	}
}

int32 AOctreeVoxelVolume::GetOctantContainingVoxel(const FBox& NodeBounds, const FVector& VoxelWorldPos) const
{
	int32 Octant = 0;
	const FVector Center = NodeBounds.GetCenter();
	if (VoxelWorldPos.X >= Center.X) Octant |= 1; 
	if (VoxelWorldPos.Y >= Center.Y) Octant |= 2; 
	if (VoxelWorldPos.Z >= Center.Z) Octant |= 4; 
	return Octant;
}

// AOctreeVoxelVolume.cpp

#include "Engine/World.h" // GetWorld()와 UWorld를 사용하기 위해 필요합니다.

bool AOctreeVoxelVolume::GetBakedAssetPath(FString& OutPackagePath, FString& OutAssetName) const
{
	if (BakedDataSaveDirectory.Path.IsEmpty())
	{
		UE_LOG(LogTemp, Error, TEXT("Cannot get asset path: 'Baked Data Save Directory' is not set."));
		return false;
	}

	// ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
	// *** 핵심 수정 부분 시작 ***
	// ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼

	// 1. 현재 월드(레벨)를 가져옵니다.
	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("Cannot get asset path: World context is not valid."));
		return false;
	}

	// 2. 월드에서 맵 이름을 가져옵니다.
	const FString RawLevelName = World->GetMapName();

	// 3. PIE (Play-In-Editor) 접두사("UEDPIE_0_")를 제거하여 순수한 레벨 이름을 얻습니다.
	const FString CleanLevelName = UWorld::RemovePIEPrefix(RawLevelName);

	// 4. 최종 에셋 이름을 "DA_레벨이름_액터이름" 형식으로 구성합니다.
	OutAssetName = FString::Printf(TEXT("DA_%s_%s"), *CleanLevelName, *GetActorLabel());

	// ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
	// *** 핵심 수정 부분 끝 ***
	// ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

	// UPROPERTY에 저장된 상대 경로 앞에 "/Game/"을 붙여 유효한 패키지 디렉토리 경로를 생성합니다.
	const FString PackageDirectory = FPaths::Combine(TEXT("/Game"), BakedDataSaveDirectory.Path);

	// 패키지 디렉토리와 에셋 이름을 결합하여 최종 패키지 경로를 만듭니다.
	OutPackagePath = FPaths::Combine(PackageDirectory, OutAssetName);

	return true;
}

void AOctreeVoxelVolume::SaveOctreeToAsset()
{
	FString OutPackageName, AssetName;
	if (!GetBakedAssetPath(OutPackageName, AssetName))
	{
		return;
	}
	
	 FString DirectoryPath = FPaths::GetPath(OutPackageName);
     UEditorAssetLibrary::MakeDirectory(DirectoryPath);

	{
		FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
		FAssetData ExistingAssetData = AssetRegistryModule.Get().GetAssetByObjectPath(FName(*OutPackageName));

		if (ExistingAssetData.IsValid())
		{
			UE_LOG(LogTemp, Log, TEXT("Found existing asset at: %s. Attempting to delete."), *OutPackageName);
            
			// 2. 존재하는 에셋 데이터를 실제 UObject로 로드합니다.
			UObject* ExistingObject = ExistingAssetData.GetAsset();
			if (ExistingObject)
			{
				// 3. ObjectTools를 사용하여 에셋을 삭제합니다. 두 번째 인자 false는 '삭제 확인' 팝업을 띄우지 않겠다는 의미입니다.
				TArray<UObject*> ObjectsToDelete;
				ObjectsToDelete.Add(ExistingObject);
				ObjectTools::DeleteObjects(ObjectsToDelete, false);
				UE_LOG(LogTemp, Log, TEXT("Successfully deleted old asset."));
			}
		}
	}
	
	UPackage* Package = CreatePackage(*OutPackageName);
	Package->FullyLoad();

	UOctreeDataAsset* NewOctreeData = NewObject<UOctreeDataAsset>(
		Package, FName(*AssetName), RF_Public | RF_Standalone
	);
	if (!NewOctreeData)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create new UOctreeDataAsset object for package: %s"), *OutPackageName);
		return;
	}
	
	NewOctreeData->Modify();

	NewOctreeData->NodePool = StaticNodePool;
	NewOctreeData->BakedMapSize = this->MapSize;
	NewOctreeData->BakedMaxOctreeDepth = this->MaxOctreeDepth;

	FAssetRegistryModule::AssetCreated(NewOctreeData);

	TArray<UPackage*> PackagesToSave;
	PackagesToSave.Add(Package);

	if (UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true))
	{
		UE_LOG(LogTemp, Log, TEXT("Successfully saved Octree data to: %s.uasset"), *OutPackageName);
		this->BakedOctreeData = NewOctreeData; 
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to save Octree data package: %s"), *OutPackageName);
	}
}
#endif

void AOctreeVoxelVolume::InitializeDynamicLayer()
{
	check(MapSize.X > 0 && MapSize.X % 128 == 0);
	check(MapSize.Y > 0);
	check(MapSize.Z > 0);
	const int32 NumChunks128 = (MapSize.X / 128) * MapSize.Y * MapSize.Z;
	DynamicObstacleBitmask.Empty(NumChunks128);
	DynamicObstacleBitmask.AddZeroed(NumChunks128);
}

void AOctreeVoxelVolume::UpdateDynamicObstacles()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(VoxelVolume_UpdateDynamicObstacles_Optimized);

	for (UDynamicObstacleComponent* Comp : RegisteredObstacles)
	{
		if (!Comp) continue;
		
		// 컴포넌트가 움직였을 때("Dirty"일 때)만 업데이트합니다.
		if (Comp->IsDirty())
		{
			// 1. 이전 위치의 복셀들을 비웁니다.
			UpdateVoxelStateForBounds(Comp->GetLastKnownBounds(), false);

			// 2. 현재 위치의 경계 상자를 계산하고, 그 위치에 복셀들을 채웁니다.
			const FBox CurrentBounds = Comp->GetCurrentBounds();
			UpdateVoxelStateForBounds(CurrentBounds, true);

			// 3. 마지막 위치 정보를 현재로 업데이트하고, Dirty 플래그를 내립니다.
			Comp->UpdateLastKnownBounds();
			Comp->ClearDirtyFlag();
		}
	}
}

void AOctreeVoxelVolume::UpdateVoxelStateForBounds(const FBox& InBounds, bool bAsObstacle)
{
	if (!InBounds.IsValid) return;

	FIntVector MinVoxelIndex = WorldToVoxelIndex(InBounds.Min);
	FIntVector MaxVoxelIndex = WorldToVoxelIndex(InBounds.Max);

	FIntVector ClampedMin, ClampedMax;
	if (!ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex, ClampedMin, ClampedMax)) return;

	for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
	{
		for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
		{
			for (int32 x = ClampedMin.X; x <= ClampedMax.X; ++x)
			{
				UpdateDynamicVoxelState({x, y, z}, bAsObstacle);
			}
		}
	}
}

void AOctreeVoxelVolume::ApplyDynamicObstacleChanges(const TSet<FIntVector>& NewFrameObstacles)
{
	const TSet<FIntVector> RemovedObstacles = CurrentDynamicObstacles.Difference(NewFrameObstacles);
	for (const FIntVector& Index : RemovedObstacles)
	{
		UpdateDynamicVoxelState(Index, false);
	}

	const TSet<FIntVector> AddedObstacles = NewFrameObstacles.Difference(CurrentDynamicObstacles);
	for (const FIntVector& Index : AddedObstacles)
	{
		UpdateDynamicVoxelState(Index, true);
	}
}

void AOctreeVoxelVolume::UpdateDynamicVoxelState(const FIntVector& Index, bool bIsObstacle)
{
	if (!IsIndexValid(Index)) return;

	const int32 ChunksPerRow = MapSize.X / 128;
	const int32 ChunkIndex = (Index.Z * MapSize.Y + Index.Y) * ChunksPerRow + (Index.X / 128);
	const int32 BitIndex = Index.X % 128;

	if (!DynamicObstacleBitmask.IsValidIndex(ChunkIndex)) return;

	F128BitChunk& TargetChunk = DynamicObstacleBitmask[ChunkIndex];
	const int32 WordIndex = BitIndex / 64;
	const uint64 BitMask = 1ULL << (BitIndex % 64);

	if (bIsObstacle)
	{
		TargetChunk.Words[WordIndex] |= BitMask;
	}
	else
	{
		TargetChunk.Words[WordIndex] &= ~BitMask;
	}
}

bool AOctreeVoxelVolume::IsRegionEmpty_StaticOctree(const FBox& QueryBox) const
{
	if (RootNodeIndex == -1) return true; 

	const FBox VolumeBounds = GetComponentsBoundingBox(true, true);
	return IsRegionEmptyRecursive(RootNodeIndex, VolumeBounds, QueryBox);
}

bool AOctreeVoxelVolume::IsRegionEmptyRecursive(int32 NodeIndex, const FBox& NodeBounds, const FBox& QueryBox) const
{
	if (!NodeBounds.Intersect(QueryBox))
	{
		return true;
	}

	if (!StaticNodePool.IsValidIndex(NodeIndex)) return true;
	const FOctreeNode& Node = StaticNodePool[NodeIndex];

	if (Node.State == EOctreeNodeState::Solid)
	{
		return false; 
	}
	if (Node.State == EOctreeNodeState::Empty)
	{
		return true;
	}

	// Mixed 상태: 자식 노드 순회
	for (int i = 0; i < 8; ++i)
	{
		if (Node.ChildrenIndices[i] != -1)
		{
			const FBox ChildBounds = CalculateChildBounds(NodeBounds, i);
			if (!IsRegionEmptyRecursive(Node.ChildrenIndices[i], ChildBounds, QueryBox))
			{
				return false;
			}
		}
	}

	return true;
}

bool AOctreeVoxelVolume::IsRegionEmpty_DynamicGrid(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
{
	const FIntVector ClampedMin(FMath::Max(MinCorner.X, 0), FMath::Max(MinCorner.Y, 0), FMath::Max(MinCorner.Z, 0));
	const FIntVector ClampedMax(FMath::Min(MaxCorner.X, MapSize.X - 1), FMath::Min(MaxCorner.Y, MapSize.Y - 1),
	                            FMath::Min(MaxCorner.Z, MapSize.Z - 1));

	if (ClampedMin.X > ClampedMax.X || ClampedMin.Y > ClampedMax.Y || ClampedMin.Z > ClampedMax.Z) return true;
	if (DynamicObstacleBitmask.Num() == 0) return true;

	auto CreateMask = [](int32 StartBit, int32 EndBit) -> __m128i
	{
		uint64 LowMask = (StartBit < 64) ? (~0ULL << StartBit) : 0;
		if (EndBit < 64) LowMask &= (~0ULL >> (63 - EndBit));
		uint64 HighMask = (EndBit >= 64) ? (~0ULL >> (127 - EndBit)) : 0;
		if (StartBit >= 64) HighMask &= (~0ULL << (StartBit - 64));
		return _mm_set_epi64x(HighMask, LowMask);
	};

	const int32 ChunksPerRow = MapSize.X / 128;
	for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
	{
		for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
		{
			const int32 RowStartChunkIndex = (z * MapSize.Y + y) * ChunksPerRow;
			const int32 StartChunkIndex = RowStartChunkIndex + ClampedMin.X / 128;
			const int32 EndChunkIndex = RowStartChunkIndex + ClampedMax.X / 128;
			const int32 StartBitInChunk = ClampedMin.X % 128;
			const int32 EndBitInChunk = ClampedMax.X % 128;

			if (StartChunkIndex == EndChunkIndex)
			{
				if (!DynamicObstacleBitmask.IsValidIndex(StartChunkIndex)) continue;
				const __m128i Mask = CreateMask(StartBitInChunk, EndBitInChunk);
				const __m128i ChunkData = _mm_loadu_si128(
					reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[StartChunkIndex]));
				if (_mm_testz_si128(ChunkData, Mask) == 0) return false;
			}
			else
			{
				if (DynamicObstacleBitmask.IsValidIndex(StartChunkIndex))
				{
					__m128i StartMask = CreateMask(StartBitInChunk, 127);
					__m128i ChunkData = _mm_loadu_si128(
						reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[StartChunkIndex]));
					if (_mm_testz_si128(ChunkData, StartMask) == 0) return false;
				}

				for (int32 i = StartChunkIndex + 1; i < EndChunkIndex; ++i)
				{
					if (!DynamicObstacleBitmask.IsValidIndex(i)) continue;
					const __m128i MiddleChunkData = _mm_loadu_si128(
						reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[i]));
					if (_mm_testz_si128(MiddleChunkData, _mm_set1_epi32(-1)) == 0) return false;
				}

				if (DynamicObstacleBitmask.IsValidIndex(EndChunkIndex))
				{
					__m128i EndMask = CreateMask(0, EndBitInChunk);
					__m128i ChunkData = _mm_loadu_si128(
						reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[EndChunkIndex]));
					if (_mm_testz_si128(ChunkData, EndMask) == 0) return false;
				}
			}
		}
	}
	return true;
}

void AOctreeVoxelVolume::GetCombinedBitmask(TArray<F128BitChunk>& CombinedBitmask) const
{
	CombinedBitmask.Empty();

	if (StaticObstacleBitmask.Num() == DynamicObstacleBitmask.Num())
	{
		CombinedBitmask = StaticObstacleBitmask;
		const TArray<F128BitChunk>& DynamicBitmask = DynamicObstacleBitmask;

		for (int32 i = 0; i < CombinedBitmask.Num(); ++i)
		{
			// SIMD를 사용하여 OR 연산을 수행합니다.
			const __m128i StaticData = _mm_load_si128(reinterpret_cast<const __m128i*>(&CombinedBitmask[i]));
			const __m128i DynamicData = _mm_load_si128(reinterpret_cast<const __m128i*>(&DynamicBitmask[i]));
			_mm_store_si128(reinterpret_cast<__m128i*>(&CombinedBitmask[i]), _mm_or_si128(StaticData, DynamicData));
		}
	}
}