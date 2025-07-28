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
#include "LandscapeComponent.h" 
#include "Landscape.h"
#include "LandscapeStreamingProxy.h"
#include "LandscapeSubsystem.h"
#include "EngineUtils.h"
#include "LandscapeInfo.h"

AOctreeVoxelVolume::AOctreeVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;

	SetIsSpatiallyLoaded(true);

	// Default value for octree depth. For a 128^3 volume, a depth of 7 provides voxel-level precision (2^7 = 128
}

//~======================================================================
// AActor Overrides
//~======================================================================

void AOctreeVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	if (UWorld* World = GetWorld())
	{
		if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
		{
			NavSubsystem->RegisterVolumeChunk(this);
		}
	}

	RequestAsyncLoad();

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
	// --- 1. 사전 검사: 데이터가 준비되었고, 위치가 유효한 범위 내에 있는지 확인 ---
	if (LoadState != EVolumeLoadState::Loaded)
	{
		return true; // 데이터가 없으면 안전을 위해 막혔다고 처리
	}
	if (!GetVolumeBounds().IsInsideOrOn(WorldLocation))
	{
		return true; // 볼륨 범위를 벗어남
	}

	// --- 2. 동적 장애물 레이어 확인 (빠른 비트마스크 검사) ---
	// 이 로직은 IsLocationBlockedByDynamicObstacle 함수와 동일하지만, 여기서 직접 처리하는 것이 더 효율적입니다.
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
			// 동적 장애물에 의해 막힘
			return true;
		}
	}

	// --- 3. 정적 장애물 레이어 확인 (수정된 옥트리 순회) ---
	if (!StaticNodePoolPtr || RootNodeIndex == -1)
	{
		// 정적 데이터가 없으면 동적 장애물 결과만 신뢰합니다.
		return false;
	}

	int32 CurrentNodeIndex = RootNodeIndex;
	int32 CurrentDepth = 0;

	while (StaticNodePoolPtr->IsValidIndex(CurrentNodeIndex))
	{
		const FOctreeNode& Node = (*StaticNodePoolPtr)[CurrentNodeIndex];

		// 최종 판단: 리프 노드에 도달했거나, 최대 깊이에 도달했을 때
		if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= GetMaxOctreeDepth())
		{
			// 이 노드가 Solid 상태일 때만 막힌 것입니다. Empty 상태는 안전합니다.
			return Node.State == EOctreeNodeState::Solid;
		}

		// 분기: Mixed 노드이므로 자식으로 내려가야 합니다.
		const int32 ChildOctant = GetOctantContainingVoxel(Node.Bounds, WorldLocation);
		CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
		CurrentDepth++;
		
		// 만약 자식 인덱스가 -1이라면, 이 분기는 비어있어서 최적화된(Collapse) 것입니다.
		// 따라서 이 위치는 막히지 않았습니다.
		if (CurrentNodeIndex == -1)
		{
			return false;
		}
	}

	// 루프가 비정상적으로 종료된 경우 (예: WorldLocation이 볼륨 밖에 있는 경우 등)
	// 안전을 위해 막혔다고 처리할 수 있지만, 위의 사전 검사로 대부분 걸러집니다.
	// 여기까지 왔다는 것은 해당 위치가 비어있다는 것을 의미합니다.
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

void AOctreeVoxelVolume::ConnectPortals_Bake(
	TArray<FOctreeNode>& InOutMyNodePool,
	TMap<int32, FVolumePortalLink>& OutPortalNodes,
	const TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>>& AllVolumeData,
	const TArray<AOctreeVoxelVolume*>& AllCreatedVolumes)
{
	UE_LOG(LogTemp, Log, TEXT("===[%s] ConnectPortals_Bake 시작. (총 노드 수: %d) ==="), *GetName(), InOutMyNodePool.Num());

	// Epsilon 값은 테스트 지점이 경계면을 확실히 넘어가도록 보장합니다.
	constexpr float Epsilon = 1.0f;
	int32 BoundaryNodeCount = 0;
	int32 CreatedPortalCount = 0;

	// 1. 현재 볼륨의 모든 노드를 순회합니다.
	for (int32 MyNodeIndex = 0; MyNodeIndex < InOutMyNodePool.Num(); ++MyNodeIndex)
	{
		FOctreeNode& MyNode = InOutMyNodePool[MyNodeIndex];

		// 2. 포털은 비어있는(Empty) 노드에만 생성될 수 있습니다.
		if (MyNode.State != EOctreeNodeState::Empty)
		{
			continue;
		}

		// 3. 6방향의 이웃을 모두 확인합니다.
		for (int32 DirIndex = 0; DirIndex < static_cast<int32>(ENeighborDirection::MAX); ++DirIndex)
		{
			// 내부적으로 이미 이웃이 연결된 경우는 건너뜁니다. 
			// -1은 볼륨의 경계면임을 의미합니다.
			if (MyNode.Neighbors[DirIndex] != -1)
			{
				continue;
			}

			BoundaryNodeCount++;

			// 4. 경계면 너머의 이웃을 탐색할 테스트 지점을 계산합니다.
			FVector TestPoint = MyNode.Bounds.GetCenter();
			const ENeighborDirection Direction = static_cast<ENeighborDirection>(DirIndex);
			switch (Direction)
			{
			case ENeighborDirection::Right: TestPoint.X = MyNode.Bounds.Max.X + Epsilon; break;
			case ENeighborDirection::Left:  TestPoint.X = MyNode.Bounds.Min.X - Epsilon; break;
			case ENeighborDirection::Front: TestPoint.Y = MyNode.Bounds.Max.Y + Epsilon; break;
			case ENeighborDirection::Back:  TestPoint.Y = MyNode.Bounds.Min.Y - Epsilon; break;
			case ENeighborDirection::Up:    TestPoint.Z = MyNode.Bounds.Max.Z + Epsilon; break;
			case ENeighborDirection::Down:  TestPoint.Z = MyNode.Bounds.Min.Z - Epsilon; break;
			}

			// 5. 이웃 볼륨 찾기: NavSubsystem 대신 전달받은 전체 볼륨 목록을 직접 순회합니다.
			// 이것이 이 로직의 핵심적인 수정 사항입니다.
			AOctreeVoxelVolume* NeighborVolume = nullptr;
			for (AOctreeVoxelVolume* PotentialNeighbor : AllCreatedVolumes)
			{
				if (PotentialNeighbor && PotentialNeighbor != this && PotentialNeighbor->GetVolumeBounds().IsInside(TestPoint))
				{
					NeighborVolume = PotentialNeighbor;
					break; // 첫 번째로 찾은 이웃이 우리가 찾는 이웃입니다.
				}
			}

			// 6. 유효한 이웃 볼륨을 찾았을 경우, 포털 생성을 시도합니다.
			if (NeighborVolume)
			{
				// 6a. 이웃 볼륨의 데이터 에셋 경로를 가져옵니다. 이것이 포털 연결의 고유 식별자가 됩니다.
				TSoftObjectPtr<UOctreeDataAsset> NeighborDataAssetPtr = NeighborVolume->GetBakedOctreeData();
				if (NeighborDataAssetPtr.IsNull())
				{
					// 이 에러는 BakeEntireWorld의 로직 순서가 잘못되었을 때 발생할 수 있습니다. (에셋 할당이 포털 연결보다 늦을 경우)
					UE_LOG(LogTemp, Error, TEXT("ConnectPortals_Bake CRITICAL: 이웃 볼륨 '%s'가 유효한 BakedOctreeData 에셋을 가지고 있지 않습니다! 포털을 생성할 수 없습니다."), *NeighborVolume->GetName());
					continue;
				}

				// 6b. 전체 데이터 맵에서 이웃 볼륨의 노드 풀(NodePool)을 가져옵니다.
				const TArray<FOctreeNode>* NeighborNodePoolPtr = AllVolumeData.Find(NeighborVolume);
				if (!NeighborNodePoolPtr)
				{
					UE_LOG(LogTemp, Warning, TEXT("ConnectPortals_Bake: 이웃 볼륨 '%s'는 찾았지만, AllVolumeData 맵에 해당 볼륨의 노드 데이터가 없습니다."), *NeighborVolume->GetName());
					continue;
				}

				// 6c. 이웃 볼륨에서 테스트 지점에 해당하는 리프 노드를 찾습니다.
				const int32 NeighborNodeIndex = NeighborVolume->FindLeafNode_Internal(*NeighborNodePoolPtr, 0, TestPoint, NeighborVolume->GetMaxOctreeDepth());

				// 6d. 찾은 이웃 노드가 유효하고, 비어있는 노드인지 최종 확인합니다.
				if (NeighborNodePoolPtr->IsValidIndex(NeighborNodeIndex))
				{
					const FOctreeNode& NeighborNode = (*NeighborNodePoolPtr)[NeighborNodeIndex];
					if (NeighborNode.State == EOctreeNodeState::Empty)
					{
						// 7. 성공! 연결 가능한 포털을 찾았으므로 링크 정보를 생성하고 저장합니다.
						UE_LOG(LogTemp, Log, TEXT("ConnectPortals_Bake SUCCESS: '%s'의 노드 %d에서 -> '%s'의 노드 %d (에셋: %s)로 가는 유효한 포털을 찾았습니다."),
							*GetName(), MyNodeIndex, *NeighborVolume->GetName(), NeighborNodeIndex, *NeighborDataAssetPtr.ToString());

						// 포털을 통과할 수 있는 최대 에이전트 크기는 두 노드 중 더 작은 쪽을 따릅니다.
						const float PortalClearance = FMath::Min(MyNode.MaxAllowedRadius, NeighborNode.MaxAllowedRadius);

						FVolumePortalLink PortalLink;
						PortalLink.TargetVolume = NeighborDataAssetPtr;
						PortalLink.TargetNodeIndex = NeighborNodeIndex;
						PortalLink.MaxAllowedRadius = PortalClearance;

						OutPortalNodes.Add(MyNodeIndex, PortalLink);
						CreatedPortalCount++;
					}
					else
					{
						// 이웃 노드가 있지만 장애물(Solid) 또는 혼합(Mixed) 상태라 연결할 수 없습니다.
						UE_LOG(LogTemp, Verbose, TEXT("ConnectPortals_Bake: 노드 %d의 이웃 노드(%d in '%s')가 Empty 상태가 아니므로 포털을 생성하지 않습니다."), MyNodeIndex, NeighborNodeIndex, *NeighborVolume->GetName());
					}
				}
				else
				{
					// 이웃 볼륨에서 유효한 리프 노드를 찾지 못했습니다. (이론적으로 발생하기 힘든 경우)
					UE_LOG(LogTemp, Warning, TEXT("ConnectPortals_Bake: 이웃 볼륨 '%s'에서 TestPoint(%s)에 해당하는 유효한 리프 노드를 찾지 못했습니다."), *NeighborVolume->GetName(), *TestPoint.ToString());
				}
			}
			// 이웃 볼륨을 찾지 못했다면, 월드의 실제 경계면이므로 아무 작업도 하지 않습니다.
		}
	}

	// 최종 결과를 로그로 남깁니다.
	UE_LOG(LogTemp, Log, TEXT("===[%s] ConnectPortals_Bake 종료. (경계면 노드 %d개 확인, 포털 %d개 생성) ==="), *GetName(), BoundaryNodeCount, CreatedPortalCount);
}
void AOctreeVoxelVolume::SaveOctreeToAsset(const TArray<FOctreeNode>& NodePoolToSave, const TMap<int32, FVolumePortalLink>& PortalsToSave)
{
	FString PackageName, AssetName;
	if (!GetBakedAssetPath(PackageName, AssetName))
	{
		UE_LOG(LogTemp, Error, TEXT("SaveOctreeToAsset 실패: '%s'의 데이터 에셋 경로를 결정할 수 없습니다."), *GetName());
		return;
	}

	// GetBakedOctreeData()를 통해 미리 생성/할당된 데이터 에셋을 가져옵니다.
	UOctreeDataAsset* DataAssetToSave = GetBakedOctreeData().Get();
	if (!DataAssetToSave)
	{
		UE_LOG(LogTemp, Error, TEXT("SaveOctreeToAsset 실패: '%s'에 할당된 데이터 에셋이 없습니다. 베이크 순서를 확인하세요."), *GetName());
		return;
	}

	// --- 저장 직전 데이터 검증 로그 ---
	UE_LOG(LogTemp, Log, TEXT("'%s'의 데이터 에셋(%s)을 저장합니다. %d개의 포털 데이터를 확인합니다..."), *GetName(), *AssetName, PortalsToSave.Num());
	for (const auto& PortalPair : PortalsToSave)
	{
		const FVolumePortalLink& PortalLink = PortalPair.Value;
		if (PortalLink.TargetVolume.IsNull())
		{
			// 이 로그가 찍힌다면, ConnectPortals_Bake에서 문제가 있었음을 의미합니다.
			UE_LOG(LogTemp, Error, TEXT("SaveOctreeToAsset CRITICAL: 노드 %d를 위한 포털의 TargetVolume이 NULL입니다! 이 데이터는 런타임에서 실패를 유발합니다."), PortalPair.Key);
		}
		else
		{
			UE_LOG(LogTemp, Verbose, TEXT("SaveOctreeToAsset: 노드 %d -> 대상 에셋 '%s' (유효)"), PortalPair.Key, *PortalLink.TargetVolume.ToString());
		}
	}
	// --- 로그 끝 ---

	DataAssetToSave->Modify();
	DataAssetToSave->PortalNodes = PortalsToSave;
	DataAssetToSave->SetAndCompressBakedData(NodePoolToSave, this->MapSize, this->GetMaxOctreeDepth());

	UE_LOG(LogTemp, Log, TEXT("'%s'의 데이터 에셋 저장을 위해 패키지를 Dirty로 표시했습니다."), *GetName());
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

// TO-DO: Check
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

// ======================================================================
// Query Helpers - Implementation
// ======================================================================

FIntVector AOctreeVoxelVolume::WorldToVoxelIndex(const FVector& WorldLocation) const
{
	// VoxelExtent가 0이거나 음수일 경우의 방어 코드
	if (VoxelExtent <= 0.f) return FIntVector::ZeroValue;

	// 볼륨의 최소 경계점을 (0,0,0) 원점으로 간주하고 계산합니다.
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
	// 복셀의 월드 좌표는 (볼륨 시작점) + (인덱스 * 복셀 크기) + (중심점 보정) 입니다.
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

void AOctreeVoxelVolume::GetDebugNodeInfoRecursive(TArray<FDebugNodeInfo>& OutNodes, int32 NodeIndex, const FBox& NodeBounds, int32 CurrentDepth) const
{
	if (!StaticNodePoolPtr || StaticNodePoolPtr->IsEmpty()) return;

	if (!StaticNodePoolPtr->IsValidIndex(NodeIndex) || CurrentDepth > GetMaxOctreeDepth())
	{
		return;
	}
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
			// 배열이 재할당될 수 있으므로, InOutNodePool[NodeIndex]를 다시 참조하지 않도록 주의해야 합니다.
			// 여기서는 괜찮지만 복잡한 로직에서는 포인터/참조를 다시 얻어야 할 수 있습니다.
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
