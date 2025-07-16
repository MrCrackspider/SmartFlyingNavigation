#include "Pathfinder/NaviVolume/OctreeVoxelVolume.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "Misc/ScopedSlowTask.h"

// 베이킹에 필요한 헤더
#if WITH_EDITOR
#include "AssetToolsModule.h"
#include "FileHelpers.h"
#endif

// SIMD 헤더 (동적 그리드용)
#include "emmintrin.h"
#include "smmintrin.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Components/BoxComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Pathfinder/PathfindingComponent.h"


AOctreeVoxelVolume::AOctreeVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;
	// 기본값 설정
	MaxOctreeDepth = 7; // 예: 128x128x128 볼륨 -> 2^7=128
}

void AOctreeVoxelVolume::BeginPlay()
{
	Super::BeginPlay();

	// 1. 베이크된 정적 옥트리 데이터 로드
	if (BakedOctreeData)
	{
		StaticNodePool = BakedOctreeData->NodePool;
		if (StaticNodePool.Num() > 0)
		{
			RootNodeIndex = 0; // 루트 노드는 항상 인덱스 0으로 가정
			UE_LOG(LogTemp, Log, TEXT("OctreeVoxelVolume: Loaded static octree with %d nodes."), StaticNodePool.Num());
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning,
		       TEXT("OctreeVoxelVolume: No BakedOctreeData asset assigned. Static layer will be empty."));
	}

	// 2. 동적 장애물 레이어 초기화 및 타이머 시작
	InitializeDynamicLayer();
	GetWorldTimerManager().SetTimer(DynamicObstacleTimer, this, &AOctreeVoxelVolume::UpdateDynamicObstacles, 0.2f, true,
	                                0.1f);
}

void AOctreeVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

//======================================================================
// --- 핵심 인터페이스 구현 ---
//======================================================================

bool AOctreeVoxelVolume::IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const
{
	const FIntVector AgentRadius(AgentRadiusInVoxels);
	const FIntVector MinCorner = VoxelIndex - AgentRadius;
	const FIntVector MaxCorner = VoxelIndex + AgentRadius;

	// 월드 좌표계의 쿼리 박스 계산
	const FVector QueryMinWorld = VoxelIndexToWorld(MinCorner);
	// Max 코너는 복셀의 끝 지점까지 포함해야 하므로 1을 더한 인덱스의 월드 위치를 사용
	const FVector QueryMaxWorld = VoxelIndexToWorld(MaxCorner + FIntVector(1, 1, 1));
	const FBox QueryBox(QueryMinWorld, QueryMaxWorld);

	// 1. 정적 옥트리 레이어 확인
	if (!IsRegionEmpty_StaticOctree(QueryBox))
	{
		return false; // 정적 지형에 막힘
	}

	// 2. 동적 그리드 레이어 확인
	if (!IsRegionEmpty_DynamicGrid(MinCorner, MaxCorner))
	{
		return false; // 동적 장애물에 막힘
	}

	// 두 레이어 모두 통과하면 걸을 수 있음
	return true;
}

void AOctreeVoxelVolume::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
	OutObstacleIndices.Empty();

	// 1. 동적 장애물 복셀 수집
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

	// 2. 정적 장애물 복셀 수집 (옥트리 순회)
	if (RootNodeIndex != -1)
	{
		TSet<FIntVector> StaticObstacles; // 중복 방지
		// 재귀적으로 Solid 리프 노드 찾기
		// ... (이 부분은 매우 느릴 수 있으므로 디버깅용으로만 사용)
	}
}

bool AOctreeVoxelVolume::IsVoxelWalkable(const FIntVector& Index) const
{
	return IsRegionWalkable(Index, 0);
}

//======================================================================
// --- 베이킹 구현 ---
//======================================================================

// 헬퍼 함수 구현
void AOctreeVoxelVolume::SafeRemoveInstances(UInstancedStaticMeshComponent* Component,
                                             TArray<int32>& InstanceIndicesToRemove)
{
	if (!Component || InstanceIndicesToRemove.Num() == 0)
	{
		return;
	}
	// 인덱스가 큰 것부터 제거해야 나머지 인덱스가 유효하게 유지됩니다.
	InstanceIndicesToRemove.Sort([](const int32& A, const int32& B)
	{
		return A > B;
	});
	Component->RemoveInstances(InstanceIndicesToRemove);
}

// 이 코드로 UpdateDebugObstacleVisualization 함수를 완전히 교체하십시오.

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

	// 1. 백그라운드 스레드에서 그려야 할 노드 정보 수집
	Async(EAsyncExecution::ThreadPool, [WeakThis, InitialBounds]()
	{
		if (!WeakThis.IsValid() || WeakThis->RootNodeIndex == -1)
		{
			if (WeakThis.IsValid()) WeakThis->bIsUpdatingVisualization = false;
			return;
		}

		TArray<FDebugNodeInfo> NodesToDraw;
		WeakThis->GetDebugNodeInfoRecursive(NodesToDraw, WeakThis->RootNodeIndex, InitialBounds, 0);

		// 2. 게임 스레드에서 인스턴스 업데이트
		AsyncTask(ENamedThreads::GameThread, [WeakThis, NodesToDraw = MoveTemp(NodesToDraw)]()
		{
			AOctreeVoxelVolume* StrongThis = WeakThis.Get();
			if (!StrongThis) return;

			// --- 모든 컴포넌트의 인스턴스를 깨끗하게 비웁니다 ---
			if (StrongThis->DebugSolidNodesComponent) StrongThis->DebugSolidNodesComponent->ClearInstances();
			if (StrongThis->DebugEmptyNodesComponent) StrongThis->DebugEmptyNodesComponent->ClearInstances();
			if (StrongThis->DebugMixedNodesComponent) StrongThis->DebugMixedNodesComponent->ClearInstances();

			if (NodesToDraw.Num() == 0)
			{
				StrongThis->bIsUpdatingVisualization = false;
				return;
			}

			// --- 그려야 할 노드들의 "로컬" Transform을 수집합니다 ---
			TArray<FTransform> SolidTransforms, EmptyTransforms, MixedTransforms;

			for (const auto& NodeInfo : NodesToDraw)
			{
				const FVector Scale = NodeInfo.Bounds.GetSize() / 100.0f;
				const FVector WorldCenter = NodeInfo.Bounds.GetCenter();

				if (NodeInfo.State == EOctreeNodeState::Solid && StrongThis->DebugSolidNodesComponent)
				{
					// "Solid" 컴포넌트의 Transform을 기준으로 로컬 좌표 계산
					const FTransform CompXform = StrongThis->DebugSolidNodesComponent->GetComponentTransform();
					const FVector LocalCenter = CompXform.InverseTransformPosition(WorldCenter);
					SolidTransforms.Emplace(FRotator::ZeroRotator, LocalCenter, Scale);
				}
				else if (NodeInfo.State == EOctreeNodeState::Empty && StrongThis->DebugEmptyNodesComponent)
				{
					// "Empty" 컴포넌트의 Transform을 기준으로 로컬 좌표 계산
					const FTransform CompXform = StrongThis->DebugEmptyNodesComponent->GetComponentTransform();
					const FVector LocalCenter = CompXform.InverseTransformPosition(WorldCenter);
					EmptyTransforms.Emplace(FRotator::ZeroRotator, LocalCenter, Scale);
				}
				else if (NodeInfo.State == EOctreeNodeState::Mixed && StrongThis->DebugMixedNodesComponent)
				{
					// "Mixed" 컴포넌트의 Transform을 기준으로 로컬 좌표 계산
					const FTransform CompXform = StrongThis->DebugMixedNodesComponent->GetComponentTransform();
					const FVector LocalCenter = CompXform.InverseTransformPosition(WorldCenter);
					MixedTransforms.Emplace(FRotator::ZeroRotator, LocalCenter, Scale);
				}
			}

			// --- 상태별로 인스턴스 일괄 추가 (bWorldSpace = false) ---
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
	if (!StaticNodePool.IsValidIndex(NodeIndex) || CurrentDepth > DebugOctreeMaxDepth) return;

	const FOctreeNode& Node = StaticNodePool[NodeIndex];

	// 리프 노드이거나, 디버그 최대 깊이에 도달한 경우에만 노드 정보를 추가
	if (Node.State != EOctreeNodeState::Mixed || CurrentDepth == DebugOctreeMaxDepth)
	{
		OutNodes.Add({NodeIndex, NodeBounds, Node.State});
	}
	else // Mixed 상태이고 더 깊이 들어갈 수 있으면 자식 노드 탐색
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

	// --- 1. 사전 준비 및 장애물 스캔 ---
	TSet<FIntVector> AllStaticObstacles;
	ScanForAllStaticObstacles(AllStaticObstacles);

	const FBox VolumeBounds = VolumeBox->Bounds.GetBox();
	if (!VolumeBounds.IsValid)
	{
		UE_LOG(LogTemp, Error, TEXT("Baking failed: Volume has invalid bounds."));
		return;
	}

	// --- 2. UI 프로그레스 바 설정 ---
	// 총 작업량 = (빌드: 장애물 개수) + (최적화: 1) + (저장: 1)
	FScopedSlowTask BakeTask(AllStaticObstacles.Num() + 2, FText::FromString(TEXT("Baking Octree Data...")));
	BakeTask.MakeDialog(); // UI 대화상자 표시

	// --- 3. 옥트리 빌드 ---
	BakeTask.EnterProgressFrame(0, FText::FromString(TEXT("Building Octree...")));

	StaticNodePool.Empty();
	RootNodeIndex = StaticNodePool.Emplace(FOctreeNode()); // 루트 노드 생성

	StaticNodePool[RootNodeIndex].Bounds = VolumeBounds;
	
	for (const FIntVector& ObstacleVoxel : AllStaticObstacles)
	{
		BakeTask.EnterProgressFrame(1);
		if (BakeTask.ShouldCancel()) return;

		// ✨✨ InsertObstacleRecursive 호출 시 VolumeBounds 인자를 제거합니다! ✨✨
		InsertObstacleRecursive(RootNodeIndex, ObstacleVoxel, 0); // <-- 이렇게 수정합니다.
	}

	// --- 4. 옥트리 최적화 ---
	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Optimizing Octree...")));
	if (BakeTask.ShouldCancel()) return;

	OptimizeAndCollapse(RootNodeIndex, VolumeBounds);

	// --- 5. 애셋 파일로 저장 ---
	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Saving data asset...")));
	if (BakeTask.ShouldCancel()) return;

	SaveOctreeToAsset(); // 헬퍼 함수 호출로 깔끔하게 처리

	UE_LOG(LogTemp, Log, TEXT("Baking complete! Final Octree has %d nodes."), StaticNodePool.Num());
}

void AOctreeVoxelVolume::ScanForAllStaticObstacles(TSet<FIntVector>& OutStaticObstacles) const
{
	OutStaticObstacles.Empty();

	// 1. 볼륨 전체 영역에 대한 박스를 계산합니다.
	const FBox VolumeWorldBounds = GetComponentsBoundingBox(true, true);

	// 2. 해당 박스 영역과 겹치는 모든 Static Mesh 액터를 찾습니다.
	TArray<AActor*> OverlappingActors;
	TArray<TEnumAsByte<EObjectTypeQuery>> ObjectTypes;
	ObjectTypes.Add(UEngineTypes::ConvertToObjectType(ECollisionChannel::ECC_WorldStatic));

	UKismetSystemLibrary::BoxOverlapActors(GetWorld(), VolumeWorldBounds.GetCenter(), VolumeWorldBounds.GetExtent(),
	                                       ObjectTypes, AActor::StaticClass(), BakeExcludeActors, OverlappingActors);

	if (OverlappingActors.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("ScanForAllStaticObstacles: No overlapping static actors found in volume."));
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("ScanForAllStaticObstacles: Found %d overlapping actors. Now checking voxels..."),
	       OverlappingActors.Num());

	// 3. 각 액터의 컴포넌트를 순회하며 복셀 단위로 정밀 검사합니다.
	for (AActor* Actor : OverlappingActors)
	{
		TArray<UPrimitiveComponent*> PrimitiveComponents;
		Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);

		for (UPrimitiveComponent* Comp : PrimitiveComponents)
		{
			if (!Comp || !Comp->IsQueryCollisionEnabled()) continue;

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
	UE_LOG(LogTemp, Log, TEXT("Scan complete. Found %d unique static obstacle voxels."), OutStaticObstacles.Num());
}

void AOctreeVoxelVolume::InsertObstacleRecursive(int32 NodeIndex, // <-- const FBox& NodeBounds 인자를 제거합니다!
												 const FIntVector& ObstacleVoxel, int32 CurrentDepth)
{
  // 1. 유효성 검사 (NodeIndex는 항상 유효해야 하지만, 방어적 코딩)
    if (!StaticNodePool.IsValidIndex(NodeIndex))
    {
        UE_LOG(LogTemp, Error, TEXT("InsertObstacleRecursive: Invalid NodeIndex %d"), NodeIndex);
        return;
    }

    // 2. 현재 노드의 참조를 가져옵니다.
    //    주의: StaticNodePool.Emplace() 호출 전에만 유효합니다.
    FOctreeNode& CurrentNodeRef = StaticNodePool[NodeIndex]; 
    const FBox CurrentNodeBounds = CurrentNodeRef.Bounds; // <-- 노드에 저장된 Bounds를 사용합니다.

    // 3. 노드 상태가 Solid이면 더 이상 진행할 필요 없음
    if (CurrentNodeRef.State == EOctreeNodeState::Solid)
    {
        return;
    }

    // 4. 최대 깊이에 도달하면 현재 노드를 Solid로 만듭니다.
    if (CurrentDepth == MaxOctreeDepth)
    {
        CurrentNodeRef.State = EOctreeNodeState::Solid;
        return;
    }

    // 5. 현재 노드가 Empty 상태이면 Mixed로 바꾸고 자식 노드를 생성합니다.
    //    이 블록은 StaticNodePool의 재할당을 유발할 수 있으므로,
    //    이전에 얻었던 CurrentNodeRef 참조가 무효화될 수 있습니다.
    //    따라서 StaticNodePool[NodeIndex]를 직접 사용하여 안전하게 접근합니다.
    if (CurrentNodeRef.State == EOctreeNodeState::Empty)
    {
        StaticNodePool[NodeIndex].State = EOctreeNodeState::Mixed; // <-- 재할당 가능성 때문에 직접 접근

        // 자식 노드를 생성하고 Bounds를 설정합니다.
        for (int i = 0; i < 8; ++i)
        {
            const int32 NewChildIndex = StaticNodePool.Emplace(FOctreeNode()); // 새 노드 추가
            StaticNodePool[NewChildIndex].Bounds = CalculateChildBounds(CurrentNodeBounds, i); // <-- 새 자식 노드의 Bounds 설정
            StaticNodePool[NodeIndex].ChildrenIndices[i] = NewChildIndex; // <-- 재할당 가능성 때문에 직접 접근
        }
    }

    // 6. 자식 노드로 재귀 호출하기 위한 준비
    //    여기서부터는 StaticNodePool에 더 이상 Emplace가 없으므로 CurrentNodeRef가 다시 유효해집니다.
    //    하지만 일관성을 위해 StaticNodePool[NodeIndex]를 계속 사용하거나,
    //    아예 이 시점에서 FOctreeNode& Node = StaticNodePool[NodeIndex]; 를 다시 가져와도 됩니다.
    //    이전 CurrentNodeRef는 Emplace 때문에 무효화되었을 가능성이 있기 때문입니다.
    FOctreeNode& Node = StaticNodePool[NodeIndex]; // <-- 안전하게 다시 참조를 가져옵니다.

    const FVector VoxelWorldPos = VoxelIndexToWorld(ObstacleVoxel);
    // GetOctantContainingVoxel 호출 시 Node.Bounds를 사용합니다.
    const int32 ChildOctant = GetOctantContainingVoxel(Node.Bounds, VoxelWorldPos); 

    const int32 ChildNodeIndex = Node.ChildrenIndices[ChildOctant];
    if (ChildNodeIndex != -1)
    {
        // 재귀 호출 시 NodeBounds 인자를 제거합니다.
        // ChildBounds는 더 이상 인자로 전달되지 않고, 재귀 호출된 함수 내부에서 Node.Bounds를 통해 접근합니다.
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

void AOctreeVoxelVolume::OptimizeAndCollapse(int32 NodeIndex, const FBox& NodeBounds)
{
	if (!StaticNodePool.IsValidIndex(NodeIndex))
	{
		return;
	}

	FOctreeNode& Node = StaticNodePool[NodeIndex];
	if (Node.State != EOctreeNodeState::Mixed)
	{
		return; // 리프 노드는 최적화 대상이 아님
	}

	// 자식들을 먼저 재귀적으로 최적화 (Post-order traversal)
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

	// 모든 자식의 상태가 동일한지 확인
	const EOctreeNodeState FirstChildState = StaticNodePool[Node.ChildrenIndices[0]].State;
	if (FirstChildState == EOctreeNodeState::Mixed) return; // 자식이 Mixed면 부모는 병합 불가

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

	// 모두 같다면 부모 노드를 자식의 상태로 바꾸고, 자식들을 제거
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
	}
	return FBox();
}

int32 AOctreeVoxelVolume::GetOctantContainingVoxel(const FBox& NodeBounds, const FVector& VoxelWorldPos) const
{
	int32 Octant = 0;
	const FVector Center = NodeBounds.GetCenter();
	if (VoxelWorldPos.X >= Center.X) Octant |= 1; // 001
	if (VoxelWorldPos.Y >= Center.Y) Octant |= 2; // 010
	if (VoxelWorldPos.Z >= Center.Z) Octant |= 4; // 100
	return Octant;
}

bool AOctreeVoxelVolume::GetBakedAssetPath(FString& OutPackagePath, FString& OutAssetName) const
{
	if (BakedDataSaveDirectory.Path.IsEmpty())
	{
		UE_LOG(LogTemp, Error, TEXT("Cannot get asset path: 'Baked Data Save Directory' is not set."));
		return false;
	}

	const FString BaseDirectory = BakedDataSaveDirectory.Path;
	const FString DataDirectory = FPaths::Combine(BaseDirectory, TEXT("Data"));
	const FString FinalDirectory = FPaths::Combine(DataDirectory, TEXT("Octree"));

	OutAssetName = FString::Printf(TEXT("DA_Octree_%s"), *GetActorLabel());
	OutPackagePath = FPaths::Combine(FinalDirectory, OutAssetName);

	return true;
}

void AOctreeVoxelVolume::SaveOctreeToAsset()
{
	FString PackageName, AssetName;
	if (!GetBakedAssetPath(PackageName, AssetName))
	{
		return; // 경로 생성 실패 시 중단
	}

	UPackage* Package = CreatePackage(*PackageName);
	Package->FullyLoad();

	UOctreeDataAsset* NewOctreeData = NewObject<
		UOctreeDataAsset>(Package, FName(*AssetName), RF_Public | RF_Standalone);
	if (!NewOctreeData)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create new UOctreeDataAsset object for package: %s"), *PackageName);
		return;
	}

	// 베이크된 데이터와 설정을 애셋에 복사
	NewOctreeData->NodePool = StaticNodePool;
	NewOctreeData->BakedMapSize = this->MapSize;
	NewOctreeData->BakedMaxOctreeDepth = this->MaxOctreeDepth;

	FAssetRegistryModule::AssetCreated(NewOctreeData);
	Package->MarkPackageDirty();

	// --- 저장 로직 변경 ---
	TArray<UPackage*> PackagesToSave;
	PackagesToSave.Add(Package);

	if (UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true))
	{
		UE_LOG(LogTemp, Log, TEXT("Successfully saved Octree data to: %s.uasset"), *PackageName);
		this->BakedOctreeData = NewOctreeData; // 자동으로 참조 할당
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to save Octree data package: %s"), *PackageName);
	}
}

#endif

//======================================================================
// --- 동적 장애물 처리 구현 (AFenwickVoxelVolume에서 가져옴) ---
//======================================================================

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
	TSet<FIntVector> NewFrameObstacles;
	GatherCurrentObstacleVoxels(NewFrameObstacles);
	ApplyDynamicObstacleChanges(NewFrameObstacles);
	CurrentDynamicObstacles = MoveTemp(NewFrameObstacles);
}

void AOctreeVoxelVolume::GatherCurrentObstacleVoxels(TSet<FIntVector>& OutVoxelIndices) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(VoxelVolume_GatherObstacles);

	if (!IsValid(ObstacleFilterProfile)) return;

	FCollisionObjectQueryParams ObjectQueryParams;
	for (const TEnumAsByte<ECollisionChannel>& Channel : ObstacleFilterProfile->CollisionChannels)
	{
		ObjectQueryParams.AddObjectTypesToQuery(Channel);
	}

	TArray<FOverlapResult> OverlapResults;
	if (!GetOverlappingComponentsInVolume(ObjectQueryParams, OverlapResults)) return;

	for (const FOverlapResult& Result : OverlapResults)
	{
		UPrimitiveComponent* OverlappedComponent = Result.GetComponent();
		AActor* OverlappedActor = Result.GetActor();

		if (OverlappedActor && OverlappedActor->FindComponentByClass<UPathfindingComponent>())
		{
			continue;
		}

		if (ObstacleFilterProfile->DoesActorMatch(OverlappedActor, OverlappedComponent))
		{
			const FBox ObstacleBounds = OverlappedComponent->Bounds.GetBox();
			FIntVector MinVoxelIndex = WorldToVoxelIndex(ObstacleBounds.Min);
			FIntVector MaxVoxelIndex = WorldToVoxelIndex(ObstacleBounds.Max - FVector(KINDA_SMALL_NUMBER));

			FIntVector ClampedMin, ClampedMax;
			if (!ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex, ClampedMin, ClampedMax)) continue;

			for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
			{
				for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
				{
					for (int32 x = ClampedMin.X; x <= ClampedMax.X; ++x)
					{
						const FIntVector CurrentVoxelIndex(x, y, z);
						if (CheckVoxelOverlap(CurrentVoxelIndex, OverlappedComponent))
						{
							OutVoxelIndices.Add(CurrentVoxelIndex);
						}
					}
				}
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

//======================================================================
// --- 쿼리 헬퍼 구현 ---
//======================================================================

bool AOctreeVoxelVolume::IsRegionEmpty_StaticOctree(const FBox& QueryBox) const
{
	if (RootNodeIndex == -1) return true; // 정적 레이어가 없으면 항상 비어있음

	const FBox VolumeBounds = GetComponentsBoundingBox(true, true);
	return IsRegionEmptyRecursive(RootNodeIndex, VolumeBounds, QueryBox);
}

bool AOctreeVoxelVolume::IsRegionEmptyRecursive(int32 NodeIndex, const FBox& NodeBounds, const FBox& QueryBox) const
{
	if (!NodeBounds.Intersect(QueryBox))
	{
		return true; // 쿼리 영역과 겹치지 않음
	}

	if (!StaticNodePool.IsValidIndex(NodeIndex)) return true; // 잘못된 인덱스는 안전하게 처리
	const FOctreeNode& Node = StaticNodePool[NodeIndex];

	if (Node.State == EOctreeNodeState::Solid)
	{
		return false; // 장애물 발견
	}
	if (Node.State == EOctreeNodeState::Empty)
	{
		return true; // 이 노드 영역은 안전함
	}

	// Mixed 상태: 자식 노드 순회
	for (int i = 0; i < 8; ++i)
	{
		if (Node.ChildrenIndices[i] != -1)
		{
			const FBox ChildBounds = CalculateChildBounds(NodeBounds, i);
			if (!IsRegionEmptyRecursive(Node.ChildrenIndices[i], ChildBounds, QueryBox))
			{
				return false; // 자식 중 하나에서 장애물 발견
			}
		}
	}

	return true;
}

bool AOctreeVoxelVolume::IsRegionEmpty_DynamicGrid(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
{
	// FThreadSafeBitmaskGrid에서 가져온 SIMD 최적화 버전 구현
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
