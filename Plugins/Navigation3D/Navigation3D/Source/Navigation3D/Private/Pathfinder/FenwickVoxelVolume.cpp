#include "Pathfinder/FenwickVoxelVolume.h"

#include "EditorLevelUtils.h"
#include "ObjectTools.h"
#include "Engine/AssetManager.h"
#include "Engine/LevelStreamingDynamic.h"
#include "Engine/StreamableManager.h"
#include "Pathfinder/PathfindingComponent.h"
#include "Pathfinder/StaticVoxelChunkData.h"
#include "Pathfinder/ChunkDataHolder.h"


#if WITH_EDITOR
#include "AssetToolsModule.h"
#include "FileHelpers.h"
#include "Factories/DataAssetFactory.h"
#include "Misc/ScopedSlowTask.h"
#endif

//======================================================================
// --- INITIALIZATION & LIFECYCLE ---
//======================================================================

AFenwickVoxelVolume::AFenwickVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;
	bIsReady = false;
}

void AFenwickVoxelVolume::BeginPlay()
{
	Super::BeginPlay();
	InitializeDataStructures();

	GetWorldTimerManager().SetTimer(StreamingUpdateTimer, this, &AFenwickVoxelVolume::UpdateStreamingState, 0.05f, true,
	                                0.5f);
	// GetWorldTimerManager().SetTimer(DynamicObstacleTimer, this, &AFenwickVoxelVolume::UpdateDynamicObstacles, 0.3f,
	//                                 true, 0.1f);

	bIsReady = true;
	UE_LOG(LogTemp, Log, TEXT("FenwickVoxelVolume streaming system initialized."));
}

void AFenwickVoxelVolume::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (GetWorld())
	{
		GetWorld()->GetTimerManager().ClearTimer(StreamingUpdateTimer);
		GetWorld()->GetTimerManager().ClearTimer(DynamicObstacleTimer);
	}
	Super::EndPlay(EndPlayReason);
}

void AFenwickVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AFenwickVoxelVolume::BuildObstacles()
{
	Super::BuildObstacles();
}

void AFenwickVoxelVolume::InitializeDataStructures()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_InitializeMap);
	check(MapSize.X > 0 && MapSize.X % CHUNK_SIZE == 0);
	check(MapSize.Y > 0 && MapSize.Y % CHUNK_SIZE == 0);
	check(MapSize.Z > 0 && MapSize.Z % CHUNK_SIZE == 0);

	FenwickTreeSize = MapSize + FIntVector(1, 1, 1);
	const int32 TotalFenwickSize = FenwickTreeSize.X * FenwickTreeSize.Y * FenwickTreeSize.Z;
	FenwickTree.Empty(TotalFenwickSize);
	FenwickTree.AddZeroed(TotalFenwickSize);

	const int32 ChunksAlongX = MapSize.X / CHUNK_SIZE;
	ObstacleBitmasks_X.Init(0, ChunksAlongX * MapSize.Y * MapSize.Z);
}

//======================================================================
// --- RUNTIME STREAMING ---
//======================================================================

void AFenwickVoxelVolume::UpdateStreamingState()
{
	TSet<FIntVector> NewRequiredChunks;
	const int32 NumChunksX = MapSize.X / CHUNK_SIZE;
	const int32 NumChunksY = MapSize.Y / CHUNK_SIZE;
	const int32 NumChunksZ = MapSize.Z / CHUNK_SIZE;

	for (auto It = StreamingAgents.CreateIterator(); It; ++It)
	{
		if (AActor* Agent = It->Get())
		{
			const FIntVector AgentChunkCoords = WorldToChunkCoords(Agent->GetActorLocation());
			for (int32 z = -StreamingRangeInChunks.Z; z <= StreamingRangeInChunks.Z; ++z)
			{
				for (int32 y = -StreamingRangeInChunks.Y; y <= StreamingRangeInChunks.Y; ++y)
				{
					for (int32 x = -StreamingRangeInChunks.X; x <= StreamingRangeInChunks.X; ++x)
					{
						const FIntVector Coords = AgentChunkCoords + FIntVector(x, y, z);
						if (Coords.X >= 0 && Coords.X < NumChunksX &&
							Coords.Y >= 0 && Coords.Y < NumChunksY &&
							Coords.Z >= 0 && Coords.Z < NumChunksZ)
						{
							NewRequiredChunks.Add(Coords);
						}
					}
				}
			}
		}
		else
		{
			It.RemoveCurrent();
		}
	}

	const TSet<FIntVector> ChunksToLoad = NewRequiredChunks.Difference(RequiredChunks);
	const TSet<FIntVector> ChunksToUnload = RequiredChunks.Difference(NewRequiredChunks);

	for (const FIntVector& Coords : ChunksToLoad)
	{
		RequestChunkLoad(Coords);
	}
	for (const FIntVector& Coords : ChunksToUnload)
	{
		UnloadChunk(Coords);
	}

	RequiredChunks = NewRequiredChunks;
}

void AFenwickVoxelVolume::RequestChunkLoad(const FIntVector& ChunkCoords)
{
	if (PendingStreamingLevels.Contains(ChunkCoords) || ActiveStreamingLevels.Contains(ChunkCoords))
	{
		return;
	}


	const FString LevelToLoadPath = GetChunkAssetPath(ChunkCoords);
	bool bSuccess = false;

	ULevelStreamingDynamic* StreamingLevel = ULevelStreamingDynamic::LoadLevelInstanceBySoftObjectPtr(
		this, TSoftObjectPtr<UWorld>(FSoftObjectPath(LevelToLoadPath)), FVector::ZeroVector, FRotator::ZeroRotator,
		bSuccess);

	if (bSuccess && StreamingLevel)
	{
		PendingStreamingLevels.Add(ChunkCoords);
		ActiveStreamingLevels.Add(ChunkCoords, StreamingLevel);
		//StreamingLevel->OnLevelLoaded.AddDynamic(this, &AFenwickVoxelVolume::OnLevelLoadCompleted);
	}
	else
	{
		// 파일이 없는 경우 (장애물이 없는 빈 청크)는 정상적인 상황이므로, 에러 로그를 남기지 않습니다.
		// UE_LOG(LogTemp, Warning, TEXT("Failed to request chunk load for %s. The .umap asset might not exist."), *ChunkCoords.ToString());
	}
}

void AFenwickVoxelVolume::UnloadChunk(const FIntVector& ChunkCoords)
{
	// 1. 활성화된 스트리밍 레벨 맵에서 해당 청크를 찾습니다.
	if (auto* StreamingLevelPtr = ActiveStreamingLevels.Find(ChunkCoords))
	{
		if (ULevelStreamingDynamic* StreamingLevel = *StreamingLevelPtr)
		{
			// 2. 레벨 스트리머에게 이 레벨이 더 이상 로드될 필요가 없다고 알립니다.
			StreamingLevel->SetShouldBeLoaded(false);
			// 3. 언로드가 완료되면 메모리에서 완전히 제거하도록 요청합니다.
			StreamingLevel->SetIsRequestingUnloadAndRemoval(true);
		}

		// 4. 우리 관리 목록에서 해당 청크를 제거합니다.
		ActiveStreamingLevels.Remove(ChunkCoords);
		UE_LOG(LogTemp, Log, TEXT("Requested unload for chunk %s."), *ChunkCoords.ToString());
	}
}

void AFenwickVoxelVolume::OnLevelLoadCompleted()
{
	// 이 콜백은 레벨이 로드되었다는 신호입니다.
	// 실제 데이터 처리는 해당 레벨의 AChunkDataHolder가 BeginPlay에서 RegisterChunkFromLevel을 호출할 때 이루어집니다.
	// 여기서는 Pending 상태를 제거하는 등의 관리 작업만 하면 됩니다.
	// 어떤 청크가 로드되었는지 식별하는 로직이 필요하지만, 지금은 단순하게 둡니다.
}

void AFenwickVoxelVolume::FenwUpdate(const FIntVector& Index, int32 Delta)
{
	for (int32 z = Index.Z + 1; z < FenwickTreeSize.Z; z += z & -z)
	{
		for (int32 y = Index.Y + 1; y < FenwickTreeSize.Y; y += y & -y)
		{
			for (int32 x = Index.X + 1; x < FenwickTreeSize.X; x += x & -x)
			{
				FenwickTree[GetFenwickFlatIndex(x, y, z)] += Delta;
			}
		}
	}
}

void AFenwickVoxelVolume::FenwUpdateCuboid(const FIntVector& MinCorner, const FIntVector& MaxCorner, int32 Delta)
{
	return;
	const FIntVector MaxPlusOne = MaxCorner + FIntVector(1, 1, 1);
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_FenwUpdateCuboid);
	FenwUpdate(FIntVector(MinCorner.X, MinCorner.Y, MinCorner.Z), +Delta);
	FenwUpdate(FIntVector(MaxPlusOne.X, MinCorner.Y, MinCorner.Z), -Delta);
	FenwUpdate(FIntVector(MinCorner.X, MaxPlusOne.Y, MinCorner.Z), -Delta);
	FenwUpdate(FIntVector(MinCorner.X, MinCorner.Y, MaxPlusOne.Z), -Delta);
	FenwUpdate(FIntVector(MaxPlusOne.X, MaxPlusOne.Y, MinCorner.Z), +Delta);
	FenwUpdate(FIntVector(MaxPlusOne.X, MinCorner.Y, MaxPlusOne.Z), +Delta);
	FenwUpdate(FIntVector(MinCorner.X, MaxPlusOne.Y, MaxPlusOne.Z), +Delta);
	FenwUpdate(FIntVector(MaxPlusOne.X, MaxPlusOne.Y, MaxPlusOne.Z), -Delta);
}

int32 AFenwickVoxelVolume::FenwQuery(const FIntVector& Index) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_FenwQuery);
	int32 Sum = 0;
	for (int32 z = Index.Z + 1; z > 0; z -= z & -z)
	{
		for (int32 y = Index.Y + 1; y > 0; y -= y & -y)
		{
			for (int32 x = Index.X + 1; x > 0; x -= x & -x)
			{
				Sum += FenwickTree[GetFenwickFlatIndex(x, y, z)];
			}
		}
	}
	return Sum;
}

void AFenwickVoxelVolume::UpdateVoxelObstacleState(const FIntVector& Index, bool bIsObstacle)
{
	if (!IsIndexValid(Index)) return;

	const uint32 ChunkX = Index.X / CHUNK_SIZE;
	const uint32 BitPos = Index.X % CHUNK_SIZE;
	const uint32 ChunkArrIndex = GetFlatIndex_X(ChunkX, Index.Y, Index.Z);
	if (bIsObstacle) ObstacleBitmasks_X[ChunkArrIndex] |= (1ULL << BitPos);
	else ObstacleBitmasks_X[ChunkArrIndex] &= ~(1ULL << BitPos);

	FIntVector ChunkCoords = WorldToChunkCoords(VoxelIndexToWorld(Index));
	int32& Counter = ChunkObstacleCounter.FindOrAdd(ChunkCoords, 0);
	if (bIsObstacle) Counter++;
	else
	{
		Counter--;
		if (Counter <= 0)
		{
			ChunkObstacleCounter.Remove(ChunkCoords);
		}
	}
}

bool AFenwickVoxelVolume::IsRegionEmpty_Bitmask(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
{
	const FIntVector ClampedMin(
		FMath::Max(MinCorner.X, 0),
		FMath::Max(MinCorner.Y, 0),
		FMath::Max(MinCorner.Z, 0)
	);

	const FIntVector ClampedMax(
		FMath::Min(MaxCorner.X, MapSize.X - 1),
		FMath::Min(MaxCorner.Y, MapSize.Y - 1),
		FMath::Min(MaxCorner.Z, MapSize.Z - 1)
	);

	for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
	{
		for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
		{
			const int32 StartChunkX = ClampedMin.X / CHUNK_SIZE;
			const int32 EndChunkX = ClampedMax.X / CHUNK_SIZE;

			for (int32 cx = StartChunkX; cx <= EndChunkX; ++cx)
			{
				const uint64 Mask = ObstacleBitmasks_X[GetFlatIndex_X(cx, y, z)];
				if (Mask == 0) continue;

				const int32 VoxelStartX = (cx == StartChunkX) ? ClampedMin.X : cx * CHUNK_SIZE;
				const int32 VoxelEndX = (cx == EndChunkX) ? ClampedMax.X : (cx + 1) * CHUNK_SIZE - 1;
				const int32 StartBit = VoxelStartX % CHUNK_SIZE;
				const int32 EndBit = VoxelEndX % CHUNK_SIZE;

				const uint64 CheckMask = (~0ULL << StartBit) & (~0ULL >> (63 - EndBit));

				if ((Mask & CheckMask) != 0)
				{
					return false;
				}
			}
		}
	}
	return true;
}

void AFenwickVoxelVolume::UpdateDynamicObstacles()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_UpdateDynamicObstacles);
	if (!IsReadyForPathfinding() || !GetWorld()) return;

	TMap<FIntVector, int32> CurrentFrameVoxelCounts;
	GatherCurrentObstacleVoxels(CurrentFrameVoxelCounts);

	ProcessRemovedAndChangedObstacles(CurrentFrameVoxelCounts);
	ProcessNewObstacles(CurrentFrameVoxelCounts);

	DynamicObstacleVoxelCounts = MoveTemp(CurrentFrameVoxelCounts);
}

void AFenwickVoxelVolume::GatherCurrentObstacleVoxels(TMap<FIntVector, int32>& OutVoxelCounts) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_GatherObstacles);
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

			if (!ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex)) continue;

			for (int32 z = MinVoxelIndex.Z; z <= MaxVoxelIndex.Z; ++z)
			{
				for (int32 y = MinVoxelIndex.Y; y <= MaxVoxelIndex.Y; ++y)
				{
					for (int32 x = MinVoxelIndex.X; x <= MaxVoxelIndex.X; ++x)
					{
						const FIntVector CurrentVoxelIndex(x, y, z);
						if (CheckVoxelOverlap(CurrentVoxelIndex, OverlappedComponent))
						{
							OutVoxelCounts.FindOrAdd(CurrentVoxelIndex, 0)++;
						}
					}
				}
			}
		}
	}
}

void AFenwickVoxelVolume::ProcessRemovedAndChangedObstacles(const TMap<FIntVector, int32>& CurrentCounts)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_ProcessRemovals);
	for (const auto& Pair : DynamicObstacleVoxelCounts)
	{
		const FIntVector& Index = Pair.Key;
		const int32 PreviousCount = Pair.Value;
		const int32 CurrentCount = CurrentCounts.FindRef(Index);

		// State Transition: Obstacle existed and now is completely gone.
		if (PreviousCount > 0 && CurrentCount == 0)
		{
			// Only update the state if there is no static obstacle at this location.
			if (!StaticObstacleVoxels.Contains(Index))
			{
				// Update Fenwick Tree using a cuboid of size 1x1x1
				FenwUpdateCuboid(Index, Index, -1);
				UpdateVoxelObstacleState(Index, false);
			}
		}
	}
}

void AFenwickVoxelVolume::ProcessNewObstacles(const TMap<FIntVector, int32>& CurrentCounts)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_ProcessAdditions);
	for (const auto& Pair : CurrentCounts)
	{
		const FIntVector& Index = Pair.Key;

		// Process only if this voxel was NOT a dynamic obstacle in the previous frame.
		if (!DynamicObstacleVoxelCounts.Contains(Index))
		{
			// State Transition: A new obstacle appeared where there was none before (dynamically).
			// We must also check if it was already occupied by a static obstacle.
			// IsVoxelWalkable checks the combined state from the bitmask.
			if (IsVoxelWalkable(Index))
			{
				// Update Fenwick Tree using a cuboid of size 1x1x1
				FenwUpdateCuboid(Index, Index, 1);
				UpdateVoxelObstacleState(Index, true);
			}
		}
	}
}

//======================================================================
// --- PUBLIC QUERY API IMPLEMENTATION ---
//======================================================================

bool AFenwickVoxelVolume::IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_IsRegionWalkable);
	if (!IsReadyForPathfinding()) return false;
	if (!IsVoxelWalkable(VoxelIndex)) return false;
	if (AgentRadiusInVoxels < 0) return false;

	const FIntVector AgentRadius(AgentRadiusInVoxels);
	const FIntVector MinCorner = VoxelIndex - AgentRadius;
	const FIntVector MaxCorner = VoxelIndex + AgentRadius;

	if (!IsIndexValid(MinCorner) || !IsIndexValid(MaxCorner)) return false;

	return IsRegionEmpty_Bitmask(MinCorner, MaxCorner);
}

bool AFenwickVoxelVolume::IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex,
                                                int32 AgentRadiusInVoxels) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_IsPathSegmentWalkable);
	if (!IsReadyForPathfinding()) return false;

	const FIntVector AgentRadius(AgentRadiusInVoxels);
	const FIntVector FromMin = FromStartIndex - AgentRadius;
	const FIntVector FromMax = FromStartIndex + AgentRadius;
	const FIntVector ToMin = ToStartIndex - AgentRadius;
	const FIntVector ToMax = ToStartIndex + AgentRadius;

	const FIntVector SweptMin(FMath::Min(FromMin.X, ToMin.X), FMath::Min(FromMin.Y, ToMin.Y),
	                          FMath::Min(FromMin.Z, ToMin.Z));
	const FIntVector SweptMax(FMath::Max(FromMax.X, ToMax.X), FMath::Max(FromMax.Y, ToMax.Y),
	                          FMath::Max(FromMax.Z, ToMax.Z));

	return FenwRangeSum(SweptMin, SweptMax) == 0;
}

bool AFenwickVoxelVolume::IsVoxelWalkable(const FIntVector& Index) const
{
	if (!IsIndexValid(Index)) return false;

	const int32 ChunkX = Index.X / CHUNK_SIZE;
	const int32 BitPos = Index.X % CHUNK_SIZE;
	const int32 ChunkArrIndex = GetFlatIndex_X(ChunkX, Index.Y, Index.Z);

	return (ObstacleBitmasks_X[ChunkArrIndex] & (1ULL << BitPos)) == 0;
}

void AFenwickVoxelVolume::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
	OutObstacleIndices.Empty();
	if (!IsReadyForPathfinding()) return;

	TSet<FIntVector> AllObstacles = StaticObstacleVoxels;
	TArray<FIntVector> DynamicKeys;
	DynamicObstacleVoxelCounts.GenerateKeyArray(DynamicKeys);
	AllObstacles.Append(DynamicKeys);

	OutObstacleIndices = AllObstacles.Array();
}


int32 AFenwickVoxelVolume::FenwRangeSum(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_FenwRangeSum);
	if (MaxCorner.X < MinCorner.X || MaxCorner.Y < MinCorner.Y || MaxCorner.Z < MinCorner.Z)
	{
		return 0;
	}

	const FIntVector Max = MaxCorner;
	const FIntVector Min = MinCorner - FIntVector(1, 1, 1);

	const FIntVector p_mmm(Min.X, Min.Y, Min.Z);
	const FIntVector p_mmp(Min.X, Min.Y, Max.Z);
	const FIntVector p_mpm(Min.X, Max.Y, Min.Z);
	const FIntVector p_mpp(Min.X, Max.Y, Max.Z);
	const FIntVector p_pmm(Max.X, Min.Y, Min.Z);
	const FIntVector p_pmp(Max.X, Min.Y, Max.Z);
	const FIntVector p_ppm(Max.X, Max.Y, Min.Z);
	const FIntVector p_ppp(Max.X, Max.Y, Max.Z);

	int32 FinalSum = 0;
	FinalSum += FenwQuery(p_ppp);
	FinalSum -= FenwQuery(p_pmp);
	FinalSum -= FenwQuery(p_mpp);
	FinalSum -= FenwQuery(p_ppm);
	FinalSum += FenwQuery(p_mpm);
	FinalSum += FenwQuery(p_pmm);
	FinalSum += FenwQuery(p_mmp);
	FinalSum -= FenwQuery(p_mmm);

	return FinalSum;
}

void AFenwickVoxelVolume::RegisterStreamingAgent(AActor* Agent)
{
	if (Agent) StreamingAgents.Add(Agent);
}

void AFenwickVoxelVolume::UnregisterStreamingAgent(AActor* Agent)
{
	if (Agent) StreamingAgents.Remove(Agent);
}

void AFenwickVoxelVolume::RegisterChunkFromLevel(AChunkDataHolder* ChunkHolder)
{
	if (!ChunkHolder || !ChunkHolder->VoxelChunkData)
	{
		UE_LOG(LogTemp, Warning,
		       TEXT("RegisterChunkFromLevel: Received an invalid ChunkHolder or it has no VoxelChunkData."));
		return;
	}

	const FIntVector& ChunkCoords = ChunkHolder->ChunkCoords;
	UStaticVoxelChunkData* LoadedData = ChunkHolder->VoxelChunkData;

	// 로드 대기 목록에서 제거
	PendingStreamingLevels.Remove(ChunkCoords);

	const FIntVector ChunkOriginGlobal = ChunkCoords * CHUNK_SIZE;

	// 청크 데이터에 포함된 모든 장애물 복셀을 순회하며 시스템에 등록합니다.
	for (int32 i = 0; i < LoadedData->ObstacleBitmasks_X.Num(); ++i)
	{
		uint64 Mask = LoadedData->ObstacleBitmasks_X[i];
		if (Mask == 0) continue;

		const int32 LocalZ = i / CHUNK_SIZE;
		const int32 LocalY = i % CHUNK_SIZE;

		// 비트마스크에서 연속된 '1' (장애물 덩어리)을 찾아 한 번에 처리합니다.
		while (Mask != 0)
		{
			unsigned long StartBitIndex;
			_BitScanForward64(&StartBitIndex, Mask);

			uint64 TempMask = Mask >> StartBitIndex;
			unsigned long EndBitOffset;
			_BitScanForward64(&EndBitOffset, ~TempMask);

			// 64비트가 모두 1인 경우 EndBitOffset이 0으로 나올 수 있어 예외 처리
			if (EndBitOffset == 0 && TempMask != 1) EndBitOffset = 64;

			const uint32 RunLength = (EndBitOffset == 0) ? 1 : EndBitOffset;
			const uint32 EndBitIndex = StartBitIndex + RunLength - 1;

			// 펜윅 트리를 업데이트하기 위한 월드 좌표 계산
			const FIntVector WorldMin = ChunkOriginGlobal + FIntVector(StartBitIndex, LocalY, LocalZ);
			const FIntVector WorldMax = ChunkOriginGlobal + FIntVector(EndBitIndex, LocalY, LocalZ);

			// 펜윅 트리에 장애물 개수(Delta=1)를 더합니다.
			FenwUpdateCuboid(WorldMin, WorldMax, 1);

			// 처리된 비트를 마스크에서 제거합니다.
			const uint64 ProcessedMask = (~0ULL >> (64 - RunLength)) << StartBitIndex;
			Mask &= ~ProcessedMask;

			// 각 복셀에 대해 개별 상태를 업데이트합니다.
			for (uint32 x = StartBitIndex; x <= EndBitIndex; ++x)
			{
				const FIntVector ObstacleIndex = ChunkOriginGlobal + FIntVector(x, LocalY, LocalZ);
				StaticObstacleVoxels.Add(ObstacleIndex);
				UpdateVoxelObstacleState(ObstacleIndex, true);
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Chunk %s registered from loaded level."), *ChunkCoords.ToString());
}

void AFenwickVoxelVolume::UnregisterChunkFromLevel(AChunkDataHolder* ChunkHolder)
{
	if (!ChunkHolder || !ChunkHolder->VoxelChunkData)
	{
		UE_LOG(LogTemp, Warning,
		       TEXT("UnregisterChunkFromLevel: Received an invalid ChunkHolder or it has no VoxelChunkData."));
		return;
	}

	const FIntVector& ChunkCoords = ChunkHolder->ChunkCoords;
	UStaticVoxelChunkData* DataAsset = ChunkHolder->VoxelChunkData;

	const FIntVector ChunkOriginGlobal = ChunkCoords * CHUNK_SIZE;

	// 청크 데이터에 포함된 모든 장애물 복셀을 순회하며 시스템에서 등록 해제합니다.
	for (int32 i = 0; i < DataAsset->ObstacleBitmasks_X.Num(); ++i)
	{
		uint64 Mask = DataAsset->ObstacleBitmasks_X[i];
		if (Mask == 0) continue;

		const int32 LocalZ = i / CHUNK_SIZE;
		const int32 LocalY = i % CHUNK_SIZE;

		while (Mask != 0)
		{
			unsigned long StartBitIndex;
			_BitScanForward64(&StartBitIndex, Mask);

			uint64 TempMask = Mask >> StartBitIndex;
			unsigned long EndBitOffset;
			_BitScanForward64(&EndBitOffset, ~TempMask);

			if (EndBitOffset == 0 && TempMask != 1) EndBitOffset = 64;

			const uint32 RunLength = (EndBitOffset == 0) ? 1 : EndBitOffset;
			const uint32 EndBitIndex = StartBitIndex + RunLength - 1;

			const FIntVector WorldMin = ChunkOriginGlobal + FIntVector(StartBitIndex, LocalY, LocalZ);
			const FIntVector WorldMax = ChunkOriginGlobal + FIntVector(EndBitIndex, LocalY, LocalZ);

			// 펜윅 트리에 장애물 개수(Delta=-1)를 뺍니다.
			FenwUpdateCuboid(WorldMin, WorldMax, -1);

			const uint64 ProcessedMask = (~0ULL >> (64 - RunLength)) << StartBitIndex;
			Mask &= ~ProcessedMask;

			// 각 복셀에 대해 개별 상태를 업데이트합니다.
			for (uint32 x = StartBitIndex; x <= EndBitIndex; ++x)
			{
				const FIntVector ObstacleIndex = ChunkOriginGlobal + FIntVector(x, LocalY, LocalZ);
				if (IsIndexValid(ObstacleIndex))
				{
					StaticObstacleVoxels.Remove(ObstacleIndex);
					// 동적 장애물이 해당 위치를 점유하고 있지 않은 경우에만 복셀 상태를 '걸을 수 있음'으로 변경합니다.
					if (!DynamicObstacleVoxelCounts.Contains(ObstacleIndex))
					{
						UpdateVoxelObstacleState(ObstacleIndex, false);
					}
				}
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Chunk %s unregistered from unloaded level."), *ChunkCoords.ToString());
}

//======================================================================
// --- EDITOR BAKING ---
//======================================================================
#if WITH_EDITOR

void AFenwickVoxelVolume::BakeStaticDataToChunks()
{
	if (!ValidateBakePrerequisites()) return;

	const FString FinalSaveDirectory = GetFinalSaveDirectory();

	
	FScopedSlowTask BakeTask(5, FText::FromString(TEXT("Baking Voxel Data...")));
	BakeTask.MakeDialog();

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Cleaning up old chunk assets...")));
	CleanUpAllChunkAssets();

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Gathering bake exclusions...")));
	TSet<FIntVector> ExcludedVoxels;
	GatherBakeExclusionVoxels(ExcludedVoxels);

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Scanning for static obstacles...")));
	TSet<FIntVector> AllStaticObstacles;
	ScanForAllStaticObstacles(ExcludedVoxels, AllStaticObstacles);

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Grouping obstacles into chunks...")));
	TMap<FIntVector, TArray<FIntVector>> ObstaclesByChunk;
	GroupObstaclesIntoChunks(AllStaticObstacles, ObstaclesByChunk);

	BakeTask.EnterProgressFrame(1, FText::FromString(TEXT("Creating new chunk assets...")));
	CreateOrUpdateChunkAssets(ObstaclesByChunk, FinalSaveDirectory);
	UE_LOG(LogTemp, Log, TEXT("Static data baking process completed successfully."));
}

#endif

//======================================================================
// --- HELPER FUNCTIONS ---
//======================================================================
FIntVector AFenwickVoxelVolume::WorldToChunkCoords(const FVector& WorldPos) const
{
	const FVector LocalPos = GetTransform().InverseTransformPosition(WorldPos);
	const float ChunkWorldSize = CHUNK_SIZE * VoxelExtent;
	return FIntVector(
		FMath::FloorToInt(LocalPos.X / ChunkWorldSize),
		FMath::FloorToInt(LocalPos.Y / ChunkWorldSize),
		FMath::FloorToInt(LocalPos.Z / ChunkWorldSize)
	);
}

FORCEINLINE int32 AFenwickVoxelVolume::GetFenwickFlatIndex(int32 x, int32 y, int32 z) const
{
	return (z * FenwickTreeSize.Y * FenwickTreeSize.X) + (y * FenwickTreeSize.X) + x;
}

FORCEINLINE uint32 AFenwickVoxelVolume::GetFlatIndex_X(uint32 ChunkX, uint32 Y, uint32 Z) const
{
	const uint32 ChunksAlongX = MapSize.X / CHUNK_SIZE;
	return (Z * MapSize.Y + Y) * ChunksAlongX + ChunkX;
}

FORCEINLINE uint32 AFenwickVoxelVolume::GetFlatIndex_Y(uint32 ChunkY, uint32 X, uint32 Z) const
{
	const uint32 ChunksAlongY = MapSize.Y / CHUNK_SIZE;
	return (Z * MapSize.X + X) * ChunksAlongY + ChunkY;
}

FORCEINLINE uint32 AFenwickVoxelVolume::GetFlatIndex_Z(uint32 ChunkZ, uint32 X, uint32 Y) const
{
	const uint32 ChunksAlongZ = MapSize.Z / CHUNK_SIZE;
	return (Y * MapSize.X + X) * ChunksAlongZ + ChunkZ;
}

bool AFenwickVoxelVolume::IsVoxelIndexValid(const FIntVector& VoxelIndex) const
{
	return VoxelIndex.X >= 0 && VoxelIndex.X < MapSize.X &&
		VoxelIndex.Y >= 0 && VoxelIndex.Y < MapSize.Y &&
		VoxelIndex.Z >= 0 && VoxelIndex.Z < MapSize.Z;
}

FString AFenwickVoxelVolume::GetUniqueAssetPrefix() const
{
	return TEXT("DA_Chunk_");
}


#if WITH_EDITOR

// 헬퍼 함수 1: 사전 조건 검사
bool AFenwickVoxelVolume::ValidateBakePrerequisites() const
{
	if (MapSize.X <= 0 || MapSize.Y <= 0 || MapSize.Z <= 0 || MapSize.X % CHUNK_SIZE != 0 || MapSize.Y % CHUNK_SIZE != 0
		|| MapSize.Z % CHUNK_SIZE != 0)
	{
		UE_LOG(LogTemp, Error, TEXT("Baking failed: MapSize must be a positive multiple of CHUNK_SIZE (%d)."),
		       CHUNK_SIZE);
		return false;
	}
	if (BakedDataSaveDirectory.IsEmpty())
	{
		UE_LOG(LogTemp, Error, TEXT("Baking failed: Please specify a valid 'Baked Data Save Directory'."));
		return false;
	}
	return true;
}

// 헬퍼 함수 2: 제외 복셀 수집
void AFenwickVoxelVolume::GatherBakeExclusionVoxels(TSet<FIntVector>& OutExcludedVoxels)
{
	OutExcludedVoxels.Empty();

	auto AddActorBoundsToExclusionSet = [this, &OutExcludedVoxels](AActor* ActorToExclude)
	{
		if (!ActorToExclude) return;
		FVector Origin, BoxExtent;
		ActorToExclude->GetActorBounds(true, Origin, BoxExtent);
		const FIntVector MinVoxel = WorldToVoxelIndex(Origin - BoxExtent);
		const FIntVector MaxVoxel = WorldToVoxelIndex(Origin + BoxExtent);

		for (int32 z = MinVoxel.Z; z <= MaxVoxel.Z; ++z)
			for (int32 y = MinVoxel.Y; y <= MaxVoxel.Y; ++y)
				for (int32 x = MinVoxel.X; x <= MaxVoxel.X; ++x)
					OutExcludedVoxels.Add(FIntVector(x, y, z));
	};

	for (AActor* ExcludedActor : BakeExcludeActors)
	{
		AddActorBoundsToExclusionSet(ExcludedActor);
	}

	for (TObjectIterator<UPathfindingComponent> It; It; ++It)
	{
		UPathfindingComponent* PF = *It;
		if (PF && PF->GetWorld() == GetWorld() && PF->GetTargetVolume() == this)
		{
			if (AActor* TargetActor = PF->GetTargetActor())
			{
				AddActorBoundsToExclusionSet(TargetActor);
			}
		}
	}
	UE_LOG(LogTemp, Log, TEXT("Gathered %d voxels to exclude from baking."), OutExcludedVoxels.Num());
}

// 헬퍼 함수 3: 정적 장애물 스캔
void AFenwickVoxelVolume::ScanForAllStaticObstacles(const TSet<FIntVector>& ExcludedVoxels,
                                                    TSet<FIntVector>& OutStaticObstacles) const
{
	OutStaticObstacles.Empty();
	FCollisionObjectQueryParams StaticObjectQueryParams(ECC_WorldStatic);
	TArray<FOverlapResult> OverlapResults;
	GetOverlappingComponentsInVolume(StaticObjectQueryParams, OverlapResults);

	for (const FOverlapResult& Result : OverlapResults)
	{
		if (UPrimitiveComponent* Comp = Result.GetComponent())
		{
			const FBox ComponentBounds = Comp->Bounds.GetBox();

			FIntVector MinVoxel = WorldToVoxelIndex(ComponentBounds.Min);
			FIntVector MaxVoxel = WorldToVoxelIndex(ComponentBounds.Max);

			if (!ClampVoxelIndicesToVolume(MinVoxel, MaxVoxel))
			{
				continue;
			}
			for (int32 z = MinVoxel.Z; z <= MaxVoxel.Z; ++z)
			{
				for (int32 y = MinVoxel.Y; y <= MaxVoxel.Y; ++y)
				{
					for (int32 x = MinVoxel.X; x <= MaxVoxel.X; ++x)
					{
						const FIntVector VoxelIndex(x, y, z);
						if (ExcludedVoxels.Contains(VoxelIndex) || OutStaticObstacles.Contains(VoxelIndex))
						{
							continue;
						}
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

// 헬퍼 함수 4: 청크로 그룹화
void AFenwickVoxelVolume::GroupObstaclesIntoChunks(const TSet<FIntVector>& AllObstacles,
                                                   TMap<FIntVector, TArray<FIntVector>>& OutObstaclesByChunk) const
{
	OutObstaclesByChunk.Empty();
	for (const FIntVector& ObstacleVoxel : AllObstacles)
	{
		const FIntVector ChunkCoord(
			FMath::FloorToInt(static_cast<float>(ObstacleVoxel.X) / CHUNK_SIZE),
			FMath::FloorToInt(static_cast<float>(ObstacleVoxel.Y) / CHUNK_SIZE),
			FMath::FloorToInt(static_cast<float>(ObstacleVoxel.Z) / CHUNK_SIZE)
		);
		OutObstaclesByChunk.FindOrAdd(ChunkCoord).Add(ObstacleVoxel);
	}
	UE_LOG(LogTemp, Log, TEXT("Grouped obstacles into %d non-empty chunks."), OutObstaclesByChunk.Num());
}

void AFenwickVoxelVolume::CleanUpAllChunkAssets() const
{
    // 1. 에디터 월드 컨텍스트 획득
    UWorld* EditorWorld = GetWorld();
    if (!EditorWorld || !EditorWorld->IsEditorWorld())
    {
        UE_LOG(LogTemp, Warning, TEXT("CleanUpAllChunkAssets: Editor world context is not available. Aborting cleanup."));
        return;
    }

    // 2. 청크(.umap) 및 데이터 애셋(.uasset) 디렉토리 경로
    const FString ChunksDirectory = FPaths::Combine(GetFinalSaveDirectory(), TEXT("Chunks"));
    const FString DataDirectory   = FPaths::Combine(GetFinalSaveDirectory(), TEXT("Data"));

    UE_LOG(LogTemp, Log, TEXT("Starting cleanup for directories: %s and %s"), *ChunksDirectory, *DataDirectory);

    // 3. 애셋 레지스트리를 통해 삭제할 모든 애셋 수집
    FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
    IAssetRegistry& AssetRegistry = AssetRegistryModule.Get();
    
    TArray<FString> PathsToScan;
    PathsToScan.Add(ChunksDirectory);
    PathsToScan.Add(DataDirectory);
    AssetRegistry.ScanPathsSynchronous(PathsToScan, /*bForceRescan=*/true);

    TArray<FAssetData> AssetsToDelete;
    AssetRegistry.GetAssetsByPath(FName(*ChunksDirectory), AssetsToDelete, /*bRecursive=*/ true);
    TArray<FAssetData> DataAssetsToDelete;
    AssetRegistry.GetAssetsByPath(FName(*DataDirectory), DataAssetsToDelete, /*bRecursive=*/ true);
    AssetsToDelete.Append(DataAssetsToDelete);

    if (AssetsToDelete.IsEmpty())
    {
        UE_LOG(LogTemp, Log, TEXT("No chunk assets found to clean up."));
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("Found %d assets to delete. Attempting to delete..."), AssetsToDelete.Num());

    const int32 DeletedCount = ObjectTools::DeleteAssets(AssetsToDelete, /*bShowConfirmation=*/ false);

    if (DeletedCount > 0)
    {
        UE_LOG(LogTemp, Log, TEXT("Successfully deleted %d assets."), DeletedCount);
        
        // ** (중요) 가비지 컬렉션을 동기식으로 강제 실행하여 메모리에서 완전히 제거합니다. **
        // 이것이 이름 중복 및 "부분 로드" 문제를 해결하는 핵심입니다.
        CollectGarbage(GARBAGE_COLLECTION_KEEPFLAGS, true);

        AssetRegistry.ScanPathsSynchronous(PathsToScan, /*bForceRescan=*/ true);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("ObjectTools::DeleteAssets did not delete any assets, though %d were found. They might be in use or read-only."), AssetsToDelete.Num());
    }
}

void AFenwickVoxelVolume::CreateOrUpdateChunkAssets(const TMap<FIntVector, TArray<FIntVector>>& ObstaclesByChunk, const FString& InFinalSaveDirectory)
{
    if (ObstaclesByChunk.Num() == 0) return;
    check(GEditor);

    const FString DataAssetDirectory = FPaths::Combine(InFinalSaveDirectory, TEXT("Data"));
    const FString ChunksDirectory = FPaths::Combine(InFinalSaveDirectory, TEXT("Chunks"));
    
    FAssetToolsModule& AssetToolsModule = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools");
    IAssetTools& AssetTools = AssetToolsModule.Get();
 
    TArray<UObject*> ObjectsToSave;

    for (const auto& Pair : ObstaclesByChunk)
    {
        const FIntVector& ChunkCoord = Pair.Key;
        
        // --- 데이터 애셋 생성 ---
        const FString DataAssetName = FString::Printf(TEXT("DA_ChunkData_%d_%d_%d"), ChunkCoord.X, ChunkCoord.Y, ChunkCoord.Z);
        
        UStaticVoxelChunkData* ChunkAsset = Cast<UStaticVoxelChunkData>(
            AssetTools.CreateAsset(DataAssetName, DataAssetDirectory, UStaticVoxelChunkData::StaticClass(), nullptr)
        );

        if (!ChunkAsset)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create Data Asset: %s"), *DataAssetName);
            continue;
        }

        ChunkAsset->ChunkCoords = ChunkCoord;
        const int32 BitmaskArraySize = CHUNK_SIZE * CHUNK_SIZE;
        ChunkAsset->ObstacleBitmasks_X.Init(0, BitmaskArraySize);

        for (const FIntVector& ObstacleVoxel : Pair.Value)
        {
            const int32 LocalX = ObstacleVoxel.X % CHUNK_SIZE;
            const int32 LocalY = ObstacleVoxel.Y % CHUNK_SIZE;
            const int32 LocalZ = ObstacleVoxel.Z % CHUNK_SIZE;
            ChunkAsset->ObstacleBitmasks_X[(LocalZ * CHUNK_SIZE) + LocalY] |= (1ULL << LocalX);
        }
        
        // 데이터 애셋은 저장이 필요하므로 목록에 추가합니다.
        ChunkAsset->MarkPackageDirty();
        ObjectsToSave.Add(ChunkAsset);

        // --- 서브 레벨 생성 및 저장 ---
        const FString LevelAssetName = FString::Printf(TEXT("Chunk_%d_%d_%d"), ChunkCoord.X, ChunkCoord.Y, ChunkCoord.Z);
        const FString LevelPackagePath = FPaths::Combine(ChunksDirectory, LevelAssetName);
        
        UWorld* SubLevelWorld = UEditorLoadingAndSavingUtils::NewBlankMap(false);
        if (!SubLevelWorld)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create new blank map for %s"), *LevelAssetName);
            continue;
        }

        FActorSpawnParameters Params;
        Params.OverrideLevel = SubLevelWorld->PersistentLevel;
        Params.ObjectFlags = RF_Transactional;
        AChunkDataHolder* Holder = SubLevelWorld->SpawnActor<AChunkDataHolder>(AChunkDataHolder::StaticClass(), FTransform::Identity, Params);

        if (Holder)
        {
            Holder->ChunkCoords = ChunkCoord;
            Holder->VoxelChunkData = ChunkAsset;
        }

        // SaveMap 함수가 맵을 디스크에 저장하는 작업을 이미 완료합니다.
        if (UEditorLoadingAndSavingUtils::SaveMap(SubLevelWorld, LevelPackagePath))
        {
            // ** (수정) 이미 저장된 월드를 다시 저장할 필요가 없으므로 이 줄을 제거/주석 처리합니다. **
            // ObjectsToSave.Add(SubLevelWorld); 
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to save sublevel: %s"), *LevelPackagePath);
        }
    }

    // 생성/수정된 모든 '데이터 애셋'들을 한 번에 저장합니다.
    if (ObjectsToSave.Num() > 0)
    {
        TSet<UPackage*> UniquePackagesToSave;
        for (UObject* ObjToSave : ObjectsToSave)
        {
            if (ObjToSave)
            {
                UniquePackagesToSave.Add(ObjToSave->GetPackage());
            }
        }
        UEditorLoadingAndSavingUtils::SavePackages(UniquePackagesToSave.Array(), true);
    }
}
#endif // WITH_EDITOR


FString AFenwickVoxelVolume::GetFinalSaveDirectory() const
{
	if (!GetWorld())
	{
		UE_LOG(LogTemp, Error, TEXT("GetFinalSaveDirectory: GetWorld() returned null. Cannot determine level name."));
		return BakedDataSaveDirectory;
	}
	const FString LevelName = FPackageName::GetShortName(GetWorld()->GetName());
	const FString ActorName = GetName();
	return FPaths::Combine(BakedDataSaveDirectory, LevelName, ActorName);
}


FString AFenwickVoxelVolume::GetChunkAssetPath(const FIntVector& ChunkCoords) const
{
	const FString ChunksDirectory = FPaths::Combine(GetFinalSaveDirectory(), TEXT("Chunks"));
	const FString AssetName = FString::Printf(TEXT("Chunk_%d_%d_%d"), ChunkCoords.X, ChunkCoords.Y, ChunkCoords.Z);
	return FPaths::Combine(ChunksDirectory, AssetName);
}
 