#include "Pathfinder/FenwickVoxelVolume.h"

#include "ObjectTools.h"
#include "Engine/AssetManager.h"
#include "Engine/StreamableManager.h"
#include "Pathfinder/PathfindingComponent.h"
#include "Pathfinder/StaticVoxelChunkData.h"


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

	GetWorldTimerManager().SetTimer(StreamingUpdateTimer, this, &AFenwickVoxelVolume::UpdateStreamingState, 1.0f, true, 0.5f);
	GetWorldTimerManager().SetTimer(DynamicObstacleTimer, this, &AFenwickVoxelVolume::UpdateDynamicObstacles, 0.3f, true, 0.1f);

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
		TWeakObjectPtr<AActor> AgentPtr = *It;
		if (AgentPtr.IsValid())
		{
			AActor* Agent = AgentPtr.Get();
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
	if (PendingLoadChunks.Contains(ChunkCoords) || ActiveChunks.Contains(ChunkCoords))
	{
		return;
	}

	const FString AssetPrefix = GetUniqueAssetPrefix();
	const FString AssetName = FString::Printf(TEXT("%s%d_%d_%d"), *AssetPrefix, ChunkCoords.X, ChunkCoords.Y, ChunkCoords.Z);
	FString AssetPath = FPaths::Combine(BakedDataSaveDirectory, AssetName) + TEXT(".") + AssetName;
	FSoftObjectPath SoftPath(AssetPath);
	
	PendingLoadChunks.Add(ChunkCoords);
	UAssetManager::Get().GetStreamableManager().RequestAsyncLoad(SoftPath, FStreamableDelegate::CreateUObject(this, &AFenwickVoxelVolume::OnChunkLoadCompleted, SoftPath, ChunkCoords));
}

void AFenwickVoxelVolume::OnChunkLoadCompleted(FSoftObjectPath LoadedAssetPath, FIntVector ChunkCoords)
{
	PendingLoadChunks.Remove(ChunkCoords);
	if (!RequiredChunks.Contains(ChunkCoords)) return;

	UStaticVoxelChunkData* LoadedData = Cast<UStaticVoxelChunkData>(LoadedAssetPath.ResolveObject());
	if (LoadedData)
	{
		ActiveChunks.Add(ChunkCoords, LoadedData);
		const FIntVector ChunkOriginGlobal = ChunkCoords * FIntVector(CHUNK_SIZE);

		for (int32 i = 0; i < LoadedData->ObstacleBitmasks_X.Num(); ++i)
		{
			uint64 Mask = LoadedData->ObstacleBitmasks_X[i];
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

				FenwUpdateCuboid(WorldMin, WorldMax, 1);
				ChunkObstacleCounter.FindOrAdd(ChunkCoords, 0) += RunLength;

				const uint64 ProcessedMask = (~0ULL >> (64 - RunLength)) << StartBitIndex;
				Mask &= ~ProcessedMask;

				for (uint32 x = StartBitIndex; x <= EndBitIndex; ++x)
				{
					const FIntVector ObstacleIndex = ChunkOriginGlobal + FIntVector(x, LocalY, LocalZ);
					StaticObstacleVoxels.Add(ObstacleIndex);
					UpdateVoxelObstacleState(ObstacleIndex, true);
				}
			}
		}
	}
	else
	{
		ActiveChunks.Add(ChunkCoords, nullptr);
	}
}

void AFenwickVoxelVolume::UnloadChunk(const FIntVector& ChunkCoords)
{
	TObjectPtr<UStaticVoxelChunkData> DataAsset;
	ChunkObstacleCounter.Remove(ChunkCoords);
	if (ActiveChunks.RemoveAndCopyValue(ChunkCoords, DataAsset))
	{
		if (DataAsset)
		{
			const FIntVector ChunkOriginGlobal = ChunkCoords * FIntVector(CHUNK_SIZE);

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

					FenwUpdateCuboid(WorldMin, WorldMax, -1);

					const uint64 ProcessedMask = (~0ULL >> (64 - RunLength)) << StartBitIndex;
					Mask &= ~ProcessedMask;

					for (uint32 x = StartBitIndex; x <= EndBitIndex; ++x)
					{
						const FIntVector ObstacleIndex = ChunkOriginGlobal + FIntVector(x, LocalY, LocalZ);
						if (IsIndexValid(ObstacleIndex))
						{
							StaticObstacleVoxels.Remove(ObstacleIndex);
							if (!DynamicObstacleVoxelCounts.Contains(ObstacleIndex))
							{
								UpdateVoxelObstacleState(ObstacleIndex, false);
							}
						}
					}
				}
			}
			UE_LOG(LogTemp, Verbose, TEXT("Chunk %s unloaded and removed from Fenwick Tree."), *ChunkCoords.ToString());
		}
	}
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
	const FIntVector MaxPlusOne = MaxCorner + FIntVector(1, 1, 1);

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
	for (int32 z = MinCorner.Z; z <= MaxCorner.Z; ++z)
	{
		for (int32 y = MinCorner.Y; y <= MaxCorner.Y; ++y)
		{
			const int32 StartChunkX = MinCorner.X / CHUNK_SIZE;
			const int32 EndChunkX = MaxCorner.X / CHUNK_SIZE;

			for (int32 cx = StartChunkX; cx <= EndChunkX; ++cx)
			{
				const uint64 Mask = ObstacleBitmasks_X[GetFlatIndex_X(cx, y, z)];
				if (Mask == 0) continue;

				const int32 VoxelStartX = (cx == StartChunkX) ? MinCorner.X : cx * CHUNK_SIZE;
				const int32 VoxelEndX = (cx == EndChunkX) ? MaxCorner.X : (cx + 1) * CHUNK_SIZE - 1;
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

bool AFenwickVoxelVolume::IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex, int32 AgentRadiusInVoxels) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_IsPathSegmentWalkable);
	if (!IsReadyForPathfinding()) return false;

	const FIntVector AgentRadius(AgentRadiusInVoxels);
	const FIntVector FromMin = FromStartIndex - AgentRadius;
	const FIntVector FromMax = FromStartIndex + AgentRadius;
	const FIntVector ToMin = ToStartIndex - AgentRadius;
	const FIntVector ToMax = ToStartIndex + AgentRadius;

	const FIntVector SweptMin(FMath::Min(FromMin.X, ToMin.X), FMath::Min(FromMin.Y, ToMin.Y), FMath::Min(FromMin.Z, ToMin.Z));
	const FIntVector SweptMax(FMath::Max(FromMax.X, ToMax.X), FMath::Max(FromMax.Y, ToMax.Y), FMath::Max(FromMax.Z, ToMax.Z));

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

//======================================================================
// --- EDITOR BAKING (No changes needed here) ---
//======================================================================
#if WITH_EDITOR
// ... The entire BakeStaticDataToChunks function and its helpers remain unchanged ...
void AFenwickVoxelVolume::BakeStaticDataToChunks()
{
	// --- (기존 유효성 검사 코드는 동일) ---
	if (MapSize.X <= 0 || MapSize.Y <= 0 || MapSize.Z <= 0 || MapSize.X % CHUNK_SIZE != 0 || MapSize.Y % CHUNK_SIZE != 0
		|| MapSize.Z % CHUNK_SIZE != 0)
	{
		UE_LOG(LogTemp, Error, TEXT("Baking failed: MapSize must be a positive multiple of CHUNK_SIZE (%d)."),
		       CHUNK_SIZE);
		return;
	}
	if (BakedDataSaveDirectory.IsEmpty())
	{
		UE_LOG(LogTemp, Error, TEXT("Baking failed: Please specify a valid 'Baked Data Save Directory'."));
		return;
	}

	// ===================================================================================
	// 1. 모든 정적 장애물 복셀을 한 번에, 수집
	// ===================================================================================
	UE_LOG(LogTemp, Log, TEXT("Step 1: Scanning for all static obstacle voxels..."));
	FScopedSlowTask VoxelScanTask(100, FText::FromString(TEXT("Scanning Static Obstacles...")));
	VoxelScanTask.MakeDialog();

	TSet<FIntVector> AllStaticObstacles;
	FCollisionObjectQueryParams StaticObjectQueryParams(ECC_WorldStatic);
	TArray<FOverlapResult> OverlapResults;
	// GetOverlappingComponentsInVolume은 볼륨 전체에 대한 컴포넌트 목록을 가져옵니다.
	GetOverlappingComponentsInVolume(StaticObjectQueryParams, OverlapResults);

	if (OverlapResults.Num() > 0)
	{
		VoxelScanTask.EnterProgressFrame(10);
		// 각 컴포넌트의 바운딩 박스 내에서만 복셀을 검사합니다.
		for (const FOverlapResult& Result : OverlapResults)
		{
			if (UPrimitiveComponent* Comp = Result.GetComponent())
			{
				const FBox ComponentBounds = Comp->Bounds.GetBox();
				FIntVector MinVoxel = WorldToVoxelIndex(ComponentBounds.Min);
				FIntVector MaxVoxel = WorldToVoxelIndex(ComponentBounds.Max);

				// 검사 범위를 볼륨 내부로 제한합니다.
				if (!ClampVoxelIndicesToVolume(MinVoxel, MaxVoxel))
				{
					continue;
				}

				// 바운딩 박스 내의 복셀만 순회
				for (int32 z = MinVoxel.Z; z <= MaxVoxel.Z; ++z)
				{
					for (int32 y = MinVoxel.Y; y <= MaxVoxel.Y; ++y)
					{
						for (int32 x = MinVoxel.X; x <= MaxVoxel.X; ++x)
						{
							const FIntVector VoxelIndex(x, y, z);
							// 이미 장애물로 판명된 복셀은 다시 검사할 필요가 없습니다.
							if (AllStaticObstacles.Contains(VoxelIndex))
							{
								continue;
							}

							// 정확한 충돌 검사
							if (CheckVoxelOverlap(VoxelIndex, Comp))
							{
								AllStaticObstacles.Add(VoxelIndex);
							}
						}
					}
				}
			}
		}
	}
	VoxelScanTask.EnterProgressFrame(40);
	UE_LOG(LogTemp, Log, TEXT("Scan complete. Found %d unique static obstacle voxels."), AllStaticObstacles.Num());


	// ===================================================================================
	// 2. 장애물이 있는 청크만 데이터로 변환
	// ===================================================================================
	UE_LOG(LogTemp, Log, TEXT("Step 2: Grouping obstacle voxels into chunks..."));

	TMap<FIntVector, TArray<FIntVector>> ObstaclesByChunk;
	for (const FIntVector& ObstacleVoxel : AllStaticObstacles)
	{
		const FIntVector ChunkCoord(
			ObstacleVoxel.X / CHUNK_SIZE,
			ObstacleVoxel.Y / CHUNK_SIZE,
			ObstacleVoxel.Z / CHUNK_SIZE
		);
		ObstaclesByChunk.FindOrAdd(ChunkCoord).Add(ObstacleVoxel);
	}
	VoxelScanTask.EnterProgressFrame(20);
	UE_LOG(LogTemp, Log, TEXT("Found %d chunks containing obstacles."), ObstaclesByChunk.Num());


	// ===================================================================================
	// ★★★ 3. 최적화: 장애물이 있는 청크에 대해서만 애셋 생성 ★★★
	// ===================================================================================
	FScopedSlowTask BakeTask(100, FText::FromString(TEXT("Baking Voxel Data...")));
	BakeTask.MakeDialog();
	
	// --- 1. 삭제할 애셋 후보 찾기 ---
	BakeTask.EnterProgressFrame(10, FText::FromString(TEXT("Finding old assets...")));

	FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>(FName("AssetRegistry"));
	TArray<FAssetData> ExistingAssets;
	AssetRegistryModule.Get().GetAssetsByPath(FName(*BakedDataSaveDirectory), ExistingAssets, true);

	
	const FString AssetPrefix = GetUniqueAssetPrefix();
	
	TSet<FName> KeepPackageNames;
	for (const auto& Pair : ObstaclesByChunk)
	{
		const FIntVector& ChunkCoord = Pair.Key;
		// 헬퍼 함수로 생성한 접두사를 사용합니다.
		const FString AssetName = FString::Printf(TEXT("%s%d_%d_%d"), *AssetPrefix, ChunkCoord.X, ChunkCoord.Y, ChunkCoord.Z);
		const FString PackagePath = FPaths::Combine(BakedDataSaveDirectory, AssetName);
		KeepPackageNames.Add(*PackagePath);
	}

	TArray<FAssetData> AssetsToDelete;
	for (const FAssetData& Asset : ExistingAssets)
	{
		if (Asset.AssetName.ToString().StartsWith(AssetPrefix) && !KeepPackageNames.Contains(Asset.PackageName))
		{
			AssetsToDelete.Add(Asset);
		}
	}

	// --- 2. 사용자에게 삭제 여부 확인 ---
	bool bPerformDeletion = false;
	if (AssetsToDelete.Num() > 0)
	{
		// FAssetData 배열을 UObject 배열로 변환해야 합니다.
		TArray<UObject*> LoadedAssetsToDelete;
		for (const FAssetData& AssetData : AssetsToDelete)
		{
			UObject* Asset = AssetData.GetAsset();
			if (Asset)
			{
				LoadedAssetsToDelete.Add(Asset);
			}
		}

		if (LoadedAssetsToDelete.Num() > 0)
		{
			UE_LOG(LogTemp, Log, TEXT("Requesting deletion of %d stale assets. Showing confirmation dialog..."), LoadedAssetsToDelete.Num());
			
			int32 DeletedAssetCount = ObjectTools::DeleteAssets(AssetsToDelete, /*bShowConfirmation=*/false);
			
			if (DeletedAssetCount > 0)
			{
				UE_LOG(LogTemp, Log, TEXT("User confirmed deletion. %d assets were deleted."), DeletedAssetCount);
			}
			else
			{
				UE_LOG(LogTemp, Log, TEXT("Asset deletion was cancelled by the user or failed."));
			}
		}
	}
	
	// --- 4. 애셋 생성/업데이트 로직 (기존과 동일) ---
	BakeTask.EnterProgressFrame(70, FText::FromString(TEXT("Baking new/updated assets...")));
	
	FAssetToolsModule& AssetToolsModule = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools");
	UDataAssetFactory* DataAssetFactory = NewObject<UDataAssetFactory>();
	DataAssetFactory->DataAssetClass = UStaticVoxelChunkData::StaticClass();
	TArray<UPackage*> PackagesToSave;

	for (const auto& Pair : ObstaclesByChunk)
	{
		// ... (이하 애셋 생성/업데이트 로직은 이전과 동일합니다) ...
		if (BakeTask.ShouldCancel()) return;

		const FIntVector& ChunkCoord = Pair.Key;
		const TArray<FIntVector>& ObstaclesInThisChunk = Pair.Value;

		const FString AssetName = FString::Printf(TEXT("%s%d_%d_%d"), *AssetPrefix, ChunkCoord.X, ChunkCoord.Y, ChunkCoord.Z);
		const FString PackagePath = FPaths::Combine(BakedDataSaveDirectory, AssetName);

		UPackage* Package = CreatePackage(*PackagePath);
		Package->FullyLoad();

		UStaticVoxelChunkData* ChunkAsset = FindObject<UStaticVoxelChunkData>(Package, *AssetName);
		if (!ChunkAsset)
		{
			ChunkAsset = Cast<UStaticVoxelChunkData>(
				DataAssetFactory->FactoryCreateNew(UStaticVoxelChunkData::StaticClass(), Package, *AssetName,
				                                   RF_Public | RF_Standalone, nullptr, GWarn));
			if(ChunkAsset) FAssetRegistryModule::AssetCreated(ChunkAsset);
		}
		
		if (ChunkAsset)
		{
			ChunkAsset->ChunkCoords = ChunkCoord;
			const int32 BitmaskArraySize = CHUNK_SIZE * CHUNK_SIZE;
			ChunkAsset->ObstacleBitmasks_X.Init(0, BitmaskArraySize);
			for (const FIntVector& ObstacleVoxel : ObstaclesInThisChunk)
			{
				const int32 LocalX = ObstacleVoxel.X % CHUNK_SIZE;
				const int32 LocalY = ObstacleVoxel.Y % CHUNK_SIZE;
				const int32 LocalZ = ObstacleVoxel.Z % CHUNK_SIZE;
				ChunkAsset->ObstacleBitmasks_X[(LocalZ * CHUNK_SIZE) + LocalY] |= (1ULL << LocalX);
			}

			Package->MarkPackageDirty();
			PackagesToSave.Add(Package);
		}
	}

	if (PackagesToSave.Num() > 0)
	{
		UEditorLoadingAndSavingUtils::SavePackages(PackagesToSave, true);
	}
}
#endif
//======================================================================
// --- HELPER FUNCTIONS (No changes needed here) ---
//======================================================================
FIntVector AFenwickVoxelVolume::WorldToChunkCoords(const FVector& WorldPos) const
{
	const FVector LocalPos = GetTransform().InverseTransformPosition(WorldPos);
	return FIntVector(
		FMath::FloorToInt(LocalPos.X / (CHUNK_SIZE * VoxelExtent)),
		FMath::FloorToInt(LocalPos.Y / (CHUNK_SIZE * VoxelExtent)),
		FMath::FloorToInt(LocalPos.Z / (CHUNK_SIZE * VoxelExtent))
	);
}

FString AFenwickVoxelVolume::GetUniqueAssetPrefix() const
{
	const FString WorldName = GetWorld()->GetName();
	FString SanitizedWorldName = WorldName;
	SanitizedWorldName.ReplaceInline(TEXT(" "), TEXT("_"));
	SanitizedWorldName.ReplaceInline(TEXT("/"), TEXT("_"));
	SanitizedWorldName.ReplaceInline(TEXT("."), TEXT("_"));

	const FString ActorName = GetName();
	FString SanitizedActorName = ActorName;
	SanitizedActorName.ReplaceInline(TEXT(" "), TEXT("_"));
	SanitizedActorName.ReplaceInline(TEXT("/"), TEXT("_"));
	SanitizedActorName.ReplaceInline(TEXT("."), TEXT("_"));

	return FString::Printf(TEXT("DA_%s_%s_VoxelChunk_"), *SanitizedWorldName, *SanitizedActorName);
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