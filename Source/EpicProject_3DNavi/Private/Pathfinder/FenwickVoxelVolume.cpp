// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/FenwickVoxelVolume.h"
#include "EpicProject_3DNavi/Voxel/DynamicObstacle.h"

AFenwickVoxelVolume::AFenwickVoxelVolume()
{
	PrimaryActorTick.bCanEverTick = true;
	bIsReady = false; // Not ready until BeginPlay is complete.
}

void AFenwickVoxelVolume::BeginPlay()
{
	Super::BeginPlay();
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_BeginPlay);

	InitializeDataStructures();
	BuildObstacles();

	// Signal that the volume is now fully initialized and safe for pathfinding queries.
	bIsReady = true;
	UE_LOG(LogTemp, Log, TEXT("FenwickVoxelVolume is now initialized and ready."));
}

void AFenwickVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (IsReadyForPathfinding())
	{
		UpdateDynamicObstacles();
	}
}

int32 AFenwickVoxelVolume::FenwRangeSum(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
{
    TRACE_CPUPROFILER_EVENT_SCOPE(JPS_FenwRangeSum);
    if (MaxCorner.X < MinCorner.X || MaxCorner.Y < MinCorner.Y || MaxCorner.Z < MinCorner.Z)
    {
        return 0;
    }

    // To calculate the sum of a 3D range [min, max], we use the inclusion-exclusion principle.
    // This requires querying the cumulative sum at the 8 corners of the expanded volume.
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
					return false; // Obstacle found
				}
			}
		}
	}
	return true;
}

#pragma region Unchanged Code
void AFenwickVoxelVolume::InitializeDataStructures()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_InitializeMap);

	// Ensure map dimensions are compatible with bitmask chunking.
	check(MapSize.X > 0 && MapSize.X % CHUNK_SIZE == 0);
	check(MapSize.Y > 0 && MapSize.Y % CHUNK_SIZE == 0);
	check(MapSize.Z > 0 && MapSize.Z % CHUNK_SIZE == 0);

	// Initialize Fenwick Tree
	FenwickTreeSize = MapSize + FIntVector(1, 1, 1);
	const int32 TotalFenwickSize = FenwickTreeSize.X * FenwickTreeSize.Y * FenwickTreeSize.Z;
	FenwickTree.Empty(TotalFenwickSize);
	FenwickTree.AddZeroed(TotalFenwickSize);

	// Initialize Bitmasks
	const int32 ChunksAlongX = MapSize.X / CHUNK_SIZE;
	ObstacleBitmasks_X.Init(0, ChunksAlongX * MapSize.Y * MapSize.Z);

	const int32 ChunksAlongY = MapSize.Y / CHUNK_SIZE;
	ObstacleBitmasks_Y.Init(0, ChunksAlongY * MapSize.X * MapSize.Z);

	const int32 ChunksAlongZ = MapSize.Z / CHUNK_SIZE;
	ObstacleBitmasks_Z.Init(0, ChunksAlongZ * MapSize.X * MapSize.Y);
}

void AFenwickVoxelVolume::BuildObstacles()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_BuildObstacles);
	
	FCollisionObjectQueryParams StaticObjectQueryParams(ECC_WorldStatic);
	StaticObstacleVoxels.Empty();
	
	TArray<FOverlapResult> OverlapResults;
	if (!GetOverlappingComponentsInVolume(StaticObjectQueryParams, OverlapResults))
	{
		return;
	}

	for (const FOverlapResult& Result : OverlapResults)
	{
		UPrimitiveComponent* OverlappedComponent = Result.GetComponent();
		if (!OverlappedComponent || !OverlappedComponent->IsQueryCollisionEnabled() || OverlappedComponent->GetOwner()->GetComponentByClass<UDynamicObstacle>())
		{
			continue;
		}

		const FBox ObstacleBounds = OverlappedComponent->Bounds.GetBox();
		FIntVector MinVoxelIndex = WorldToVoxelIndex(ObstacleBounds.Min);
		FIntVector MaxVoxelIndex = WorldToVoxelIndex(ObstacleBounds.Max - FVector(KINDA_SMALL_NUMBER));

		if (!ClampVoxelIndicesToVolume(MinVoxelIndex, MaxVoxelIndex))
		{
			continue;
		}
		
		for (int32 z = MinVoxelIndex.Z; z <= MaxVoxelIndex.Z; ++z)
		{
			for (int32 y = MinVoxelIndex.Y; y <= MaxVoxelIndex.Y; ++y)
			{
				for (int32 x = MinVoxelIndex.X; x <= MaxVoxelIndex.X; ++x)
				{
					const FIntVector CurrentVoxelIndex(x, y, z);
					if (CheckVoxelOverlap(CurrentVoxelIndex, OverlappedComponent))
					{
						UpdateVoxelObstacleState(CurrentVoxelIndex, true);
						StaticObstacleVoxels.Add(CurrentVoxelIndex);
						FenwUpdate(CurrentVoxelIndex, 1);
					}
				}
			}
		}
	}
}

bool AFenwickVoxelVolume::IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_IsRegionWalkable);

	if (!IsReadyForPathfinding())
	{
		return false;
	}
	if (!IsVoxelWalkable(VoxelIndex))
	{
		return false;
	}
	if (AgentRadiusInVoxels < 0)
	{
		return false;
	}
	const FIntVector AgentRadius(AgentRadiusInVoxels);
	const FIntVector MinCorner = VoxelIndex - AgentRadius;
	const FIntVector MaxCorner = VoxelIndex + AgentRadius;

	if (!IsIndexValid(MinCorner) || !IsIndexValid(MaxCorner))
	{
		return false;
	}

	// Broad-phase: Use fast bitmask check to see if the region is guaranteed to be empty.
	if (IsRegionEmpty_Bitmask(MinCorner, MaxCorner))
	{
		return true;
	}
	
	// Narrow-phase: If bitmask fails, use the precise Fenwick Tree to count obstacles.
	return FenwRangeSum(MinCorner, MaxCorner) == 0;
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

void AFenwickVoxelVolume::UpdateDynamicObstacles()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_UpdateDynamicObstacles);
	if (!GetWorld()) return;

	TMap<FIntVector, int32> CurrentFrameVoxelCounts;
	GatherCurrentObstacleVoxels(CurrentFrameVoxelCounts);

	ProcessRemovedAndChangedObstacles(CurrentFrameVoxelCounts);
	ProcessNewObstacles(CurrentFrameVoxelCounts);

	DynamicObstacleVoxelCounts = MoveTemp(CurrentFrameVoxelCounts);
}

void AFenwickVoxelVolume::GatherCurrentObstacleVoxels(TMap<FIntVector, int32>& OutVoxelCounts) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_GatherObstacles);
	OutVoxelCounts.Empty();

	FCollisionObjectQueryParams DynamicObjectQueryParams;
	DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_WorldDynamic);
	DynamicObjectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);

	TArray<FOverlapResult> OverlapResults;
	if (!GetOverlappingComponentsInVolume(DynamicObjectQueryParams, OverlapResults)) return;

	for (const FOverlapResult& Result : OverlapResults)
	{
		UPrimitiveComponent* OverlappedComponent = Result.GetComponent();
		if (!OverlappedComponent || !OverlappedComponent->IsQueryCollisionEnabled() || !OverlappedComponent->GetOwner()->GetComponentByClass<UDynamicObstacle>())
		{
			continue;
		}

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

void AFenwickVoxelVolume::ProcessRemovedAndChangedObstacles(const TMap<FIntVector, int32>& CurrentCounts)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(FenwickVolume_ProcessRemovals);
	for (const auto& Pair : DynamicObstacleVoxelCounts)
	{
		const FIntVector& Index = Pair.Key;
		const int32 PreviousCount = Pair.Value;
		const int32 CurrentCount = CurrentCounts.FindRef(Index); // Returns 0 if not found

		if (CurrentCount != PreviousCount)
		{
			FenwUpdate(Index, CurrentCount - PreviousCount);
			if (CurrentCount == 0) // Obstacle was fully removed
			{
				// Only clear the bit if no static obstacle is also present
				if (!StaticObstacleVoxels.Contains(Index))
				{
					UpdateVoxelObstacleState(Index, false);
				}
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
		if (!DynamicObstacleVoxelCounts.Contains(Index))
		{
			FenwUpdate(Index, Pair.Value);
			UpdateVoxelObstacleState(Index, true);
		}
	}
}

bool AFenwickVoxelVolume::IsVoxelWalkable(const FIntVector& Index) const
{
	if (!IsIndexValid(Index)) return false;

	const int32 ChunkX = Index.X / CHUNK_SIZE;
	const int32 BitPos = Index.X % CHUNK_SIZE;
	const int32 ChunkArrIndex = GetFlatIndex_X(ChunkX, Index.Y, Index.Z);

	return (ObstacleBitmasks_X[ChunkArrIndex] & (1ULL << BitPos)) == 0;
}

void AFenwickVoxelVolume::UpdateVoxelObstacleState(const FIntVector& Index, bool bIsObstacle)
{
	if (!IsIndexValid(Index)) return;

	// X-axis bitmask
	{
		const uint32 ChunkX = Index.X / CHUNK_SIZE;
		const uint32 BitPos = Index.X % CHUNK_SIZE;
		const uint32 ChunkArrIndex = GetFlatIndex_X(ChunkX, Index.Y, Index.Z);
		if (bIsObstacle) ObstacleBitmasks_X[ChunkArrIndex] |= (1ULL << BitPos);
		else ObstacleBitmasks_X[ChunkArrIndex] &= ~(1ULL << BitPos);
	}
	// Y-axis bitmask
	{
		const int32 ChunkY = Index.Y / CHUNK_SIZE;
		const int32 BitPos = Index.Y % CHUNK_SIZE;
		const int32 ChunkArrIndex = GetFlatIndex_Y(ChunkY, Index.X, Index.Z);
		if (bIsObstacle) ObstacleBitmasks_Y[ChunkArrIndex] |= (1ULL << BitPos);
		else ObstacleBitmasks_Y[ChunkArrIndex] &= ~(1ULL << BitPos);
	}
	// Z-axis bitmask
	{
		const int32 ChunkZ = Index.Z / CHUNK_SIZE;
		const int32 BitPos = Index.Z % CHUNK_SIZE;
		const int32 ChunkArrIndex = GetFlatIndex_Z(ChunkZ, Index.X, Index.Y);
		if (bIsObstacle) ObstacleBitmasks_Z[ChunkArrIndex] |= (1ULL << BitPos);
		else ObstacleBitmasks_Z[ChunkArrIndex] &= ~(1ULL << BitPos);
	}
}

void AFenwickVoxelVolume::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
	OutObstacleIndices.Empty();
	if (!IsReadyForPathfinding()) return;

	OutObstacleIndices.Reserve(StaticObstacleVoxels.Num() + DynamicObstacleVoxelCounts.Num());
	OutObstacleIndices.Append(StaticObstacleVoxels.Array());
	DynamicObstacleVoxelCounts.GetKeys(OutObstacleIndices);
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
#pragma endregion