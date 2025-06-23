// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/VoxelVolume.h"

// Sets default values
AVoxelVolume::AVoxelVolume() : MapSize(64, 64, 64), VoxelExtent(100.f)
{
	PrimaryActorTick.bCanEverTick = true;

}

void AVoxelVolume::BeginPlay()
{
	Super::BeginPlay();
	
	// Strategy를 어떻게 Initiaize할지 결정해야함	

	//CreateMap(MapSize);
}

void AVoxelVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

bool AVoxelVolume::IsVoxelWalkable(const FIntVector& Index) const
{
	if (!IsIndexValid(Index)) return false;

	const int32 ChunkX = Index.X / 64;
	const int32 BitPos = Index.X % 64;
	const int32 ChunkArrIndex = GetFlatIndex_X(ChunkX, Index.Y, Index.Z);

	return (ObstacleBitmasks_X[ChunkArrIndex] & (1ULL << BitPos)) == 0;
}

uint64 AVoxelVolume::GetObstacleChunk_X(int32 ChunkX, int32 Y, int32 Z) const
{
	const int32 ChunksAlongX = MapSize.X / static_cast<int32>(CHUNK_SIZE);

	if (ChunkX < 0 || ChunkX >= ChunksAlongX || Y < 0 || Y >= MapSize.Y || Z < 0 || Z >= MapSize.Z)
	{
		return ~0ULL;
	}
	return ObstacleBitmasks_X[GetFlatIndex_X(ChunkX, Y, Z)];
}

uint64 AVoxelVolume::GetObstacleChunk_Y(int32 ChunkY, int32 X, int32 Z) const
{
	const int32 ChunksAlongY = MapSize.Y / static_cast<int32>(CHUNK_SIZE);

	if (ChunkY < 0 || ChunkY >= ChunksAlongY || X < 0 || X >= MapSize.X || Z < 0 || Z >= MapSize.Z)
	{
		return ~0ULL;
	}
	return ObstacleBitmasks_Y[GetFlatIndex_Y(ChunkY, X, Z)];
}

uint64 AVoxelVolume::GetObstacleChunk_Z(int32 ChunkZ, int32 X, int32 Y) const
{
	const int32 ChunksAlongZ = MapSize.Z / static_cast<int32>(CHUNK_SIZE);

	if (ChunkZ < 0 || ChunkZ >= ChunksAlongZ || X < 0 || X >= MapSize.X || Y < 0 || Y >= MapSize.Y)
	{
		return ~0ULL;
	}
	return ObstacleBitmasks_Z[GetFlatIndex_Z(ChunkZ, X, Y)];
}

void AVoxelVolume::CreateMap(const FIntVector& InMapSize)
{
	MapSize = InMapSize;

	// 비트마스크 최적화를 위해 모든 축의 크기가 CHUNK_SIZE의 배수여야 함
	check(MapSize.X % CHUNK_SIZE == 0);
	check(MapSize.Y % CHUNK_SIZE == 0);
	check(MapSize.Z % CHUNK_SIZE == 0);

	// X축 우선 배열 초기화
	const int32 ChunksAlongX = MapSize.X / CHUNK_SIZE;
	const int32 TotalChunksX = ChunksAlongX * MapSize.Y * MapSize.Z;
	ObstacleBitmasks_X.Init(0, TotalChunksX);

	// Y축 우선 배열 초기화
	const int32 ChunksAlongY = MapSize.Y / CHUNK_SIZE;
	const int32 TotalChunksY = ChunksAlongY * MapSize.X * MapSize.Z;
	ObstacleBitmasks_Y.Init(0, TotalChunksY);

	// Z축 우선 배열 초기화
	const int32 ChunksAlongZ = MapSize.Z / CHUNK_SIZE;
	const int32 TotalChunksZ = ChunksAlongZ * MapSize.X * MapSize.Y;
	ObstacleBitmasks_Z.Init(0, TotalChunksZ);
}

void AVoxelVolume::SetStaticObstacle(const FIntVector& Index, bool bIsObstacle)
{
	if (!IsIndexValid(Index)) return;

	// 1. X축 비트마스크 업데이트
	{
		const uint32 ChunkX = Index.X / CHUNK_SIZE;
		const uint32 BitPos = Index.X % CHUNK_SIZE;
		const uint32 ChunkArrIndex = GetFlatIndex_X(ChunkX, Index.Y, Index.Z);
		if (bIsObstacle)
			ObstacleBitmasks_X[ChunkArrIndex] |= (1ULL << BitPos);
		else
			ObstacleBitmasks_X[ChunkArrIndex] &= ~(1ULL << BitPos);
	}

	// 2. Y축 비트마스크 업데이트
	{
		const int32 ChunkY = Index.Y / CHUNK_SIZE;
		const int32 BitPos = Index.Y % CHUNK_SIZE;
		const int32 ChunkArrIndex = GetFlatIndex_Y(ChunkY, Index.X, Index.Z);
		if (bIsObstacle)
			ObstacleBitmasks_Y[ChunkArrIndex] |= (1ULL << BitPos);
		else
			ObstacleBitmasks_Y[ChunkArrIndex] &= ~(1ULL << BitPos);
	}

	// 3. Z축 비트마스크 업데이트
	{
		const int32 ChunkZ = Index.Z / CHUNK_SIZE;
		const int32 BitPos = Index.Z % CHUNK_SIZE;
		const int32 ChunkArrIndex = GetFlatIndex_Z(ChunkZ, Index.X, Index.Y);
		if (bIsObstacle)
			ObstacleBitmasks_Z[ChunkArrIndex] |= (1ULL << BitPos);
		else
			ObstacleBitmasks_Z[ChunkArrIndex] &= ~(1ULL << BitPos);
	}
}

bool AVoxelVolume::IsIndexValid(const FIntVector& Index) const
{
	return Index.X >= 0 && Index.X < MapSize.X &&
		Index.Y >= 0 && Index.Y < MapSize.Y &&
		Index.Z >= 0 && Index.Z < MapSize.Z;
}

FVector AVoxelVolume::VoxelIndexToWorld(const FIntVector& VoxelIndex) const
{
	const FVector LocalCenter = FVector(VoxelIndex) * VoxelExtent + (VoxelExtent * 0.5f);
	return GetActorTransform().TransformPosition(LocalCenter);
}

FIntVector AVoxelVolume::WorldToVoxelIndex(const FVector& WorldPosition) const
{
	if (VoxelExtent <= 0.f)
	{
		UE_LOG(LogTemp, Error, TEXT("AVoxelVolume::WorldToVoxelIndex - VoxelExtent is zero or negative."));
		return FIntVector(EForceInit::ForceInitToZero);
	}

	const FVector LocalPosition = GetActorTransform().InverseTransformPosition(WorldPosition);
	const int32 IndexX = FMath::FloorToInt(LocalPosition.X / VoxelExtent);
	const int32 IndexY = FMath::FloorToInt(LocalPosition.Y / VoxelExtent);
	const int32 IndexZ = FMath::FloorToInt(LocalPosition.Z / VoxelExtent);

	return FIntVector(IndexX, IndexY, IndexZ);
}
