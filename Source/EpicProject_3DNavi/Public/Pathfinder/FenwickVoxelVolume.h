// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UniformGridVolume.h"
#include "Pathfinder/VoxelVolume.h"
#include "FenwickVoxelVolume.generated.h"

UCLASS()
class EPICPROJECT_3DNAVI_API AFenwickVoxelVolume : public AUniformGridVolume
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AFenwickVoxelVolume();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	void SetStaticObstacle(const FIntVector& Index, bool bIsObstacle);

	virtual void BuildObstacles() override;
	virtual void UpdateDynamicObstacles();
	virtual bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const override;
	virtual void GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const override;
	bool IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex, int32 AgentRadiusInVoxels) const;

	void FenwUpdate(const FIntVector& Index, int32 Delta);
	int32 FenwQuery(const FIntVector& Index) const;
	int32 FenwRangeSum(const FIntVector& Min, const FIntVector& Max) const;
	FORCEINLINE int32 GetFenwickFlatIndex(int32 x, int32 y, int32 z) const;
	void InitializeMap(const FIntVector& InMapSize);

	virtual bool IsVoxelWalkable(const FIntVector& Index) const override;

	uint32 GetFlatIndex_X(uint32 ChunkX, uint32 Y, uint32 Z) const
	{
		const uint32 ChunksAlongX = MapSize.X / CHUNK_SIZE;
		return (Z * MapSize.Y + Y) * ChunksAlongX + ChunkX;
	}

	// Y축 우선으로 정렬된 청크 배열의 인덱스를 계산
	// 메모리 레이아웃: [ (Z=0, X=0, Y=0..n), (Z=0, X=1, Y=0..n), ... ]
	uint32 GetFlatIndex_Y(uint32 ChunkY, uint32 X, uint32 Z) const
	{
		const uint32 ChunksAlongY = MapSize.Y / CHUNK_SIZE;
		return (Z * MapSize.X + X) * ChunksAlongY + ChunkY;
	}

	// Z축 우선으로 정렬된 청크 배열의 인덱스를 계산
	// 메모리 레이아웃: [ (Y=0, X=0, Z=0..n), (Y=0, X=1, Z=0..n), ... ]
	uint32 GetFlatIndex_Z(uint32 ChunkZ, uint32 X, uint32 Y) const
	{
		const uint32 ChunksAlongZ = MapSize.Z / CHUNK_SIZE;
		return (Y * MapSize.X + X) * ChunksAlongZ + ChunkZ;
	}

private:
	FIntVector FenwickTreeSize;
	TArray<int32> FenwickTree;
	bool bIsObstacleUpdateInProgress;

	// --- 비트마스크 데이터 (SetStaticObstacle에 필요) ---
	TArray<uint64> ObstacleBitmasks_X;
	TArray<uint64> ObstacleBitmasks_Y;
	TArray<uint64> ObstacleBitmasks_Z;

	// --- 기타 장애물 관리 데이터 ---
	TSet<FIntVector> StaticObstacleVoxels;
	TMap<FIntVector, int32> DynamicObstacleVoxelCounts;
};
