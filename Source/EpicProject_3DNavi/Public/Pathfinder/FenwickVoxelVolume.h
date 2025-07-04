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
	AFenwickVoxelVolume();

	/** Checks if the volume has been fully initialized and is safe to query for pathfinding. Thread-safe. */
	bool IsReadyForPathfinding() const { return bIsReady.Load(); }

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

public:
	//~ Begin AUniformGridVolume Interface
	virtual void BuildObstacles() override;
	virtual bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const override;
	virtual bool IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex, int32 AgentRadiusInVoxels) const override;
	virtual bool IsVoxelWalkable(const FIntVector& Index) const override;
	virtual void GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const override;
	//~ End AUniformGridVolume Interface

	/** Calculates the total number of obstacles within a rectangular region using the Fenwick Tree. */
	int32 FenwRangeSum(const FIntVector& Min, const FIntVector& Max) const;

private:
	/** Initializes the core data structures like the Fenwick Tree and Bitmasks. */
	void InitializeDataStructures();

	/** Scans the world for dynamic obstacles and updates the Fenwick Tree with any changes. */
	void UpdateDynamicObstacles();

	/** Gathers all voxels currently occupied by dynamic obstacles. */
	void GatherCurrentObstacleVoxels(TMap<FIntVector, int32>& OutVoxelCounts) const;

	/** Compares previous and current obstacle maps to find voxels that are no longer occupied or have a changed count. */
	void ProcessRemovedAndChangedObstacles(const TMap<FIntVector, int32>& CurrentCounts);

	/** Compares previous and current obstacle maps to find newly occupied voxels. */
	void ProcessNewObstacles(const TMap<FIntVector, int32>& CurrentCounts);
	
	/**
	 * Updates the bitmasks to mark a voxel as an obstacle or empty.
	 * @param Index The voxel index to update.
	 * @param bIsObstacle True to mark as an obstacle, false to mark as empty.
	 */
	void UpdateVoxelObstacleState(const FIntVector& Index, bool bIsObstacle);

	/**
	 * Uses bitmasks to perform a very fast check if a region contains any obstacles.
	 * This is a broad-phase check; a `false` return means the region *might* have an obstacle,
	 * while `true` guarantees it is empty.
	 */
	bool IsRegionEmpty_Bitmask(const FIntVector& MinCorner, const FIntVector& MaxCorner) const;

	/** Adds a value (delta) to a voxel's count in the Fenwick Tree. */
	void FenwUpdate(const FIntVector& Index, int32 Delta);

	/** Queries the Fenwick Tree for the cumulative sum from the origin to the given index. */
	int32 FenwQuery(const FIntVector& Index) const;

	
	/** Calculates the flattened 1D index for a 3D coordinate in the Fenwick Tree array. */
	FORCEINLINE int32 GetFenwickFlatIndex(int32 x, int32 y, int32 z) const;

	/** Calculates the flattened index for the X-axis-major bitmask array. */
	FORCEINLINE uint32 GetFlatIndex_X(uint32 ChunkX, uint32 Y, uint32 Z) const;
	/** Calculates the flattened index for the Y-axis-major bitmask array. */
	FORCEINLINE uint32 GetFlatIndex_Y(uint32 ChunkY, uint32 X, uint32 Z) const;
	/** Calculates the flattened index for the Z-axis-major bitmask array. */
	FORCEINLINE uint32 GetFlatIndex_Z(uint32 ChunkZ, uint32 X, uint32 Y) const;

private:
	/** The size of the Fenwick Tree, which is MapSize + 1 on each axis. */
	FIntVector FenwickTreeSize;

	/** The core Fenwick Tree data structure. Stores cumulative obstacle counts. */
	TArray<int32> FenwickTree;

	/** Thread-safe flag indicating that BeginPlay has completed and the volume is ready for queries. */
	TAtomic<bool> bIsReady;

	/** Size of a bitmask chunk along one axis. Must be 64. */
	static constexpr uint32 CHUNK_SIZE = 64;

	/** Bitmasks for fast obstacle checks, organized with X-axis as the inner loop. */
	TArray<uint64> ObstacleBitmasks_X;
	TArray<uint64> ObstacleBitmasks_Y;
	TArray<uint64> ObstacleBitmasks_Z;

	/** Set of voxels occupied by static geometry. */
	TSet<FIntVector> StaticObstacleVoxels;

	/** Map of voxels occupied by dynamic obstacles and the number of obstacles in each. Used to track changes frame-to-frame. */
	TMap<FIntVector, int32> DynamicObstacleVoxelCounts;
};