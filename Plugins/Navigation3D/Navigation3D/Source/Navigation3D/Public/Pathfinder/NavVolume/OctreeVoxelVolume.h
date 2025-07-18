// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/NavVolume/VoxelVolumeBase.h"
#include "Pathfinder/NavData/StaticVoxelChunkData.h"
#include "Pathfinder/NavData/OctreeDataAsset.h"
#include "OctreeVoxelVolume.generated.h"

class UDynamicObstacleComponent;
class UVoxelObstacleFilter;

struct FDebugNodeInfo
{
	int32 NodeIndex;
	FBox Bounds;
	EOctreeNodeState State; 
};

/**
 * @class AOctreeVoxelVolume
 * @brief A navigation volume that employs a hybrid data structure for pathfinding.
 *
 * This volume combines a pre-baked static octree for unchanging world geometry with a dynamic
 * bitmask grid for moving obstacles. This dual approach allows for highly efficient queries
 * against the static environment while providing fast, real-time updates for dynamic entities
 * like characters or physics objects.
 */
UCLASS()
class NAVIGATION3D_API AOctreeVoxelVolume : public AVoxelVolumeBase
{
	GENERATED_BODY()

public:
	/** Default constructor. */
	AOctreeVoxelVolume();

	//~======================================================================
	// Core Public API
	//~======================================================================

	/**
	 * Checks if a specified region is walkable for an agent of a given size.
	 * This query considers both the static octree (world geometry) and the dynamic obstacle grid.
	 *
	 * @param VoxelIndex The center of the query region in voxel coordinates.
	 * @param AgentRadiusInVoxels The radius of the agent in number of voxels, defining the query's extent.
	 * @return True if the entire region is free of both static and dynamic obstacles, false otherwise.
	 */
	bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const;

	/**
	 * Registers a component to be treated as a dynamic obstacle.
	 * The component's bounds will be voxelized and updated periodically.
	 *
	 * @param ObstacleComponent The component to register.
	 */
	void RegisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent);

	/**
	 * Unregisters a dynamic obstacle component.
	 * Its occupied voxels will be cleared from the dynamic obstacle grid.
	 *
	 * @param ObstacleComponent The component to unregister.
	 */
	void UnregisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent);

	//~======================================================================
	// Accessors
	//~======================================================================

	/** Returns a thread-safe copy of the node pool for the static octree. */
	FORCEINLINE TArray<FOctreeNode> GetStaticNodePool() const { return StaticNodePool; }

	/** Returns the index of the root node within the static node pool. */
	FORCEINLINE int32 GetRootNodeIndex() const { return RootNodeIndex; }

	/** Returns the configured maximum depth of the octree. */
	FORCEINLINE int32 GetMaxOctreeDepth() const { return MaxOctreeDepth; }

	/** Returns a thread-safe copy of the bitmask used for dynamic obstacles. */
	FORCEINLINE TArray<F128BitChunk> GetDynamicObstacleBitmask() const { return DynamicObstacleBitmask; }

	/** Returns the filter used to identify static objects during the baking process. */
	TObjectPtr<UVoxelObstacleFilter> GetStaticObstacleFilter() const { return StaticObjectFilter; }

	/** Returns the data asset where the baked octree data is stored. */
	TObjectPtr<UOctreeDataAsset> GetBakedOctreeData() const { return BakedOctreeData; }

	/**
	 * Sets the data asset to use for the baked octree.
	 * @param NewData The new data asset to assign.
	 */
	void SetBakedOctreeData(TObjectPtr<UOctreeDataAsset> NewData) { BakedOctreeData = NewData; }

	/**
	 * Sets the maximum depth for the octree.
	 * @param Depth The new maximum depth.
	 */
	void SetMaxOctreeDepth(int32 Depth) { MaxOctreeDepth = Depth; }

	/**
	 * Sets the filter used to identify static objects during the baking process.
	 * @param Filter The new filter to use.
	 */
	void SetStaticObstacleFilter(TObjectPtr<UVoxelObstacleFilter> Filter) { StaticObjectFilter = Filter; }

	//~======================================================================
	// Debugging
	//~======================================================================

	/**
	 * Collects the voxel indices of all known obstacles for debugging purposes.
	 * This includes static obstacles from the octree and dynamic obstacles from the grid.
	 *
	 * @param OutObstacleIndices TArray to be populated with the indices of obstacle voxels.
	 */
	void GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const;

protected:
	//~======================================================================
	// AActor Overrides
	//~======================================================================

	/**
	 * Called when the game starts or when spawned.
	 * Initializes the volume, loads baked octree data, and sets up the dynamic obstacle update timer.
	 */
	virtual void BeginPlay() override;

	/** Called every frame. */
	virtual void Tick(float DeltaTime) override;

	//~======================================================================
	// Editor & Baking
	//~======================================================================

#if WITH_EDITOR
public:
	/**
	 * Initiates the process of baking static geometry within this volume into an octree data asset.
	 * This function should only be called from the editor.
	 */
	void BakeStaticData();

	/**
	 * Scans the volume to identify all static actors that should be treated as obstacles,
	 * based on the assigned StaticObjectFilter.
	 *
	 * @param OutStaticObstacles A set to be populated with the voxel coordinates of detected static obstacles.
	 */
	void ScanForAllStaticObstacles(TSet<FIntVector>& OutStaticObstacles) const;
#endif

#if WITH_EDITORONLY_DATA

	UPROPERTY(meta=(ContentDir))
	FDirectoryPath BakedDataSaveDirectory;

protected:
	/**
	 * Overrides the base class visualization to draw the octree structure for debugging.
	 * This is called automatically in the editor when debug drawing is enabled.
	 */
	virtual void UpdateDebugObstacleVisualization() override;
#endif

private:
	//~======================================================================
	// Dynamic Obstacle Internals
	//~======================================================================

	/** Initializes the dynamic obstacle grid based on the volume's dimensions. */
	void InitializeDynamicLayer();

	/** Periodically updates the state of all registered dynamic obstacles. Called by a timer. */
	void UpdateDynamicObstacles();

	/**
	 * Voxelizes the bounding box of a dynamic obstacle, setting or clearing the corresponding bits in the grid.
	 * @param InBounds The world-space bounding box of the obstacle.
	 * @param bAsObstacle True to mark the voxels as obstacles, false to clear them.
	 */
	void UpdateVoxelStateForBounds(const FBox& InBounds, bool bAsObstacle);

	/**
	 * Sets the state of a single voxel in the dynamic obstacle bitmask.
	 * @param Index The 3D index of the voxel to modify.
	 * @param bIsObstacle True to set the voxel as an obstacle, false to clear it.
	 */
	void UpdateDynamicVoxelState(const FIntVector& Index, bool bIsObstacle);

	//~======================================================================
	// Query Helpers
	//~======================================================================

	/**
	 * Internal recursive function to check for collisions within the static octree.
	 * @param QueryBox The box to check for intersections.
	 * @return True if the region is empty, false if it contains an obstacle.
	 */
	bool IsRegionEmpty_StaticOctree(const FBox& QueryBox) const;

	/**
	 * The recursive implementation for checking a region against the static octree.
	 * @param NodeIndex The index of the current node being checked.
	 * @param NodeBounds The bounding box of the current node.
	 * @param QueryBox The original query box.
	 * @return True if the region is empty, false otherwise.
	 */
	bool IsRegionEmptyRecursive(int32 NodeIndex, const FBox& NodeBounds, const FBox& QueryBox) const;

	/**
	 * Internal function to check for collisions within the dynamic obstacle grid.
	 * Uses SIMD instructions for performance where possible.
	 * @param MinCorner The minimum corner of the query box in voxel coordinates.
	 * @param MaxCorner The maximum corner of the query box in voxel coordinates.
	 * @return True if the region is empty, false if it contains a dynamic obstacle.
	 */
	bool IsRegionEmpty_DynamicGrid(const FIntVector& MinCorner, const FIntVector& MaxCorner) const;

	//~======================================================================
	// Baking Internals
	//~======================================================================

#if WITH_EDITOR
private:
	/** Recursively inserts a single obstacle voxel into the octree during the baking process. */
	void InsertObstacleRecursive(int32 NodeIndex, const FIntVector& ObstacleVoxel, int32 CurrentDepth);

	/** Traverses the octree to find the leaf node containing a specific world position. For bake-time use only. */
	int32 FindLeafNodeAt_BakeTime(const FVector& WorldPos) const;

	/** After building the tree, this pre-calculates and stores neighbors for each leaf node to accelerate pathfinding. */
	void PrecomputeNeighbors();

	/** Traverses the octree bottom-up to merge child nodes that have the same state, optimizing memory usage. */
	void OptimizeAndCollapse(int32 NodeIndex, const FBox& NodeBounds);

	/** Calculates the bounding box for one of the 8 children of a parent node. */
	FBox CalculateChildBounds(const FBox& ParentBounds, int32 ChildIndex) const;

	/** Determines which of the 8 octants a given voxel belongs to within a node's bounds. */
	int32 GetOctantContainingVoxel(const FBox& NodeBounds, const FVector& VoxelWorldPos) const;

	/** Constructs the full package path and asset name for the baked data asset. */
	bool GetBakedAssetPath(FString& OutPackagePath, FString& OutAssetName) const;

	/** Handles the creation, saving, and assignment of the UOctreeDataAsset. */
	void SaveOctreeToAsset();

	/** Recursively gathers information about octree nodes for debug visualization. */
	void GetDebugNodeInfoRecursive(TArray<FDebugNodeInfo>& OutNodes, int32 NodeIndex, const FBox& NodeBounds, int32 CurrentDepth) const;
#endif

public:
	//~======================================================================
	// Properties
	//~======================================================================

	/**
	 * The maximum depth of the octree. Determines the smallest possible node size.
	 * For a 128x128x128 volume, a depth of 7 (2^7=128) is appropriate for voxel-level precision.
	 */
	UPROPERTY(EditAnywhere, Category = "Octree")
	int32 MaxOctreeDepth;

	/**
	 * The data asset containing the pre-baked octree for static geometry.
	 * This asset is loaded at runtime to initialize the static collision data.
	 */
	UPROPERTY(EditAnywhere, Category = "Octree|Baking", meta = (DisplayName = "Baked Octree Data"))
	TObjectPtr<UOctreeDataAsset> BakedOctreeData;

	/**
	 * A filter that defines which actors and components should be considered static obstacles
	 * during the baking process.
	 */
	UPROPERTY(EditAnywhere, Category = "Octree|Baking", meta = (DisplayName = "Static Object Filter"))
	TObjectPtr<UVoxelObstacleFilter> StaticObjectFilter;

#if WITH_EDITORONLY_DATA

	/** A list of actors to explicitly ignore during the baking process. */
	UPROPERTY(EditAnywhere, Category = "Octree|Baking")
	TArray<AActor*> BakeExcludeActors;
#endif

private:
	//~======================================================================
	// Runtime Data Members
	//~======================================================================

	/** The runtime pool of nodes for the static octree, loaded from BakedOctreeData. */
	TArray<FOctreeNode> StaticNodePool;

	/** The index of the root node in the StaticNodePool array. Initialized to -1. */
	int32 RootNodeIndex = -1;

	/** A bitmask grid representing the locations of dynamic obstacles in real-time. */
	TArray<F128BitChunk> DynamicObstacleBitmask;

	/** A timer handle for the periodic update of dynamic obstacles. */
	FTimerHandle DynamicObstacleTimer;
};