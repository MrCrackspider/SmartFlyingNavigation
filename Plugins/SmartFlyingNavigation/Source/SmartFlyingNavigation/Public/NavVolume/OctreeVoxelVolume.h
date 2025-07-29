// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavVolume/VoxelVolumeBase.h"
#include "NavData/StaticVoxelChunkData.h"
#include "NavData/OctreeDataAsset.h"
#include "OctreeVoxelVolume.generated.h"

class UDynamicObstacleComponent;
class UVoxelObstacleProfile;
class UOctreeNavSubsystem;
struct FStreamableHandle;

UENUM(BlueprintType)
enum class EVolumeLoadState : uint8
{
	//Unloading,
	Unloaded,
	Loading,
	Loaded,
	Failed
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
class SMARTFLYINGNAVIGATION_API AOctreeVoxelVolume : public AVoxelVolumeBase
{
	GENERATED_BODY()
 
public:
	/** Default constructor. */
	AOctreeVoxelVolume();

	//~======================================================================
	// Core Public API
	//~======================================================================

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
	FORCEINLINE const TArray<FOctreeNode>* GetNodePoolPtr() const { return StaticNodePoolPtr; }
	// ...
	/** Returns the index of the root node within the static node pool. */
	FORCEINLINE int32 GetRootNodeIndex() const { return RootNodeIndex; }


	/** Returns a thread-safe copy of the bitmask used for dynamic obstacles. */
	FORCEINLINE TArray<F128BitChunk> GetDynamicObstacleBitmask() const { return DynamicObstacleBitmask; }

	/** Returns the filter used to identify static objects during the baking process. */
	TObjectPtr<UVoxelObstacleProfile> GetStaticObstacleProfile() const { return StaticObjectProfile; }

	/** Returns the data asset where the baked octree data is stored. */
	TSoftObjectPtr<UOctreeDataAsset> GetBakedOctreeData() const { return BakedOctreeData; }

	/** 
	 * Sets the data asset to use for the baked octree.
	 * @param NewData The new data asset to assign.
	 */

	// RAWPOINTER로 해도 괜찮을지 검토 필요
	// void SetBakedOctreeData(TObjectPtr<UOctreeDataAsset> NewData) { BakedOctreeData = NewData; }
	void SetBakedOctreeData(UOctreeDataAsset* NewData) { BakedOctreeData = NewData; }

	/**
	 * Sets the maximum depth for the octree.
	 * @param Depth The new maximum depth.
	 */
	void SetMaxOctreeDepth(int32 Depth) { MaxOctreeDepth = Depth; }

	/** Returns the configured maximum depth of the octree. */
	FORCEINLINE int32 GetMaxOctreeDepth() const { return static_cast<int>(std::log2(GetMapSize().X));; }

	/**
	 * Sets the filter used to identify static objects during the baking process.
	 * @param Filter The new filter to use.
	 */
	void SetStaticObstacleProfile(TObjectPtr<UVoxelObstacleProfile> Profile) { StaticObjectProfile = Profile; }

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

	UFUNCTION(BlueprintCallable, Category = "Octree")
	bool IsLocationInBlockedVoxel(const FVector& WorldLocation) const;
protected:
	//~======================================================================
	// AActor Overrides
	//~======================================================================

	/**
	 * Called when the game starts or when spawned.
	 * Initializes the volume, loads baked octree data, and sets up the dynamic obstacle update timer.
	 */
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	virtual void PostInitializeComponents() override;

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
	bool BakeStaticData(UWorld* World, TArray<FOctreeNode>& OutNodePool);


	void ConnectPortals_Bake(
		TArray<FOctreeNode>& InOutMyNodePool,
		TMap<int32, FVolumePortalLinks>& OutPortalNodes,
		const TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>>& AllVolumeData,
		const TArray<AOctreeVoxelVolume*>& AllCreatedVolumes
	);
	
	// 컴포넌트를 복셀화하여 장애물 목록에 추가
	void VoxelizeComponent(const FBox& ComponentBounds, const FCollisionObjectQueryParams& ObjectQueryParams, TSet<FIntVector>& OutStaticObstacles) const;
	/**
	 * Scans the volume to identify all static actors that should be treated as obstacles,
	 * based on the assigned StaticObjectFilter.
	 *
	 * @param OutStaticObstacles A set to be populated with the voxel coordinates of detected static obstacles.
	 */
	void ScanForAllStaticObstacles(TSet<FIntVector>& OutStaticObstacles, UObject* WorldContextObject, const FBox& ScanWorldBounds) const;
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
	
	// --- Query Helpers ---
public:
	FIntVector WorldToVoxelIndex(const FVector& WorldLocation) const;

	FVector VoxelIndexToWorld(const FIntVector& VoxelIndex) const;

	bool IsIndexValid(const FIntVector& Index) const;

	bool ClampVoxelIndicesToVolume(const FIntVector& InMin, const FIntVector& InMax, FIntVector& OutClampedMin, FIntVector& OutClampedMax) const;

	int32 FindLeafNodeAt(const FVector& WorldPos) const;

	void DrawDebugDynamicObstaclesWithBoxes();
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
	bool IsRegionEmpty(const FBox& QueryBox) const;

	/**
	 * The recursive implementation for checking a region against the static octree.
	 * @param NodeIndex The index of the current node being checked.
	 * @param NodeBounds The bounding box of the current node.
	 * @param QueryBox The original query box.
	 * @return True if the region is empty, false otherwise.
	 */
	bool IsRegionEmpty_Internal(int32 NodeIndex, const FBox& NodeBounds, const FBox& QueryBox) const;

	//~======================================================================
	// Baking Internals
	//~======================================================================

	/** Calculates the bounding box for one of the 8 children of a parent node. */
	FBox CalculateChildBounds(const FBox& ParentBounds, int32 ChildIndex) const;

#if WITH_EDITOR
private:
	
	struct FDebugNodeInfo
	{
		int32 NodeIndex;
		FBox Bounds;
		EOctreeNodeState State; 
	};
	/** Determines which of the 8 octants a given voxel belongs to within a node's bounds. */
	int32 GetOctantContainingVoxel(const FBox& NodeBounds, const FVector& VoxelWorldPos) const;
		
	/** Recursively gathers information about octree nodes for debug visualization. */
	void GetDebugNodeInfoRecursive(TArray<FDebugNodeInfo>& OutNodes, int32 NodeIndex, const FBox& NodeBounds, int32 CurrentDepth) const;

public:
	/** Constructs the full package path and asset name for the baked data asset. */
	bool GetBakedAssetPath(FString& OutPackagePath, FString& OutAssetName) const;

	/** Handles the creation, saving, and assignment of the UOctreeDataAsset. */
	void SaveOctreeToAsset(const TArray<FOctreeNode>& NodePoolToSave, const TMap<int32, FVolumePortalLinks>& PortalsToSave);

	/** Recursively inserts a single obstacle voxel into the octree during the baking process. */
	void InsertObstacleRecursive_Internal(
		TArray<FOctreeNode>& InOutNodePool,
		int32 NodeIndex,
		const FIntVector& ObstacleVoxel,
		int32 CurrentDepth,
		int32 MaxDepth
	);
	/** Traverses the octree to find the leaf node containing a specific world position. For bake-time use only. */
	int32 FindLeafNode_Internal(
		const TArray<FOctreeNode>& InNodePool,
		int32 CurrentNodeIndex,
		const FVector& WorldPos,
		int32 MaxDepth,
		int32 CurrentDepth = 0
	);
	/** Traverses the octree bottom-up to merge child nodes that have the same state, optimizing memory usage. */
	void OptimizeAndCollapse_Internal(TArray<FOctreeNode>& InOutNodePool, int32 NodeIndex);
	/** After building the tree, this pre-calculates and stores neighbors for each leaf node to accelerate pathfinding. */
	void PrecomputeNeighbors_Internal(TArray<FOctreeNode>& InOutNodePool, const FBox& TotalVolumeBounds, int32 MaxDepth);

#endif

public:
	//~======================================================================
	// Properties
	//~======================================================================


	/**
	 * The data asset containing the pre-baked octree for static geometry.
	 * This asset is loaded at runtime to initialize the static collision data.
	 */
	UPROPERTY()
	TSoftObjectPtr<UOctreeDataAsset> BakedOctreeData;

	/**
	 * A filter that defines which actors and components should be considered static obstacles
	 * during the baking process.
	 */
	UPROPERTY()
	TObjectPtr<UVoxelObstacleProfile> StaticObjectProfile;

#if WITH_EDITORONLY_DATA

	/** A list of actors to explicitly ignore during the baking process. */
	UPROPERTY(EditAnywhere, Category = "Octree|Baking")
	TArray<AActor*> BakeExcludeActors;
#endif

private:
	//~======================================================================
	// Runtime Data Members
	//~======================================================================
	/**
	 * The maximum depth of the octree. Determines the smallest possible node size.
	 * For a 128x128x128 volume, a depth of 7 (2^7=128) is appropriate for voxel-level precision.
	 */
	UPROPERTY()
	int32 MaxOctreeDepth=1;

	/** The runtime pool of nodes for the static octree, loaded from BakedOctreeData. */
	/** 로드된 데이터 에셋의 노드 풀을 가리키는 포인터. LoadedDataAsset이 소유권을 가짐. */
	TArray<FOctreeNode>* StaticNodePoolPtr = nullptr; 

	/** The index of the root node in the StaticNodePool array. Initialized to -1. */
	int32 RootNodeIndex = -1;

	/** A bitmask grid representing the locations of dynamic obstacles in real-time. */
	TArray<F128BitChunk> DynamicObstacleBitmask;

	/** A timer handle for the periodic update of dynamic obstacles. */
	FTimerHandle DynamicObstacleTimer;
	
	// Begin Test

	// 비동기 로딩이 완료되었을 때 호출될 콜백 함수
	void OnDataAssetLoaded();

	// 현재 볼륨 데이터의 로딩 상태
	EVolumeLoadState LoadState = EVolumeLoadState::Unloaded;

	// 비동기 로딩 핸들
	TSharedPtr<FStreamableHandle> AsyncLoadHandle;

	// 로드된 데이터 에셋의 소유권을 관리하기 위한 포인터
	UPROPERTY()
	TObjectPtr<UOctreeDataAsset> LoadedDataAsset;

public:
	// 비동기 데이터 로딩을 요청하는 함수
	void RequestAsyncLoad();

	// 데이터가 로드되어 길찾기에 사용될 준비가 되었는지 확인하는 함수
	FORCEINLINE bool IsReadyForNavigation() const { return LoadState == EVolumeLoadState::Loaded; }
	FORCEINLINE EVolumeLoadState GetLoadState() const { return LoadState; }
	// 볼륨의 월드 바운드를 가져오는 함수
	FBox GetVolumeBounds() const;
	// End Test
};