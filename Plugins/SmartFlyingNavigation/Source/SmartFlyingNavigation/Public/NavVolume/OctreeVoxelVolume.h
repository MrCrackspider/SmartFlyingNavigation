// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavVolume/VoxelVolumeBase.h"
#include "NavCore/NavNodeTypes.h"
#include "NavCore/NavDistanceField.h"
#include "NavCore/OctreePathfindingData.h"
#include "OctreeVoxelVolume.generated.h"

class AOctreeVoxelVolume;
class UDynamicObstacleComponent;
class UVoxelObstacleProfile;
class UOctreeNavSubsystem;
struct FOctreePathfindingData;

UENUM(BlueprintType)
enum class EVolumeLoadState : uint8
{
	Unloaded,
	Loading,
	Loaded,
	Failed
};

/**
 * @class AOctreeVoxelVolume
 * @brief A navigation volume chunk backed by a pre-baked static octree.
 *
 * Static geometry queries run against the baked octree loaded from the world
 * navigation manifest. Dynamic obstacles are tracked in a sparse bitmask grid
 * that is updated periodically at runtime.
 */
UCLASS(Config = Game)
class SMARTFLYINGNAVIGATION_API AOctreeVoxelVolume : public AVoxelVolumeBase
{
	GENERATED_BODY()

public:
	AOctreeVoxelVolume();

	//~======================================================================
	// Accessors
	//~======================================================================

	FORCEINLINE const TArray<FOctreeNode>* GetNodePoolPtr() const
	{
		return (ChunkData && ChunkData->NodePool.Num() > 0) ? &ChunkData->NodePool : nullptr;
	}

	FORCEINLINE const FAirwayGraph* GetAirwayGraphPtr() const
	{
		return (ChunkData && !ChunkData->AirwayGraph.IsEmpty()) ? &ChunkData->AirwayGraph : nullptr;
	}

	/** Baked navigation distance field, or nullptr when the chunk was baked without one. */
	FORCEINLINE const FNavDistanceField* GetDistanceField() const
	{
		return (ChunkData && ChunkData->DistanceField.IsValid()) ? &ChunkData->DistanceField : nullptr;
	}

	/** Returns the index of the root node within the node pool. */
	FORCEINLINE int32 GetRootNodeIndex() const { return RootNodeIndex; }

	/** Returns the filter used to identify static objects during the baking process. */
	TObjectPtr<UVoxelObstacleProfile> GetStaticObstacleProfile() const { return StaticObjectProfile; }

	/**
	 * Sets the maximum depth for the octree.
	 * @param Depth The new maximum depth.
	 */
	void SetMaxOctreeDepth(int32 Depth) { MaxOctreeDepth = Depth; }

	/** Returns the configured maximum depth of the octree. */
	FORCEINLINE int32 GetMaxOctreeDepth() const
	{
		if (MapSize.X <= 0)
		{
			UE_LOG(LogTemp, Error, TEXT("AOctreeVoxelVolume::GetMaxOctreeDepth - MapSize.X is invalid (%d). Returning default depth 1."), MapSize.X);
			return 1;
		}

		const int32 CalculatedDepth = FMath::FloorToInt(FMath::Log2(static_cast<float>(MapSize.X)));

		return FMath::Max(0, CalculatedDepth);
	}

	/**
	 * Sets the filter used to identify static objects during the baking process.
	 */
	void SetStaticObstacleProfile(TObjectPtr<UVoxelObstacleProfile> Profile) { StaticObjectProfile = Profile; }

	//~======================================================================
	// Debugging
	//~======================================================================

	UFUNCTION(BlueprintCallable, Category = "Octree")
	bool IsLocationInBlockedVoxel(const FVector& WorldLocation) const;

	/**
	 * Builds a thread-safe snapshot for async pathfinding.
	 * @param bSnapshotDynamicObstacles When false, the (allocating) dynamic obstacle
	 *        snapshot is skipped — use for game-thread-only static queries.
	 */
	void CreatePathfindingDataView(FOctreePathfindingData& OutData, bool bSnapshotDynamicObstacles = true) const;

	/**
	 * Game-thread point/segment safety probes. Cheap: no dynamic obstacle snapshot
	 * is allocated; dynamic obstacles are tested directly against the world registry.
	 */
	bool IsPointBlockedForAgent(const FVector& Location, float AgentRadius, bool bCheckStatic = true,
	                            bool bCheckDynamic = true) const;
	bool IsSegmentBlockedForAgent(const FVector& Start, const FVector& End, float AgentRadius,
	                              bool bCheckStatic = true, bool bCheckDynamic = true) const;

protected:
	//~======================================================================
	// AActor Overrides
	//~======================================================================

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void PostInitializeComponents() override;

public:
	FVector VoxelIndexToWorld(const FIntVector& VoxelIndex) const;

#if WITH_EDITORONLY_DATA

protected:
	/**
	 * Overrides the base class visualization to draw the octree structure for debugging.
	 */
	virtual void UpdateDebugObstacleVisualization() override;
#endif

	// --- Query Helpers ---
public:
	FIntVector WorldToVoxelIndex(const FVector& WorldLocation) const;

	bool IsIndexValid(const FIntVector& Index) const;

	bool ClampVoxelIndicesToVolume(const FIntVector& InMin, const FIntVector& InMax, FIntVector& OutClampedMin,
	                               FIntVector& OutClampedMax) const;

	int32 FindLeafNodeAt(const FVector& WorldPos) const;

private:
	//~======================================================================
	// Query Helpers
	//~======================================================================

	/**
	 * Internal recursive function to check for collisions within the static octree.
	 */
	bool IsRegionEmpty(const FBox& QueryBox) const;

	bool IsRegionEmpty_Internal(int32 NodeIndex, const FBox& NodeBounds, const FBox& QueryBox) const;

	//~======================================================================
	// Octree Geometry Helpers
	//~======================================================================

	/** Calculates the bounding box for one of the 8 children of a parent node. */
	FBox CalculateChildBounds(const FBox& ParentBounds, int32 ChildIndex) const;

	/** Determines which of the 8 octants a given voxel belongs to within a node's bounds. */
	int32 GetOctantContainingVoxel(const FBox& NodeBounds, const FVector& VoxelWorldPos) const;

#if WITH_EDITOR

private:
	struct FDebugNodeInfo
	{
		int32 NodeIndex;
		FBox Bounds;
		EOctreeNodeState State;
	};
#endif

public:
	//~======================================================================
	// Bake Filter Settings
	//~======================================================================

	/** Collision channels to include when baking. Actors on these channels will be treated as obstacles. */
	UPROPERTY(EditAnywhere, Category = "Octree|Baking", meta = (DisplayName = "Obstacle Channels"))
	TArray<TEnumAsByte<ECollisionChannel>> BakeCollisionChannels = { ECC_WorldStatic };

	/** When true, actors/components with bCanEverAffectNavigation=false are skipped (follows UE convention). */
	UPROPERTY(EditAnywhere, Category = "Octree|Baking", meta = (DisplayName = "Respect CanAffectNavigation"))
	bool bRespectCanAffectNavigation = true;

	/**
	 * (Optional, Advanced) A profile asset for complex filtering rules.
	 * When assigned, this overrides the direct collision channel settings above.
	 */
	UPROPERTY(EditAnywhere, Category = "Octree|Baking", AdvancedDisplay, meta = (DisplayName = "Advanced: Obstacle Profile Override"))
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
	 */
	UPROPERTY()
	int32 MaxOctreeDepth = 1;

	/**
	 * The baked chunk payload. Swapped under ChunkDataCS and never mutated after load.
	 * Shared (thread-safe) so pathfinding snapshots keep the data alive while
	 * worker tasks are in flight, even if the volume streams out mid-task.
	 */
	TSharedPtr<FLoadedNavChunkData, ESPMode::ThreadSafe> ChunkData;

	int32 LoadedChunkId = INDEX_NONE;

	/** Allocated bytes of the loaded payload (node pool + airway + distance field), for stats. */
	int64 LoadedChunkBytes = 0;

	mutable FCriticalSection ChunkDataCS;

	/** The index of the root node in the node pool. Initialized to -1. */
	int32 RootNodeIndex = -1;

	EVolumeLoadState LoadState = EVolumeLoadState::Unloaded;

public:
	bool InitializeFromManifestPayload(int32 ChunkId, TArray<FOctreeNode>&& InNodePool,
	                                   FAirwayGraph&& InAirwayGraph,
	                                   FNavDistanceField&& InDistanceField);

	FORCEINLINE bool IsReadyForNavigation() const { return LoadState == EVolumeLoadState::Loaded; }
	FORCEINLINE EVolumeLoadState GetLoadState() const { return LoadState; }
	FORCEINLINE int32 GetLoadedChunkId() const { return LoadedChunkId; }
	FORCEINLINE int64 GetLoadedChunkBytes() const { return LoadedChunkBytes; }

	/** Per-part memory of the loaded payload, for diagnostics (sfn.DumpNavStats). */
	void GetLoadedChunkMemoryBreakdown(int64& OutNodePoolBytes, int64& OutAirwayBytes, int64& OutDistanceFieldBytes) const
	{
		OutNodePoolBytes = ChunkData ? ChunkData->NodePool.GetAllocatedSize() : 0;
		OutAirwayBytes = ChunkData ? ChunkData->AirwayGraph.GetAllocatedBytes() : 0;
		OutDistanceFieldBytes = ChunkData ? ChunkData->DistanceField.GetAllocatedBytes() : 0;
	}

	FBox GetVolumeBounds() const;
};
