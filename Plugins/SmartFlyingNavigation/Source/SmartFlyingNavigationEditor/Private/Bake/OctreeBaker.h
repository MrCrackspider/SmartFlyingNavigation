// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavCore/NavNodeTypes.h"
#include "NavCore/NavDistanceField.h"

class AOctreeVoxelVolume;
class UVoxelObstacleProfile;
class UWorld;

/**
 * Bake-time portal connection to a neighboring volume. Converted into
 * FNavigationChunkPortal entries when the world manifest is written.
 */
struct FNavPortalBakeLink
{
	TWeakObjectPtr<AOctreeVoxelVolume> TargetVolume;
	int32 TargetNodeIndex = INDEX_NONE;
	float MaxAllowedRadius = 0.f;
};

/**
 * Plain description of a voxel volume so the baker never needs the actor for math.
 */
struct FVoxelVolumeDesc
{
	FBox Bounds = FBox(ForceInit);
	float VoxelExtent = 100.f;
	FIntVector MapSize = FIntVector::ZeroValue;
	int32 MaxDepth = 1;

	// --- Obstacle filter (authoring data, read from the volume actor) ---
	TArray<TEnumAsByte<ECollisionChannel>> CollisionChannels;
	bool bRespectCanAffectNavigation = true;
	const UVoxelObstacleProfile* ObstacleProfile = nullptr;
	const AActor* VolumeActor = nullptr; // ignored during scans; used for logs
	TArray<const AActor*> ExcludeActors;

	FString GetLabel() const;

	FVector VoxelIndexToWorld(const FIntVector& VoxelIndex) const
	{
		return Bounds.Min + (FVector(VoxelIndex) * VoxelExtent) + FVector(VoxelExtent * 0.5f);
	}

	FIntVector WorldToVoxelIndex(const FVector& WorldLocation) const
	{
		const FVector Local = WorldLocation - Bounds.Min;
		return FIntVector(
			FMath::FloorToInt(Local.X / VoxelExtent),
			FMath::FloorToInt(Local.Y / VoxelExtent),
			FMath::FloorToInt(Local.Z / VoxelExtent));
	}

	FORCEINLINE int64 FlatIndex(int32 X, int32 Y, int32 Z) const
	{
		return (static_cast<int64>(Z) * MapSize.Y + Y) * MapSize.X + X;
	}

	FORCEINLINE int64 NumVoxels() const
	{
		return static_cast<int64>(MapSize.X) * MapSize.Y * MapSize.Z;
	}
};

/**
 * Editor-only octree bake pipeline. Three explicit stages:
 *   A. Occupancy   — per-component narrow-phase voxelization into a dense bitset.
 *   B. Distance    — separable exact squared Euclidean distance transform (Meijster),
 *                    used for accurate per-leaf clearance (and the nav SDF).
 *   C. Octree      — insert/collapse/neighbor pipeline producing the node pool.
 */
class FOctreeBaker
{
public:
	/** Result of the occupancy + distance stages, consumed by the octree stage (and later the SDF). */
	struct FVoxelField
	{
		TBitArray<> Occupancy;          // NumVoxels bits, true = solid
		TArray<int32> SquaredDistance;  // NumVoxels entries, squared distance in voxels to nearest solid (clamped)
		int32 BandVoxels = 0;           // distance clamp band, in voxels
	};

	/** Runs stages A+B+C. Returns false on invalid input or user cancel. */
	static bool BakeVolume(UWorld* World, const FVoxelVolumeDesc& Desc, TArray<FOctreeNode>& OutNodePool, FVoxelField* OutField = nullptr);

	/** Quantizes the EDT result into the brick-sparse runtime distance field. */
	static void BuildDistanceField(const FVoxelVolumeDesc& Desc, const FVoxelField& Field, FNavDistanceField& OutDistanceField);

	/** Builds a desc from a volume actor (also normalizes the actor's MapSize against its bounds). */
	static bool BuildDescFromVolume(AOctreeVoxelVolume* Volume, FVoxelVolumeDesc& OutDesc);

	/**
	 * Connects boundary nodes of a freshly baked volume to leaf nodes of neighboring volumes.
	 * Mirrors the per-volume node pools in AllVolumeData.
	 */
	static void ConnectPortals(
		UWorld* World,
		AOctreeVoxelVolume* SourceVolume,
		TArray<FOctreeNode>& InOutMyNodePool,
		TMap<int32, TArray<FNavPortalBakeLink>>& OutPortalNodes,
		const TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>>& AllVolumeData,
		const TArray<AOctreeVoxelVolume*>& AllCreatedVolumes);

private:
	// --- Stage A ---
	static bool BuildOccupancy(UWorld* World, const FVoxelVolumeDesc& Desc, TBitArray<>& OutOccupancy);

	// --- Stage B ---
	static void ComputeSquaredDistanceField(const FVoxelVolumeDesc& Desc, const TBitArray<>& Occupancy, int32 BandVoxels, TArray<int32>& OutSquaredDistance);

	// --- Stage C ---
	static bool BuildOctree(const FVoxelVolumeDesc& Desc, const FVoxelField& Field, TArray<FOctreeNode>& OutNodePool);

	static void InsertObstacleRecursive(const FVoxelVolumeDesc& Desc, TArray<FOctreeNode>& NodePool, int32 NodeIndex, const FIntVector& ObstacleVoxel, int32 CurrentDepth);
	static void OptimizeAndCollapse(TArray<FOctreeNode>& NodePool, int32 NodeIndex);
	static void PrecomputeNeighborsAndClearance(const FVoxelVolumeDesc& Desc, const FVoxelField& Field, TArray<FOctreeNode>& NodePool);

	// --- Shared math ---
	static FBox CalculateChildBounds(const FBox& ParentBounds, int32 ChildIndex);
	static int32 GetOctantContainingPoint(const FBox& NodeBounds, const FVector& Point);
	static int32 FindLeafNode(const TArray<FOctreeNode>& NodePool, int32 RootIndex, const FVector& WorldPos, int32 MaxDepth);
};
