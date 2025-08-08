// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "OctreeDataAsset.generated.h"

USTRUCT(BlueprintType)
struct FVolumePortalLink
{
	GENERATED_BODY()

	UPROPERTY()
	TSoftObjectPtr<UOctreeDataAsset> TargetVolume;

	UPROPERTY()
	int32 TargetNodeIndex = -1;

	UPROPERTY()
	float MaxAllowedRadius = 0.f;
};

USTRUCT(BlueprintType)
struct FVolumePortalLinks
{
	GENERATED_BODY()

	UPROPERTY()
	TArray<FVolumePortalLink> Links;
};

UENUM(BlueprintType)
enum class EOctreeNodeState : uint8
{
	Empty,
	Solid,
	Mixed
};

UENUM()
enum class ENeighborDirection : uint8
{
	Right, Left, Front, Back, Up, Down, MAX
};

USTRUCT(BlueprintType)
struct FOctreeNode
{
	GENERATED_BODY()

	UPROPERTY()
	int32 ChildrenIndices[8];

	UPROPERTY()
	EOctreeNodeState State;

	UPROPERTY()
	FBox Bounds;

	UPROPERTY()
	int32 Neighbors[static_cast<int32>(ENeighborDirection::MAX)];

	UPROPERTY()
	float MaxAllowedRadius = 0.f;

	FOctreeNode() : State(EOctreeNodeState::Empty)
	{
		FMemory::Memset(ChildrenIndices, -1, sizeof(ChildrenIndices));
		FMemory::Memset(Neighbors, -1, sizeof(Neighbors));
	}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FOctreeNode& Node)
{
	for (int32& ChildIndex : Node.ChildrenIndices) { Ar << ChildIndex; }
	Ar << Node.State;
	Ar << Node.Bounds;
	for (int32& NeighborIndex : Node.Neighbors) { Ar << NeighborIndex; }
	Ar << Node.MaxAllowedRadius;
	return Ar;
}


UCLASS(BlueprintType)
class SMARTFLYINGNAVIGATION_API UOctreeDataAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	FIntVector BakedVolumeSize;

	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	int32 BakedMaxOctreeDepth;

	static const FName OctreeAssetType;

	UPROPERTY(Transient, VisibleAnywhere, Category = "OctreeData")
	TArray<FOctreeNode> NodePool;

	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	TMap<int32, FVolumePortalLinks> PortalNodes;

	UPROPERTY()
	TArray<uint8> CompressedNodePool;

	UPROPERTY()
	int32 UncompressedNodePoolSize;

public:
	void DecompressAndInitialize();
#if WITH_EDITOR
	void SetAndCompressBakedData(const TArray<FOctreeNode>& InNodePool, const FIntVector& InMapSize, int32 InMaxDepth);
#endif
	virtual FPrimaryAssetId GetPrimaryAssetId() const override;
};