// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "OctreeDataAsset.generated.h"

USTRUCT()
struct FVolumePortalLink
{
	GENERATED_BODY()

	UPROPERTY()
	TSoftObjectPtr<UOctreeDataAsset> TargetVolume;

	UPROPERTY()
	int32 TargetNodeIndex; 

	UPROPERTY()
	float MaxAllowedRadius = 0.f;
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
	Right,  // +X
	Left,   // -X
	Front,  // +Y
	Back,   // -Y
	Up,     // +Z
	Down,   // -Z
	MAX
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
		for (int i = 0; i < 8; ++i)
		{
			ChildrenIndices[i] = -1;
		}

		for (int i = 0; i < static_cast<int32>(ENeighborDirection::MAX); ++i)
		{
			Neighbors[i] = -1;
		}
	}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FOctreeNode& Node)
{
	for (int32& ChildIndex : Node.ChildrenIndices)
	{
		Ar << ChildIndex;
	}

	Ar << Node.State;
	Ar << Node.Bounds;
	
	// Serialize the neighbors array element by element.
	for (int32& NeighborIndex : Node.Neighbors)
	{
		Ar << NeighborIndex;
	}

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

	UPROPERTY()
	TArray<uint8> CompressedData;

	UPROPERTY()
	int32 UncompressedSize;

	UPROPERTY()
	TMap<int32, FVolumePortalLink> PortalNodes;
public:
	void DecompressAndInitialize();
#if WITH_EDITOR
	void SetAndCompressBakedData(const TArray<FOctreeNode>& InNodePool, const FIntVector& InMapSize, int32 InMaxDepth);
#endif
	virtual FPrimaryAssetId GetPrimaryAssetId() const override;
};