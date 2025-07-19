// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "OctreeDataAsset.generated.h"


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

UCLASS(BlueprintType)
class SMARTFLYINGNAVIGATION_API UOctreeDataAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	static const FName OctreeAssetType;

	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	TArray<FOctreeNode> NodePool;

	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	FIntVector BakedMapSize;

	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	int32 BakedMaxOctreeDepth;

	virtual FPrimaryAssetId GetPrimaryAssetId() const override;
};