// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "OctreeDataAsset.generated.h"

// [수정] USTRUCT()를 USTRUCT(BlueprintType)으로 변경합니다.
// 이 구조체를 사용하는 다른 구조체(FVolumePortalLinks)가 BlueprintType이므로,
// UHT가 리플렉션 데이터를 올바르게 생성하기 위해 이 변경이 반드시 필요합니다.
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

// [추가된 구조체] 하나의 노드가 가질 수 있는 여러 개의 포털 링크를 담는 컨테이너입니다.
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

// 커스텀 직렬화를 위한 operator<< 입니다. FOctreeNode만 처리합니다.
FORCEINLINE FArchive& operator<<(FArchive& Ar, FOctreeNode& Node)
{
	for (int32& ChildIndex : Node.ChildrenIndices) { Ar << ChildIndex; }
	Ar << Node.State;
	Ar << Node.Bounds;
	for (int32& NeighborIndex : Node.Neighbors) { Ar << NeighborIndex; }
	Ar << Node.MaxAllowedRadius;
	return Ar;
}

// [삭제] FVolumePortalLink와 FVolumePortalLinks에 대한 operator<<는 사용하지 않으므로 삭제합니다.

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

	// NodePool은 런타임에 압축 해제되므로 Transient로 선언합니다.
	UPROPERTY(Transient, VisibleAnywhere, Category = "OctreeData")
	TArray<FOctreeNode> NodePool;

	// PortalNodes는 표준 UPROPERTY 직렬화 시스템으로 저장되므로 Transient가 아닙니다.
	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	TMap<int32, FVolumePortalLinks> PortalNodes;

	// NodePool 데이터만 담는 압축 데이터입니다.
	UPROPERTY()
	TArray<uint8> CompressedNodePool;

	UPROPERTY()
	int32 UncompressedNodePoolSize;

public:
	void DecompressAndInitialize();
#if WITH_EDITOR
	// 이 함수는 이제 NodePool 데이터만 처리합니다.
	void SetAndCompressBakedData(const TArray<FOctreeNode>& InNodePool, const FIntVector& InMapSize, int32 InMaxDepth);
#endif
	virtual FPrimaryAssetId GetPrimaryAssetId() const override;
};