#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "OctreeDataAsset.generated.h"

// 옥트리 노드의 상태
UENUM(BlueprintType)
enum class EOctreeNodeState : uint8
{
	Empty,    // 완전히 비어있음
	Solid,    // 완전히 막혀있음
	Mixed     // 자식 노드를 가짐
};

// 옥트리 노드 구조체
// UPROPERTY()를 붙여야 파일로 저장(직렬화)되고 다시 불러올 수 있습니다.
USTRUCT(BlueprintType)
struct FOctreeNode
{
	GENERATED_BODY()

	// 8개의 자식 노드 인덱스. -1이면 자식이 없음을 의미합니다.
	UPROPERTY()
	int32 ChildrenIndices[8];
	
	// 노드의 현재 상태
	UPROPERTY()
	EOctreeNodeState State;

	// 생성자: 기본값으로 초기화
	FOctreeNode() : State(EOctreeNodeState::Empty)
	{
		for (int i = 0; i < 8; ++i)
		{
			ChildrenIndices[i] = -1;
		}
	}
};

/**
 * 베이킹된 정적 옥트리 데이터를 저장하는 UPrimaryDataAsset입니다.
 * 이 에셋은 에디터의 베이킹 과정을 통해 생성되며, 런타임에 AOctreeVoxelVolume에 의해 로드됩니다.
 */
UCLASS(BlueprintType)
class NAVIGATION3D_API UOctreeDataAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	/** 이 에셋 타입을 식별하기 위한 고유 이름입니다. */
	static const FName OctreeAssetType;

	//~======================================================================
	// 저장될 데이터
	//~======================================================================

	/** 옥트리를 구성하는 모든 노드가 저장되는 메모리 풀입니다. */
	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	TArray<FOctreeNode> NodePool;

	/** 이 데이터가 베이킹될 당시의 볼륨 맵 크기입니다. */
	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	FIntVector BakedMapSize;

	/** 이 데이터가 베이킹될 당시의 옥트리 최대 깊이입니다. */
	UPROPERTY(VisibleAnywhere, Category = "OctreeData")
	int32 BakedMaxOctreeDepth;

	/**
	 * 언리얼의 에셋 매니저가 이 에셋을 식별할 수 있도록 Primary Asset ID를 반환합니다.
	 * 스트리밍이나 비동기 로드에 활용될 수 있습니다.
	 */
	virtual FPrimaryAssetId GetPrimaryAssetId() const override;
};