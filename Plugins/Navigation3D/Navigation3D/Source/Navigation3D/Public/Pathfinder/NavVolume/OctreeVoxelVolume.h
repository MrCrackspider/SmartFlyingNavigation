// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/NavVolume/VoxelVolumeBase.h"
#include "Pathfinder/NavData/StaticVoxelChunkData.h"
#include "Pathfinder/NavData/OctreeDataAsset.h"
#include "OctreeVoxelVolume.generated.h"
class UVoxelObstacleFilter;

UCLASS()
class NAVIGATION3D_API AOctreeVoxelVolume : public AVoxelVolumeBase
{
	GENERATED_BODY()

public:
	AOctreeVoxelVolume();

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

public:
	//~======================================================================
	// AUniformGridVolume 인터페이스 구현 (하이브리드 쿼리)
	//~======================================================================
	virtual bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const override;
	virtual void GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const override;
	virtual bool IsVoxelWalkable(const FIntVector& VoxelIndex) const override;
	virtual void RegisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent) override;
	virtual void UnregisterDynamicObstacle(UDynamicObstacleComponent* ObstacleComponent) override;
	/** 런타임에 로드된 정적 옥트리 노드 풀의 복사본을 반환합니다. (스레드 안전) */
	FORCEINLINE TArray<FOctreeNode> GetStaticNodePool() const { return StaticNodePool; }

	/** 옥트리의 루트 노드 인덱스를 반환합니다. */
	FORCEINLINE int32 GetRootNodeIndex() const { return RootNodeIndex; }

	/** 설정된 최대 옥트리 깊이를 반환합니다. */
	FORCEINLINE int32 GetMaxOctreeDepth() const { return MaxOctreeDepth; }

	FORCEINLINE void SetMaxOctreeDepth(int32 Depth) { MaxOctreeDepth = Depth; }
	
	/** 동적 장애물 비트마스크의 복사본을 반환합니다. (스레드 안전) */
	FORCEINLINE TArray<F128BitChunk> GetDynamicObstacleBitmask() const { return DynamicObstacleBitmask; }

	TObjectPtr<UVoxelObstacleFilter> GetStaticObstacleFilter() const { return StaticObjectFilter;}
	void SetStaticObstacleFilter(TObjectPtr<UVoxelObstacleFilter> Filter) {StaticObjectFilter=Filter;}

	TObjectPtr<UOctreeDataAsset> GetBakedOctreeData() {return BakedOctreeData;};
	void SetBakedOctreeData(TObjectPtr<UOctreeDataAsset> NewData) {BakedOctreeData = NewData;}

	UPROPERTY(EditAnywhere, Category = "Voxel|Octree")
	int32 MaxOctreeDepth = 7; // 예: 128x128x128 복셀 -> 2^7=128

	
#if WITH_EDITOR
	// 옥트리 베이킹 함수
	virtual void BakeStaticData() override;
	void ScanForAllStaticObstacles(TSet<FIntVector>& OutStaticObstacles) const;
#endif

#if WITH_EDITORONLY_DATA

	UPROPERTY(EditAnywhere, Category = "Octree|Baking", meta=(ContentDir))
	FDirectoryPath BakedDataSaveDirectory;

	void SafeRemoveInstances(UInstancedStaticMeshComponent* Component, TArray<int32>& InstanceIndicesToRemove);

	// 옥트리 베이킹 함수
	struct FDebugNodeInfo
	{
		int32 NodeIndex;
		FBox Bounds;
		EOctreeNodeState State;
	};
 
	virtual void UpdateDebugObstacleVisualization() override;
	void GetDebugNodeInfoRecursive(TArray<FDebugNodeInfo>& OutNodes, int32 NodeIndex, const FBox& NodeBounds,
	                               int32 CurrentDepth) const;
	// 현재 그려진 노드 인덱스 추적
	UPROPERTY()
	TMap<int32, int32> DisplayedSolidNodeInstanceMap;

	UPROPERTY()
	TMap<int32, int32> DisplayedEmptyNodeInstanceMap;

	UPROPERTY()
	TMap<int32, int32> DisplayedMixedNodeInstanceMap;

	// UPROPERTY(EditAnywhere, Category = "Voxel|Debug", meta = (EditCondition = "bDrawDebugVolumes"))
	// int32 DebugOctreeMaxDepth = 4;


#endif

protected:
	//~======================================================================
	// 동적 장애물 처리 (AFenwickVoxelVolume에서 가져옴)
	//~======================================================================
	void InitializeDynamicLayer();
	void UpdateDynamicObstacles();
	void UpdateVoxelStateForBounds(const FBox& InBounds, bool bAsObstacle);
	void ApplyDynamicObstacleChanges(const TSet<FIntVector>& NewFrameObstacles);
	void UpdateDynamicVoxelState(const FIntVector& Index, bool bIsObstacle);

private:
	//~======================================================================
	// 쿼리 헬퍼 함수
	//~======================================================================

	/** 정적 옥트리 레이어에서 해당 영역이 비어있는지 확인합니다. */
	bool IsRegionEmpty_StaticOctree(const FBox& QueryBox) const;
	bool IsRegionEmptyRecursive(int32 NodeIndex, const FBox& NodeBounds, const FBox& QueryBox) const;

	/** 동적 비트마스크 레이어에서 해당 영역이 비어있는지 확인합니다. */
	bool IsRegionEmpty_DynamicGrid(const FIntVector& MinCorner, const FIntVector& MaxCorner) const;

private:
	//~======================================================================
	// 베이킹 헬퍼 함수
	//~======================================================================
#if WITH_EDITOR
	void InsertObstacleRecursive(int32 NodeIndex, const FIntVector& ObstacleVoxel, int32 CurrentDepth);
	int32 FindLeafNodeAt_BakeTime(const FVector& WorldPos) const;
	void PrecomputeNeighbors();
	void OptimizeAndCollapse(int32 NodeIndex, const FBox& NodeBounds);
	FBox CalculateChildBounds(const FBox& ParentBounds, int32 ChildIndex) const;
	int32 GetOctantContainingVoxel(const FBox& NodeBounds, const FVector& VoxelWorldPos) const;
	bool GetBakedAssetPath(FString& OutPackagePath, FString& OutAssetName) const;
	void SaveOctreeToAsset();
#endif

protected:
	UPROPERTY(EditAnywhere, Category = "Baking", meta=(DisplayName="Static Object Filter"))
	TObjectPtr<UVoxelObstacleFilter> StaticObjectFilter;
	
private:
	//~======================================================================
	// 데이터 멤버
	//~======================================================================

	// --- 1. 정적(Static) 장애물 레이어 ---
	UPROPERTY(EditAnywhere, Category = "Voxel|Baking")
	TObjectPtr<UOctreeDataAsset> BakedOctreeData;
	//
	/** 런타임에 로드된 정적 옥트리 노드 풀 (읽기 전용) */
	TArray<FOctreeNode> StaticNodePool;
	int32 RootNodeIndex = -1;
	
	// --- 2. 동적(Dynamic) 장애물 레이어 ---
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Voxel|Filter", meta=(AllowPrivateAccess="true"))
	TObjectPtr<UVoxelObstacleFilter> ObstacleFilterProfile;

	/** 동적 장애물을 위한 비트마스크 그리드 (실시간 업데이트) */
	TArray<F128BitChunk> DynamicObstacleBitmask;
	TSet<FIntVector> CurrentDynamicObstacles;
	FTimerHandle DynamicObstacleTimer;



	// Begin Test

	/**
	 * Uses bitmasks to perform a very fast check if a region contains any obstacles.
	 * This is a broad-phase check; a `false` return means the region *might* have an obstacle,
	 * while `true` guarantees it is empty.
	 */

public:
	void GetCombinedBitmask(TArray<F128BitChunk>& CombinedBitmask) const;

	/** Bitmasks for fast obstacle checks, organized with X-axis as the inner loop. */
	TArray<F128BitChunk> StaticObstacleBitmask;


	// End Test

};
