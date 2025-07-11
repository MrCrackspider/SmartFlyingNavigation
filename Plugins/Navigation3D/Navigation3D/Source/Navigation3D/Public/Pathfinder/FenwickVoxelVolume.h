 // Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UniformGridVolume.h"
#include "StaticVoxelChunkData.h"
#include "Navigation3D/VoxelObstacleFilter.h"
#include "FenwickVoxelVolume.generated.h"

class AChunkDataHolder;

USTRUCT()
struct FActiveFenwickChunk
{
	GENERATED_BODY()

	UPROPERTY()
	FIntVector Coords;

	UPROPERTY()
	TObjectPtr<UStaticVoxelChunkData> SourceAsset = nullptr;
};

class ULevelStreamingDynamic;
UCLASS()
class NAVIGATION3D_API AFenwickVoxelVolume : public AUniformGridVolume
{
	GENERATED_BODY()

public:
	AFenwickVoxelVolume();

	/** Checks if the volume has been fully initialized and is safe to query for pathfinding. Thread-safe. */
	bool IsReadyForPathfinding() const { return bIsReady.Load(); }

	
#if WITH_EDITOR
	virtual void BakeStaticDataToChunks() override;
#endif
	
protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaTime) override;

public:
	//~ Begin AUniformGridVolume Interface
	virtual void BuildObstacles() override;
	virtual bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const override;
	virtual bool IsPathSegmentWalkable(const FIntVector& FromStartIndex, const FIntVector& ToStartIndex,
	                                   int32 AgentRadiusInVoxels) const override;
	virtual bool IsVoxelWalkable(const FIntVector& Index) const override;
	virtual void GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const override;
	//~ End AUniformGridVolume Interface


protected:
	//======================================================================
	// --- CHUNK STREAMING INTERNALS ---
	//======================================================================

	/** 등록된 에이전트들의 위치를 기반으로 필요한 청크를 로드/언로드하는 주기적인 함수입니다. */
	virtual void UpdateStreamingState();

	/** 특정 좌표의 청크(서브레벨)에 대한 로드를 요청합니다. */
	void RequestChunkLoad(const FIntVector& ChunkCoords);

	/** 특정 좌표의 청크(서브레벨)를 언로드합니다. */
	void UnloadChunk(const FIntVector& ChunkCoords);

	/** 서브레벨이 비동기적으로 로드 완료되었을 때 호출되는 콜백 함수입니다. */
	UFUNCTION()
	void OnLevelLoadCompleted();

public:
	/** Calculates the total number of obstacles within a rectangular region using the Fenwick Tree. */
	int32 FenwRangeSum(const FIntVector& Min, const FIntVector& Max) const;

public:
	//~=============================================================================
	// AGENT REGISTRATION API
	//~=============================================================================

	/** 스트리밍의 중심이 될 에이전트를 이 볼륨에 등록합니다. */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Streaming")
	virtual void RegisterStreamingAgent(AActor* Agent) override;

	/** 더 이상 스트리밍의 중심이 아닐 때, 에이전트를 등록 해제합니다. */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Streaming")
	virtual void UnregisterStreamingAgent(AActor* Agent) override;

	/** 로드된 서브레벨의 ChunkDataHolder가 자신을 볼륨에 등록할 때 호출됩니다. */
	void RegisterChunkFromLevel(AChunkDataHolder* ChunkHolder);

	/** 언로드되는 서브레벨의 ChunkDataHolder가 자신을 볼륨에서 해제할 때 호출됩니다. */
	void UnregisterChunkFromLevel(AChunkDataHolder* ChunkHolder);
	
	/**
	 * Uses bitmasks to perform a very fast check if a region contains any obstacles.
	 * This is a broad-phase check; a `false` return means the region *might* have an obstacle,
	 * while `true` guarantees it is empty.
	 */
	bool IsRegionEmpty_Bitmask(const FIntVector& MinCorner, const FIntVector& MaxCorner) const;

private:
	/** Initializes the core data structures like the Fenwick Tree and Bitmasks. */
	void InitializeDataStructures();

	/** Scans the world for dynamic obstacles and updates the Fenwick Tree with any changes. */
	void UpdateDynamicObstacles();

	/** Gathers all voxels currently occupied by dynamic obstacles. */
	void GatherCurrentObstacleVoxels(TMap<FIntVector, int32>& OutVoxelCounts) const;

	/** Compares previous and current obstacle maps to find voxels that are no longer occupied or have a changed count. */
	void ProcessRemovedAndChangedObstacles(const TMap<FIntVector, int32>& CurrentCounts);

	/** Compares previous and current obstacle maps to find newly occupied voxels. */
	void ProcessNewObstacles(const TMap<FIntVector, int32>& CurrentCounts);

	/**
	 * Updates the bitmasks to mark a voxel as an obstacle or empty.
	 * @param Index The voxel index to update.
	 * @param bIsObstacle True to mark as an obstacle, false to mark as empty.
	 */
	void UpdateVoxelObstacleState(const FIntVector& Index, bool bIsObstacle);

	
	/** Adds a value (delta) to a voxel's count in the Fenwick Tree. */
	void FenwUpdate(const FIntVector& Index, int32 Delta);

	void FenwUpdateCuboid(const FIntVector& MinCorner, const FIntVector& MaxCorner, int32 Delta);
	
	/** Queries the Fenwick Tree for the cumulative sum from the origin to the given index. */
	int32 FenwQuery(const FIntVector& Index) const;


	FString GetUniqueAssetPrefix() const;
	
	/** Calculates the flattened 1D index for a 3D coordinate in the Fenwick Tree array. */
	FORCEINLINE int32 GetFenwickFlatIndex(int32 x, int32 y, int32 z) const;

	/** Calculates the flattened index for the X-axis-major bitmask array. */
	FORCEINLINE uint32 GetFlatIndex_X(uint32 ChunkX, uint32 Y, uint32 Z) const;
	/** Calculates the flattened index for the Y-axis-major bitmask array. */
	FORCEINLINE uint32 GetFlatIndex_Y(uint32 ChunkY, uint32 X, uint32 Z) const;
	/** Calculates the flattened index for the Z-axis-major bitmask array. */
	FORCEINLINE uint32 GetFlatIndex_Z(uint32 ChunkZ, uint32 X, uint32 Y) const;

	FORCEINLINE bool IsVoxelIndexValid(const FIntVector& VoxelIndex) const;

#if WITH_EDITOR
	/** 베이크 전 맵 크기, 저장 경로 등의 유효성을 검사합니다. */
	bool ValidateBakePrerequisites() const;

	/** 베이크 과정에서 제외되어야 할 모든 복셀의 목록을 수집합니다. */
	void GatherBakeExclusionVoxels(TSet<FIntVector>& OutExcludedVoxels);

	/** 월드를 스캔하여 모든 정적 장애물 복셀을 찾습니다. (const TSet<FIntVector>& ExcludedVoxels)를 입력으로 받습니다. */
	void ScanForAllStaticObstacles(const TSet<FIntVector>& ExcludedVoxels, TSet<FIntVector>& OutStaticObstacles) const;

	/** 모든 장애물 복셀을 청크 좌표에 따라 그룹화합니다. */
	void GroupObstaclesIntoChunks(const TSet<FIntVector>& AllObstacles, TMap<FIntVector, TArray<FIntVector>>& OutObstaclesByChunk) const;

	/** 청크 데이터 애셋을 전부 삭제합니다. */
	void CleanUpAllChunkAssets() const;

	/** 청크 데이터를 기반으로 데이터 애셋을 생성하거나 업데이트합니다. */
	void CreateOrUpdateChunkAssets(const TMap<FIntVector, TArray<FIntVector>>& ObstaclesByChunk, const FString& InFinalSaveDirectory);
#endif

private:
	//======================================================================
	// --- CHUNK HELPER FUNCTIONS ---
	//======================================================================

	/** 월드 좌표를 청크 좌표로 변환합니다. */
	FIntVector WorldToChunkCoords(const FVector& WorldPos) const;

	/** 이 볼륨의 베이크된 데이터가 저장될 최종 디렉토리 경로를 반환합니다. (월드/액터 이름 포함) */
	FString GetFinalSaveDirectory() const;

	/** 특정 청크 좌표에 대한 서브레벨(.umap) 애셋의 패키지 경로를 반환합니다. */
	FString GetChunkAssetPath(const FIntVector& ChunkCoords) const;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Voxel|Filter", meta=(AllowPrivateAccess="true"))
	TObjectPtr<UVoxelObstacleFilter> ObstacleFilterProfile;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Baking")
	TMap<FIntVector, TObjectPtr<UStaticVoxelChunkData>> BakedStaticDataAssets;

	TMap<FIntVector, int32> ChunkObstacleCounter;
private:
	/** The size of the Fenwick Tree, which is MapSize + 1 on each axis. */
	FIntVector FenwickTreeSize;

	/** The core Fenwick Tree data structure. Stores cumulative obstacle counts. */
	TArray<int32> FenwickTree;

	/** Thread-safe flag indicating that BeginPlay has completed and the volume is ready for queries. */
	TAtomic<bool> bIsReady;

	/** Size of a bitmask chunk along one axis. Must be 64. */
	static constexpr uint32 CHUNK_SIZE = 64;
	
	/** Set of voxels occupied by static geometry. */
	TSet<FIntVector> StaticObstacleVoxels;

	/** Map of voxels occupied by dynamic obstacles and the number of obstacles in each. Used to track changes frame-to-frame. */
	TMap<FIntVector, int32> DynamicObstacleVoxelCounts;

	TMap<FIntVector, TObjectPtr<UStaticVoxelChunkData>> ActiveChunks;
public:
	/** Bitmasks for fast obstacle checks, organized with X-axis as the inner loop. */
	TArray<uint64> ObstacleBitmasks_X;
	// TArray<uint64> ObstacleBitmasks_Y;
	// TArray<uint64> ObstacleBitmasks_Z;
protected:
	//~=============================================================================
	// STREAMING & BAKING PROPERTIES (Configurable in Editor)
	//~=============================================================================

	/** 플레이어 주변으로 스트리밍할 청크의 반경입니다. (X, Y, Z) */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Voxel|Streaming")
	FIntVector StreamingRangeInChunks = FIntVector(2, 2, 2);

	/** 베이킹된 데이터 애셋을 저장하고 찾아올 경로입니다. */
	UPROPERTY(EditAnywhere, Category = "Voxel|Baking", meta = (ContentDir))
	FString BakedDataSaveDirectory = TEXT("/Game/Pathfinding/BakedData");
	
	/** 스트리밍의 중심이 되는 에이전트(플레이어 등) 목록입니다. */
	TSet<TWeakObjectPtr<AActor>> StreamingAgents;
private:
	/** 스트리밍 상태를 업데이트하는 타이머 핸들입니다. */
	FTimerHandle StreamingUpdateTimer;
	FTimerHandle DynamicObstacleTimer;
	
	/** 현재 로드되어 있어야 하는 청크 좌표의 집합입니다. */
	TSet<FIntVector> RequiredChunks;

	/** 현재 로드 중인(스트리밍 중인) 서브레벨 목록입니다. */
	UPROPERTY()
	TMap<FIntVector, TObjectPtr<ULevelStreamingDynamic>> ActiveStreamingLevels;

	/** 로드를 요청했지만 아직 완료되지 않은 청크 좌표의 집합입니다. */
	TSet<FIntVector> PendingStreamingLevels;
};