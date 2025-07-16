 // Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UniformGridVolume.h"
#include "StaticVoxelChunkData.h"
#include "Containers/BitArray.h"
#include "Navigation3D/VoxelObstacleFilter.h"
#include "FenwickVoxelVolume.generated.h"

class AChunkDataHolder;
struct FStreamableHandle;
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
	virtual void BakeStaticData() override;
#endif
	
protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaTime) override;

public:
	//~ Begin AUniformGridVolume Interface
	virtual bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const override;
	virtual bool IsVoxelWalkable(const FIntVector& Index) const override;
	virtual void GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const override;
	//~ End AUniformGridVolume Interface

	void OnChunkLoadCompleted(FIntVector ChunkCoords, FStreamableHandle* StreamableHandle);

protected:
	//======================================================================
	// --- CHUNK STREAMING INTERNALS ---
	//======================================================================

	virtual void UpdateStreamingState();
	
	void RequestChunkLoad(const FIntVector& ChunkCoords);
		
	void UnloadChunk(const FIntVector& ChunkCoords);

	// UFUNCTION()
	// void OnLevelLoadCompleted();

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

	void RegisterChunk(UStaticVoxelChunkData* ChunkData);
	void UnregisterChunk(const FIntVector& ChunkCoords);
	
	/**
	 * Uses bitmasks to perform a very fast check if a region contains any obstacles.
	 * This is a broad-phase check; a `false` return means the region *might* have an obstacle,
	 * while `true` guarantees it is empty.
	 */
	bool IsRegionEmpty_Bitmask(const FIntVector& MinCorner, const FIntVector& MaxCorner) const;
	
	uint32 GetChunkSize() {return CHUNK_SIZE; }
	
	void GetCombinedBitmask(TArray<F128BitChunk>& CombinedBitmask) const;

	
private:
	/** Initializes the core data structures like the Fenwick Tree and Bitmasks. */
	void InitializeDataStructures();

	/** Scans the world for dynamic obstacles and updates the Fenwick Tree with any changes. */
	void UpdateDynamicObstacles();

	void UpdateDynamicVoxelState(const FIntVector& Index, bool bIsObstacle);

	void ApplyDynamicObstacleChanges(const TSet<FIntVector>& NewFrameObstacles);
	
	/** Gathers all voxels currently occupied by dynamic obstacles. */
	void GatherCurrentObstacleVoxels(TSet<FIntVector>& OutVoxelIndices) const;

	FORCEINLINE bool IsVoxelIndexValid(const FIntVector& VoxelIndex) const;

#if WITH_EDITOR
	bool SaveCurrentWorld(UPackage* WorldPackage);
	
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

	virtual void UpdateDebugObstacleVisualization() override;
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
	static constexpr uint32 CHUNK_SIZE = 128;
	
	/** Set of voxels occupied by static geometry. */
	TSet<FIntVector> StaticObstacleVoxels;

	/** Map of voxels occupied by dynamic obstacles and the number of obstacles in each. Used to track changes frame-to-frame. */
	TMap<FIntVector, int32> DynamicObstacleVoxelCounts;

	TMap<FIntVector, TObjectPtr<UStaticVoxelChunkData>> ActiveChunks;
public:
	/** Bitmasks for fast obstacle checks, organized with X-axis as the inner loop. */
	TArray<F128BitChunk> StaticObstacleBitmask;
	TArray<F128BitChunk> DynamicObstacleBitmask;
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
	TSet<FIntVector> CurrentDynamicObstacles;
	TMap<FIntVector, TSharedPtr<FStreamableHandle>> PendingChunkHandles;

	FRWLock VolumeDataLock; 
};