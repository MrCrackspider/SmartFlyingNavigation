// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VoxelVolume.generated.h"

static constexpr uint32 CHUNK_SIZE = 64;

struct FDebugBoxCullingResult
{
	TArray<FVector> VisibleStaticVoxels;
	TArray<FVector> VisibleDynamicVoxels;
};

struct FAsyncCullingTaskParams
{
	TSet<FIntVector> StaticVoxels;
	TArray<FIntVector> DynamicVoxels;
	FMatrix ViewProjectionMatrix;
	FTransform ActorToWorldTransform;
	float VoxelExtent;
};
UCLASS()
class EPICPROJECT_3DNAVI_API AVoxelVolume : public AActor
{
	GENERATED_BODY()

public:
	AVoxelVolume();

	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	bool IsVoxelWalkable(const FIntVector& Index) const;

	uint64 GetObstacleChunk_X(int32 ChunkX, int32 Y, int32 Z) const;
	uint64 GetObstacleChunk_Y(int32 ChunkY, int32 X, int32 Z) const;
	uint64 GetObstacleChunk_Z(int32 ChunkZ, int32 X, int32 Y) const;

	void InitializeMap(const FIntVector& InMapSize);

	virtual void BuildStaticObstacles();
	virtual void UpdateDynamicObstacles();
	void DrawDebugVoxelObstacles() const;

	void SetStaticObstacle(const FIntVector& Index, bool bIsObstacle);

	bool IsIndexValid(const FIntVector& Index) const;

	UFUNCTION(BlueprintPure, Category = "VoxelVolume|Coordinates")
	FVector VoxelIndexToWorld(const FIntVector& VoxelIndex) const;
	UFUNCTION(BlueprintPure, Category = "VoxelVolume|Coordinates")
	FIntVector WorldToVoxelIndex(const FVector& WorldPosition) const;
	bool CheckVoxelOverlap(const FIntVector& VoxelIndex, UPrimitiveComponent* ComponentToTest) const;
	bool ClampVoxelIndicesToVolume(FIntVector& MinVoxelIndex, FIntVector& MaxVoxelIndex) const;
	bool GetOverlappingComponentsInVolume(const FCollisionObjectQueryParams& ObjectQueryParams,
	                                      TArray<FOverlapResult>& OutResults) const;

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
	FIntVector MapSize; // 들어가는 복셀의 갯수
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinding", meta = (AllowPrivateAccess = "true"))
	float VoxelExtent;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VoxelVolume|Debug")
	bool bDrawDebugObstacles = true; // 디버그 드로잉 활성화 여부

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VoxelVolume|Debug",
		meta = (EditCondition = "bDrawDebugObstacles"))
	FColor DebugObstacleColor = FColor::Red;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VoxelVolume|Debug",
		meta = (EditCondition = "bDrawDebugObstacles"))
	float DebugObstacleThickness = 2.0f;

protected:
	// 비트마스크 데이터. 각 축을 64개 단위로 묶음
	TArray<uint64_t> ObstacleBitmasks_X;
	TArray<uint64_t> ObstacleBitmasks_Y;
	TArray<uint64_t> ObstacleBitmasks_Z;

	TMap<FIntVector, int32> DynamicObstacleVoxelCounts;

public:
	FIntVector GetMapSize() const { return MapSize; }
	float GetVoxelExtent() const { return VoxelExtent; }

	// X축 우선으로 정렬된 청크 배열의 인덱스를 계산
	// 메모리 레이아웃: [ (Z=0, Y=0, X=0..n), (Z=0, Y=1, X=0..n), ... ]
	uint32 GetFlatIndex_X(uint32 ChunkX, uint32 Y, uint32 Z) const
	{
		const uint32 ChunksAlongX = MapSize.X / CHUNK_SIZE;
		return (Z * MapSize.Y + Y) * ChunksAlongX + ChunkX;
	}

	// Y축 우선으로 정렬된 청크 배열의 인덱스를 계산
	// 메모리 레이아웃: [ (Z=0, X=0, Y=0..n), (Z=0, X=1, Y=0..n), ... ]
	uint32 GetFlatIndex_Y(uint32 ChunkY, uint32 X, uint32 Z) const
	{
		const uint32 ChunksAlongY = MapSize.Y / CHUNK_SIZE;
		return (Z * MapSize.X + X) * ChunksAlongY + ChunkY;
	}

	// Z축 우선으로 정렬된 청크 배열의 인덱스를 계산
	// 메모리 레이아웃: [ (Y=0, X=0, Z=0..n), (Y=0, X=1, Z=0..n), ... ]
	uint32 GetFlatIndex_Z(uint32 ChunkZ, uint32 X, uint32 Y) const
	{
		const uint32 ChunksAlongZ = MapSize.Z / CHUNK_SIZE;
		return (Y * MapSize.X + X) * ChunksAlongZ + ChunkZ;
	}

protected: // Draw DebugBox
	TSet<FIntVector> StaticObstacleVoxels;

	void StartAsyncCullingTask();
	bool IsVoxelVisible(const UWorld* World, const FVector& ViewLocation, const FVector& VoxelWorldPos,
	                    const FVector& VoxelBoxExtent, const FCollisionQueryParams& OcclusionParams) const;
	TSharedPtr<FDebugBoxCullingResult> LastCompletedCullingResult;

	TFuture<TSharedPtr<FDebugBoxCullingResult>> CullingTaskFuture;
};
