// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/PathfindingRequest.h"
#include "PathfindingStrategy.h"
#include "Pathfinder/Utility/MemoryPool.h"
#include "JPSPlusStrategy.generated.h"
constexpr int32 NUM_ALL_DIRECTIONS = 26;
struct FPathNode;
/**
 * 
 */
class AFenwickVoxelVolume;
class AVoxelVolumeBase;

USTRUCT()
struct FNeighborWalkability
{
	GENERATED_BODY()
	uint32 WalkableMask = 0;

	bool IsWalkable(int32 DirIndex) const
	{
		return (WalkableMask & (1 << DirIndex)) != 0;
	}
};

USTRUCT()
struct FMoveFeasibilityKey
{
	GENERATED_BODY()

	UPROPERTY()
	FIntVector From;

	UPROPERTY()
	FIntVector To;

	FMoveFeasibilityKey() = default;

	FMoveFeasibilityKey(const FIntVector& InFrom, const FIntVector& InTo) : From(InFrom), To(InTo)
	{
	}

	bool operator==(const FMoveFeasibilityKey& Other) const
	{
		return From == Other.From && To == Other.To;
	}
};

inline uint32 GetTypeHash(const FMoveFeasibilityKey& Key)
{
	return HashCombine(GetTypeHash(Key.From), GetTypeHash(Key.To));
}


struct FThreadSafeBitmaskGrid
{
    TArray<uint64> ObstacleBitmasks_X;
    FIntVector GridSize;
    
    // 이 구조체는 UObject를 포함하지 않으므로 여러 스레드에서 안전하게 읽을 수 있습니다.

    // 헬퍼 상수
    static const int32 CHUNK_SIZE = 64; // FenwickVoxelVolume의 CHUNK_SIZE와 일치해야 합니다.

    FThreadSafeBitmaskGrid() : GridSize(FIntVector::ZeroValue) {}

    // X축 비트마스크 배열의 1차원 인덱스를 계산하는 헬퍼 함수
    // (AFenwickVoxelVolume::GetFlatIndex_X 와 동일한 로직)
    uint32 GetFlatIndex_X(uint32 ChunkX, uint32 Y, uint32 Z) const
    {
        const uint32 ChunksAlongX = GridSize.X / CHUNK_SIZE;
        return (Z * GridSize.Y + Y) * ChunksAlongX + ChunkX;
    }

	bool IsIndexValid(const FIntVector& Index) const
    {
    	return Index.X >= 0 && Index.X < GridSize.X &&
			Index.Y >= 0 && Index.Y < GridSize.Y &&
			Index.Z >= 0 && Index.Z < GridSize.Z;
    }

    // IsRegionEmpty_Bitmask의 스레드 안전 버전.
    // UObject 멤버에 접근하는 대신, 복사된 데이터로만 연산합니다.
    bool IsRegionEmpty(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
    {
        // 경계 값을 맵 크기 안으로 제한합니다.
        const FIntVector ClampedMin(
            FMath::Max(MinCorner.X, 0),
            FMath::Max(MinCorner.Y, 0),
            FMath::Max(MinCorner.Z, 0)
        );

        const FIntVector ClampedMax(
            FMath::Min(MaxCorner.X, GridSize.X - 1),
            FMath::Min(MaxCorner.Y, GridSize.Y - 1),
            FMath::Min(MaxCorner.Z, GridSize.Z - 1)
        );

        // Z, Y 축을 순회하며 비트마스크로 X축을 한 번에 검사합니다.
        for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
        {
            for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
            {
                const int32 StartChunkX = ClampedMin.X / CHUNK_SIZE;
                const int32 EndChunkX = ClampedMax.X / CHUNK_SIZE;

                for (int32 cx = StartChunkX; cx <= EndChunkX; ++cx)
                {
                    const uint64 Mask = ObstacleBitmasks_X[GetFlatIndex_X(cx, y, z)];
                    if (Mask == 0) continue; // 이 청크(64개 복셀)에 장애물이 없으면 통과

                    const int32 VoxelStartX = (cx == StartChunkX) ? ClampedMin.X : cx * CHUNK_SIZE;
                    const int32 VoxelEndX = (cx == EndChunkX) ? ClampedMax.X : (cx + 1) * CHUNK_SIZE - 1;
                    const int32 StartBit = VoxelStartX % CHUNK_SIZE;
                    const int32 EndBit = VoxelEndX % CHUNK_SIZE;

                    // 검사할 범위에 해당하는 비트만 1로 채운 마스크를 생성합니다.
                    const uint64 CheckMask = (~0ULL << StartBit) & (~0ULL >> (63 - EndBit));
                    
                    // 장애물 비트마스크와 검사 마스크를 AND 연산하여 겹치는 부분이 있는지 확인합니다.
                    if ((Mask & CheckMask) != 0)
                    {
                        return false; // 장애물 발견!
                    }
                }
            }
        }
        return true; // 장애물 없음
    }
};

class FJPSPlusPathfinder
{
public:
	// 생성자: 길찾기에 필요한 모든 데이터를 안전하게 복사/참조합니다.
	FJPSPlusPathfinder(const FPathfindingRequest& InRequest,FPathNodeMemoryPool& InNodePool, const FThreadSafeBitmaskGrid& InGrid, const TArray<FIntVector>& InAllDirections, const TMap<FIntVector, int32>& InDirectionToIndexMap, FIntVector InActualGoal);
	// 실제 길찾기를 수행하는 메인 함수
	EPathfindingResult FindPath(TArray<FIntVector>& OutPath);

private:
	// UJPSPlusStrategy에 있던 모든 알고리즘 함수들을 이곳으로 이동시킵니다.
	void IdentifySuccessors(FPathNode* Node, TArray<FPathNode*>& OutSuccessors);
	TOptional<FIntVector> Jump(const FIntVector& StartPos, const FIntVector& Direction);
	void GetPrunedDirections(const FIntVector& Dir, TArray<FIntVector>& OutDirections) const;
	void OptimizePathFromJumpPoints(FPathNode* EndNode, TArray<FIntVector>& OutOptimizedPath) const;
	bool HasLineOfSight(const FIntVector& From, const FIntVector& To) const;
	bool IsAgentMoveFeasible_ThreadSafe(const FIntVector& From, const FIntVector& To) const;
	bool HasForcedNeighbour(const FIntVector& NodePos, const FIntVector& CheckDir, const FIntVector& ParentDir) const;
	uint64 CalculateForcedNeighborMask(const FNeighborWalkability& Neighbors, int32 ParentDirIndex) const;
	int32 CalculateHeuristic(const FIntVector& A, const FIntVector& B) const;
	int32 GetMoveCost(const FIntVector& A, const FIntVector& B) const;

	// UJPSPlusStrategy에 있던 모든 상태 변수들을 이곳으로 이동시킵니다.
	const FPathfindingRequest& Request;
	const FThreadSafeBitmaskGrid& WalkabilityGrid;
	const TArray<FIntVector>& AllDirections;
	const TMap<FIntVector, int32>& DirectionToIndexMap;

	FIntVector ActualGoalIndex;
	
	int32 AgentRadiusInVoxels;

	FPathNodeMemoryPool& NodePool;
	TArray<FPathNode*> OpenSet;
	TMap<FIntVector, FPathNode*> OpenSetLookup;
	TMap<FIntVector, FPathNode*> ClosedSet;
};


UCLASS()
class NAVIGATION3D_API UJPSPlusStrategy : public UPathfindingStrategy
{
	GENERATED_BODY()

public:
	// UObject로서의 역할만 수행합니다.
	virtual void Initialize(AVoxelVolumeBase* InVolume) override;
	virtual bool FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FIntVector>& OutPath) override;
	virtual bool ShouldRepathForOptimization(const FVector& CurrentAgentLocation, const TArray<FVector>& CurrentPath,
										 int32 CurrentPathIndex, const FVector& GoalLocation,
										 uint32 AgentRadius) const override;
	virtual FName GetStrategyName() const override;

private:
	// 멤버 변수들이 대폭 줄어듭니다.
	TWeakObjectPtr<AFenwickVoxelVolume> Volume;
    
	// 이 데이터는 한 번만 계산하고 모든 '일꾼'에게 공유합니다.
	TUniquePtr<FPathNodeMemoryPool> NodePool;
	TArray<FIntVector> AllDirections;
	TMap<FIntVector, int32> DirectionToIndexMap;
};
