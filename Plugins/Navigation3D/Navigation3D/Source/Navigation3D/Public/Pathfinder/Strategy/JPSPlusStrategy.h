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

UCLASS()
class NAVIGATION3D_API UJPSPlusStrategy : public UPathfindingStrategy
{
	GENERATED_BODY()

public:
	// =============================================================================
	// UPathfindingStrategy 인터페이스 (Public API)
	// =============================================================================
	virtual void Initialize(AVoxelVolumeBase* InVolume) override;

	virtual bool FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FIntVector>& OutPath) override;
	
	virtual bool ShouldRepathForOptimization(const FVector& CurrentAgentLocation, const TArray<FVector>& CurrentPath,
	                                         int32 CurrentPathIndex, const FVector& GoalLocation,
	                                         uint32 AgentRadius) const override;

	virtual FName GetStrategyName() const override;

protected:
	// =============================================================================
	// 메인 알고리즘 흐름 (Main Algorithm Flow)
	// =============================================================================
	EPathfindingResult FindPathInternal(const FPathfindingRequest& InRequest, TArray<FIntVector>& OutPath);

	// =============================================================================
	// JPS+ 핵심 로직 (JPS+ Core Logic)
	// =============================================================================
	/** @brief Pruned Directions에 대해 Jump를 수행하여 후속 점프 포인트(Successors)를 식별합니다. */
	void IdentifySuccessors(FPathNode* Node, const FIntVector& Goal, int32 AgentRadiusInVoxels,
	                        TArray<FPathNode*>& OutSuccessors);

	/** @brief 한 방향으로 점프하며 Goal, Forced Neighbour 등을 찾습니다. */
	TOptional<FIntVector> Jump(const FIntVector& CurrentPos, const FIntVector& Direction, const FIntVector& Goal,
	                           int32 AgentRadiusInVoxels);

	/** @brief 부모 방향을 기반으로, Pruning 규칙에 따라 탐색할 방향을 결정합니다. */
	void GetPrunedDirections(const FIntVector& Dir, TArray<FIntVector>& OutDirections) const;

	// =============================================================================
	// 경로 후처리 및 유효성 검사 (Path Post-Processing & Validation)
	// =============================================================================
	/** @brief 점프 포인트 목록을 Theta* 스타일의 LOS 검사를 통해 직선화하여 최종 경로를 생성합니다. */
	void OptimizePathFromJumpPoints(FPathNode* EndNode, TArray<FIntVector>& OutOptimizedPath,
	                                uint32 AgentRadiusInVoxels) const;

	/** @brief 두 지점 사이에 장애물이 없는지(Line of Sight) 안전하게 확인합니다. */
	bool HasLineOfSight(const FIntVector& From, const FIntVector& To, int32 AgentRadiusInVoxels) const;

	/** @brief 에이전트의 부피를 고려하여 두 지점 간 이동이 가능한지 엄밀하게 판단합니다. */
	bool IsAgentMoveFeasible(const FIntVector& From, const FIntVector& To, int32 AgentRadiusInVoxels) const;

	/** @brief 현재 위치가 코너를 돌기 위해 반드시 거쳐야 하는 '강제 이웃'을 가지고 있는지 확인합니다. */
	bool HasForcedNeighbour(const FIntVector& NodePos, const FIntVector& CheckDir, const FIntVector& ParentDir,
	                        uint32 AgentRadiusInVoxels) const;

	/** @brief 미리 계산된 이웃 정보를 비트마스크로 받아 강제 이웃을 더 빠르게 판단합니다. */
	bool HasForcedNeighbourBitMask(const FNeighborWalkability& Neighbors, int32 CheckDirIndex,
	                               int32 ParentDirIndex) const;
	uint64 CalculateForcedNeighborMask(const FNeighborWalkability& Neighbors, int32 ParentDirIndex) const;
	// =============================================================================
	// 유틸리티 (Utilities)
	// =============================================================================
	int32 CalculateHeuristic(const FIntVector& A, const FIntVector& B) const;
	int32 GetMoveCost(const FIntVector& A, const FIntVector& B) const;

private:
	// =============================================================================
	// 멤버 변수 (Member Variables)
	// =============================================================================

	// --- 엔진/월드 의존성 ---
	TWeakObjectPtr<AFenwickVoxelVolume> Volume;
	TUniquePtr<FPathNodeMemoryPool> NodePool;

	// --- 경로탐색 상태 데이터 (매번 초기화됨) ---
	TArray<FPathNode*> OpenSet;
	TMap<FIntVector, FPathNode*> OpenSetLookup;
	TMap<FIntVector, FPathNode*> ClosedSet;

	// --- 사전 계산된 데이터 (초기화 시 생성) ---
	TArray<FIntVector> AllDirections;
	TMap<FIntVector, int32> DirectionToIndexMap;

	// --- 스레드 안전 캐시 ---
	mutable FCriticalSection CacheMutex;
	mutable TMap<FMoveFeasibilityKey, bool> MoveFeasibilityCache;
};
