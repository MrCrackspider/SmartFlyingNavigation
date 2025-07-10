// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/PathfindingRequest.h"
#include "PathfindingStrategy.h"
#include "Pathfinder/ThreadSafeBitmask.h"
#include "Pathfinder/Utility/MemoryPool.h"
#include "JPSPlusStrategy.generated.h"


constexpr int32 NUM_ALL_DIRECTIONS = 26;
struct FPathNode;
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


class FJPSPlusPathfinder
{
public:
	// 생성자: 길찾기에 필요한 모든 데이터를 안전하게 복사/참조합니다.

	FJPSPlusPathfinder(const FPathfindingRequest& InRequest,
	                   const TArray<FIntVector>& InAllDirections,
	                   const TMap<FIntVector, int32>& InDirectionToIndexMap,
	                   FIntVector InActualGoal);
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

	//FPathNodeMemoryPool& NodePool;
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
	//TUniquePtr<FPathNodeMemoryPool> NodePool;
	TArray<FIntVector> AllDirections;
	TMap<FIntVector, int32> DirectionToIndexMap;
};
