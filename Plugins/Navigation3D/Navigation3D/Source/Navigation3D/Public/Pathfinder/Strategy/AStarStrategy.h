// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/Strategy/PathfindingStrategy.h"
#include "AStarStrategy.generated.h"

// --- 전방 선언 ---
struct FPathfindingRequest;
struct FOctreePathfindingData;
struct FAstarPathNode;

class FAstarPathfinder
{
public:
	FAstarPathfinder(const FOctreePathfindingData& InOctreeData, int32 InStartNodeIndex, int32 InGoalNodeIndex, float InAgentRadius, TNodeMemoryPool<FAstarPathNode>& InMemoryPool);
	EPathfindingResult FindPath(TArray<FVector>& OutPath);

private:
	void IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors);
	int64 CalculateHeuristic(const FVector& A, const FVector& B) const;
	int64 GetMoveCost(const FVector& A, const FVector& B) const;
	void ReconstructPath(FAstarPathNode* EndNode, TArray<FVector>& OutPath) const;

	const FOctreePathfindingData& OctreeData;
	const int32 StartNodeIndex;
	const int32 GoalNodeIndex;
	const float AgentRadius;
	TNodeMemoryPool<FAstarPathNode>& MemoryPool;
	
	TArray<FAstarPathNode*> OpenSet;
	TMap<int32, FAstarPathNode*> OpenSetLookup; // Key: NodeIndex
	TMap<int32, FAstarPathNode*> ClosedSet;     // Key: NodeIndex
};

UCLASS()
class NAVIGATION3D_API UAStarStrategy : public UPathfindingStrategy
{
	GENERATED_BODY()

public:
	virtual void Initialize() override;
	virtual bool FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath) override;
	
	virtual bool ShouldRepathForOptimization(const FIntVector& CurrentAgentVoxel,
	                                         const TArray<FIntVector>& CurrentPathVoxels,
	                                         int32 CurrentPathIndex,
	                                         const FIntVector& GoalVoxel,
	                                         uint32 AgentRadiusVoxels) const override;

	virtual FName GetStrategyName() const override;

private:
	FThreadSafeBitmaskGrid Grid;
	TArray<FIntVector> AllDirections;
	TMap<FIntVector, int32> DirectionToIndexMap;
};
