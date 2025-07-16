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
	FAstarPathfinder(const FOctreePathfindingData& InOctreeData, int32 InStartIndex, int32 InEndIndex,
	                 const FVector& InStartLocation, const FVector& InEndLocation, float InAgentRadius,
	                 TNodeMemoryPool<FAstarPathNode>& InMemoryPool);
	EPathfindingResult FindPath(TArray<FVector>& OutPath);

private:
	void IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors);
	int64 CalculateHeuristic(const FVector& A, const FVector& B) const;
	int64 GetMoveCost(const FVector& A, const FVector& B) const;
	void ReconstructPath(FAstarPathNode* EndNode, TArray<FVector>& OutPath) const;
	void ReconstructAndSmoothPath(FAstarPathNode* EndNode, TArray<FVector>& OutPath);
	bool HasLineOfSightWithRadius(const FVector& Start, const FVector& End) const;

	const FOctreePathfindingData& OctreeData;
	const int32 StartNodeIndex;
	const int32 GoalNodeIndex;
	const float AgentRadius;
	const FVector ActualStartLocation;
	const FVector ActualGoalLocation;
	TNodeMemoryPool<FAstarPathNode>& MemoryPool;

	TArray<FAstarPathNode*> OpenSet;
	TMap<int32, FAstarPathNode*> OpenSetLookup; // Key: NodeIndex
	TMap<int32, FAstarPathNode*> ClosedSet; // Key: NodeIndex
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

public:
	UPROPERTY(EditDefaultsOnly, Category = "A* | Fallback Search", meta=(ClampMin="50.0"))
	float NearbyGoalSearchRadius = 300.f;

	/** 주변 탐색을 몇 번까지 확장하며 시도할지 결정합니다. (SearchRadius * Iteration) */
	UPROPERTY(EditDefaultsOnly, Category = "A* | Fallback Search", meta=(ClampMin="1"))
	int32 NearbyGoalSearchIterations = 5;

private:
	FThreadSafeBitmaskGrid Grid;
	TArray<FIntVector> AllDirections;
	TMap<FIntVector, int32> DirectionToIndexMap;
};
