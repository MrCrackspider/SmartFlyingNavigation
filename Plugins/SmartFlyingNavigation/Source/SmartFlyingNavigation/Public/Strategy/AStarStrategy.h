// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Strategy/PathfindingStrategy.h"
#include "AStarStrategy.generated.h"

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
	bool TryToRepairPartialPath(FAstarPathNode* PartialEndNode, TArray<FVector>& OutPath);
	int64 CalculateBoundHeuristic(int32 NodeIndex) const;
	int64 GetMoveCost(const FVector& A, const FVector& B) const;
	void ReconstructAndSmoothPath(FAstarPathNode* EndNode, TArray<FVector>& OutPath);
	bool HasClearPath(const FVector& Start, const FVector& End) const;
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
class SMARTFLYINGNAVIGATION_API UAStarStrategy : public UPathfindingStrategy
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
	float NearbyGoalSearchRadius = 500.f;

	UPROPERTY(EditDefaultsOnly, Category = "A* | Fallback Search", meta=(ClampMin="1"))
	int32 NearbyGoalSearchIterations = 5;
};
