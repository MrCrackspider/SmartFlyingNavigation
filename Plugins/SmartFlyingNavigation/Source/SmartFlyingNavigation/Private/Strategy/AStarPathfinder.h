// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Strategy/PathfindingStrategy.h"
#include "Utility/AStarStrategyUtils.h"
#include "NavCore/OctreePathfindingData.h"

/**
 * Lazy Theta* pathfinder over a baked octree chunk. Implementation detail of
 * UAStarStrategy — not part of the public plugin API.
 */
class FAstarPathfinder
{
public:
	FAstarPathfinder(const FOctreePathfindingData& InOctreeData, int32 InStartIndex, int32 InEndIndex,
	                 const FVector& InOriginalStartLocation, const FVector& InEffectiveStartLocation,
	                 const FVector& InEndLocation, float InAgentRadius,
	                 TNodeMemoryPool<FAstarPathNode>& InMemoryPool,
	                 float InClearancePenaltyWeight = 0.f, float InPreferredClearanceMultiplier = 2.f);

	EPathfindingResult FindPath(TArray<FVector>& OutPath);

	bool HasClearPath(const FVector& Start, const FVector& End) const;

private:
	void IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors);
	bool TryToRepairPartialPath(FAstarPathNode* PartialEndNode, TArray<FVector>& OutPath);
	int64 CalculateBoundHeuristic(int32 NodeIndex) const;
	int64 GetMoveCost(const FVector& A, const FVector& B) const;
	void ReconstructAndSmoothPath(FAstarPathNode* EndNode, TArray<FVector>& OutPath);

	const FOctreePathfindingData& OctreeData;
	const int32 StartNodeIndex;
	const int32 GoalNodeIndex;
	const float AgentRadius;

	const FVector OriginalStartLocation;
	const FVector EffectiveStartLocation;

	const FVector ActualGoalLocation;
	TNodeMemoryPool<FAstarPathNode>& MemoryPool;

	/** Clearance-based cost shaping: penalize moves that hug obstacles (requires a distance field). */
	const float ClearancePenaltyWeight;
	const float PreferredClearanceMultiplier;

	TArray<FAstarPathNode*> OpenSet;
	TMap<int32, FAstarPathNode*> OpenSetLookup; // Key: NodeIndex
	TMap<int32, FAstarPathNode*> ClosedSet; // Key: NodeIndex
};
