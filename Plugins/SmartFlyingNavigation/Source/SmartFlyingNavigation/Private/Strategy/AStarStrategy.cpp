// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Strategy/AStarStrategy.h"
#include "Strategy/AStarPathfinder.h"
#include "Utility/AStarStrategyUtils.h"

void UAStarStrategy::Initialize()
{
}

bool UAStarStrategy::FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_FindPathWithStrategy);

	const FOctreePathfindingData& OctreeData = InRequest.NavigationData;
	if (!OctreeData.StaticNodePoolPtr)
	{
		UE_LOG(LogTemp, Error, TEXT("AStarStrategy: StaticNodePoolPtr is null. Pathfinding aborted."));
		return false;
	}
	const TArray<FOctreeNode>& PoolRef = *OctreeData.StaticNodePoolPtr;

	int32 StartNodeIndex = -1;
	FVector EffectiveStartLocation = InRequest.StartLocation;

	const bool bIsTrulyObstructed = OctreeData.IsPointBlockedForAgent(
		InRequest.StartLocation, InRequest.AgentRadius);

	if (bIsTrulyObstructed)
	{
		StartNodeIndex = OctreeData.FindNearestValidLeafForAgent(
			InRequest.StartLocation, InRequest.AgentRadius, NearbyGoalSearchRadius, NearbyGoalSearchIterations);

		if (!PoolRef.IsValidIndex(StartNodeIndex))
		{
			UE_LOG(LogTemp, Error, TEXT("AStarStrategy: Start is obstructed AND no safe alternative was found. Pathfinding failed."));
			return false;
		}
		EffectiveStartLocation = PoolRef[StartNodeIndex].Bounds.GetCenter();
	}
	else
	{
		StartNodeIndex = OctreeData.FindLeafNodeAt(InRequest.StartLocation);

		if (!PoolRef.IsValidIndex(StartNodeIndex))
		{
			UE_LOG(LogTemp, Warning, TEXT("AStarStrategy: Start is safe but on a boundary. Finding nearest reference node (start location will NOT be moved)."));

			StartNodeIndex = OctreeData.FindNearestValidLeafForAgent(
				InRequest.StartLocation, InRequest.AgentRadius, NearbyGoalSearchRadius, 1);

			if (!PoolRef.IsValidIndex(StartNodeIndex))
			{
				UE_LOG(LogTemp, Error, TEXT("AStarStrategy: Could not find any reference node near the boundary start. Pathfinding failed."));
				return false;
			}
		}
	}

	int32 GoalNodeIndex = OctreeData.FindLeafNodeAt(InRequest.EndLocation);

	if (!PoolRef.IsValidIndex(GoalNodeIndex) || PoolRef[GoalNodeIndex].State != EOctreeNodeState::Empty)
	{
		UE_LOG(LogTemp, Warning, TEXT("AStarStrategy: Goal location is inside a blocked voxel. A* will attempt to find the closest possible path."));
	}

	FBox GoalQueryBox = AStarStrategyUtils::MakeAgentQueryBox(InRequest.EndLocation, InRequest.AgentRadius);
	if (OctreeData.IsRegionDynamicallyBlocked(GoalQueryBox))
	{
		UE_LOG(LogTemp, Error, TEXT("AStarStrategy: Goal location is blocked by a dynamic obstacle. Pathfinding failed."));
		return false;
	}

	FPathNodeMemoryPoolBase* BasePool = InRequest.MemoryPool;
	if (!BasePool)
	{
		UE_LOG(LogTemp, Error, TEXT("AStarStrategy: MemoryPool is NULL! Pathfinding aborted."));
		return false;
	}
	auto& NodePool = *static_cast<TNodeMemoryPool<FAstarPathNode>*>(BasePool);

	checkf(PoolRef.IsValidIndex(StartNodeIndex), TEXT("AStarStrategy: StartNodeIndex (%d) is invalid right before creating Pathfinder! Logic is flawed."), StartNodeIndex);

	FAstarPathfinder Pathfinder(OctreeData, StartNodeIndex, GoalNodeIndex,
	                            InRequest.StartLocation, EffectiveStartLocation, InRequest.EndLocation,
	                            InRequest.AgentRadius, NodePool,
	                            ClearancePenaltyWeight, PreferredClearanceMultiplier);

	const EPathfindingResult Result = Pathfinder.FindPath(OutPath);

	return (Result == EPathfindingResult::Success || Result == EPathfindingResult::Success_Partial);
}

bool UAStarStrategy::ShouldRepathForOptimization(const FIntVector& CurrentAgentVoxel,
                                                 const TArray<FIntVector>& CurrentPathVoxels,
                                                 int32 CurrentPathIndex,
                                                 const FIntVector& GoalVoxel,
                                                 uint32 AgentRadiusVoxels) const
{
	return false;
}

FName UAStarStrategy::GetStrategyName() const
{
	return TEXT("AStarStrategy");
}
