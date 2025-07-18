// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Pathfinder/Strategy/AStarStrategy.h"
#include "Pathfinder/Utility/AStarStrategyUtils.h"

void UAStarStrategy::Initialize()
{
}

bool UAStarStrategy::FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_FindPathWithStrategy);
	
	const FOctreePathfindingData& OctreeData = InRequest.NavigationData;

	const int32 StartNodeIndex = OctreeData.FindLeafNodeAt(InRequest.StartLocation);
	int32 GoalNodeIndex = OctreeData.FindLeafNodeAt(InRequest.EndLocation);

	bool bIsGoalNodeInvalid = (GoalNodeIndex == -1) ||
		(OctreeData.StaticNodePool[GoalNodeIndex].State != EOctreeNodeState::Empty) ||
		(OctreeData.StaticNodePool[GoalNodeIndex].MaxAllowedRadius < InRequest.AgentRadius);

	if (StartNodeIndex == -1 || OctreeData.StaticNodePool[StartNodeIndex].State != EOctreeNodeState::Empty)
	{
		UE_LOG(LogTemp, Warning, TEXT("A* (Octree) Failed: Start location is invalid or inside a solid node."));
	}

	if (bIsGoalNodeInvalid)
	{
		const int32 NewGoalNodeIndex = OctreeData.FindNearestValidLeafForAgent(
			InRequest.EndLocation,
			InRequest.AgentRadius,
			NearbyGoalSearchRadius,
			NearbyGoalSearchIterations);

		if (NewGoalNodeIndex == -1)
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("A* (Octree) Failed: Could not find any valid (and wide enough) nearby goal location."));
			return false;
		}

		GoalNodeIndex = NewGoalNodeIndex;
		UE_LOG(LogTemp, Log, TEXT("A* (Octree): Found alternative valid goal at node index %d."), GoalNodeIndex);
	}


	if (OctreeData.IsRegionDynamicallyBlocked(OctreeData.GetNodeBounds(StartNodeIndex)))
	{
		UE_LOG(LogTemp, Warning, TEXT("A* (Octree) Failed: Start location is dynamically blocked."));
		return false;
	}
	if (OctreeData.IsRegionDynamicallyBlocked(OctreeData.GetNodeBounds(GoalNodeIndex)))
	{
		UE_LOG(LogTemp, Warning, TEXT("A* (Octree) Failed: Goal location is dynamically blocked."));
		return false;
	}

	FPathNodeMemoryPoolBase* BasePool = InRequest.MemoryPool;
	if (!BasePool)
	{
		UE_LOG(LogTemp, Error, TEXT("A* Strategy failed to lease a memory pool."));
		return false;
	}
	auto& NodePool = *static_cast<TNodeMemoryPool<FAstarPathNode>*>(BasePool);

	FAstarPathfinder Pathfinder(OctreeData, StartNodeIndex, GoalNodeIndex,
	                            InRequest.StartLocation, InRequest.EndLocation,
	                            InRequest.AgentRadius, NodePool);

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


// =======================================================================
// FAstarPathfinder
// =======================================================================

FAstarPathfinder::FAstarPathfinder(const FOctreePathfindingData& InOctreeData, int32 InStartIndex, int32 InEndIndex,
                                   const FVector& InStartLocation, const FVector& InEndLocation, float InAgentRadius,
                                   TNodeMemoryPool<FAstarPathNode>& InMemoryPool)
	: OctreeData(InOctreeData)
	  , StartNodeIndex(InStartIndex)
	  , GoalNodeIndex(InEndIndex)
	  , ActualStartLocation(InStartLocation)
	  , ActualGoalLocation(InEndLocation)
	  , AgentRadius(InAgentRadius)
	  , MemoryPool(InMemoryPool)
{
}

EPathfindingResult FAstarPathfinder::FindPath(TArray<FVector>& OutPath)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_Pathfinder_FindPath_Internal);

	MemoryPool.Reset();
	OpenSet.Empty();
	OpenSetLookup.Empty();
	ClosedSet.Empty();

	const auto& Pool = OctreeData.StaticNodePool;
	const FVector GoalCenter = Pool[GoalNodeIndex].Bounds.GetCenter();
	const TPathNodePtrCompare<FAstarPathNode> Predicate;

	TArray<FAstarPathNode*> Successors;
	Successors.Reserve(8);

	FAstarPathNode* StartNode = MemoryPool.NewNode();
	if (!StartNode) return EPathfindingResult::Failed_OutOfMemory;

	StartNode->NodeIndex = StartNodeIndex;
	StartNode->WorldPosition = OctreeData.StaticNodePool[StartNodeIndex].Bounds.GetCenter();
	StartNode->HCost = CalculateHeuristic(StartNode->WorldPosition, GoalCenter);
	StartNode->CalculateAndSetFCost();

	OpenSet.HeapPush(StartNode, Predicate);
	OpenSetLookup.Add(StartNodeIndex, StartNode);

	FAstarPathNode* BestPartialNode = StartNode;

	while (!OpenSet.IsEmpty())
	{
		FAstarPathNode* CurrentNode;
		OpenSet.HeapPop(CurrentNode, Predicate);

		OpenSetLookup.Remove(CurrentNode->NodeIndex);
		ClosedSet.Add(CurrentNode->NodeIndex, CurrentNode);

		if (CurrentNode->HCost < BestPartialNode->HCost)
		{
			BestPartialNode = CurrentNode;
		}

		if (CurrentNode->NodeIndex == GoalNodeIndex)
		{
			ReconstructAndSmoothPath(CurrentNode, OutPath);
			return EPathfindingResult::Success;
		}

		Successors.Reset();
		IdentifySuccessors(CurrentNode, Successors);

		for (FAstarPathNode* Successor : Successors)
		{
			const int32 SuccIdx = Successor->NodeIndex;
			if (ClosedSet.Contains(SuccIdx))
			{
				continue;
			}

			FAstarPathNode* ParentNode = CurrentNode;

			const float NodeRadius = Pool[SuccIdx].MaxAllowedRadius;
			if (NodeRadius < AgentRadius)
			{
				continue;
			}


			const int64 MoveCost = GetMoveCost(ParentNode->WorldPosition, Successor->WorldPosition);
			const int64 NewGCost = ParentNode->GCost + MoveCost;

			if (FAstarPathNode** Existing = OpenSetLookup.Find(SuccIdx))
			{
				FAstarPathNode* ExistNode = *Existing;
				if (NewGCost < ExistNode->GCost)
				{
					ExistNode->SetParent(ParentNode, NewGCost, ExistNode->HCost);
					int32 HeapIndex = OpenSet.Find(ExistNode);
					if (ensure(HeapIndex != INDEX_NONE))
					{
						AlgoImpl::HeapSiftUp(
							OpenSet.GetData(),
							0,
							HeapIndex,
							FIdentityFunctor(),
							Predicate
						);
					}

					OpenSet.Heapify(Predicate);
				}
			}
			else
			{
				Successor->SetParent(ParentNode, NewGCost, CalculateHeuristic(Successor->WorldPosition, GoalCenter));
				OpenSet.HeapPush(Successor, Predicate);
				OpenSetLookup.Add(Successor->NodeIndex, Successor);
			}
		}
	}

	if (BestPartialNode)
	{
		ReconstructAndSmoothPath(BestPartialNode, OutPath);
		return EPathfindingResult::Success_Partial;
	}

	return EPathfindingResult::Failed_Unreachable;
}

bool FAstarPathfinder::HasFastLineOfSight(const FVector& Start, const FVector& End) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(ThetaStar_HasLineOfSightWithRadius_Hierarchical);

	TArray<int32> CenterlineNodes;
	return OctreeData.VoxelRayMarch(Start, End, CenterlineNodes);
}

void FAstarPathfinder::ReconstructAndSmoothPath(FAstarPathNode* EndNode, TArray<FVector>& OutPath)
{
	TArray<FVector> RawPath;
	FAstarPathNode* CurrentNode = EndNode;

	while (CurrentNode != nullptr)
	{
		RawPath.Add(CurrentNode->WorldPosition);
		CurrentNode = CurrentNode->Parent;
	}
	Algo::Reverse(RawPath);

	if (RawPath.Num() <= 1)
	{
		OutPath.Empty();
		if (RawPath.Num() > 0) OutPath.Add(ActualStartLocation);
		OutPath.Add(ActualGoalLocation);
		return;
	}

	RawPath[0] = ActualStartLocation;
	if (HasFastLineOfSight(RawPath.Last(), ActualGoalLocation))
	{
		RawPath.Last() = ActualGoalLocation;
	}

	OutPath.Empty();
	OutPath.Add(RawPath[0]);

	int32 CurrentPointIndex = 0;
	while (CurrentPointIndex < RawPath.Num() - 1)
	{
		int32 LookAheadLow = CurrentPointIndex + 1;
		int32 LookAheadHigh = RawPath.Num() - 1;
		int32 BestLookAheadIndex = LookAheadLow;

		while (LookAheadLow <= LookAheadHigh)
		{
			const int32 Mid = LookAheadLow + (LookAheadHigh - LookAheadLow) / 2;

			if (HasFastLineOfSight(OutPath.Last(), RawPath[Mid]))
			{
				BestLookAheadIndex = Mid;
				LookAheadLow = Mid + 1;
			}
			else
			{
				LookAheadHigh = Mid - 1;
			}
		}

		OutPath.Add(RawPath[BestLookAheadIndex]);

		if (BestLookAheadIndex == RawPath.Num() - 1)
		{
			break;
		}

		CurrentPointIndex = BestLookAheadIndex;
	}
}

bool FAstarPathfinder::HasGuaranteedLineOfSight(const FVector& Start, const FVector& End) const
{
	return !OctreeData.CheckForSolidNodesInCapsule(Start, End, AgentRadius);
}

void FAstarPathfinder::IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_IdentifySuccessors_Internal);
	OutSuccessors.Empty();

	TArray<int32> AdjacentNodeIndices;
	OctreeData.FindAdjacentEmptyLeaves(Node->NodeIndex, AdjacentNodeIndices);

	for (const int32 NeighborIndex : AdjacentNodeIndices)
	{
		const auto& NeighborNode = OctreeData.StaticNodePool[NeighborIndex];
		const FBox NeighborBounds = NeighborNode.Bounds;
		const FVector Center = NeighborBounds.GetCenter();


		// 1) If the cell itself is already big enough, we have instant clearance.
		if (NeighborNode.MaxAllowedRadius < AgentRadius)
		{
			continue;
		}

		// 3) Finally, also make sure it’s not dynamically blocked
		if (!OctreeData.IsRegionDynamicallyBlocked(NeighborBounds))
		{
			FAstarPathNode* Succ = MemoryPool.NewNode();
			Succ->NodeIndex = NeighborIndex;
			Succ->WorldPosition = Center;
			OutSuccessors.Add(Succ);
		}
	}
}

int64 FAstarPathfinder::CalculateHeuristic(const FVector& A, const FVector& B) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_CalculateHeuristic);
	return static_cast<int64>(FVector::DistSquared(A, B));
}

int64 FAstarPathfinder::GetMoveCost(const FVector& A, const FVector& B) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_GetMoveCost);
	return static_cast<int64>(FVector::DistSquared(A, B));
}

void FAstarPathfinder::ReconstructPath(FAstarPathNode* EndNode, TArray<FVector>& OutPath) const
{
	OutPath.Empty();
	FAstarPathNode* CurrentNode = EndNode;
	while (CurrentNode != nullptr)
	{
		OutPath.Add(CurrentNode->WorldPosition);
		CurrentNode = CurrentNode->Parent;
	}
	Algo::Reverse(OutPath);
}
