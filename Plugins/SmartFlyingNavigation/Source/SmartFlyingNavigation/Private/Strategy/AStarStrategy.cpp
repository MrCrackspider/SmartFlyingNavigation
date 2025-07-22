// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Strategy/AStarStrategy.h"
#include "Utility/AStarStrategyUtils.h"

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


	if (bIsGoalNodeInvalid)
	{
		const int32 NewGoalNodeIndex = OctreeData.FindNearestValidLeafForAgent(
			InRequest.EndLocation,
			InRequest.AgentRadius,
			NearbyGoalSearchRadius,
			NearbyGoalSearchIterations);

		if (NewGoalNodeIndex == -1)
		{
			return false;
		}

		GoalNodeIndex = NewGoalNodeIndex;
	}
	
	FBox GoalQueryBox = AStarStrategyUtils::MakeAgentQueryBox(InRequest.EndLocation, InRequest.AgentRadius);
	if (OctreeData.IsRegionDynamicallyBlocked(GoalQueryBox))
	{
		return false;
	}

	FPathNodeMemoryPoolBase* BasePool = InRequest.MemoryPool;
	if (!BasePool)
	{
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
	  , AgentRadius(InAgentRadius)
	  , ActualStartLocation(InStartLocation)
	  , ActualGoalLocation(InEndLocation)
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




		// --- LAZY THETA* CORE LOGIC (CLEANED UP) ---
		// This is the moment to check for a shortcut from the grandparent.
		if (CurrentNode->Parent != nullptr && CurrentNode->Parent->Parent != nullptr)
		{
			FAstarPathNode* GrandparentNode = CurrentNode->Parent->Parent;
			if (HasGuaranteedLineOfSight(GrandparentNode->WorldPosition, CurrentNode->WorldPosition))
			{
				// A shortcut is possible. Update the node's parent and its G-cost.
				// This is the only line needed to change the path's structure.
				const int64 NewGCost = GrandparentNode->GCost + GetMoveCost(GrandparentNode->WorldPosition, CurrentNode->WorldPosition);
				CurrentNode->SetParent(GrandparentNode, NewGCost, CurrentNode->HCost);
			}
		}
		// --- END OF LAZY THETA* LOGIC ---
		OpenSetLookup.Remove(CurrentNode->NodeIndex);
		ClosedSet.Add(CurrentNode->NodeIndex, CurrentNode);

		if (CurrentNode->HCost < BestPartialNode->HCost)
		{
			BestPartialNode = CurrentNode;
		}

		if (CurrentNode->NodeIndex == GoalNodeIndex)
		{
		 	ReconstructPath(CurrentNode, OutPath);
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

bool FAstarPathfinder::HasSimpleLineOfSight(const FVector& Start, const FVector& End) const
{
	TArray<int32> IgnoredNodes; // We don't need the list of nodes here.
	return OctreeData.VoxelRayMarch(Start, End, IgnoredNodes);
}

void FAstarPathfinder::ReconstructAndSmoothPath(FAstarPathNode* EndNode, TArray<FVector>& OutPath)
{
	// --- Step 1: Reconstruct the initial path from A* nodes (reversed) ---
	TArray<FVector> RawPath;
	FAstarPathNode* CurrentNode = EndNode;
	while (CurrentNode != nullptr)
	{
		RawPath.Add(CurrentNode->WorldPosition);
		CurrentNode = CurrentNode->Parent;
	}
	Algo::Reverse(RawPath);

	// --- Step 2: Handle trivial cases ---
	if (RawPath.Num() < 2)
	{
		OutPath = RawPath;
		// Ensure actual start/goal are used if possible
		if (OutPath.Num() > 0) OutPath[0] = ActualStartLocation;
		OutPath.Add(ActualGoalLocation);
		return;
	}

	// --- Step 3: Anchor the true start and end locations ---
	// This ensures the smoothing process works from the agent's exact position to the goal's exact position.
	RawPath[0] = ActualStartLocation;
	RawPath.Last() = ActualGoalLocation;

	// --- Step 4: Perform the "String Pulling" algorithm ---
	OutPath.Empty();
	OutPath.Add(RawPath[0]); // Start the final path with the first point.

	int32 AnchorIndex = 0;
	while (AnchorIndex < RawPath.Num() - 1)
	{
		int32 FurthestVisibleIndex = AnchorIndex + 1;

		// Find the furthest point in RawPath that is visible from the current anchor point.
		// We iterate forward from the next point.
		for (int32 LookAheadIndex = AnchorIndex + 2; LookAheadIndex < RawPath.Num(); ++LookAheadIndex)
		{
			// Check line-of-sight from the anchor to the look-ahead point.
			if (HasGuaranteedLineOfSight(RawPath[AnchorIndex], RawPath[LookAheadIndex]))
			{
				// If we can see this point, it's our new best candidate.
				FurthestVisibleIndex = LookAheadIndex;
			}
			else
			{
				// As soon as we can't see a point, we stop looking further.
				// The previous point was the last one in our line of sight.
				break;
			}
		}

		// Add the furthest visible point to our final path.
		OutPath.Add(RawPath[FurthestVisibleIndex]);

		// The point we just added becomes the new anchor for the next iteration.
		AnchorIndex = FurthestVisibleIndex;
	}
}
bool FAstarPathfinder::HasGuaranteedLineOfSight(const FVector& Start, const FVector& End) const
{
	return OctreeData.IsPathClearWithVoxelSweep(Start, End, AgentRadius);
}

void FAstarPathfinder::IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_IdentifySuccessors_Internal);
	OutSuccessors.Empty();

	TArray<int32> AdjacentNodeIndices;
	OctreeData.FindAdjacentEmptyLeaves(Node->NodeIndex, AdjacentNodeIndices);
	const FVector CurrentNodeCenter = Node->WorldPosition;

	for (const int32 NeighborIndex : AdjacentNodeIndices)
	{
		const auto& NeighborNode = OctreeData.StaticNodePool[NeighborIndex];
		const FBox NeighborBounds = NeighborNode.Bounds;
		FVector NeighborCenter = NeighborBounds.GetCenter();

		// 1) If the cell itself is already big enough, we have instant clearance.
		if (!HasFastLineOfSight(CurrentNodeCenter, NeighborCenter))
		{
			continue;
		}
		
		FBox NeighborQueryBox = AStarStrategyUtils::MakeAgentQueryBox(NeighborCenter,AgentRadius);
		
		if (!OctreeData.IsRegionDynamicallyBlocked(NeighborQueryBox))
		{
			FAstarPathNode* Successor = MemoryPool.NewNode();
			Successor->NodeIndex      = NeighborIndex;
			Successor->WorldPosition  = NeighborCenter;
			OutSuccessors.Add(Successor);
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
