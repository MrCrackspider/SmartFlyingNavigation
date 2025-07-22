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

	if (StartNodeIndex == -1)
	{
		UE_LOG(LogTemp, Warning, TEXT("A* Pathfinding failed: Start node is invalid."));
		return false;;
	}

	int32 GoalNodeIndex = OctreeData.FindLeafNodeAt(InRequest.EndLocation);

	const TArray<FOctreeNode>& PoolRef = *(OctreeData.StaticNodePoolPtr);
	bool bIsGoalNodeInvalid = (GoalNodeIndex == -1) ||
		(PoolRef[GoalNodeIndex].State != EOctreeNodeState::Empty) ||
		(PoolRef[GoalNodeIndex].MaxAllowedRadius < InRequest.AgentRadius);


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

	const TPathNodePtrCompare<FAstarPathNode> Predicate;

	TArray<FAstarPathNode*> Successors;
	Successors.Reserve(8);

	FAstarPathNode* StartNode = MemoryPool.NewNode();
	if (!StartNode) return EPathfindingResult::Failed_OutOfMemory;

	const TArray<FOctreeNode>& PoolRef = *(OctreeData.StaticNodePoolPtr);

	StartNode->NodeIndex = StartNodeIndex;
	StartNode->WorldPosition = PoolRef[StartNodeIndex].Bounds.GetCenter();
	StartNode->HCost = CalculateBoundHeuristic(StartNode->NodeIndex);
	StartNode->CalculateAndSetFCost();

	OpenSet.HeapPush(StartNode, Predicate);
	OpenSetLookup.Add(StartNodeIndex, StartNode);

	FAstarPathNode* BestPartialNode = StartNode;

	while (!OpenSet.IsEmpty())
	{
		FAstarPathNode* CurrentNode;
		OpenSet.HeapPop(CurrentNode, Predicate);

		// --- LAZY THETA* CORE LOGIC ---
		// This is the moment to check for a shortcut from the grandparent.
		if (CurrentNode->Parent != nullptr && CurrentNode->Parent->Parent != nullptr)
		{
			FAstarPathNode* GrandparentNode = CurrentNode->Parent->Parent;
			if (HasGuaranteedLineOfSight(GrandparentNode->WorldPosition, CurrentNode->WorldPosition))
			{
				// A shortcut is possible. Update the node's parent and its G-cost.
				// This is the only line needed to change the path's structure.
				const int64 NewGCost = GrandparentNode->GCost + GetMoveCost(
					GrandparentNode->WorldPosition, CurrentNode->WorldPosition);
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
				Successor->SetParent(ParentNode, NewGCost, CalculateBoundHeuristic(SuccIdx));
				OpenSet.HeapPush(Successor, Predicate);
				OpenSetLookup.Add(Successor->NodeIndex, Successor);
			}
		}
	}

	if (BestPartialNode)
	{
		if (TryToRepairPartialPath(BestPartialNode, OutPath))
		{
			return EPathfindingResult::Success; // Repair was successful!
		}
		ReconstructAndSmoothPath(BestPartialNode, OutPath);
		return EPathfindingResult::Success_Partial;
	}

	return EPathfindingResult::Failed_Unreachable;
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

	// --- Step 2: Handle trivial cases (path is too short) ---
	if (RawPath.Num() < 2)
	{
		OutPath.Empty();
		OutPath.Add(ActualStartLocation);
		if (HasGuaranteedLineOfSight(ActualStartLocation, ActualGoalLocation))
		{
			OutPath.Add(ActualGoalLocation);
		}
		else if (RawPath.Num() > 0)
		{
			// If no direct path to goal, use the first safe node from A*.
			OutPath.Add(RawPath[0]);
		}
		return;
	}

	// --- Step 3: Safely tether the actual start and end locations to the raw path. ---
	if (HasGuaranteedLineOfSight(ActualStartLocation, RawPath[1]))
	{
		RawPath[0] = ActualStartLocation;
	}

	// 2. Connect the End Point:
	if (HasGuaranteedLineOfSight(RawPath.Last(), ActualGoalLocation))
	{
		RawPath.Add(ActualGoalLocation);
	}

	// --- Step 4: Perform the "String Pulling" algorithm ---
	// Now, perform the smoothing on the safely-modified path.
	OutPath.Empty();
	if (RawPath.Num() > 0)
	{
		OutPath.Add(RawPath[0]);
	}

	int32 AnchorIndex = 0;
	while (AnchorIndex < RawPath.Num() - 1)
	{
		int32 FurthestVisibleIndex = AnchorIndex + 1;

		for (int32 LookAheadIndex = AnchorIndex + 2; LookAheadIndex < RawPath.Num(); ++LookAheadIndex)
		{
			if (HasGuaranteedLineOfSight(RawPath[AnchorIndex], RawPath[LookAheadIndex]))
			{
				FurthestVisibleIndex = LookAheadIndex;
			}
			else
			{
				break;
			}
		}
		OutPath.Add(RawPath[FurthestVisibleIndex]);
		AnchorIndex = FurthestVisibleIndex;
	}
}

bool FAstarPathfinder::HasGuaranteedLineOfSight(const FVector& Start, const FVector& End) const
{
	return !OctreeData.CheckForSolidNodesInCapsule(Start, End, AgentRadius * 0.98f);
}

void FAstarPathfinder::IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_IdentifySuccessors_Internal);
	OutSuccessors.Empty();

	TArray<int32> AdjacentNodeIndices;
	OctreeData.FindAdjacentEmptyLeaves(Node->NodeIndex, AdjacentNodeIndices);
	const TArray<FOctreeNode>& PoolRef = *(OctreeData.StaticNodePoolPtr);
	for (const int32 NeighborIndex : AdjacentNodeIndices)
	{
		const auto& NeighborNode = PoolRef[NeighborIndex];
		const FBox NeighborBounds = NeighborNode.Bounds;
		FVector NeighborCenter = NeighborBounds.GetCenter();
		
		if (HasGuaranteedLineOfSight(Node->WorldPosition, NeighborCenter))
		{
			FBox NeighborQueryBox = AStarStrategyUtils::MakeAgentQueryBox(NeighborCenter, AgentRadius);
			if (!OctreeData.IsRegionDynamicallyBlocked(NeighborQueryBox))
			{
				FAstarPathNode* Successor = MemoryPool.NewNode();
				Successor->NodeIndex = NeighborIndex;
				Successor->WorldPosition = NeighborCenter;
				OutSuccessors.Add(Successor);
			}
		}
	}
}

int64 FAstarPathfinder::GetMoveCost(const FVector& A, const FVector& B) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_GetMoveCost);
	return static_cast<int64>(FVector::DistSquared(A, B));
}

int64 FAstarPathfinder::CalculateBoundHeuristic(int32 NodeIndex) const
{
	const TArray<FOctreeNode>& PoolRef = *(OctreeData.StaticNodePoolPtr);
	const FBox& Bounds = PoolRef[NodeIndex].Bounds;
	const FVector ClosestPoint = Bounds.GetClosestPointTo(ActualGoalLocation);
	const float RawDist = (ClosestPoint - ActualGoalLocation).Size();
	const float EffectiveDist = FMath::Max(0.0f, RawDist - AgentRadius);
	return static_cast<int64>(EffectiveDist * EffectiveDist);
}

bool FAstarPathfinder::TryToRepairPartialPath(FAstarPathNode* PartialEndNode, TArray<FVector>& OutPath)
{
	FVector PartialPos = PartialEndNode->WorldPosition;
	float Radius = AgentRadius * 2;
	FVector BestPoint;

	for (int32 i = 0; i < 16; ++i)
	{
		FVector Dir = FMath::VRand();
		FVector Candidate = PartialPos + Dir * FMath::FRandRange(0, Radius);
		if (!OctreeData.CheckForSolidNodesInCapsule(Candidate, Candidate, AgentRadius)
			&& HasGuaranteedLineOfSight(PartialPos, Candidate)
			&& HasGuaranteedLineOfSight(Candidate, ActualGoalLocation))
		{
			BestPoint = Candidate;
			OutPath.Empty();
			ReconstructAndSmoothPath(PartialEndNode, OutPath);
			OutPath.Add(BestPoint);
			OutPath.Add(ActualGoalLocation);
			return true;
		}
	}
	return false;
}
