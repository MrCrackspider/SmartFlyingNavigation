// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Strategy/AStarPathfinder.h"
#include "Algo/Reverse.h"

FAstarPathfinder::FAstarPathfinder(const FOctreePathfindingData& InOctreeData, int32 InStartIndex, int32 InEndIndex,
                                   const FVector& InOriginalStartLocation, const FVector& InEffectiveStartLocation,
                                   const FVector& InEndLocation, float InAgentRadius,
                                   TNodeMemoryPool<FAstarPathNode>& InMemoryPool,
                                   float InClearancePenaltyWeight, float InPreferredClearanceMultiplier)
	: OctreeData(InOctreeData)
	  , StartNodeIndex(InStartIndex)
	  , GoalNodeIndex(InEndIndex)
	  , AgentRadius(InAgentRadius)
	  , OriginalStartLocation(InOriginalStartLocation)
	  , EffectiveStartLocation(InEffectiveStartLocation)
	  , ActualGoalLocation(InEndLocation)
	  , MemoryPool(InMemoryPool)
	  , ClearancePenaltyWeight(InClearancePenaltyWeight)
	  , PreferredClearanceMultiplier(InPreferredClearanceMultiplier)
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

	StartNode->NodeIndex = StartNodeIndex;
	StartNode->WorldPosition = EffectiveStartLocation;
	StartNode->HCost = CalculateBoundHeuristic(StartNode->NodeIndex);
	StartNode->CalculateAndSetFCost();

	OpenSet.HeapPush(StartNode, Predicate);
	OpenSetLookup.Add(StartNodeIndex, StartNode);

	FAstarPathNode* BestPartialNode = StartNode;

	// Safety guard: prevent runaway searches on pathologically large/malformed graphs.
	constexpr int32 MaxIterations = 100000;
	int32 IterationCount = 0;

	while (!OpenSet.IsEmpty())
	{
		if (++IterationCount > MaxIterations)
		{
			UE_LOG(LogTemp, Warning, TEXT("A* exceeded %d iterations. Returning best partial path."), MaxIterations);
			break;
		}

		FAstarPathNode* CurrentNode;
		OpenSet.HeapPop(CurrentNode, Predicate);

		// Skip stale heap entries from lazy deletion.
		// A node is stale if it's already closed or if the lookup points to a different (newer) node.
		if (ClosedSet.Contains(CurrentNode->NodeIndex))
		{
			continue;
		}
		{
			FAstarPathNode** LookupNode = OpenSetLookup.Find(CurrentNode->NodeIndex);
			if (LookupNode && *LookupNode != CurrentNode)
			{
				continue; // This is a superseded entry - a better path was found.
			}
		}

		// --- LAZY THETA* CORE LOGIC ---
		// This is the moment to check for a shortcut from the grandparent.
		if (CurrentNode->Parent != nullptr && CurrentNode->Parent->Parent != nullptr)
		{
			FAstarPathNode* GrandparentNode = CurrentNode->Parent->Parent;
			if (HasClearPath(GrandparentNode->WorldPosition, CurrentNode->WorldPosition))
			{
				// A shortcut is possible. Update the node's parent and its G-cost.
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
					// Lazy deletion: create a new node with the better cost and push it.
					// The old entry stays in the heap but becomes "stale" - it will be
					// skipped when popped because the lookup won't match it.
					FAstarPathNode* BetterNode = MemoryPool.NewNode();
					if (BetterNode)
					{
						BetterNode->NodeIndex = SuccIdx;
						BetterNode->WorldPosition = ExistNode->WorldPosition;
						BetterNode->SetParent(ParentNode, NewGCost, ExistNode->HCost);
						OpenSet.HeapPush(BetterNode, Predicate);
						OpenSetLookup[SuccIdx] = BetterNode; // Redirect lookup to new node
					}
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

	OutPath.Empty();
	OutPath.Add(OriginalStartLocation);

	// --- Step 2: Handle trivial cases (path is too short) ---
	if (RawPath.Num() < 2)
	{
		OutPath.Empty();
		OutPath.Add(OriginalStartLocation);
		if (HasClearPath(OriginalStartLocation, ActualGoalLocation))
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
	if (HasClearPath(OriginalStartLocation, RawPath[1]))
	{
		RawPath[0] = OriginalStartLocation;
	}

	// 2. Connect the End Point:
	if (HasClearPath(RawPath.Last(), ActualGoalLocation))
	{
		RawPath.Add(ActualGoalLocation);
	}

	// --- Step 4: Perform the "String Pulling" algorithm ---
	OutPath.Empty();
	if (RawPath.Num() > 0)
	{
		OutPath.Add(RawPath[0]);
	}

	int32 AnchorIndex = 0;
	while (AnchorIndex < RawPath.Num() - 1)
	{
		int32 FurthestVisibleIndex = AnchorIndex + 1;
		int32 Low = AnchorIndex + 2;
		int32 High = RawPath.Num() - 1;

		while (Low <= High)
		{
			const int32 Mid = Low + (High - Low) / 2;
			if (HasClearPath(RawPath[AnchorIndex], RawPath[Mid]))
			{
				FurthestVisibleIndex = Mid;
				Low = Mid + 1;
			}
			else
			{
				High = Mid - 1;
			}
		}
		OutPath.Add(RawPath[FurthestVisibleIndex]);
		AnchorIndex = FurthestVisibleIndex;
	}
}

bool FAstarPathfinder::HasClearPath(const FVector& Start, const FVector& End) const
{
	return !OctreeData.IsSegmentBlockedForAgent(Start, End, AgentRadius);
}

void FAstarPathfinder::IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_IdentifySuccessors_PortalCentric);
	OutSuccessors.Empty();

	const TArray<FOctreeNode>& PoolRef = *(OctreeData.StaticNodePoolPtr);
	if (!PoolRef.IsValidIndex(Node->NodeIndex))
	{
		return;
	}

	const FOctreeNode& CurrentNode = PoolRef[Node->NodeIndex];

	for (int32 DirIndex = 0; DirIndex < static_cast<int32>(ENeighborDirection::MAX); ++DirIndex)
	{
		const int32 NeighborNodeIndex = CurrentNode.Neighbors[DirIndex];

		if (PoolRef.IsValidIndex(NeighborNodeIndex) && PoolRef[NeighborNodeIndex].State == EOctreeNodeState::Empty)
		{
			const FOctreeNode& NeighborNode = PoolRef[NeighborNodeIndex];

			// Cheap clearance gate: the baked per-leaf clearance must admit the agent.
			if (NeighborNode.MaxAllowedRadius > 0.f && NeighborNode.MaxAllowedRadius < AgentRadius)
			{
				continue;
			}

			const FBox& NeighborBounds = NeighborNode.Bounds;
			FVector NeighborCenter = NeighborBounds.GetCenter();

			if (HasClearPath(Node->WorldPosition, NeighborCenter))
			{
				FAstarPathNode* Successor = MemoryPool.NewNode();
				Successor->NodeIndex = NeighborNodeIndex;

				Successor->WorldPosition = NeighborCenter;

				OutSuccessors.Add(Successor);
			}
		}
	}
}

int64 FAstarPathfinder::GetMoveCost(const FVector& A, const FVector& B) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_GetMoveCost);

	float Cost = FVector::Distance(A, B);

	// Clearance shaping: prefer routes with breathing room around obstacles.
	// Multiplier >= 1 keeps the heuristic admissible.
	if (ClearancePenaltyWeight > 0.f && OctreeData.ShouldUseDistanceField())
	{
		const float Clearance = OctreeData.DistanceFieldPtr->SampleClearance(B);
		const float PreferredClearance = FMath::Max(AgentRadius * PreferredClearanceMultiplier, KINDA_SMALL_NUMBER);
		const float Penalty = FMath::Clamp(1.f - Clearance / PreferredClearance, 0.f, 1.f);
		Cost *= 1.f + ClearancePenaltyWeight * Penalty;
	}

	return static_cast<int64>(Cost * 100.0f);
}

int64 FAstarPathfinder::CalculateBoundHeuristic(int32 NodeIndex) const
{
	const TArray<FOctreeNode>& PoolRef = *(OctreeData.StaticNodePoolPtr);
	const FBox& Bounds = PoolRef[NodeIndex].Bounds;
	const FVector NodeCenter = Bounds.GetCenter();
	const float RawDist = (NodeCenter - ActualGoalLocation).Size();
	const float EffectiveDist = FMath::Max(0.0f, RawDist - AgentRadius);
	return static_cast<int64>(EffectiveDist * 100.0f);
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
		if (HasClearPath(PartialPos, Candidate) && HasClearPath(Candidate, ActualGoalLocation))
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
