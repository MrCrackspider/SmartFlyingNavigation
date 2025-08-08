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
	if (!OctreeData.StaticNodePoolPtr)
	{
		UE_LOG(LogTemp, Error, TEXT("AStarStrategy: StaticNodePoolPtr is null. Pathfinding aborted."));
		return false;
	}
	const TArray<FOctreeNode>& PoolRef = *OctreeData.StaticNodePoolPtr;

	int32 StartNodeIndex = -1;
	FVector EffectiveStartLocation = InRequest.StartLocation;
	
	const bool bIsTrulyObstructed = 
		OctreeData.CheckForSolidNodesInCapsule(InRequest.StartLocation, InRequest.StartLocation, InRequest.AgentRadius) ||
		OctreeData.IsPathDynamicallyBlocked_Sweep(InRequest.StartLocation, InRequest.StartLocation, InRequest.AgentRadius);

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
			UE_LOG(LogTemp, Log, TEXT("AStarStrategy: Start is safe but on a boundary. Finding nearest reference node (start location will NOT be moved)."));
			
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
                                   const FVector& InOriginalStartLocation, const FVector& InEffectiveStartLocation,
                                   const FVector& InEndLocation, float InAgentRadius,
                                   TNodeMemoryPool<FAstarPathNode>& InMemoryPool)
	: OctreeData(InOctreeData)
	  , StartNodeIndex(InStartIndex)
	  , GoalNodeIndex(InEndIndex)
	  , AgentRadius(InAgentRadius)
	  , OriginalStartLocation(InOriginalStartLocation)
	  , EffectiveStartLocation(InEffectiveStartLocation)
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

	StartNode->NodeIndex = StartNodeIndex;
	StartNode->WorldPosition = EffectiveStartLocation;
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
			if (HasClearPath(GrandparentNode->WorldPosition, CurrentNode->WorldPosition))
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
	if (OctreeData.CheckForSolidNodesInCapsule(Start, End, AgentRadius))
	{
		return false;
	}

	if (OctreeData.IsPathDynamicallyBlocked_Sweep(Start, End, AgentRadius))
	{
		return false;
	}

	return true;
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
	return static_cast<int64>(FVector::Distance(A, B) * 100.0f);
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
