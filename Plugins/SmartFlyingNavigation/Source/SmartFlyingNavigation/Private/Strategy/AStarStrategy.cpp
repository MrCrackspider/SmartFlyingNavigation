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
	const TArray<FOctreeNode>& PoolRef = *(OctreeData.StaticNodePoolPtr);

	int32 StartNodeIndex = OctreeData.FindLeafNodeAt(InRequest.StartLocation);

	bool bIsStartNodeInvalid = (StartNodeIndex == -1) || (PoolRef[StartNodeIndex].State != EOctreeNodeState::Empty);


	if (bIsStartNodeInvalid)
	{
		// If invalid, search for the nearest valid leaf node for the agent.
		UE_LOG(LogTemp, Warning, TEXT("AStarStrategy: Start location is inside a blocked voxel. Searching for a nearby valid start node."));
		const int32 NewStartNodeIndex = OctreeData.FindNearestValidLeafForAgent(
			InRequest.StartLocation,
			InRequest.AgentRadius,
			NearbyGoalSearchRadius,
			NearbyGoalSearchIterations);

		if (NewStartNodeIndex == -1)
		{
			// If even the nearby search fails, then pathfinding is truly impossible from this area.
			UE_LOG(LogTemp, Error, TEXT("AStarStrategy: Could not find any valid start node near the agent. Pathfinding failed."));
			return false;
		}
		
		// We found a valid nearby node, use it as our starting point.
		StartNodeIndex = NewStartNodeIndex;
	}

	

	
	int32 GoalNodeIndex = OctreeData.FindLeafNodeAt(InRequest.EndLocation);

	bool bIsGoalNodeInvalid = (GoalNodeIndex == -1) ||
		(PoolRef[GoalNodeIndex].State != EOctreeNodeState::Empty);


	if (bIsGoalNodeInvalid)
	{
		UE_LOG(LogTemp, Warning, TEXT("AStarStrategy: Goal location is inside a blocked voxel. A* will attempt to find the closest possible path."));
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
	StartNode->WorldPosition = ActualStartLocation;
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
				const float HeuristicWeight = 2.f; // 1.0보다 큰 값 (예: 1.5 ~ 2.0)
				Successor->SetParent(ParentNode, NewGCost, CalculateBoundHeuristic(SuccIdx) * HeuristicWeight);
				OpenSet.HeapPush(Successor, Predicate);
				OpenSetLookup.Add(Successor->NodeIndex, Successor);
				
				
			}
		}
	}
	// const bool bSolid = OctreeData.CheckForSolidNodesInCapsule(BestPartialNode->WorldPosition, ActualGoalLocation, AgentRadius);
	// UE_LOG(LogTemp, Error, TEXT("Partial Path - Anchor=(%.2f,%.2f,%.2f) Goal=(%.2f,%.2f,%.2f) CapsuleBlocked=%d"),
	// 	BestPartialNode->WorldPosition.X, BestPartialNode->WorldPosition.Y, BestPartialNode->WorldPosition.Z, 
	// 	ActualGoalLocation.X, ActualGoalLocation.Y, ActualGoalLocation.Z, 
	// 	bSolid ? 1 : 0
	// );

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
		if (HasClearPath(ActualStartLocation, ActualGoalLocation))
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
	if (HasClearPath(ActualStartLocation, RawPath[1]))
	{
		RawPath[0] = ActualStartLocation;
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
		return false; // 고정 장애물과 충돌
	}

	// 2. 동적 장애물 검사는 그대로 수행
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
			
			FVector PortalCenter;
			const FBox& CurrentBounds = CurrentNode.Bounds;
			const FBox& NeighborBounds = NeighborNode.Bounds;

			const FBox PortalBounds = CurrentBounds.Overlap(NeighborBounds);
			PortalCenter = PortalBounds.GetCenter();

			if (HasClearPath(Node->WorldPosition, PortalCenter))
			{
				FAstarPathNode* Successor = MemoryPool.NewNode();
				Successor->NodeIndex = NeighborNodeIndex;
				
				Successor->WorldPosition = PortalCenter;
				
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
		if(HasClearPath(PartialPos, Candidate) && HasClearPath(Candidate, ActualGoalLocation))
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
