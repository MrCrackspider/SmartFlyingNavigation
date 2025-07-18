// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/Strategy/AStarStrategy.h"
#include "Pathfinder/Utility/AStarStrategyUtils.h"

#include "Pathfinder/Utility/StrategyUtils.h"

void UAStarStrategy::Initialize()
{
}

bool UAStarStrategy::FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_FindPathWithStrategy); // Insights에서 이 이름으로 표시됩니다.

	// 1. 요청에서 옥트리 데이터 스냅샷을 가져옵니다.
	const FOctreePathfindingData& OctreeData = InRequest.NavigationData.Get<FOctreePathfindingData>();

	// 2. 시작점과 끝점이 유효한 'Empty' 리프 노드에 있는지 확인합니다.
	const int32 StartNodeIndex = OctreeData.FindLeafNodeAt(InRequest.StartLocation);
	int32 GoalNodeIndex = OctreeData.FindLeafNodeAt(InRequest.EndLocation);

	bool bIsGoalNodeInvalid = (GoalNodeIndex == -1) ||
		(OctreeData.StaticNodePool[GoalNodeIndex].State != EOctreeNodeState::Empty) ||
		(OctreeData.StaticNodePool[GoalNodeIndex].MaxAllowedRadius < InRequest.AgentRadius);

	if (StartNodeIndex == -1 || OctreeData.StaticNodePool[StartNodeIndex].State != EOctreeNodeState::Empty)
	{
		UE_LOG(LogTemp, Warning, TEXT("A* (Octree) Failed: Start location is invalid or inside a solid node."));
		return false;
	}

	if (bIsGoalNodeInvalid)
	{
		// 목표 지점이 유효하지 않으면, 에이전트 크기를 고려하여 "진짜 유효한" 주변 노드를 찾습니다.
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

	// 3. '일꾼' 객체를 생성하고 작업을 위임합니다.
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
// FAstarPathfinder: 전문가(일꾼) 역할
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
			if (CurrentNode->Parent && HasLineOfSightWithRadius(CurrentNode->Parent->WorldPosition,
			                                                    Successor->WorldPosition))
			{
				ParentNode = CurrentNode->Parent;
			}

			const float NodeRadius = Pool[SuccIdx].MaxAllowedRadius;
			if (NodeRadius < AgentRadius)
			{
				continue; // 좁아서 통과 불가 -> 후보에서 완전 제거
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

bool FAstarPathfinder::HasLineOfSightWithRadius(const FVector& Start, const FVector& End) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(ThetaStar_HasLineOfSightWithRadius_Hierarchical);

	// --- 1단계: 초고속 센터라인 검사 ---
	TArray<int32> CenterlineNodes;
	if (!OctreeData.VoxelRayMarch(Start, End, CenterlineNodes))
	{
		return false;
	}

	// --- 2단계: 안전 여유 공간(Clearance) 검사 ---
	for (const int32 NodeIndex : CenterlineNodes)
	{
		const float NodeMinHalfExtent = OctreeData.StaticNodePool[NodeIndex].Bounds.GetExtent().GetMin();
		if (NodeMinHalfExtent < AgentRadius)
		{
			return !OctreeData.CheckForSolidNodesInCapsule(Start, End, AgentRadius);
		}
	}
	return true;
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
	if (HasLineOfSightWithRadius(RawPath.Last(), ActualGoalLocation))
	{
		RawPath.Last() = ActualGoalLocation;
	}

	OutPath.Empty();
	OutPath.Add(RawPath[0]);

	int32 CurrentPointIndex = 0;
	while (CurrentPointIndex < RawPath.Num() - 1)
	{
		// --- 이진 탐색 로직 시작 ---
		int32 LookAheadLow = CurrentPointIndex + 1;
		int32 LookAheadHigh = RawPath.Num() - 1;
		int32 BestLookAheadIndex = LookAheadLow;

		while (LookAheadLow <= LookAheadHigh)
		{
			const int32 Mid = LookAheadLow + (LookAheadHigh - LookAheadLow) / 2;

			// 현재 지점(OutPath.Last())에서 중간 지점(Mid)까지 시야가 확보되는지 확인
			if (HasLineOfSightWithRadius(OutPath.Last(), RawPath[Mid]))
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

		// 만약 찾은 점이 최종 목적지였다면 루프를 완전히 종료
		if (BestLookAheadIndex == RawPath.Num() - 1)
		{
			break;
		}

		// 현재 위치를 방금 추가한 웨이포인트로 점프
		CurrentPointIndex = BestLookAheadIndex;
	}
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
