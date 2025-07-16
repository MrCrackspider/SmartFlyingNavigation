// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/Strategy/AStarStrategy.h"
#include "Pathfinder/Utility/AStarStrategyUtils.h"

#include "Pathfinder/Utility/StrategyUtils.h"

void UAStarStrategy::Initialize()
{
	if (AllDirections.Num() == 0)
	{
		for (int32 x = -1; x <= 1; ++x)
			for (int32 y = -1; y <= 1; ++y)
				for (int32 z = -1; z <= 1; ++z)
					if (x != 0 || y != 0 || z != 0)
						AllDirections.Emplace(x, y, z);
	}

	DirectionToIndexMap.Empty();
	for (int32 i = 0; i < AllDirections.Num(); ++i)
	{
		DirectionToIndexMap.Add(AllDirections[i], i);
	}
}

bool UAStarStrategy::FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_FindPathWithStrategy); // Insights에서 이 이름으로 표시됩니다.

	// 1. 요청에서 옥트리 데이터 스냅샷을 가져옵니다.
	const FOctreePathfindingData& OctreeData = InRequest.NavigationData.Get<FOctreePathfindingData>();

	// 2. 시작점과 끝점이 유효한 'Empty' 리프 노드에 있는지 확인합니다.
	const int32 StartNodeIndex = OctreeData.FindLeafNodeAt(InRequest.StartLocation);
	int32 GoalNodeIndex = OctreeData.FindLeafNodeAt(InRequest.EndLocation);

	if (StartNodeIndex == -1 || OctreeData.StaticNodePool[StartNodeIndex].State != EOctreeNodeState::Empty)
	{
		UE_LOG(LogTemp, Warning, TEXT("A* (Octree) Failed: Start location is invalid or inside a solid node."));
		return false;
	}
	if (GoalNodeIndex == -1 || OctreeData.StaticNodePool[GoalNodeIndex].State != EOctreeNodeState::Empty)
	{
		const int32 NewGoalNodeIndex = OctreeData.FindNearestEmptyLeaf(InRequest.EndLocation, NearbyGoalSearchRadius, NearbyGoalSearchIterations);

		if (NewGoalNodeIndex == -1)
		{
			UE_LOG(LogTemp, Warning, TEXT("A* (Octree) Failed: Could not find any valid nearby goal location."));
			return false; // 주변 탐색으로도 실패 시, 최종 실패 처리
		}

		GoalNodeIndex = NewGoalNodeIndex; // 찾은 새 노드를 목표로 설정
		UE_LOG(LogTemp, Log, TEXT("A* (Octree): Found alternative goal at node index %d."), GoalNodeIndex);
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
	// Basic safety check: ensure the path has at least two points and the index is valid.
	if (CurrentPathIndex + 1 >= CurrentPathVoxels.Num())
	{
		// No upcoming voxel to check, so no repath is needed for this reason.
		return false;
	}

	// Check if we have a direct line of sight to the final destination of the current path segment.
	if (CurrentPathIndex < CurrentPathVoxels.Num() - 1)
	{
		const FIntVector& NextVoxel = CurrentPathVoxels.Last();
		if (FStrategyUtils::HasLineOfSightLight(Grid, CurrentAgentVoxel, NextVoxel, AgentRadiusVoxels))
		{
			return false;
		}
	}

	// Now it's safe to access the upcoming voxel.
	const FIntVector& UpcomingVoxel = CurrentPathVoxels[CurrentPathIndex + 1];
	if (!Grid.IsRegionWalkable(UpcomingVoxel, AgentRadiusVoxels))
	{
		// The next step on our path is blocked, so we need to repath.
		return true;
	}

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

	// 비교 함수 객체 생성 (매번 호출 시 넘겨주어야 함)
	const TPathNodePtrCompare<FAstarPathNode> Predicate;
	FAstarPathNode* StartNode = MemoryPool.NewNode();
	if (!StartNode) return EPathfindingResult::Failed_OutOfMemory;

	StartNode->NodeIndex = StartNodeIndex;
	StartNode->WorldPosition = OctreeData.GetNodeBounds(StartNodeIndex).GetCenter();
	StartNode->HCost = CalculateHeuristic(StartNode->WorldPosition,
	                                      OctreeData.GetNodeBounds(GoalNodeIndex).GetCenter());
	StartNode->CalculateAndSetFCost();

	OpenSet.HeapPush(StartNode, Predicate);
	OpenSetLookup.Add(StartNodeIndex, StartNode);

	FAstarPathNode* BestPartialNode = StartNode;

	while (OpenSet.Num() > 0)
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

		TArray<FAstarPathNode*> Successors;
		IdentifySuccessors(CurrentNode, Successors);

		for (FAstarPathNode* Successor : Successors)
		{
			if (ClosedSet.Contains(Successor->NodeIndex))
			{
				continue;
			}

			// === THETA* 로직 시작 ===
			FAstarPathNode* ParentNode = CurrentNode; // 기본 부모는 현재 노드 (A*와 동일)

			// 현재 노드에 부모가 있고(시작 노드가 아님), "부모의 부모"와 "Successor" 사이에 가시선이 확보된다면
			if (CurrentNode->Parent && HasLineOfSightWithRadius(CurrentNode->Parent->WorldPosition,
			                                                    Successor->WorldPosition))
			{
				// 부모를 "부모의 부모"로 설정하여 경로를 단축합니다.
				ParentNode = CurrentNode->Parent;
			}
			// === THETA* 로직 끝 ===

			const int64 NewGCost = ParentNode->GCost + GetMoveCost(ParentNode->WorldPosition, Successor->WorldPosition);

			FAstarPathNode** ExistingNodePtr = OpenSetLookup.Find(Successor->NodeIndex);
			if (ExistingNodePtr) // OpenSet에 이미 있는 경우
			{
				if (NewGCost < (*ExistingNodePtr)->GCost)
				{
					// 더 짧은 경로를 찾았으므로 부모와 비용을 업데이트합니다.
					(*ExistingNodePtr)->SetParent(ParentNode, NewGCost, (*ExistingNodePtr)->HCost);
					// 참고: TArray를 힙으로 사용할 때, 요소의 우선순위가 변경되면 힙을 재정렬해야 합니다.
					// OpenSet.Heapify(Predicate); // 혹은 해당 요소만 재정렬하는 함수를 사용
				}
			}
			else // OpenSet에 없는 새로운 노드인 경우
			{
				Successor->SetParent(ParentNode, NewGCost,
				                     CalculateHeuristic(Successor->WorldPosition,
				                                        OctreeData.GetNodeBounds(GoalNodeIndex).GetCenter()));

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
	TRACE_CPUPROFILER_EVENT_SCOPE(ThetaStar_HasLineOfSightWithRadius);

	return !OctreeData.CheckForSolidNodesInCapsule(Start, End, AgentRadius);
}

/**
 * 경로를 재구성하고, 더 부드럽게 만들기 위해 스무딩(String Pulling)을 적용합니다.
 */
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

	RawPath[0] = ActualStartLocation;

	if (HasLineOfSightWithRadius(RawPath.Last(), ActualGoalLocation))
	{
		RawPath.Last() = ActualGoalLocation;
	}

	if (RawPath.Num() < 2)
	{
		OutPath = RawPath;
		return;
	}

	OutPath.Empty();
	OutPath.Add(RawPath[0]); // 수정된 실제 시작점 추가

	int32 CurrentPointIndex = 0;
	while (CurrentPointIndex < RawPath.Num() - 1)
	{
		int32 LookAheadIndex = RawPath.Num() - 1;
		// 경로의 끝에서부터 현재 지점까지 가시선이 통하는 가장 먼 지점을 찾습니다.
		while (LookAheadIndex > CurrentPointIndex + 1)
		{
			if (HasLineOfSightWithRadius(OutPath.Last(), RawPath[LookAheadIndex]))
			{
				break; // 찾았다!
			}
			LookAheadIndex--;
		}
		
		OutPath.Add(RawPath[LookAheadIndex]); // 가시선이 확보된 가장 먼 지점을 경로에 추가
		
		// 만약 마지막 지점을 추가한 것이라면 루프를 종료합니다.
		if (LookAheadIndex == RawPath.Num() - 1)
		{
			break;
		}
		
		CurrentPointIndex = LookAheadIndex; 
	}
}

void FAstarPathfinder::IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(AStar_IdentifySuccessors_Internal);

	OutSuccessors.Empty();

	TArray<int32> AdjacentNodeIndices;

	{
		TRACE_CPUPROFILER_EVENT_SCOPE(AStar_FindAdjacentEmptyLeaves);
		OctreeData.FindAdjacentEmptyLeaves(Node->NodeIndex, AdjacentNodeIndices);
	}
	for (const int32 NeighborIndex : AdjacentNodeIndices)
	{
		{
			TRACE_CPUPROFILER_EVENT_SCOPE(AStar_CheckAndCreateSuccessor);
			const FBox NeighborBounds = OctreeData.GetNodeBounds(NeighborIndex);

			const FVector NeighborCenter = NeighborBounds.GetCenter();

			if (OctreeData.CheckForSolidNodesInCapsule(NeighborCenter, NeighborCenter, AgentRadius))
			{
				continue;
			}

			if (!OctreeData.IsRegionDynamicallyBlocked(NeighborBounds))
			{
				FAstarPathNode* SuccessorNode = MemoryPool.NewNode();
				if (SuccessorNode)
				{
					SuccessorNode->NodeIndex = NeighborIndex;
					SuccessorNode->WorldPosition = NeighborBounds.GetCenter();
					OutSuccessors.Add(SuccessorNode);
				}
			}
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
