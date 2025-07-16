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
	// 1. 요청에서 옥트리 데이터 스냅샷을 가져옵니다.
	const FOctreePathfindingData& OctreeData = InRequest.NavigationData.Get<FOctreePathfindingData>();

	// 2. 시작점과 끝점이 유효한 'Empty' 리프 노드에 있는지 확인합니다.
	const int32 StartNodeIndex = OctreeData.FindLeafNodeAt(InRequest.StartLocation);
	const int32 GoalNodeIndex = OctreeData.FindLeafNodeAt(InRequest.EndLocation);

	if (StartNodeIndex == -1 || OctreeData.StaticNodePool[StartNodeIndex].State != EOctreeNodeState::Empty)
	{
		UE_LOG(LogTemp, Warning, TEXT("A* (Octree) Failed: Start location is invalid or inside a solid node."));
		return false;
	}
	if (GoalNodeIndex == -1 || OctreeData.StaticNodePool[GoalNodeIndex].State != EOctreeNodeState::Empty)
	{
		UE_LOG(LogTemp, Warning, TEXT("A* (Octree) Failed: Goal location is invalid or inside a solid node."));
		// TODO: 여기서 FindNearestWalkable 같은 로직을 호출하여 대안 목표를 찾을 수 있습니다.
		return false;
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
	FAstarPathfinder Pathfinder(OctreeData, StartNodeIndex, GoalNodeIndex, InRequest.AgentRadius, NodePool);
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

FAstarPathfinder::FAstarPathfinder(const FOctreePathfindingData& InOctreeData, int32 InStartNodeIndex,
                                   int32 InGoalNodeIndex, float InAgentRadius,
                                   TNodeMemoryPool<FAstarPathNode>& InMemoryPool)
	: OctreeData(InOctreeData)
	  , StartNodeIndex(InStartNodeIndex)
	  , GoalNodeIndex(InGoalNodeIndex)
	  , AgentRadius(InAgentRadius)
	  , MemoryPool(InMemoryPool)
{
}

EPathfindingResult FAstarPathfinder::FindPath(TArray<FVector>& OutPath)
{
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

	OpenSet.HeapPush(StartNode,Predicate);
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
			ReconstructPath(CurrentNode, OutPath);
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

			const int64 NewGCost = CurrentNode->GCost + GetMoveCost(CurrentNode->WorldPosition,
			                                                        Successor->WorldPosition);

			FAstarPathNode** ExistingNodePtr = OpenSetLookup.Find(Successor->NodeIndex);
			if (ExistingNodePtr)
			{
				if (NewGCost < (*ExistingNodePtr)->GCost)
				{
					(*ExistingNodePtr)->SetParent(CurrentNode, NewGCost, (*ExistingNodePtr)->HCost);

					//OpenSet.Heapify(Predicate);
				}
			}
			else
			{
				Successor->SetParent(CurrentNode, NewGCost,
				                     CalculateHeuristic(Successor->WorldPosition,
				                                        OctreeData.GetNodeBounds(GoalNodeIndex).GetCenter()));

				OpenSet.HeapPush(Successor, Predicate);
				OpenSetLookup.Add(Successor->NodeIndex, Successor);
			}
		}
	}

	if (BestPartialNode)
	{
		ReconstructPath(BestPartialNode, OutPath);
		return EPathfindingResult::Success_Partial;
	}

	return EPathfindingResult::Failed_Unreachable;
}


void FAstarPathfinder::IdentifySuccessors(FAstarPathNode* Node, TArray<FAstarPathNode*>& OutSuccessors)
{
	OutSuccessors.Empty();

	// 1. 현재 노드와 인접한 모든 '정적' Empty 리프 노드를 찾습니다.
	TArray<int32> AdjacentNodeIndices;
	OctreeData.FindAdjacentEmptyLeaves(Node->NodeIndex, AdjacentNodeIndices);

	for (const int32 NeighborIndex : AdjacentNodeIndices)
	{
		// 2. 각 이웃이 에이전트 크기를 고려했을 때 '동적으로' 막혔는지 확인합니다.
		const FBox NeighborBounds = OctreeData.GetNodeBounds(NeighborIndex);
		if (!OctreeData.IsRegionDynamicallyBlocked(NeighborBounds)) // 에이전트 크기는 IsRegionDynamicallyBlocked 내부에서 고려되어야 함
		{
			// 3. 통과 가능하다면 후계자 노드로 생성
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

int64 FAstarPathfinder::CalculateHeuristic(const FVector& A, const FVector& B) const
{
	// 옥타일 거리 또는 유클리드 거리를 사용할 수 있습니다. 여기서는 유클리드 거리.
	return static_cast<int64>(FVector::Distance(A, B));
}

int64 FAstarPathfinder::GetMoveCost(const FVector& A, const FVector& B) const
{
	// 실제 이동 비용도 두 노드 중심점 간의 거리로 계산합니다.
	return static_cast<int64>(FVector::Distance(A, B));
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
