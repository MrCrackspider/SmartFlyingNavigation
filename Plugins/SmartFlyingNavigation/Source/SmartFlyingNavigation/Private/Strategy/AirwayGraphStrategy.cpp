// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Strategy/AirwayGraphStrategy.h"
#include "Utility/Parallel/PathfindingRequest.h"
#include "Algo/Reverse.h"

struct FAirwaySearchNode
{
	int32 NodeIndex = -1;
	double FCost = 0.0;
};

struct FAirwaySearchNodeCompare
{
	bool operator()(const FAirwaySearchNode& A, const FAirwaySearchNode& B) const
	{
		return A.FCost < B.FCost;
	}
};

void UAirwayGraphStrategy::Initialize()
{
}

bool UAirwayGraphStrategy::FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath)
{
	const FAirwayGraph* GraphPtr = InRequest.NavigationData.AirwayGraphPtr;
	if (!GraphPtr || GraphPtr->IsEmpty())
	{
		return false;
	}

	const FAirwayGraph& Graph = *GraphPtr;
	const int32 StartNode = FindNearestUsableNode(Graph, InRequest.StartLocation, InRequest.AgentRadius);
	const int32 GoalNode = FindNearestUsableNode(Graph, InRequest.EndLocation, InRequest.AgentRadius);

	if (!Graph.IsValidIndex(StartNode) || !Graph.IsValidIndex(GoalNode))
	{
		return false;
	}

	if (StartNode == GoalNode)
	{
		OutPath = { InRequest.StartLocation, InRequest.EndLocation };
		return true;
	}

	const FVector GoalPosition = Graph.GetPosition(GoalNode);

	TArray<double> GScore;
	GScore.Init(TNumericLimits<double>::Max(), Graph.Num());

	TArray<int32> CameFrom;
	CameFrom.Init(-1, Graph.Num());

	TArray<uint8> bClosed;
	bClosed.Init(0, Graph.Num());

	TArray<FAirwaySearchNode> OpenSet;
	const FAirwaySearchNodeCompare Predicate;

	GScore[StartNode] = 0.0;
	OpenSet.HeapPush({ StartNode, FVector::Dist(Graph.GetPosition(StartNode), GoalPosition) }, Predicate);

	constexpr int32 MaxIterations = 20000;
	int32 IterationCount = 0;

	while (!OpenSet.IsEmpty())
	{
		if (++IterationCount > MaxIterations)
		{
			break;
		}

		FAirwaySearchNode Current;
		OpenSet.HeapPop(Current, Predicate);
		if (!Graph.IsValidIndex(Current.NodeIndex) || bClosed[Current.NodeIndex])
		{
			continue;
		}

		if (Current.NodeIndex == GoalNode)
		{
			ReconstructPath(CameFrom, StartNode, GoalNode, Graph, InRequest.StartLocation, InRequest.EndLocation, OutPath);
			return true;
		}

		bClosed[Current.NodeIndex] = 1;
		const FVector CurrentPosition = Graph.GetPosition(Current.NodeIndex);

		uint32 EdgeBegin = 0, EdgeEnd = 0;
		Graph.GetEdgeRange(Current.NodeIndex, EdgeBegin, EdgeEnd);

		for (uint32 EdgeIndex = EdgeBegin; EdgeIndex < EdgeEnd; ++EdgeIndex)
		{
			const uint32 Edge = Graph.Edges[EdgeIndex];
			const int32 NeighborIndex = FAirwayGraph::GetEdgeTarget(Edge);
			if (!Graph.IsValidIndex(NeighborIndex) || bClosed[NeighborIndex])
			{
				continue;
			}

			// Gate on both the destination node's clearance and the edge clearance.
			if (Graph.GetClearance(NeighborIndex) < InRequest.AgentRadius ||
				FAirwayGraph::GetEdgeClearance(Edge) < InRequest.AgentRadius)
			{
				continue;
			}

			const FVector NeighborPosition = Graph.GetPosition(NeighborIndex);
			const double TentativeG = GScore[Current.NodeIndex] + FVector::Dist(CurrentPosition, NeighborPosition);
			if (TentativeG < GScore[NeighborIndex])
			{
				CameFrom[NeighborIndex] = Current.NodeIndex;
				GScore[NeighborIndex] = TentativeG;
				const double H = FVector::Dist(NeighborPosition, GoalPosition);
				OpenSet.HeapPush({ NeighborIndex, TentativeG + H }, Predicate);
			}
		}
	}

	return false;
}

FName UAirwayGraphStrategy::GetStrategyName() const
{
	return TEXT("AirwayGraphStrategy");
}

int32 UAirwayGraphStrategy::FindNearestUsableNode(const FAirwayGraph& Graph, const FVector& Location, float AgentRadius) const
{
	int32 BestIndex = -1;
	double BestDistanceSq = TNumericLimits<double>::Max();

	for (int32 NodeIndex = 0; NodeIndex < Graph.Num(); ++NodeIndex)
	{
		if (Graph.GetClearance(NodeIndex) < AgentRadius)
		{
			continue;
		}

		const double DistanceSq = FVector::DistSquared(Location, Graph.GetPosition(NodeIndex));
		if (DistanceSq < BestDistanceSq)
		{
			BestDistanceSq = DistanceSq;
			BestIndex = NodeIndex;
		}
	}

	return BestIndex;
}

void UAirwayGraphStrategy::ReconstructPath(const TArray<int32>& CameFrom, int32 StartNode, int32 GoalNode, const FAirwayGraph& Graph, const FVector& StartLocation, const FVector& EndLocation, TArray<FVector>& OutPath) const
{
	TArray<int32> ReversedNodes;
	int32 Current = GoalNode;
	while (Current != -1)
	{
		ReversedNodes.Add(Current);
		if (Current == StartNode)
		{
			break;
		}
		Current = CameFrom.IsValidIndex(Current) ? CameFrom[Current] : -1;
	}

	Algo::Reverse(ReversedNodes);

	OutPath.Reset(ReversedNodes.Num() + 2);
	OutPath.Add(StartLocation);
	for (const int32 NodeIndex : ReversedNodes)
	{
		if (Graph.IsValidIndex(NodeIndex))
		{
			OutPath.Add(Graph.GetPosition(NodeIndex));
		}
	}
	OutPath.Add(EndLocation);
}
