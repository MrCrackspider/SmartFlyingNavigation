#include "Pathfinder/Strategy/JPSStrategy.h"
#include "Pathfinder/VoxelVolume.h"
#include "Pathfinder/Utility/StrategyUtils.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Containers/BinaryHeap.h"
#include "Algo/Reverse.h"

void UJPSStrategy::Initialize(AVoxelVolume* InVolume)
{
	Volume = InVolume;
}

bool UJPSStrategy::FindPathWithStrategy(const FIntVector& StartIndex, const FIntVector& EndIndex, TArray<FIntVector>& OutPath)
{
	if (!Volume.IsValid() || !Volume->IsVoxelWalkable(StartIndex) || !Volume->IsVoxelWalkable(EndIndex))
	{
		return false;
	}
	return JumpPointSearch(StartIndex, EndIndex, OutPath);
}

FName UJPSStrategy::GetStrategyName() const
{
	return FName("JPS_Strategy");
}

bool UJPSStrategy::JumpPointSearch(const FIntVector& StartIndex, const FIntVector& EndIndex, TArray<FIntVector>& OutPath)
{
    FPathNodePtrCompare NodeCompare;

    TSharedPtr<FPathNode> StartNode = MakeShared<FPathNode>(StartIndex);
    StartNode->HCost = FStrategyUtils::Heuristic_Octile(StartIndex, EndIndex);
	StartNode->GCost = 0;
	StartNode->CalculateAndSetFCost();

    TArray<TSharedPtr<FPathNode>> OpenSet;
    OpenSet.HeapPush(StartNode, NodeCompare);

    TMap<FIntVector, TSharedPtr<FPathNode>> AllNodes;
    AllNodes.Add(StartIndex, StartNode);

    TSet<FIntVector> ClosedSet;

    while (OpenSet.Num() > 0)
    {
        TSharedPtr<FPathNode> CurrentNode;
        OpenSet.HeapPop(CurrentNode, NodeCompare, false);

        if (ClosedSet.Contains(CurrentNode->Position))
        {
            continue;
        }
        ClosedSet.Add(CurrentNode->Position);

        if (CurrentNode->Position == EndIndex)
        {
            RetracePath(CurrentNode.ToSharedRef(), OutPath);
            return true;
        }

        TArray<FIntVector> Successors = IdentifySuccessors(CurrentNode, EndIndex);
        for (const FIntVector& SuccessorPos : Successors)
        {
            if (ClosedSet.Contains(SuccessorPos))
            {
                continue;
            }

            int32 NewGCost = CurrentNode->GCost + FStrategyUtils::Heuristic_Octile(CurrentNode->Position, SuccessorPos);

            TSharedPtr<FPathNode>* SuccessorNodePtr = AllNodes.Find(SuccessorPos);
            bool bIsNewNode = (SuccessorNodePtr == nullptr);

            if (bIsNewNode || NewGCost < (*SuccessorNodePtr)->GCost)
            {
                TSharedPtr<FPathNode> SuccessorNode = bIsNewNode ? MakeShared<FPathNode>(SuccessorPos) : *SuccessorNodePtr;
                SuccessorNode->Parent = CurrentNode;
                SuccessorNode->GCost = NewGCost;
                SuccessorNode->HCost = FStrategyUtils::Heuristic_Octile(SuccessorPos, EndIndex);
				SuccessorNode->CalculateAndSetFCost();

                if (bIsNewNode)
                {
                    AllNodes.Add(SuccessorPos, SuccessorNode);
                }

                OpenSet.HeapPush(SuccessorNode, NodeCompare);
            }
        }
    }

    return false;
}

TArray<FIntVector> UJPSStrategy::IdentifySuccessors(const TSharedPtr<FPathNode>& Node, const FIntVector& Goal) const
{
    TArray<FIntVector> Successors;
    if (!Node || !Volume.IsValid())
    {
        return Successors;
    }

    UWorld* World = Volume->GetWorld();
    if (!World)
    {
        return Successors;
    }

    const TArray<FIntVector> Neighbors = FindPrunedNeighbors(Node);

    for (const FIntVector& NeighborPos : Neighbors)
    {
        const FIntVector Direction = FStrategyUtils::GetNormalizedDirection(Node->Position, NeighborPos);
        const TOptional<FIntVector> JumpPoint = Jump(Node->Position, Direction, Goal);

        if (JumpPoint.IsSet())
        {
            const FIntVector JumpPointLocation = JumpPoint.GetValue();
            Successors.AddUnique(JumpPointLocation);

            const FVector WorldLocation = Volume->VoxelIndexToWorld(JumpPointLocation);

            DrawDebugSphere(
                World,
                WorldLocation,
                20.0f,
                12,
                FColor::Yellow,
                false,
                5.0f,
                0,
                1.0f
            );
        }
    }

    return Successors;
}

void UJPSStrategy::RetracePath(TSharedRef<FPathNode> EndNode, TArray<FIntVector>& OutPath)
{
    OutPath.Empty();
    TSharedPtr<FPathNode> CurrentNode = EndNode;

    while (CurrentNode.IsValid())
    {
        OutPath.Add(CurrentNode->Position);
        CurrentNode = CurrentNode->Parent;
    }

    Algo::Reverse(OutPath);
}

TArray<FIntVector> UJPSStrategy::FindPrunedNeighbors(const TSharedPtr<FPathNode>& Node) const
{
    TArray<FIntVector> Neighbors;
    if (!Node.IsValid() || !Volume.IsValid()) return Neighbors;

    const FIntVector& CurrentPos = Node->Position;

    if (!Node->Parent.IsValid())
    {
        for (int32 OffsetX = -1; OffsetX <= 1; ++OffsetX)
        {
            for (int32 OffsetY = -1; OffsetY <= 1; ++OffsetY)
            {
                for (int32 OffsetZ = -1; OffsetZ <= 1; ++OffsetZ)
                {
                    if (OffsetX == 0 && OffsetY == 0 && OffsetZ == 0) continue;
                    FIntVector NeighborPos = CurrentPos + FIntVector(OffsetX, OffsetY, OffsetZ);
                    if (Volume->IsVoxelWalkable(NeighborPos))
                    {
                        Neighbors.Add(NeighborPos);
                    }
                }
            }
        }
        return Neighbors;
    }

    const FIntVector ParentPos = Node->Parent->Position;
    const FIntVector Direction = FStrategyUtils::GetNormalizedDirection(ParentPos, CurrentPos);

    if (Direction.X != 0 && Direction.Y != 0 && Direction.Z != 0) // 공간 대각선
    {
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z })) Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z })) Neighbors.AddUnique({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X, CurrentPos.Y, CurrentPos.Z + Direction.Z })) Neighbors.AddUnique({ CurrentPos.X, CurrentPos.Y, CurrentPos.Z + Direction.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y + Direction.Y, CurrentPos.Z })) Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y + Direction.Y, CurrentPos.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z + Direction.Z })) Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z + Direction.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z + Direction.Z })) Neighbors.AddUnique({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z + Direction.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y + Direction.Y, CurrentPos.Z + Direction.Z })) Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y + Direction.Y, CurrentPos.Z + Direction.Z });
    }
    else if (Direction.X != 0 && Direction.Y != 0) // XY 평면 대각선
    {
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z })) Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z })) Neighbors.AddUnique({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y + Direction.Y, CurrentPos.Z })) Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y + Direction.Y, CurrentPos.Z });
    }
    else if (Direction.X != 0 && Direction.Z != 0) // XZ 평면 대각선
    {
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z })) Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X, CurrentPos.Y, CurrentPos.Z + Direction.Z })) Neighbors.AddUnique({ CurrentPos.X, CurrentPos.Y, CurrentPos.Z + Direction.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z + Direction.Z })) Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y, CurrentPos.Z + Direction.Z });
    }
    else if (Direction.Y != 0 && Direction.Z != 0) // YZ 평면 대각선
    {
        if (Volume->IsVoxelWalkable({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z })) Neighbors.AddUnique({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X, CurrentPos.Y, CurrentPos.Z + Direction.Z })) Neighbors.AddUnique({ CurrentPos.X, CurrentPos.Y, CurrentPos.Z + Direction.Z });
        if (Volume->IsVoxelWalkable({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z + Direction.Z })) Neighbors.AddUnique({ CurrentPos.X, CurrentPos.Y + Direction.Y, CurrentPos.Z + Direction.Z });
    }
    else
    {
        if (Volume->IsVoxelWalkable({ CurrentPos.X + Direction.X, CurrentPos.Y + Direction.Y, CurrentPos.Z + Direction.Z }))
            Neighbors.AddUnique({ CurrentPos.X + Direction.X, CurrentPos.Y + Direction.Y, CurrentPos.Z + Direction.Z });
    }

    for (int32 OffsetX = -1; OffsetX <= 1; ++OffsetX)
    {
        for (int32 OffsetY = -1; OffsetY <= 1; ++OffsetY)
        {
            for (int32 OffsetZ = -1; OffsetZ <= 1; ++OffsetZ)
            {
                if (OffsetX == 0 && OffsetY == 0 && OffsetZ == 0) continue;
                if (OffsetX == Direction.X && OffsetY == Direction.Y && OffsetZ == Direction.Z) continue;


                FIntVector CheckObstaclePos = CurrentPos + FIntVector(OffsetX, OffsetY, OffsetZ);
                FIntVector ForcedNeighborPos = CurrentPos + FIntVector(Direction.X + OffsetX, Direction.Y + OffsetY, Direction.Z + OffsetZ);

                if (!Volume->IsVoxelWalkable(CheckObstaclePos) && Volume->IsVoxelWalkable(ForcedNeighborPos))
                {
                    Neighbors.AddUnique(ForcedNeighborPos);
                }
            }
        }
    }
    return Neighbors;
}

TOptional<FIntVector> UJPSStrategy::Jump(FIntVector CurrentPos, const FIntVector& Direction, const FIntVector& Goal) const
{
    const FIntVector NextPos = CurrentPos + Direction;

    if (!Volume.IsValid() || !Volume->IsVoxelWalkable(NextPos))
    {
        return TOptional<FIntVector>();
    }

    if (NextPos == Goal)
    {
        return TOptional<FIntVector>(NextPos);
    }

    for (int32 OffsetX = -1; OffsetX <= 1; ++OffsetX)
    {
        for (int32 OffsetY = -1; OffsetY <= 1; ++OffsetY)
        {
            for (int32 OffsetZ = -1; OffsetZ <= 1; ++OffsetZ)
            {
                if (OffsetX == 0 && OffsetY == 0 && OffsetZ == 0) continue;

                FIntVector CheckObstacleBehindPos = NextPos - Direction + FIntVector(OffsetX, OffsetY, OffsetZ);
                FIntVector ForcedCandidatePos = NextPos + FIntVector(OffsetX, OffsetY, OffsetZ);

                if (Volume->IsVoxelWalkable(ForcedCandidatePos) && !Volume->IsVoxelWalkable(CheckObstacleBehindPos))
                {
                    return TOptional<FIntVector>(NextPos);
                }
            }
        }
    }

    if (Direction.X != 0 && Direction.Y != 0 && Direction.Z != 0)
    {
        if (Jump(NextPos, { Direction.X, 0, 0 }, Goal).IsSet() || Jump(NextPos, { 0, Direction.Y, 0 }, Goal).IsSet() || Jump(NextPos, { 0, 0, Direction.Z }, Goal).IsSet() ||
            Jump(NextPos, { Direction.X, Direction.Y, 0 }, Goal).IsSet() || Jump(NextPos, { Direction.X, 0, Direction.Z }, Goal).IsSet() || Jump(NextPos, { 0, Direction.Y, Direction.Z }, Goal).IsSet())
        {
            return TOptional<FIntVector>(NextPos);
        }
    }
    else if (Direction.X != 0 && Direction.Y != 0) // XY 평면
    {
        if (Jump(NextPos, { Direction.X, 0, 0 }, Goal).IsSet() || Jump(NextPos, { 0, Direction.Y, 0 }, Goal).IsSet())
        {
            return TOptional<FIntVector>(NextPos);
        }
    }
    else if (Direction.X != 0 && Direction.Z != 0) // XZ 평면
    {
        if (Jump(NextPos, { Direction.X, 0, 0 }, Goal).IsSet() || Jump(NextPos, { 0, 0, Direction.Z }, Goal).IsSet())
        {
            return TOptional<FIntVector>(NextPos);
        }
    }
    else if (Direction.Y != 0 && Direction.Z != 0) // YZ 평면
    {
        if (Jump(NextPos, { 0, Direction.Y, 0 }, Goal).IsSet() || Jump(NextPos, { 0, 0, Direction.Z }, Goal).IsSet())
        {
            return TOptional<FIntVector>(NextPos);
        }
    }

    return Jump(NextPos, Direction, Goal);
}