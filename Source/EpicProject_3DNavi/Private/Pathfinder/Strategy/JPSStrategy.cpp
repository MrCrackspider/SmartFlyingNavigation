// JPSStrategy.cpp
#include "Pathfinder/Strategy/JPSStrategy.h"
#include "Pathfinder/Utility/StrategyUtils.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Containers/BinaryHeap.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"
#include "UniformGridVolume.h"
#include "Pathfinder/Utility/StrategyUtils.h"

void UJPSStrategy::Initialize(AVoxelVolumeBase* InVolume)
{
    Volume = Cast<AUniformGridVolume>(InVolume);
    if (Volume.IsValid() == false)
    {
        UE_LOG(LogTemp, Error, TEXT("UJPSPlusStrategy: The volume is invalid, Need AUniformGridVolume!! "));
        
        return;
    }
    
    if (!NodePool.IsValid())
    {
        AUniformGridVolume* GridVolume = Volume.Get();
        int64 VolumeNodeCount = (int64)GridVolume->MapSize.X * GridVolume->MapSize.Y * GridVolume->MapSize.Z;
    
        NodePool = MakeUnique<FPathNodeMemoryPool>(VolumeNodeCount); //일단 16384개 노드만
    }
    
    if (AllDirections.Num() == 0)
    {
        for (int32 x = -1; x <= 1; ++x)
            for (int32 y = -1; y <= 1; ++y)
                for (int32 z = -1; z <= 1; ++z)
                    if (x != 0 || y != 0 || z != 0)
                        AllDirections.Emplace(x, y, z);
    }
}

bool UJPSStrategy::FindPathWithStrategy(const FIntVector& StartIndex, const FIntVector& EndIndex, TArray<FIntVector>& OutPath)
{
    const EPathfindingResult Result = FindPathInternal(StartIndex, EndIndex, OutPath);

    // 실패 원인에 대한 로그 기록 (디버깅 용이성)
    if (Result != EPathfindingResult::Success)
    {
        FString ErrorMessage;
        switch (Result)
        {
        case EPathfindingResult::Failed_InvalidStart:
            ErrorMessage = FString::Printf(TEXT("Invalid Start Node: %s"), *StartIndex.ToString());
            break;
        case EPathfindingResult::Failed_InvalidGoal:
            ErrorMessage = FString::Printf(TEXT("Invalid Goal Node: %s"), *EndIndex.ToString());
            break;
        case EPathfindingResult::Failed_Unreachable:
            ErrorMessage = FString::Printf(TEXT("Goal %s is unreachable from Start %s."), *EndIndex.ToString(), *StartIndex.ToString());
            break;
        default:
            ErrorMessage = TEXT("An unknown error occurred.");
            break;
        }
        UE_LOG(LogTemp, Warning, TEXT("JPS Pathfinding failed. Reason: %s"), *ErrorMessage);
        return false;
    }

    return true;
}

FName UJPSStrategy::GetStrategyName() const
{
    return FName("JPS_3D_Robust_Complete");
}

EPathfindingResult UJPSStrategy::FindPathInternal(const FIntVector& StartIndex, const FIntVector& EndIndex, TArray<FIntVector>& OutPath)
{
    if (!Volume.IsValid() || !Volume->IsIndexValid(StartIndex) || !Volume->IsVoxelWalkable(StartIndex))
    {
        return EPathfindingResult::Failed_InvalidStart;
    }
    if (!Volume->IsIndexValid(EndIndex) || !Volume->IsVoxelWalkable(EndIndex))
    {
        return EPathfindingResult::Failed_InvalidGoal;
    }

    //길찾기 시작 시 메모리풀 리셋
    NodePool->Reset();
    
    OpenSet.Empty();
    OpenSetLookup.Empty();
    ClosedSet.Empty();
    OutPath.Empty();

    FPathNode* StartNode = NodePool->NewNode(StartIndex);
    StartNode->HCost = CalculateHeuristic(StartIndex, EndIndex);
    StartNode->CalculateAndSetFCost();

    OpenSet.HeapPush(StartNode, FPathNodePtrCompare());
    OpenSetLookup.Add(StartIndex, StartNode);
    
    FPathNode* GoalNode = nullptr;

    while (OpenSet.Num() > 0)
    {
        FPathNode* CurrentNode;
        OpenSet.HeapPop(CurrentNode, FPathNodePtrCompare());
        OpenSetLookup.Remove(CurrentNode->Position);
        ClosedSet.Add(CurrentNode->Position, CurrentNode);

        if (CurrentNode->Position == EndIndex)
        {
            GoalNode = CurrentNode;
            break;
        }

        TArray<FPathNode*> Successors;
        IdentifySuccessors(CurrentNode, EndIndex, Successors);

        for (const auto& Successor : Successors)
        {
            if (ClosedSet.Contains(Successor->Position)) continue;
            
            const int64 NewGCost = CurrentNode->GCost + GetMoveCost(CurrentNode->Position, Successor->Position);

            if (FPathNode** ExistingNodePtr = OpenSetLookup.Find(Successor->Position))
            {
                if (NewGCost < (*ExistingNodePtr)->GCost)
                {
                    (*ExistingNodePtr)->SetParent(CurrentNode, NewGCost, (*ExistingNodePtr)->HCost);
                    OpenSet.Heapify(FPathNodePtrCompare());
                }
            }
            else
            {
                Successor->HCost = CalculateHeuristic(Successor->Position, EndIndex);
                Successor->SetParent(CurrentNode, NewGCost, Successor->HCost);
                OpenSet.HeapPush(Successor, FPathNodePtrCompare());
                OpenSetLookup.Add(Successor->Position, Successor);
            }
        }
    }

    if (GoalNode)
    {
        TArray<FIntVector> GridPath;
        RetracePath(GoalNode, GridPath);
        OptimizePath_ThetaStarStyle(GridPath, OutPath);
        return EPathfindingResult::Success;
    }

    return EPathfindingResult::Failed_Unreachable;
}

void UJPSStrategy::GetPrunedDirections(const FIntVector& Dir, TArray<FIntVector>& OutDirections) const
{
    OutDirections.Empty();
    if (Dir == FIntVector::ZeroValue) // 시작 노드
    {
        OutDirections = AllDirections;
        return;
    }
    
    OutDirections.Reserve(14); // 3D 대각선은 최대 7(자연)+7(강제) 정도.

    const int32 dx = Dir.X, dy = Dir.Y, dz = Dir.Z;

    // 1. 자연스러운 이웃(Natural Neighbours) 추가
    OutDirections.Add(Dir); // 원래 진행 방향은 항상 추가
    if (dx != 0 && dy != 0 && dz != 0) // 3D 대각선
    {
        OutDirections.Add(FIntVector(dx, 0, 0)); OutDirections.Add(FIntVector(0, dy, 0)); OutDirections.Add(FIntVector(0, 0, dz));
        OutDirections.Add(FIntVector(dx, dy, 0)); OutDirections.Add(FIntVector(dx, 0, dz)); OutDirections.Add(FIntVector(0, dy, dz));
    }
    else if (dx != 0 && dy != 0) { OutDirections.Add(FIntVector(dx, 0, 0)); OutDirections.Add(FIntVector(0, dy, 0)); } // 2D XY
    else if (dx != 0 && dz != 0) { OutDirections.Add(FIntVector(dx, 0, 0)); OutDirections.Add(FIntVector(0, 0, dz)); } // 2D XZ
    else if (dy != 0 && dz != 0) { OutDirections.Add(FIntVector(0, dy, 0)); OutDirections.Add(FIntVector(0, 0, dz)); } // 2D YZ

    // 2. 강제된 이웃(Forced Neighbours) 추가
    // (HasForcedNeighbour 함수가 현재 노드 기준이므로, 여기서는 부모 방향을 전달하여 검사)
    // 이 부분은 HasForcedNeighbour 함수에서 처리하는 것이 더 직관적임.
    // IdentifySuccessors에서 현재 노드를 기준으로 모든 방향에 대해 HasForcedNeighbour를 검사하는 것이 더 명확한 구조.
    // 여기서는 자연스러운 이웃만 반환하고, IdentifySuccessors에서 강제 이웃을 추가하는 것으로 역할을 분리.
}

void UJPSStrategy::IdentifySuccessors(FPathNode*& Node, const FIntVector& Goal, TArray<FPathNode*>& OutSuccessors)
{
    OutSuccessors.Empty();
    TArray<FIntVector> DirsToSearch;
    const FIntVector ParentDir = Node->Parent ? FStrategyUtils::GetNormalizedDirection(Node->Parent->Position, Node->Position) : FIntVector::ZeroValue;
    
    // 1. 자연스러운 이웃 방향 가져오기
    GetPrunedDirections(ParentDir, DirsToSearch);

    // 2. 강제된 이웃 방향 추가하기
    for(const auto& Dir : AllDirections)
    {
        if (HasForcedNeighbour(Node->Position, Dir, ParentDir))
        {
            DirsToSearch.AddUnique(Dir);
        }
    }

    // 3. 최종 방향 목록에 대해 점프 수행
    for (const FIntVector& Dir : DirsToSearch)
    {
        TOptional<FIntVector> JumpPoint = Jump(Node->Position, Dir, Goal);
        if (JumpPoint.IsSet())
        {
            FPathNode* SuccessorNode = NodePool->NewNode(JumpPoint.GetValue());
            if(SuccessorNode) // 풀이 가득 차지 않았는지 항상 확인 -> 풀이 가득차면 다른 조치를 취해야할듯?
            {
                OutSuccessors.Add(SuccessorNode);
            }
            // OutSuccessors.Add(MakeShared<FPathNode>(JumpPoint.GetValue()));
        }
    }
}

TOptional<FIntVector> UJPSStrategy::Jump(const FIntVector& CurrentPos, const FIntVector& Direction, const FIntVector& Goal)
{
    const FIntVector NextPos = CurrentPos + Direction;
    const FIntVector ParentDir = Direction; // 점프 시에는 현재 방향이 부모 방향이 됨

    if (!IsMoveFeasible(CurrentPos, NextPos)) return TOptional<FIntVector>();
    if (NextPos == Goal) return NextPos;
    
    // 다음 노드(NextPos)를 기준으로 강제 이웃 검사
    for(const auto& Dir : AllDirections)
    {
        if (HasForcedNeighbour(NextPos, Dir, ParentDir))
        {
            return NextPos;
        }
    }

    // 하위 차원 점프 탐색
    const int32 Dim = FMath::Abs(Direction.X) + FMath::Abs(Direction.Y) + FMath::Abs(Direction.Z);
    if (Dim > 1) // 대각선 이동 시
    {
        // 직선 구성요소 먼저 탐색
        if (Direction.X != 0 && Jump(NextPos, FIntVector(Direction.X, 0, 0), Goal).IsSet()) return NextPos;
        if (Direction.Y != 0 && Jump(NextPos, FIntVector(0, Direction.Y, 0), Goal).IsSet()) return NextPos;
        if (Direction.Z != 0 && Jump(NextPos, FIntVector(0, 0, Direction.Z), Goal).IsSet()) return NextPos;
    }

    // 같은 방향으로 계속 점프
    return Jump(NextPos, Direction, Goal);
}

bool UJPSStrategy::IsMoveFeasible(const FIntVector& From, const FIntVector& To) const
{
    // 1. 기본 유효성 검사
    if (!Volume.IsValid() || !Volume->IsIndexValid(To) || !Volume->IsVoxelWalkable(To))
    {
        return false;
    }

    const FIntVector Move = To - From;
    const int32 dx = Move.X;
    const int32 dy = Move.Y;
    const int32 dz = Move.Z;
    const int32 Dim = FMath::Abs(dx) + FMath::Abs(dy) + FMath::Abs(dz);

    // 2. 직선 이동은 항상 가능 (도착지는 이미 위에서 검사 완료)
    if (Dim <= 1)
    {
        return true;
    }

    // 3. 2D 대각선 이동: 측면의 두 인접 노드가 '모두' walkable 이어야 함.
    if (Dim == 2)
    {
        // XY 평면
        if (dx != 0 && dy != 0)
        {
            return Volume->IsVoxelWalkable(From + FIntVector(dx, 0, 0)) && Volume->IsVoxelWalkable(From + FIntVector(0, dy, 0));
        }
        // XZ 평면
        if (dx != 0 && dz != 0)
        {
            return Volume->IsVoxelWalkable(From + FIntVector(dx, 0, 0)) && Volume->IsVoxelWalkable(From + FIntVector(0, 0, dz));
        }
        // YZ 평면
        if (dy != 0 && dz != 0)
        {
            return Volume->IsVoxelWalkable(From + FIntVector(0, dy, 0)) && Volume->IsVoxelWalkable(From + FIntVector(0, 0, dz));
        }
    }

    // 4. 3D 대각선 이동: 3개의 축 인접 노드와 3개의 면 인접 노드가 '모두' walkable 이어야 함.
    if (Dim == 3)
    {
        // 3개의 축 방향 인접 노드 검사
        const bool bAxisWalkable = Volume->IsVoxelWalkable(From + FIntVector(dx, 0, 0)) &&
                                   Volume->IsVoxelWalkable(From + FIntVector(0, dy, 0)) &&
                                   Volume->IsVoxelWalkable(From + FIntVector(0, 0, dz));
        
        // 3개의 면 방향 인접 노드 검사
        const bool bPlaneWalkable = Volume->IsVoxelWalkable(From + FIntVector(dx, dy, 0)) &&
                                    Volume->IsVoxelWalkable(From + FIntVector(dx, 0, dz)) &&
                                    Volume->IsVoxelWalkable(From + FIntVector(0, dy, dz));

        return bAxisWalkable && bPlaneWalkable;
    }

    // 위에서 처리되지 않은 경우는 없지만, 안전을 위해 false 반환
    return false;
}

void UJPSStrategy::RetracePath(FPathNode* EndNode, TArray<FIntVector>& OutGridPath) const
{
    // 안전한 경로 복원: Bresenham 알고리즘을 이용한 보간
    OutGridPath.Empty();
    TArray<FIntVector> JumpPoints;
    
    FPathNode* Current = EndNode;
    while (Current)
    {
        JumpPoints.Add(Current->Position);
        Current = Current->Parent;
    }
    Algo::Reverse(JumpPoints);

    if (JumpPoints.Num() < 2)
    {
        OutGridPath = JumpPoints;
        return;
    }

    OutGridPath.Add(JumpPoints[0]);
    for (int32 i = 0; i < JumpPoints.Num() - 1; ++i)
    {
        const FIntVector& From = JumpPoints[i];
        const FIntVector& To = JumpPoints[i + 1];
        
        // Bresenham 알고리즘으로 From과 To 사이의 모든 셀을 순회하며 경로에 추가
        FIntVector Diff = To - From;
        FIntVector Step(FMath::Sign(Diff.X), FMath::Sign(Diff.Y), FMath::Sign(Diff.Z));
        FIntVector AbsDiff(FMath::Abs(Diff.X), FMath::Abs(Diff.Y), FMath::Abs(Diff.Z));
        FIntVector CurrentVoxel = From;

        // 주축(dominant axis)을 따라 한 칸씩 이동
        if (AbsDiff.X >= AbsDiff.Y && AbsDiff.X >= AbsDiff.Z)
        {
            int32 Err1 = 2 * AbsDiff.Y - AbsDiff.X;
            int32 Err2 = 2 * AbsDiff.Z - AbsDiff.X;
            for (int32 k = 0; k < AbsDiff.X; ++k)
            {
                if (Err1 > 0) { CurrentVoxel.Y += Step.Y; Err1 -= 2 * AbsDiff.X; }
                if (Err2 > 0) { CurrentVoxel.Z += Step.Z; Err2 -= 2 * AbsDiff.X; }
                Err1 += 2 * AbsDiff.Y;
                Err2 += 2 * AbsDiff.Z;
                CurrentVoxel.X += Step.X;
                OutGridPath.Add(CurrentVoxel);
            }
        }
        else if (AbsDiff.Y >= AbsDiff.X && AbsDiff.Y >= AbsDiff.Z)
        {
            int32 Err1 = 2 * AbsDiff.X - AbsDiff.Y;
            int32 Err2 = 2 * AbsDiff.Z - AbsDiff.Y;
            for (int32 k = 0; k < AbsDiff.Y; ++k)
            {
                if (Err1 > 0) { CurrentVoxel.X += Step.X; Err1 -= 2 * AbsDiff.Y; }
                if (Err2 > 0) { CurrentVoxel.Z += Step.Z; Err2 -= 2 * AbsDiff.Y; }
                Err1 += 2 * AbsDiff.X;
                Err2 += 2 * AbsDiff.Z;
                CurrentVoxel.Y += Step.Y;
                OutGridPath.Add(CurrentVoxel);
            }
        }
        else
        {
            int32 Err1 = 2 * AbsDiff.X - AbsDiff.Z;
            int32 Err2 = 2 * AbsDiff.Y - AbsDiff.Z;
            for (int32 k = 0; k < AbsDiff.Z; ++k)
            {
                if (Err1 > 0) { CurrentVoxel.X += Step.X; Err1 -= 2 * AbsDiff.Z; }
                if (Err2 > 0) { CurrentVoxel.Y += Step.Y; Err2 -= 2 * AbsDiff.Z; }
                Err1 += 2 * AbsDiff.X;
                Err2 += 2 * AbsDiff.Y;
                CurrentVoxel.Z += Step.Z;
                OutGridPath.Add(CurrentVoxel);
            }
        }
    }
}

void UJPSStrategy::OptimizePath_ThetaStarStyle(const TArray<FIntVector>& GridPath, TArray<FIntVector>& OutOptimizedPath) const
{
    OutOptimizedPath.Empty();
    if (GridPath.Num() < 2)
    {
        OutOptimizedPath = GridPath;
        return;
    }

    OutOptimizedPath.Add(GridPath[0]);
    int32 CurrentIndex = 0;

    while (CurrentIndex < GridPath.Num() - 1)
    {
        int32 LookaheadIndex = GridPath.Num() - 1;
        // 경로의 끝에서부터 거꾸로 오면서 LOS가 확보되는 가장 먼 노드를 찾음
        while (LookaheadIndex > CurrentIndex + 1)
        {
            if (HasLineOfSight(GridPath[CurrentIndex], GridPath[LookaheadIndex]))
            {
                break;
            }
            LookaheadIndex--;
        }
        OutOptimizedPath.Add(GridPath[LookaheadIndex]);
        CurrentIndex = LookaheadIndex;
    }
}

bool UJPSStrategy::HasLineOfSight(const FIntVector& From, const FIntVector& To) const
{
    FIntVector Diff = To - From;
    FIntVector Step(FMath::Sign(Diff.X), FMath::Sign(Diff.Y), FMath::Sign(Diff.Z));
    FIntVector AbsDiff(FMath::Abs(Diff.X), FMath::Abs(Diff.Y), FMath::Abs(Diff.Z));
    FIntVector Current = From;

    // 주축(dominant axis)을 따라 한 칸씩 이동
    if (AbsDiff.X >= AbsDiff.Y && AbsDiff.X >= AbsDiff.Z) // X가 주축
    {
        int32 Err1 = 2 * AbsDiff.Y - AbsDiff.X;
        int32 Err2 = 2 * AbsDiff.Z - AbsDiff.X;
        for (int32 i = 0; i < AbsDiff.X; ++i)
        {
            FIntVector Prev = Current;
            if (Err1 > 0) { Current.Y += Step.Y; Err1 -= 2 * AbsDiff.X; }
            if (Err2 > 0) { Current.Z += Step.Z; Err2 -= 2 * AbsDiff.X; }
            Current.X += Step.X;

            // ★★★ 매 스텝마다 최종 IsMoveFeasible 검사 ★★★
            if (!IsMoveFeasible(Prev, Current))
            {
                return false;
            }
            Err1 += 2 * AbsDiff.Y;
            Err2 += 2 * AbsDiff.Z;
        }
    }
    else if (AbsDiff.Y >= AbsDiff.X && AbsDiff.Y >= AbsDiff.Z) // Y가 주축
    {
        int32 Err1 = 2 * AbsDiff.X - AbsDiff.Y;
        int32 Err2 = 2 * AbsDiff.Z - AbsDiff.Y;
        for (int32 i = 0; i < AbsDiff.Y; ++i)
        {
            FIntVector Prev = Current;
            if (Err1 > 0) { Current.X += Step.X; Err1 -= 2 * AbsDiff.Y; }
            if (Err2 > 0) { Current.Z += Step.Z; Err2 -= 2 * AbsDiff.Y; }
            Current.Y += Step.Y;

            if (!IsMoveFeasible(Prev, Current))
            {
                return false;
            }
            Err1 += 2 * AbsDiff.X;
            Err2 += 2 * AbsDiff.Z;
        }
    }
    else // Z가 주축
    {
        int32 Err1 = 2 * AbsDiff.X - AbsDiff.Z;
        int32 Err2 = 2 * AbsDiff.Y - AbsDiff.Z;
        for (int32 i = 0; i < AbsDiff.Z; ++i)
        {
            FIntVector Prev = Current;
            if (Err1 > 0) { Current.X += Step.X; Err1 -= 2 * AbsDiff.Z; }
            if (Err2 > 0) { Current.Y += Step.Y; Err2 -= 2 * AbsDiff.Z; }
            Current.Z += Step.Z;

            if (!IsMoveFeasible(Prev, Current))
            {
                return false;
            }
            Err1 += 2 * AbsDiff.X;
            Err2 += 2 * AbsDiff.Y;
        }
    }

    // 최종 목적지까지 모든 스텝이 안전했는지 확인
    return true;
}

int32 UJPSStrategy::CalculateHeuristic(const FIntVector& A, const FIntVector& B) const
{
    return FStrategyUtils::Heuristic_Octile(A, B);
}

int32 UJPSStrategy::GetMoveCost(const FIntVector& A, const FIntVector& B) const
{
    return FStrategyUtils::Heuristic_Octile(A, B);
}
bool UJPSStrategy::HasForcedNeighbour(const FIntVector& NodePos, const FIntVector& CheckDir, const FIntVector& ParentDir) const
{
    // 아이디어: CheckDir 방향으로 가야만 하는 이유가 있는가?
    // -> CheckDir의 반대편에 장애물이 있고, 그 장애물이 ParentDir로부터의 직선 경로를 막고 있을 때
    const FIntVector BackDir = FStrategyUtils::GetNormalizedDirection(CheckDir, FIntVector::ZeroValue); // -CheckDir
    const FIntVector ObstaclePos = NodePos + BackDir;

    // 1. 장애물이 없으면 강제 이웃이 아님
    if (Volume->IsVoxelWalkable(ObstaclePos))
    {
        return false;
    }
    
    // 2. 장애물이 있지만, 더 나은 경로(ParentDir에서 직접 가는)가 열려있으면 강제 이웃이 아님
    const FIntVector ParentToObstacleDir = FStrategyUtils::GetNormalizedDirection(NodePos + ParentDir, ObstaclePos);
    if (IsMoveFeasible(NodePos + ParentDir, NodePos + ParentDir + ParentToObstacleDir))
    {
        return false;
    }

    // 3. 위 조건들을 모두 통과하고, CheckDir 방향이 실제로 갈 수 있는 길이라면 강제 이웃임
    if (IsMoveFeasible(NodePos, NodePos + CheckDir))
    {
        return true;
    }

    return false;
}
