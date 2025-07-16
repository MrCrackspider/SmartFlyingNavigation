// JPSStrategy.cpp
#include "Pathfinder/Strategy/JPSPlusStrategy.h"
#include "Pathfinder/Utility/StrategyUtils.h"
#include "Pathfinder/Utility/MemoryPool.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"

void UJPSPlusStrategy::Initialize()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_Initialize);

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


bool UJPSPlusStrategy::FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FIntVector>& OutPath)
{
	FIntVector ActualGoalIndex = InRequest.EndIndex;
	auto GridData = InRequest.NavigationData.Get<FThreadSafeBitmaskGrid>();
	
	if (GridData.IsRegionWalkable(InRequest.EndIndex, InRequest.AgentRadiusInVoxels))
	{
		FIntVector NearestWalkableVoxel;
		if (GridData.FindNearestWalkableVoxel_BFS(InRequest.EndIndex, InRequest.AgentRadiusInVoxels,
		                                                           50, NearestWalkableVoxel))
		{
			ActualGoalIndex = NearestWalkableVoxel;
		}
	}
	FPathNodeMemoryPoolBase* BasePool = InRequest.MemoryPool;
	if (!BasePool)
	{
		UE_LOG(LogTemp, Error, TEXT("JPS+ Strategy failed to lease a memory pool."));
		return false;
	}
	auto& NodePool = *static_cast<TNodeMemoryPool<FPathNode>*>(BasePool);

	// 3. '일꾼' 객체 생성 및 작업 위임
	FJPSPlusPathfinder Pathfinder(InRequest,GridData, AllDirections, DirectionToIndexMap, ActualGoalIndex,NodePool);

	const EPathfindingResult Result = Pathfinder.FindPath(OutPath);
	
	// 실패 원인에 대한 로그 기록 (디버깅 용이성)
	if (Result != EPathfindingResult::Success && Result != EPathfindingResult::Success_Partial)
	{
		FString ErrorMessage;
		switch (Result)
		{
		case EPathfindingResult::Failed_InvalidStart:
			ErrorMessage = FString::Printf(TEXT("Invalid Start Node: %s"), *InRequest.StartIndex.ToString());
			break;
		case EPathfindingResult::Failed_InvalidGoal:
			ErrorMessage = FString::Printf(TEXT("Invalid Goal Node: %s"), *InRequest.EndIndex.ToString());
			break;
		case EPathfindingResult::Failed_Unreachable:
			ErrorMessage = FString::Printf(
				TEXT("Goal %s is unreachable from Start %s."), *InRequest.EndIndex.ToString(),
				*InRequest.StartIndex.ToString());
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

bool UJPSPlusStrategy::ShouldRepathForOptimization(const FIntVector& CurrentAgentVoxel,
	const TArray<FIntVector>& CurrentPathVoxels, int32 CurrentPathIndex, const FIntVector& GoalVoxel,
	uint32 AgentRadiusVoxels) const
{
	return Super::ShouldRepathForOptimization(CurrentAgentVoxel, CurrentPathVoxels, CurrentPathIndex, GoalVoxel,
	                                          AgentRadiusVoxels);
}

FName UJPSPlusStrategy::GetStrategyName() const
{
	return "JPSPlusStrategy";
}


// =======================================================================
// FJPSPlusPathfinder: 순수 C++ '일꾼' 클래스 구현
// =======================================================================

FJPSPlusPathfinder::FJPSPlusPathfinder(const FPathfindingRequest& InRequest,
										const FThreadSafeBitmaskGrid& InGrid,
                                       const TArray<FIntVector>& InAllDirections,
                                       const TMap<FIntVector, int32>& InDirectionToIndexMap,
                                       FIntVector InActualGoal,
                                        TNodeMemoryPool<FPathNode>& InMemoryPool)
	: Request(InRequest)
	  , WalkabilityGrid(InGrid)
	  , AllDirections(InAllDirections)
	  , DirectionToIndexMap(InDirectionToIndexMap)
	  , ActualGoalIndex(InActualGoal)
	  , MemoryPool(InMemoryPool)
	  , AgentRadiusInVoxels(InRequest.AgentRadiusInVoxels)
{
}

EPathfindingResult FJPSPlusPathfinder::FindPath(TArray<FIntVector>& OutPath)
{
	SCOPE_LOG_TIME_FUNC()

	MemoryPool.Reset();
	OpenSet.Empty();
	OpenSetLookup.Empty();
	ClosedSet.Empty();

	FPathNode* StartNode = MemoryPool.NewNode(Request.StartIndex);
	if (!StartNode) return EPathfindingResult::Failed_OutOfMemory;


	StartNode->HCost = CalculateHeuristic(Request.StartIndex, ActualGoalIndex);
	StartNode->CalculateAndSetFCost();

	OpenSet.HeapPush(StartNode, FPathNodePtrCompare());
	OpenSetLookup.Add(Request.StartIndex, StartNode);

	FPathNode* GoalNode = nullptr;
	FPathNode* BestPartialNode = nullptr;
	int64 MinHeuristic = MAX_int64;

	while (OpenSet.Num() > 0)
	{
		FPathNode* CurrentNode;
		OpenSet.HeapPop(CurrentNode, FPathNodePtrCompare());
		OpenSetLookup.Remove(CurrentNode->Position);
		ClosedSet.Add(CurrentNode->Position, CurrentNode);

		if (CurrentNode->HCost < MinHeuristic)
		{
			if (CurrentNode->Parent)
			{
				if (IsAgentMoveFeasible_ThreadSafe(CurrentNode->Parent->Position, CurrentNode->Position))
				{
					MinHeuristic = CurrentNode->HCost;
					BestPartialNode = CurrentNode;
				}
			}
			else
			{
				MinHeuristic = CurrentNode->HCost;
				BestPartialNode = CurrentNode;
			}
		}

		if (CurrentNode->Position == ActualGoalIndex)
		{
			GoalNode = CurrentNode;
			break;
		}

		TArray<FPathNode*> Successors;
		IdentifySuccessors(CurrentNode, Successors);

		for (const auto& Successor : Successors)
		{
			if (ClosedSet.Contains(Successor->Position)) continue;

			const int64 NewGCost = CurrentNode->GCost + GetMoveCost(CurrentNode->Position, Successor->Position);

			FPathNode** ExistingNodePtr = OpenSetLookup.Find(Successor->Position);
			if (ExistingNodePtr)
			{
				if (NewGCost < (*ExistingNodePtr)->GCost)
				{
					(*ExistingNodePtr)->SetParent(CurrentNode, NewGCost, (*ExistingNodePtr)->HCost);
				}
			}
			else
			{
				Successor->HCost = CalculateHeuristic(Successor->Position, ActualGoalIndex);
				Successor->SetParent(CurrentNode, NewGCost, Successor->HCost);
				OpenSet.HeapPush(Successor, FPathNodePtrCompare());
				OpenSetLookup.Add(Successor->Position, Successor);
			}
		}
	}

	OutPath.Empty();

	if (GoalNode)
	{
		OptimizePathFromJumpPoints(GoalNode, OutPath);
		return EPathfindingResult::Success;
	}
	if (BestPartialNode)
	{
		OptimizePathFromJumpPoints(BestPartialNode, OutPath);
		return EPathfindingResult::Success_Partial;
	}

	return EPathfindingResult::Failed_Unreachable;
}

void FJPSPlusPathfinder::IdentifySuccessors(FPathNode* Node, TArray<FPathNode*>& OutSuccessors)
{
    OutSuccessors.Empty();
    const FIntVector CurrentPos = Node->Position;

    // 1. 부모 점프 포인트로부터 온 방향을 계산합니다. 시작 노드는 부모가 없습니다.
    const FIntVector ParentPos = Node->Parent ? Node->Parent->Position : CurrentPos;
    const FIntVector ParentDir = Node->Parent ? FStrategyUtils::GetNormalizedDirection(ParentPos, CurrentPos) : FIntVector::ZeroValue;

    // 2. 현재 위치에서 점프를 시작할 '자연스러운 방향'만 가져옵니다.
    //    (강제 이웃을 여기서 미리 찾으려고 할 필요가 없습니다. 그건 Jump의 역할입니다.)
    TArray<FIntVector> DirectionsToJump;
    GetPrunedDirections(ParentDir, DirectionsToJump);

    // 3. 각 방향으로 Jump를 실행하여 후계자(다음 점프 포인트)를 찾습니다.
    TArray<FIntVector> AllJumpPoints;
    FCriticalSection JumpPointMutex;

    ParallelFor(DirectionsToJump.Num(),
        [this, &AllJumpPoints, &JumpPointMutex, &DirectionsToJump, CurrentPos](int32 Index)
        {
            const FIntVector& Dir = DirectionsToJump[Index];

            // Jump를 시작할 때, 첫 스텝의 부모는 '자기 자신(CurrentPos)'입니다.
            // 이렇게 해야 Jump 함수 내부의 JumperPrevPos가 올바르게 초기화됩니다.
            TOptional<FIntVector> JP = Jump(CurrentPos, Dir, CurrentPos);

            if (JP.IsSet())
            {
                FScopeLock Lock(&JumpPointMutex);
                AllJumpPoints.Add(JP.GetValue());
            }
        });

    // 4. 찾은 점프 포인트들을 후계자 노드로 생성합니다.
    OutSuccessors.Reserve(AllJumpPoints.Num());
    for (const FIntVector& P : AllJumpPoints)
    {
        if (FPathNode* NewNode = MemoryPool.NewNode(P))
        {
            OutSuccessors.Add(NewNode);
        }
    }
}

TOptional<FIntVector> FJPSPlusPathfinder::Jump(const FIntVector& StartPos, const FIntVector& Direction, const FIntVector& ParentOfStartPos, float AccumulatedPenalty)
{
    FIntVector CurrentPos = StartPos;
    FIntVector PrevPos = ParentOfStartPos;

	const int hVal      = CalculateHeuristic(StartPos, ActualGoalIndex);
	const float L       = (1.2f - 1.0f) * hVal;  // 논문: L=(t-1)*h(n) :contentReference[oaicite:0]{index=0}
	//이 로직을 사용한다면, 1.2는 변수로 Property로 뺄것.

	auto recDir = GetRecommendedDirection(Direction, CurrentPos);
	const int sigma  = CalcPenalty(ParentOfStartPos, StartPos, recDir);
	AccumulatedPenalty+= sigma;
	if (AccumulatedPenalty > L)
	{
		// 페널티 초과 → 현재 위치를 pseudo-jump point로 반환 :contentReference[oaicite:1]{index=1}
		return StartPos;
	}
	
	if (HasLineOfSight(CurrentPos, ActualGoalIndex))
	{
		return ActualGoalIndex;
	}

	while (true)
    {
        // 1. 다음 위치로 이동 가능한지 먼저 확인
        const FIntVector NextPos = CurrentPos + Direction;
        if (!WalkabilityGrid.IsIndexValid(NextPos) || !IsAgentMoveFeasible_ThreadSafe(CurrentPos, NextPos))
        {
            return {}; // 이동 불가, 점프 실패
        }

        // 2. 위치 갱신 (순서가 매우 중요!)
        //    먼저 현재 위치를 이전 위치로 기록하고, 그 다음에 현재 위치를 다음 칸으로 이동시킵니다.
        PrevPos = CurrentPos;
        CurrentPos = NextPos;

        // 3. 목표에 도달했는지 확인
        if (CurrentPos == ActualGoalIndex)
        {
            return CurrentPos;
        }
    	
        // 4. 새 위치에서 강제 이웃이 있는지 확인 (이제 PrevPos는 올바른 '바로 이전 칸'입니다)
        for (const auto& CheckDir : AllDirections)
        {
            if (CheckDir == Direction || CheckDir == Direction * -1) continue;
            if (HasForcedNeighbour(CurrentPos, CheckDir, PrevPos))
            {
                return CurrentPos; // 강제 이웃 발견, 현재 위치가 새로운 점프 포인트임
            }
        }
    	
        // 5. (필수) 대각선 이동의 정확성을 위한 재귀 탐색 (정찰)
        const int32 dx = Direction.X, dy = Direction.Y, dz = Direction.Z;	

        if (dx != 0 && dy != 0 && dz != 0) // 3D
        {
        	if (Jump(CurrentPos, FIntVector(dx, 0, 0), CurrentPos).IsSet() ||
			   Jump(CurrentPos, FIntVector(0, dy, 0), CurrentPos).IsSet() ||
			   Jump(CurrentPos, FIntVector(0, 0, dz), CurrentPos).IsSet())
        	{
        		return CurrentPos;
        	}
            if (Jump(CurrentPos, FIntVector(dx, dy, 0), CurrentPos).IsSet() ||
                Jump(CurrentPos, FIntVector(dx, 0, dz), CurrentPos).IsSet() ||
                Jump(CurrentPos, FIntVector(0, dy, dz), CurrentPos).IsSet())
            {
                return CurrentPos;
            }
        }
        else if (dx != 0 && dy != 0) // 2D XY
        {
            if (Jump(CurrentPos, FIntVector(dx, 0, 0), CurrentPos).IsSet() ||
                Jump(CurrentPos, FIntVector(0, dy, 0), CurrentPos).IsSet())
            {
                return CurrentPos;
            }
        }
        else if (dx != 0 && dz != 0) // 2D XZ
        {
            if (Jump(CurrentPos, FIntVector(dx, 0, 0), CurrentPos).IsSet() ||
                Jump(CurrentPos, FIntVector(0, 0, dz), CurrentPos).IsSet())
            {
                return CurrentPos;
            }
        }
        else if (dy != 0 && dz != 0) // 2D YZ
        {
            if (Jump(CurrentPos, FIntVector(0, dy, 0), CurrentPos).IsSet() ||
                Jump(CurrentPos, FIntVector(0, 0, dz), CurrentPos).IsSet())
            {
                return CurrentPos;
            }
        }
    	
    }
	
	return {};
}

void FJPSPlusPathfinder::GetPrunedDirections(const FIntVector& Dir, TArray<FIntVector>& OutDirections) const
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
		OutDirections.Add(FIntVector(dx, 0, 0));
		OutDirections.Add(FIntVector(0, dy, 0));
		OutDirections.Add(FIntVector(0, 0, dz));
		OutDirections.Add(FIntVector(dx, dy, 0));
		OutDirections.Add(FIntVector(dx, 0, dz));
		OutDirections.Add(FIntVector(0, dy, dz));
	}
	else if (dx != 0 && dy != 0)
	{
		OutDirections.Add(FIntVector(dx, 0, 0));
		OutDirections.Add(FIntVector(0, dy, 0));
	} // 2D XY
	else if (dx != 0 && dz != 0)
	{
		OutDirections.Add(FIntVector(dx, 0, 0));
		OutDirections.Add(FIntVector(0, 0, dz));
	} // 2D XZ
	else if (dy != 0 && dz != 0)
	{
		OutDirections.Add(FIntVector(0, dy, 0));
		OutDirections.Add(FIntVector(0, 0, dz));
	}
}

void FJPSPlusPathfinder::OptimizePathFromJumpPoints(FPathNode* EndNode, TArray<FIntVector>& OutOptimizedPath) const
{
	OutOptimizedPath.Empty();

	if (!EndNode)
	{
		return;
	}

	// 1. EndNode에서 시작하여 부모를 따라가며 Jump Point 목록을 생성합니다.
	TArray<FIntVector> JumpPoints;
	FPathNode* CurrentNode = EndNode;

	while (CurrentNode)
	{
		JumpPoints.Add(CurrentNode->Position);
		CurrentNode = CurrentNode->Parent;
	}

	Algo::Reverse(JumpPoints);

	if (JumpPoints.Num() < 2)
	{
		OutOptimizedPath = JumpPoints;
		return;
	}

	// 3. Jump Point 목록을 직접 순회하며 Theta* 스타일 최적화를 수행합니다.
	//    (기존 OptimizePath_ThetaStarStyle 로직을 GridPath가 아닌 JumpPoints에 적용)
	OutOptimizedPath.Add(JumpPoints[0]);
	int32 CurrentIndex = 0;

	while (CurrentIndex < JumpPoints.Num() - 1)
	{
		int32 LookaheadIndex = JumpPoints.Num() - 1;
		// 경로의 끝에서부터 거꾸로 오면서 현재 지점(CurrentIndex)과
		// 시야(LOS)가 확보되는 가장 먼 Jump Point를 찾습니다.
		while (LookaheadIndex > CurrentIndex + 1)
		{
			if (HasLineOfSight(JumpPoints[CurrentIndex], JumpPoints[LookaheadIndex]))
			{
				// 시야가 확보되면 더 이상 앞으로 갈 필요가 없습니다.
				break;
			}
			LookaheadIndex--;
		}

		// 찾은 가장 먼 점을 최종 경로에 추가하고, 그 위치에서 다시 탐색을 시작합니다.
		OutOptimizedPath.Add(JumpPoints[LookaheadIndex]);
		CurrentIndex = LookaheadIndex;
	}
}

bool FJPSPlusPathfinder::HasLineOfSight(const FIntVector& From, const FIntVector& To) const
{
	const FIntVector AgentRadiusVector(AgentRadiusInVoxels);

	// 시작점과 끝점에서의 에이전트 바운딩 박스
	const FIntVector FromMin = From - AgentRadiusVector;
	const FIntVector FromMax = From + AgentRadiusVector;
	const FIntVector ToMin = To - AgentRadiusVector;
	const FIntVector ToMax = To + AgentRadiusVector;

	// 두 박스를 모두 포함하는 스윕된(Swept) 바운딩 박스
	const FIntVector SweptMin(FMath::Min(FromMin.X, ToMin.X), FMath::Min(FromMin.Y, ToMin.Y),
	                          FMath::Min(FromMin.Z, ToMin.Z));
	const FIntVector SweptMax(FMath::Max(FromMax.X, ToMax.X), FMath::Max(FromMax.Y, ToMax.Y),
	                          FMath::Max(FromMax.Z, ToMax.Z));

	// 이 거대한 박스 안에 장애물이 하나라도 있다면, 시야는 즉시 차단된 것으로 간주합니다.
	// 이것은 매우 빠르고 효율적인 1차 필터링입니다.
	if (!WalkabilityGrid.IsRegionEmpty(SweptMin, SweptMax))
	{
		return false;
	}


	// 1. From에서 To까지 중심선을 따라 복셀 목록을 생성합니다.
	TArray<FIntVector> CenterlineVoxels;
	FStrategyUtils::GenerateBresenhamLine(From, To, CenterlineVoxels);

	// 경로가 없거나 점이 하나뿐이면 시야가 확보된 것으로 간주합니다.
	if (CenterlineVoxels.Num() < 2)
	{
		return true;
	}

	for (int32 i = 0; i < CenterlineVoxels.Num() - 1; ++i)
	{
		const FIntVector& CurrentStep = CenterlineVoxels[i];
		const FIntVector& NextStep = CenterlineVoxels[i + 1];

		if (!IsAgentMoveFeasible_ThreadSafe(CurrentStep, NextStep))
		{
			return false;
		}
	}
	return true;
}

bool FJPSPlusPathfinder::IsAgentMoveFeasible_ThreadSafe(const FIntVector& From, const FIntVector& To) const
{
	const FIntVector AgentRadiusVector(AgentRadiusInVoxels);
	if (!WalkabilityGrid.IsRegionEmpty(To - AgentRadiusVector, To + AgentRadiusVector))
	{
		return false;
	}

	// 2. 코너 커팅 방지 로직 (원본 로직을 스레드 안전 그리드로 재구성)
	const FIntVector Delta = To - From;
	const int32 MoveDimension = FMath::Abs(Delta.X) + FMath::Abs(Delta.Y) + FMath::Abs(Delta.Z);

	if (MoveDimension > 1) // 대각선 이동일 경우
	{
		if (Delta.X != 0 && Delta.Y != 0 && Delta.Z != 0)
		{
			// 3D에서는 더 많은 코너 조합을 검사해야 합니다.
			// 가장 간단한 방법은 3개의 2D 평면 검사를 모두 통과해야 하는 것으로 처리하는 것입니다.
			const FIntVector CornerX = From + FIntVector(Delta.X, 0, 0);
			const FIntVector CornerY = From + FIntVector(0, Delta.Y, 0);
			const FIntVector CornerZ = From + FIntVector(0, 0, Delta.Z);

			// XY 평면 코너 검사
			if (!WalkabilityGrid.IsRegionEmpty(CornerX - AgentRadiusVector, CornerX + AgentRadiusVector)
				&& !WalkabilityGrid.IsRegionEmpty(CornerY - AgentRadiusVector, CornerY + AgentRadiusVector))
				return false;
			// XZ 평면 코너 검사
			if (!WalkabilityGrid.IsRegionEmpty(CornerX - AgentRadiusVector, CornerX + AgentRadiusVector)
				&& !WalkabilityGrid.IsRegionEmpty(CornerZ - AgentRadiusVector, CornerZ + AgentRadiusVector))
				return false;
			// YZ 평면 코너 검사
			if (!WalkabilityGrid.IsRegionEmpty(CornerY - AgentRadiusVector, CornerY + AgentRadiusVector)
				&& !WalkabilityGrid.IsRegionEmpty(CornerZ - AgentRadiusVector, CornerZ + AgentRadiusVector))
				return false;
		}
		// 2D XY 평면 코너 검사
		else if (Delta.X != 0 && Delta.Y != 0 && Delta.Z == 0)
		{
			const FIntVector Corner1 = From + FIntVector(Delta.X, 0, 0);
			const FIntVector Corner2 = From + FIntVector(0, Delta.Y, 0);
			// 두 코너가 모두 막혀있으면 통과 불가
			if (!WalkabilityGrid.IsRegionEmpty(Corner1 - AgentRadiusVector, Corner1 + AgentRadiusVector) &&
				!WalkabilityGrid.IsRegionEmpty(Corner2 - AgentRadiusVector, Corner2 + AgentRadiusVector))
			{
				return false;
			}
		}


		// 2D 대각선 이동 (XZ 평면)
		else if (Delta.X != 0 && Delta.Z != 0 && Delta.Y == 0)
		{
			const FIntVector Corner1 = From + FIntVector(Delta.X, 0, 0);
			const FIntVector Corner2 = From + FIntVector(0, 0, Delta.Z);
			// 두 코너가 모두 막혀있으면 통과 불가
			if (!WalkabilityGrid.IsRegionEmpty(Corner1 - AgentRadiusVector, Corner1 + AgentRadiusVector) &&
				!WalkabilityGrid.IsRegionEmpty(Corner2 - AgentRadiusVector, Corner2 + AgentRadiusVector))
			{
				return false;
			}
		}
		// 2D 대각선 이동 (YZ 평면)
		else if (Delta.Y != 0 && Delta.Z != 0 && Delta.X == 0)
		{
			const FIntVector Corner1 = From + FIntVector(0, Delta.Y, 0);
			const FIntVector Corner2 = From + FIntVector(0, 0, Delta.Z);
			// 두 코너가 모두 막혀있으면 통과 불가
			if (!WalkabilityGrid.IsRegionEmpty(Corner1 - AgentRadiusVector, Corner1 + AgentRadiusVector) &&
				!WalkabilityGrid.IsRegionEmpty(Corner2 - AgentRadiusVector, Corner2 + AgentRadiusVector))
			{
				return false;
			}
		}
	}

	// 3. 이동 경로 전체에 대한 스윕(Sweep) 검사 (선택적이지만 정확도를 높임)
	// From에서 To로 이동할 때 에이전트가 차지하는 전체 공간에 장애물이 없는지 확인합니다.
	const FIntVector FromMin = From - AgentRadiusVector;
	const FIntVector FromMax = From + AgentRadiusVector;
	const FIntVector ToMin = To - AgentRadiusVector;
	const FIntVector ToMax = To + AgentRadiusVector;

	const FIntVector SweptMin(FMath::Min(FromMin.X, ToMin.X), FMath::Min(FromMin.Y, ToMin.Y),
	                          FMath::Min(FromMin.Z, ToMin.Z));
	const FIntVector SweptMax(FMath::Max(FromMax.X, ToMax.X), FMath::Max(FromMax.Y, ToMax.Y),
	                          FMath::Max(FromMax.Z, ToMax.Z));

	// 스윕된 영역이 비어있는지 최종 확인
	return WalkabilityGrid.IsRegionEmpty(SweptMin, SweptMax);
}

bool FJPSPlusPathfinder::HasForcedNeighbour(const FIntVector& NodePos, const FIntVector& CheckDir,
                                            const FIntVector& JumperPrevPos) const
{
	const FIntVector CorneringTargetPos = NodePos + CheckDir;

	if (!IsAgentMoveFeasible_ThreadSafe(JumperPrevPos, CorneringTargetPos))
	{
		if (IsAgentMoveFeasible_ThreadSafe(NodePos, CorneringTargetPos))
		{
			return true;
		}
	}

	// 그 외의 모든 경우는 강제 이웃이 아닙니다.
	return false;
}

uint64 FJPSPlusPathfinder::CalculateForcedNeighborMask(const FNeighborWalkability& Neighbors,
                                                       int32 ParentDirIndex) const
{
	uint64 ForcedMask = 0;
	const FIntVector& ParentDir = AllDirections[ParentDirIndex];
	const int32 pdx = ParentDir.X, pdy = ParentDir.Y, pdz = ParentDir.Z;

	// 모든 방향을 순회하며 검사 (이 루프는 Jump 함수 밖에서 한 번만 실행됨)
	for (int32 CheckDirIndex = 0; CheckDirIndex < AllDirections.Num(); ++CheckDirIndex)
	{
		// 1. 해당 방향으로 직선 이동이 불가능하면 후보가 아님
		if (!Neighbors.IsWalkable(CheckDirIndex)) continue;

		// 2. 코너를 막는 장애물이 있는지 검사
		const FIntVector& CheckDir = AllDirections[CheckDirIndex];
		const int32 cdx = CheckDir.X, cdy = CheckDir.Y, cdz = CheckDir.Z;

		bool bIsForced = false;
		if (pdx != 0 && (cdy != 0 || cdz != 0)) // X축 이동 중
		{
			if (cdy != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({pdx, -cdy, 0}))) bIsForced = true;
			if (cdz != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({pdx, 0, -cdz}))) bIsForced = true;
		}
		if (pdy != 0 && (cdx != 0 || cdz != 0)) // Y축 이동 중
		{
			if (cdx != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({-cdx, pdy, 0}))) bIsForced = true;
			if (cdz != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({0, pdy, -cdz}))) bIsForced = true;
		}
		if (pdz != 0 && (cdx != 0 || cdy != 0)) // Z축 이동 중
		{
			if (cdx != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({-cdx, 0, pdz}))) bIsForced = true;
			if (cdy != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({0, -cdy, pdz}))) bIsForced = true;
		}

		if (bIsForced)
		{
			ForcedMask |= (1ULL << CheckDirIndex);
		}
	}
	return ForcedMask;
}

int32 FJPSPlusPathfinder::CalculateHeuristic(const FIntVector& A, const FIntVector& B) const
{
	return FStrategyUtils::Heuristic_Octile(A, B);
}

int32 FJPSPlusPathfinder::GetMoveCost(const FIntVector& A, const FIntVector& B) const
{
	return FStrategyUtils::Heuristic_Octile(A, B);
}

FIntVector FJPSPlusPathfinder::GetRecommendedDirection(
	const FIntVector& dir,
	const FIntVector& CurrentPos
) const {
	// 1) 대각선 분해 또는 그대로 반환
	TArray<FIntVector> candidates;
	if (dir.X != 0 && dir.Y != 0 && dir.Z != 0) {
		candidates = { {dir.X,0,0}, {0,dir.Y,0}, {0,0,dir.Z} };
	}
	else if (dir.X != 0 && dir.Y != 0) {
		candidates = { {dir.X,0,0}, {0,dir.Y,0} };
	}
	else if (dir.X != 0 && dir.Z != 0) {
		candidates = { {dir.X,0,0}, {0,0,dir.Z} };
	}
	else if (dir.Y != 0 && dir.Z != 0) {
		candidates = { {0,dir.Y,0}, {0,0,dir.Z} };
	}
	else {
		// 이미 축 이동이면 그대로 추천
		return dir;
	}

	// 2) 목표까지 휴리스틱이 최소인 방향 선택
	FIntVector bestDir = candidates[0];
	int bestH = TNumericLimits<int>::Max();
	for (const auto& cand : candidates) {
		FIntVector nextPos = CurrentPos + cand;
		int h = CalculateHeuristic(nextPos, ActualGoalIndex);
		if (h < bestH) {
			bestH = h;
			bestDir = cand;
		}
	}
	return bestDir;
}

int FJPSPlusPathfinder::CalcPenalty(const FIntVector& from, const FIntVector& to, const FIntVector& recDir)
{
	const int h_xn = CalculateHeuristic(from, to);
	const int h_nr = CalculateHeuristic(to, ActualGoalIndex);
	const int h_xr = CalculateHeuristic(from, ActualGoalIndex);
	return h_xn + h_nr - h_xr;
}
