// JPSStrategy.cpp
#include "Pathfinder/Strategy/JPSPlusStrategy.h"
#include "Pathfinder/Utility/StrategyUtils.h"
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
	if (InRequest.WalkabilityGrid.IsRegionWalkable(InRequest.EndIndex, InRequest.AgentRadiusInVoxels))
	{
		FIntVector NearestWalkableVoxel;
		if (InRequest.WalkabilityGrid.FindNearestWalkableVoxel_BFS(InRequest.EndIndex, InRequest.AgentRadiusInVoxels,50, NearestWalkableVoxel))
		{
			ActualGoalIndex = NearestWalkableVoxel;
		}
	}
	// 3. '일꾼' 객체 생성 및 작업 위임
	FJPSPlusPathfinder Pathfinder(InRequest, AllDirections, DirectionToIndexMap, ActualGoalIndex);

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

bool UJPSPlusStrategy::ShouldRepathForOptimization(const FVector& CurrentAgentLocation,
                                                   const TArray<FVector>& CurrentPath, int32 CurrentPathIndex,
                                                   const FVector& GoalLocation, uint32 AgentRadius) const
{
	return Super::ShouldRepathForOptimization(CurrentAgentLocation, CurrentPath, CurrentPathIndex, GoalLocation,
	                                          AgentRadius);
}

FName UJPSPlusStrategy::GetStrategyName() const
{
	return "JPSPlusStrategy";
}


// =======================================================================
// FJPSPlusPathfinder: 순수 C++ '일꾼' 클래스 구현
// =======================================================================
thread_local static FPathNodeMemoryPool ThreadLocalPool(1024);

FJPSPlusPathfinder::FJPSPlusPathfinder(const FPathfindingRequest& InRequest,
									   const TArray<FIntVector>& InAllDirections,
									   const TMap<FIntVector, int32>& InDirectionToIndexMap,
									   FIntVector InActualGoal)
  : Request(InRequest)
  , WalkabilityGrid(InRequest.WalkabilityGrid)
  , AllDirections(InAllDirections)
  , DirectionToIndexMap(InDirectionToIndexMap)
  , ActualGoalIndex(InActualGoal)
  , AgentRadiusInVoxels(InRequest.AgentRadiusInVoxels)
{
}

EPathfindingResult FJPSPlusPathfinder::FindPath(TArray<FIntVector>& OutPath)
{
	SCOPE_LOG_TIME_FUNC()
	
	ThreadLocalPool.Reset();
	OpenSet.Empty();
	OpenSetLookup.Empty();
	ClosedSet.Empty();

	FPathNode* StartNode = ThreadLocalPool.NewNode(Request.StartIndex);
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
	const FPathNode* CurrentNode = Node;

	const FIntVector ParentDir = CurrentNode->Parent
		                             ? FStrategyUtils::GetNormalizedDirection(Node->Parent->Position, Node->Position)
		                             : FIntVector::ZeroValue;

	// ===================================================================================
	// --- 1. 정보 사전 계산 (The Pre-computation Step) ---
	// 강제 이웃 판단에 필요한 모든 정보를 여기서 한 번에 계산합니다.
	// ===================================================================================

	// 정보 1: 현재 노드에서 각 방향으로의 '직선 이동' 가능 여부
	bool bIsStraightMoveWalkable[NUM_ALL_DIRECTIONS];
	for (int32 i = 0; i < AllDirections.Num(); ++i)
	{
		bIsStraightMoveWalkable[i] = IsAgentMoveFeasible_ThreadSafe(CurrentNode->Position,
		                                                            CurrentNode->Position + AllDirections[i]);
	}

	// 정보 2: 부모 노드에서 각 방향으로의 '코너링 이동' 가능 여부
	bool bIsCorneringMoveWalkable[NUM_ALL_DIRECTIONS] = {false};
	if (CurrentNode->Parent)
	{
		for (int32 i = 0; i < AllDirections.Num(); ++i)
		{
			// ★★★ 수정: 스레드 안전 헬퍼 함수 사용 ★★★
			bIsCorneringMoveWalkable[i] = IsAgentMoveFeasible_ThreadSafe(
				CurrentNode->Parent->Position, CurrentNode->Position + AllDirections[i]);
		}
	}

	// ===================================================================================
	// --- 2. 후계자(Successor) 탐색 ---
	// ===================================================================================
	TSet<FIntVector> DirsToSearch;

	// 2a. 자연스러운 이웃 추가: GetPrunedDirections가 제안하는 방향 중, 실제로 이동 가능한 곳만 추가합니다.
	TArray<FIntVector> NaturalDirs;
	GetPrunedDirections(ParentDir, NaturalDirs);
	for (const FIntVector& Dir : NaturalDirs)
	{
		const int32 DirIndex = DirectionToIndexMap.FindRef(Dir);
		if (bIsStraightMoveWalkable[DirIndex])
		{
			DirsToSearch.Add(Dir);
		}
	}

	// 2b. 강제 이웃 추가: 사전 계산된 정보로 초고속 판단
	if (Node->Parent)
	{
		for (int32 i = 0; i < AllDirections.Num(); ++i)
		{
			// 새로운 판단 로직:
			// 1. 직선 이동은 가능하고 (bIsStraightMoveWalkable == true)
			// 2. 코너링 이동은 불가능하다면 (bIsCorneringMoveWalkable == false)
			// => 이것이 바로 정확한 강제 이웃입니다.
			if (bIsStraightMoveWalkable[i] && !bIsCorneringMoveWalkable[i])
			{
				DirsToSearch.Add(AllDirections[i]);
			}
		}
	}

	// ===================================================================================
	// --- 3. 병렬로 Jump 실행 ---
	// ===================================================================================
	TArray<FIntVector> AllJumpPoints;
	FCriticalSection JumpPointMutex;

	const FIntVector NodePos = Node->Position;

	const TArray<FIntVector> DirsToSearchArray = DirsToSearch.Array();

	ParallelFor(DirsToSearchArray.Num(),
	            [this, &AllJumpPoints, &JumpPointMutex, &DirsToSearchArray, NodePos](int32 Index)
	            {
		            const FIntVector& Dir = DirsToSearchArray[Index];

		            // 스레드 안전 Jump 함수를 호출합니다.
		            TOptional<FIntVector> JumpPoint = Jump(NodePos, Dir);

		            if (JumpPoint.IsSet())
		            {
			            // 여러 스레드에서 동시에 배열에 쓰지 못하도록 반드시 잠가야 합니다.
			            FScopeLock Lock(&JumpPointMutex);
			            AllJumpPoints.Add(JumpPoint.GetValue());
		            }
	            });

	// --- FIX: Process the results from the single, thread-safe array. ---
	OutSuccessors.Reserve(AllJumpPoints.Num());

	for (const FIntVector& Point : AllJumpPoints)
	{
		// NodePool access is now safely back on the main thread.
		FPathNode* node = ThreadLocalPool.NewNode(Point);
		if (node)
		{
			OutSuccessors.Add(node);
		}
	}
}

TOptional<FIntVector> FJPSPlusPathfinder::Jump(const FIntVector& StartPos, const FIntVector& Direction)
{
	FIntVector CurrentPos = StartPos;
	if (HasLineOfSight(CurrentPos, ActualGoalIndex))
	{
		return ActualGoalIndex;
	}

	while (true)
	{
		const FIntVector NextPos = CurrentPos + Direction;

		if (!WalkabilityGrid.IsIndexValid(NextPos))
			return {};  // 맵 경계를 벗어나면 즉시 실패 처리
		
		if (!IsAgentMoveFeasible_ThreadSafe(CurrentPos, NextPos))
		{
			return {};
		}

		// 안전성이 확인되었으므로, 상태를 다음 위치로 전진시킵니다.
		CurrentPos = NextPos;

		// --- 2. 목표 도달 검사 ---
		// 현재 위치가 목표 지점이라면, 최고의 점프 포인트를 찾은 것이므로 즉시 반환합니다.
		if (CurrentPos == ActualGoalIndex)
		{
			return CurrentPos;
		}

		// // --- 3. 강제 이웃 검사 ---
		// 현재 위치(CurrentPos)를 기준으로 강제 이웃이 있는지 확인합니다.
		// 이 검사는 점프의 방향을 바꿔야 할 '코너'를 찾는 역할을 합니다.
		for (const auto& CheckDir : AllDirections)
		{
			if (CheckDir == Direction || CheckDir == (Direction * -1))
				continue;
			// 부모 방향(Direction)을 전달하여, 현재 진행 방향을 기준으로 한 강제 이웃을 찾습니다.
			if (HasForcedNeighbour(CurrentPos, CheckDir, Direction))
			{
				// 강제 이웃이 발견되면, 현재 위치는 중요한 분기점이므로 점프 포인트가 됩니다.
				return CurrentPos;
			}
		}
		
		
		const FIntVector DirToGoal = ActualGoalIndex - CurrentPos;
		const int64 DistSqToGoal = (int64)DirToGoal.X * DirToGoal.X + (int64)DirToGoal.Y * DirToGoal.Y + (int64)
			DirToGoal.Z * DirToGoal.Z;

		// 현재 위치에서 다음 스텝까지의 거리 제곱을 직접 계산합니다. (Direction 벡터의 길이 제곱)
		const int64 DistSqToNext = (int64)Direction.X * Direction.X + (int64)Direction.Y * Direction.Y + (int64)
			Direction.Z * Direction.Z;

		if (DistSqToNext > DistSqToGoal)
		{
			// 다음 스텝이 목표보다 멀리 있다면, 목표를 지나친 것입니다.
			// 이 경우, 현재 위치에서 목표까지 직접 갈 수 있는지 최종 확인합니다.
			if (IsAgentMoveFeasible_ThreadSafe(CurrentPos, ActualGoalIndex))
			{
				return ActualGoalIndex;
			}
		}

		FNeighborWalkability Walkability;
		const FIntVector AgentRadiusVector(AgentRadiusInVoxels);
		for (int32 i = 0; i < AllDirections.Num(); ++i)
		{
			const FIntVector NeighborPos = CurrentPos + AllDirections[i];
		
			if (WalkabilityGrid.IsRegionEmpty(NeighborPos - AgentRadiusVector, NeighborPos + AgentRadiusVector))
			{
				Walkability.WalkableMask |= (1 << i);
			}
		}

		const int32 ParentDirIndex = DirectionToIndexMap.FindRef(Direction);
		uint64 ForcedNeighborCandidateMask = CalculateForcedNeighborMask(Walkability, ParentDirIndex);

		// 2c. CTZ를 사용해 강제 이웃 후보들을 빠르게 순회
		while (ForcedNeighborCandidateMask != 0)
		{
			unsigned long Index;
			// 가장 오른쪽에 있는 후보(1 비트)의 인덱스를 찾는다
			_BitScanForward64(&Index, ForcedNeighborCandidateMask); // CTZ 명령어

			return CurrentPos;

			// 만약 모든 강제 이웃을 다 찾아야 한다면, 처리한 비트를 끄고 루프를 계속합니다.
			// ForcedNeighborCandidateMask &= ForcedNeighborCandidateMask - 1;
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
		// 2D XY 평면 코너 검사
		if (Delta.X != 0 && Delta.Y != 0 && Delta.Z == 0)
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

		// 3D 대각선 이동
		else if (Delta.X != 0 && Delta.Y != 0 && Delta.Z != 0)
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
                                            const FIntVector& ParentDir) const
{
	if (ParentDir == FIntVector::ZeroValue)
	{
		return false;
	}

	const FIntVector ParentPos = NodePos - ParentDir;
	const FIntVector CorneringTargetPos = NodePos + CheckDir;

	if (!IsAgentMoveFeasible_ThreadSafe(ParentPos, CorneringTargetPos))
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