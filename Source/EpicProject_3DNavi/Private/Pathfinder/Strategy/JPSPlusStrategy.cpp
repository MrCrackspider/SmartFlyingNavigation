// JPSStrategy.cpp
#include "Pathfinder/Strategy/JPSPlusStrategy.h"
#include "Pathfinder/FenwickVoxelVolume.h"
#include "Pathfinder/Utility/StrategyUtils.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"


#include "Pathfinder/Utility/StrategyUtils.h"

void UJPSPlusStrategy::Initialize(AVoxelVolumeBase* InVolume)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(JPS_Initialize);
	
	Volume = Cast<AFenwickVoxelVolume>(InVolume);
	if (Volume.IsValid() == false)
	{
		UE_LOG(LogTemp, Error, TEXT("UJPSPlusStrategy: The volume is invalid, Need AUniformGridVolume!! "));
	}

	if (!NodePool.IsValid())
	{
		int64 VolumeNodeCount = Volume->MapSize.X * Volume->MapSize.Y * Volume->MapSize.Z;

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

	DirectionToIndexMap.Empty();
	for (int32 i = 0; i < AllDirections.Num(); ++i)
	{
		DirectionToIndexMap.Add(AllDirections[i], i);
	}
}

bool UJPSPlusStrategy::FindPathWithStrategy(const FIntVector& StartIndex, const FIntVector& EndIndex,
                                            int32 AgentRadiusInVoxel, TArray<FIntVector>& OutPath)
{
	if (!Volume.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("UJPSPlusStrategy: VoxelVolume is not valid. Pathfinding aborted."));
		OutPath.Empty();
		return false;
	}

	const EPathfindingResult Result = FindPathInternal(StartIndex, EndIndex, OutPath, AgentRadiusInVoxel);

	// 실패 원인에 대한 로그 기록 (디버깅 용이성)
	if (Result != EPathfindingResult::Success && Result != EPathfindingResult::Success_Partial)
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
			ErrorMessage = FString::Printf(
				TEXT("Goal %s is unreachable from Start %s."), *EndIndex.ToString(), *StartIndex.ToString());
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
	if (!Volume.IsValid() || CurrentPath.Num() < 2)
	{
		return false;
	}

	// --- Line of Sight 기반 지름길 검사 ---
	const int32 AgentRadiusInVoxels = FMath::RoundToInt(AgentRadius / Volume->VoxelExtent);


	const FIntVector StartIndex = Volume->WorldToVoxelIndex(CurrentAgentLocation);
	const FIntVector EndIndex = Volume->WorldToVoxelIndex(GoalLocation);

	if (HasLineOfSight(StartIndex, EndIndex, AgentRadiusInVoxels))
	{
		// 시야가 확보되었다면, 현재 경로가 이미 거의 직선인지 확인하여 불필요한 재탐색을 방지합니다.
		if (CurrentPathIndex >= CurrentPath.Num() - 1)
		{
			return false;
		}

		return true;
	}

	return false;
}

FName UJPSPlusStrategy::GetStrategyName() const
{
	return FName("JPS_3D_Robust_Complete");
}

EPathfindingResult UJPSPlusStrategy::FindPathInternal(const FIntVector& StartIndex, const FIntVector& EndIndex,
                                                      TArray<FIntVector>& OutPath, uint32 AgentRadiusInVoxels)
{
	SCOPE_LOG_TIME_FUNC()
	if (!Volume.IsValid())
	{
		return EPathfindingResult::Failed_InvalidStart;
	}

	MoveFeasibilityCache.Empty();
	NodePool->Reset();
	OpenSet.Empty();
	OpenSetLookup.Empty();
	ClosedSet.Empty();
	// OutPath.Empty();

	FPathNode* StartNode = NodePool->NewNode(StartIndex);
	StartNode->HCost = CalculateHeuristic(StartIndex, EndIndex);
	StartNode->CalculateAndSetFCost();

	OpenSet.HeapPush(StartNode, FPathNodePtrCompare());
	OpenSetLookup.Add(StartIndex, StartNode);

	FPathNode* GoalNode = nullptr;
	// --- 실시간 최적 노드 추적을 위한 변수 추가 ---
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
				if (IsAgentMoveFeasible(CurrentNode->Parent->Position, CurrentNode->Position, AgentRadiusInVoxels))
				{
					// 이동이 가능할 때만 최적 후보로 인정하고 업데이트.
					MinHeuristic = CurrentNode->HCost;
					BestPartialNode = CurrentNode;
				}
			}
			else
			{
				// 부모가 없는 경우 (시작 노드) 항상 유효.
				MinHeuristic = CurrentNode->HCost;
				BestPartialNode = CurrentNode;
			}
		}
		if (CurrentNode->Position == EndIndex)
		{
			GoalNode = CurrentNode;
			break;
		}
		
		TArray<FPathNode*> Successors;
		IdentifySuccessors(CurrentNode, EndIndex, AgentRadiusInVoxels, Successors);

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
				Successor->HCost = CalculateHeuristic(Successor->Position, EndIndex);
				Successor->SetParent(CurrentNode, NewGCost, Successor->HCost);
				OpenSet.HeapPush(Successor, FPathNodePtrCompare());
				OpenSetLookup.Add(Successor->Position, Successor);
			}
		}
	}

	//길찾기 끝나고 갱신 바로 전 초기화
	OutPath.Empty();
	
	if (GoalNode)
	{
		OptimizePathFromJumpPoints(GoalNode, OutPath, AgentRadiusInVoxels);
		return EPathfindingResult::Success;
	}
	else // 길찾기에 실패한 경우 (목표가 무효하거나 도달 불가)
	{
		UE_LOG(LogTemp, Warning, TEXT("Full path not found. Attempting to find a partial path..."));

		if (BestPartialNode)
		{
			GoalNode = BestPartialNode;

			// --- 스레드 안전성 강화 ---
			if (!Volume.IsValid()) return EPathfindingResult::Failed_Unreachable;
			OptimizePathFromJumpPoints(GoalNode, OutPath, AgentRadiusInVoxels);
			return EPathfindingResult::Success_Partial;
		}
	}

	return EPathfindingResult::Failed_Unreachable;
}

void UJPSPlusStrategy::GetPrunedDirections(const FIntVector& Dir, TArray<FIntVector>& OutDirections) const
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
	} // 2D YZ

	// 2. 강제된 이웃(Forced Neighbours) 추가
	// (HasForcedNeighbour 함수가 현재 노드 기준이므로, 여기서는 부모 방향을 전달하여 검사)
	// 이 부분은 HasForcedNeighbour 함수에서 처리하는 것이 더 직관적임.
	// IdentifySuccessors에서 현재 노드를 기준으로 모든 방향에 대해 HasForcedNeighbour를 검사하는 것이 더 명확한 구조.
	// 여기서는 자연스러운 이웃만 반환하고, IdentifySuccessors에서 강제 이웃을 추가하는 것으로 역할을 분리.
}

void UJPSPlusStrategy::IdentifySuccessors(FPathNode* Node, const FIntVector& Goal,
                                          int32 AgentRadiusInVoxels, TArray<FPathNode*>& OutSuccessors)
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
		bIsStraightMoveWalkable[i] = IsAgentMoveFeasible(CurrentNode->Position,
		                                                 CurrentNode->Position + AllDirections[i], AgentRadiusInVoxels);
	}

	// 정보 2: 부모 노드에서 각 방향으로의 '코너링 이동' 가능 여부
	bool bIsCorneringMoveWalkable[NUM_ALL_DIRECTIONS] = {false}; // 기본값 false로 초기화
	if (CurrentNode->Parent)
	{
		for (int32 i = 0; i < AllDirections.Num(); ++i)
		{
			bIsCorneringMoveWalkable[i] = IsAgentMoveFeasible(CurrentNode->Parent->Position,
			                                                  CurrentNode->Position + AllDirections[i],
			                                                  AgentRadiusInVoxels);
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

	TWeakObjectPtr<UJPSPlusStrategy> StrategyPtr(this);
	const FIntVector NodePos = Node->Position;

	const TArray<FIntVector> DirsToSearchArray = DirsToSearch.Array();

	ParallelFor(DirsToSearchArray.Num(),
	            [StrategyPtr, &AllJumpPoints, &JumpPointMutex, &DirsToSearchArray, NodePos, Goal, AgentRadiusInVoxels](
	            int32 Index)
	            {
		            if (!StrategyPtr.IsValid())
		            {
			            return;
		            }

		            const FIntVector& Dir = DirsToSearchArray[Index];

		            TOptional<FIntVector> JumpPoint = StrategyPtr->Jump(NodePos, Dir, Goal, AgentRadiusInVoxels);

		            if (JumpPoint.IsSet())
		            {
			            // --- FIX: Lock before modifying the shared array. ---
			            FScopeLock Lock(&JumpPointMutex);
			            AllJumpPoints.Add(JumpPoint.GetValue());
		            }
	            });

	if (!StrategyPtr.IsValid())
	{
		return;
	}

	// --- FIX: Process the results from the single, thread-safe array. ---
	OutSuccessors.Reserve(AllJumpPoints.Num());

	for (const FIntVector& Point : AllJumpPoints)
	{
		// NodePool access is now safely back on the main thread.
		OutSuccessors.Add(NodePool->NewNode(Point));
	}
}

TOptional<FIntVector> UJPSPlusStrategy::Jump(const FIntVector& StartPos, const FIntVector& Direction,
                                             const FIntVector& Goal, int32 AgentRadiusInVoxels)
{
	if (!Volume.IsValid())
	{
		return {}; // Volume이 사라졌으면 즉시 점프 실패
	}
	//const int32 MaxJumpDistance = Volume->GetDistanceToNextObstacle(StartPos, Direction);
	FIntVector CurrentPos = StartPos;
	if (HasLineOfSight(CurrentPos, Goal, AgentRadiusInVoxels))
	{
		return Goal;
	}
	for (int32 Step = 0; ; ++Step)
	{
		const FIntVector NextPos = CurrentPos + Direction;

		if (!IsAgentMoveFeasible(CurrentPos, NextPos, AgentRadiusInVoxels))
		{
			return {};
		}

		// 안전성이 확인되었으므로, 상태를 다음 위치로 전진시킵니다.
		CurrentPos = NextPos;

		// --- 2. 목표 도달 검사 ---
		// 현재 위치가 목표 지점이라면, 최고의 점프 포인트를 찾은 것이므로 즉시 반환합니다.
		if (CurrentPos == Goal)
		{
			return CurrentPos;
		}

		// --- 3. 강제 이웃 검사 ---
		// 현재 위치(CurrentPos)를 기준으로 강제 이웃이 있는지 확인합니다.
		// 이 검사는 점프의 방향을 바꿔야 할 '코너'를 찾는 역할을 합니다.
		for (const auto& CheckDir : AllDirections)
		{
			if (CheckDir == Direction || CheckDir == (Direction * -1))
				continue;
			// 부모 방향(Direction)을 전달하여, 현재 진행 방향을 기준으로 한 강제 이웃을 찾습니다.
			if (HasForcedNeighbour(CurrentPos, CheckDir, Direction, AgentRadiusInVoxels))
			{
				// 강제 이웃이 발견되면, 현재 위치는 중요한 분기점이므로 점프 포인트가 됩니다.
				return CurrentPos;
			}
		}

		//Question: 밑에 코드 필요있나? 어짜피 한칸씩가는거 아닌가
		const FIntVector DirToGoal = Goal - CurrentPos;
		const int64 DistSqToGoal = (int64)DirToGoal.X * DirToGoal.X + (int64)DirToGoal.Y * DirToGoal.Y + (int64)
			DirToGoal.Z * DirToGoal.Z;

		// 현재 위치에서 다음 스텝까지의 거리 제곱을 직접 계산합니다. (Direction 벡터의 길이 제곱)
		const int64 DistSqToNext = (int64)Direction.X * Direction.X + (int64)Direction.Y * Direction.Y + (int64)
			Direction.Z * Direction.Z;

		if (DistSqToNext > DistSqToGoal)
		{
			// 다음 스텝이 목표보다 멀리 있다면, 목표를 지나친 것입니다.
			// 이 경우, 현재 위치에서 목표까지 직접 갈 수 있는지 최종 확인합니다.
			if (IsAgentMoveFeasible(CurrentPos, Goal, AgentRadiusInVoxels))
			{
				return Goal;
			}
		}


		FNeighborWalkability Walkability;
		for (int32 i = 0; i < AllDirections.Num(); ++i)
		{
			if (Volume.IsValid() && Volume->IsRegionWalkable(CurrentPos + AllDirections[i], AgentRadiusInVoxels))
			{
				Walkability.WalkableMask |= (1 << i);
			}
		}

		const int32 DirIndex = DirectionToIndexMap.FindRef(Direction);
		const int32 ParentDirIndex = DirectionToIndexMap.FindRef(Direction);
		uint64 ForcedNeighborCandidateMask = CalculateForcedNeighborMask(Walkability, ParentDirIndex);

		// 2c. CTZ를 사용해 강제 이웃 후보들을 빠르게 순회
		while (ForcedNeighborCandidateMask != 0)
		{
			unsigned long Index;
			// 가장 오른쪽에 있는 후보(1 비트)의 인덱스를 찾는다
			_BitScanForward64(&Index, ForcedNeighborCandidateMask); // CTZ 명령어

			// 여기서 Index는 강제 이웃이 될 수 있는 방향의 인덱스입니다.
			// 이 방향으로 점프를 시도하거나, 현재 위치를 점프 포인트로 즉시 반환할 수 있습니다.
			// JPS+의 규칙에 따라, 강제 이웃이 하나라도 발견되면 현재 위치가 점프 포인트가 됩니다.
			return CurrentPos; 

			// 만약 모든 강제 이웃을 다 찾아야 한다면, 처리한 비트를 끄고 루프를 계속합니다.
			// ForcedNeighborCandidateMask &= ForcedNeighborCandidateMask - 1;
		}
	}

	return {};
}

bool UJPSPlusStrategy::IsAgentMoveFeasible(const FIntVector& From, const FIntVector& To,
                                           int32 AgentRadiusInVoxels) const
{
	if (Volume.IsValid() == false)
	{
		return false;
	}

	const FMoveFeasibilityKey Key(From, To);
	{
		FScopeLock Lock(&CacheMutex);
		if (const bool* bCachedResult = MoveFeasibilityCache.Find(Key))
		{
			return *bCachedResult;
		}
	}

	if (Volume.IsValid() && !Volume->IsRegionWalkable(To, AgentRadiusInVoxels))
	{
		FScopeLock Lock(&CacheMutex);
		MoveFeasibilityCache.Add(Key, false);
		return false;
	} 

	const FIntVector FromMin = From - FIntVector(AgentRadiusInVoxels);
	const FIntVector FromMax = From + FIntVector(AgentRadiusInVoxels);
	const FIntVector ToMin = To - FIntVector(AgentRadiusInVoxels);
	const FIntVector ToMax = To + FIntVector(AgentRadiusInVoxels);

	const FIntVector SweptMin(FMath::Min(FromMin.X, ToMin.X), FMath::Min(FromMin.Y, ToMin.Y),
	                          FMath::Min(FromMin.Z, ToMin.Z));
	const FIntVector SweptMax(FMath::Max(FromMax.X, ToMax.X), FMath::Max(FromMax.Y, ToMax.Y),
	                          FMath::Max(FromMax.Z, ToMax.Z));

	// 스윕 볼륨 내에 장애물이 있는지 Fenwick Tree로 빠르게 검사합니다.
	const bool bIsFeasible = ( Volume.IsValid() && Volume->FenwRangeSum(SweptMin, SweptMax) == 0);

	// 계산 결과를 캐시에 저장합니다.
	{
		FScopeLock Lock(&CacheMutex);
		MoveFeasibilityCache.Add(Key, bIsFeasible);
	}
	return bIsFeasible;
}


void UJPSPlusStrategy::OptimizePathFromJumpPoints(FPathNode* EndNode, TArray<FIntVector>& OutOptimizedPath,
                                                  uint32 AgentRadiusInVoxels) const
{
	OutOptimizedPath.Empty();
	if (!EndNode || !Volume.IsValid())
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
			if (HasLineOfSight(JumpPoints[CurrentIndex], JumpPoints[LookaheadIndex], AgentRadiusInVoxels))
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

bool UJPSPlusStrategy::HasLineOfSight(const FIntVector& From, const FIntVector& To, int32 AgentRadiusInVoxels) const
{
	if (!Volume.IsValid())
	{
		return false;
	}

	// 1. From에서 To까지 중심선을 따라 복셀 목록을 생성
	TArray<FIntVector> CenterlineVoxels;
	FStrategyUtils::GenerateBresenhamLine(From, To, CenterlineVoxels);

	// 2. 중심선의 각 복셀을 순회하며 국소 검사 수행
	for (const FIntVector& VoxelOnLine : CenterlineVoxels)
	{
		// 3. 현재 복셀을 중심으로 에이전트 반경만큼의 작은 바운딩 박스를 계산
		const FIntVector LocalMin = VoxelOnLine - FIntVector(AgentRadiusInVoxels);
		const FIntVector LocalMax = VoxelOnLine + FIntVector(AgentRadiusInVoxels);

		// 4. 이 '작은 영역'에 대해서만 Fenwick Tree 합계 검사를 수행
		if (Volume.IsValid() && Volume->FenwRangeSum(LocalMin, LocalMax) > 0)
		{
			// 경로 상의 한 지점에서 장애물이 발견되면 즉시 실패 반환
			return false;
		}
	}

	// 모든 검사를 통과했다면 시야가 확보된 것임
	return true;
}
int32 UJPSPlusStrategy::CalculateHeuristic(const FIntVector& A, const FIntVector& B) const
{
	return FStrategyUtils::Heuristic_Octile(A, B);
}

int32 UJPSPlusStrategy::GetMoveCost(const FIntVector& A, const FIntVector& B) const
{
	return FStrategyUtils::Heuristic_Octile(A, B);
}

bool UJPSPlusStrategy::HasForcedNeighbour(const FIntVector& NodePos, const FIntVector& CheckDir,
                                          const FIntVector& ParentDir, uint32 AgentRadiusInVoxels) const
{
	//Question: 이거 바깥에서 기록했던 bool값 쓰면 되는거아닌가?
	
	if (ParentDir == FIntVector::ZeroValue)
	{
		return false;
	}

	// 1. 코너링 경로(ParentPos -> NodePos + CheckDir)가 가능한지 검사합니다.
	// ParentPos는 NodePos의 부모 노드 위치입니다.
	const FIntVector ParentPos = NodePos - ParentDir;
	const FIntVector CorneringTargetPos = NodePos + CheckDir;

	// 만약 코너링 경로가 장애물에 막혀 불가능하고,
	if (!IsAgentMoveFeasible(ParentPos, CorneringTargetPos, AgentRadiusInVoxels))
	{
		// 2. 현재 위치에서 CheckDir 방향으로의 직선 이동은 가능하다면,
		// 이것이 바로 '코너를 돌아가기 위해 반드시 필요한' 강제된 이동입니다.
		if (IsAgentMoveFeasible(NodePos, CorneringTargetPos, AgentRadiusInVoxels))
		{
			return true;
		}
	}

	// 그 외의 모든 경우는 강제 이웃이 아닙니다.
	return false;
}
uint64 UJPSPlusStrategy::CalculateForcedNeighborMask(const FNeighborWalkability& Neighbors, int32 ParentDirIndex) const
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
bool UJPSPlusStrategy::HasForcedNeighbourBitMask(const FNeighborWalkability& Neighbors, int32 CheckDirIndex,
                                                 int32 ParentDirIndex) const
{
	// A forced neighbor exists if:
	// 1. The path straight into the neighbor (CheckDir) is walkable.
	if (!Neighbors.IsWalkable(CheckDirIndex))
	{
		return false;
	}

	// 2. The path around the corner obstacle is BLOCKED.
	const FIntVector& ParentDir = AllDirections[ParentDirIndex];
	const FIntVector& CheckDir = AllDirections[CheckDirIndex];

	// Decompose the parent direction (e.g., {1,1,0} -> dx=1, dy=1, dz=0)
	const int32 pdx = ParentDir.X, pdy = ParentDir.Y, pdz = ParentDir.Z;
	// Decompose the neighbor direction
	const int32 cdx = CheckDir.X, cdy = CheckDir.Y, cdz = CheckDir.Z;

	// Check for blockers based on the movement plane
	// Example: Moving along X-axis (ParentDir={1,0,0}). A forced neighbor at Y-axis (CheckDir={0,1,0})
	// is caused by a blocker at {1,-1,0}.
	if (pdx != 0 && (cdy != 0 || cdz != 0)) // Moving on X, checking Y/Z plane
	{
		if (cdy != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({pdx, -cdy, 0}))) return true;
		if (cdz != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({pdx, 0, -cdz}))) return true;
	}
	if (pdy != 0 && (cdx != 0 || cdz != 0)) // Moving on Y, checking X/Z plane
	{
		if (cdx != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({-cdx, pdy, 0}))) return true;
		if (cdz != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({0, pdy, -cdz}))) return true;
	}
	if (pdz != 0 && (cdx != 0 || cdy != 0)) // Moving on Z, checking X/Y plane
	{
		if (cdx != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({-cdx, 0, pdz}))) return true;
		if (cdy != 0 && !Neighbors.IsWalkable(DirectionToIndexMap.FindRef({0, -cdy, pdz}))) return true;
	}

	return false;
}
