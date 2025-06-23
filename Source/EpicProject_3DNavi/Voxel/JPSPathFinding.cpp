#include "JPSPathFinding.h"
#include "Public/VoxelGrid.h"

TArray<FVector> UJPSPathFinding::FindPathJPS(UVoxelGrid* VoxelGrid, const FVector& StartWorldPos, const FVector& EndWorldPos)
{
	if (!VoxelGrid) return {};

	const FIntVector StartPos = VoxelGrid->WorldToGridCoordinates(StartWorldPos);
	const FIntVector GoalPos = VoxelGrid->WorldToGridCoordinates(EndWorldPos);

	if (!IsWalkable(StartPos, VoxelGrid) || !IsWalkable(GoalPos, VoxelGrid))
	{
		//시작 또는 끝이 장애물이면 경로 탐색 불가
		return {};
	}

	TArray<FJPSNode> OpenList;		//우선순위큐로 사용할 배열
	TSet<FIntVector> ClosedList;	//이미 탐색한 노드 집합
	TMap<FIntVector, FIntVector> CameFrom;	//경로 추적을 위한 부모 노드 맵
	TMap<FIntVector, float> GCostMap;		//각 노드의 G-Cost 저장

	FJPSNode StartNode(StartPos);
	StartNode.HCost = Heuristic(StartPos, GoalPos);
	StartNode.FCost = StartNode.HCost;
	StartNode.ParentPosition = FIntVector(TNumericLimits<int32>::Min()); //부모 없는걸로 초기화
	//StartNode의 부모노드 초기화 
	
	//시작하기 위해 시작노드 삽입
	OpenList.HeapPush(StartNode);
	GCostMap.Add(StartPos, 0.f);

	while (OpenList.Num() > 0)
	{
		FJPSNode CurrentNode;
		OpenList.HeapPop(CurrentNode); //가장 우선순위가 높은 노드 꺼내기(HeapPush로 정렬돼서 들어감)

		if (CurrentNode.Position == GoalPos)
		{
			//목표 찾으면 경로 구성 후 반환
			return ReconstructPath(GoalPos, CameFrom, VoxelGrid);
		}

		ClosedList.Add(CurrentNode.Position);
		TArray<FIntVector> Successors;
		//여기서 갈 수 있는 JumpPoint(Successor) 찾기
		IdentifySuccessors(CurrentNode, GoalPos, VoxelGrid, Successors);

		//찾은 Successor 순회
		for (const FIntVector& SuccessorPos : Successors)
		{
			if (ClosedList.Contains(SuccessorPos))
			{
				continue;
			}

			//다음노드의 잠정적인 GCost = 현재포지션의 GCost + 현재노드와 다음노드의 거리
			const float TentativeGCost = GCostMap.FindRef(CurrentNode.Position) + FVector::Distance(FVector(CurrentNode.Position), FVector(SuccessorPos));

			//처음 방문하는 노드이거나 현재 GCost보다 값이 비싸서 갱신해야 하는 경우 
			if (!GCostMap.Contains(SuccessorPos) || TentativeGCost < GCostMap.FindRef(SuccessorPos))
			{
				CameFrom.Add(SuccessorPos, CurrentNode.Position);
				GCostMap.Add(SuccessorPos, TentativeGCost);
				
				FJPSNode SuccessorNode(SuccessorPos);
				SuccessorNode.GCost = TentativeGCost;
				SuccessorNode.HCost = Heuristic(SuccessorPos, GoalPos);
				SuccessorNode.FCost = SuccessorNode.GCost + SuccessorNode.HCost;

				//OpenList에 이미 있다면 제거하고 다시 추가 (업데이트 로직 필요할 수도 있음)
				//일단 TArray Heap은 업데이트를 직접 지원하지 않으므로, 간단하게 그냥 추가(중복이 생길 수 있음)
				//TODO: TArray의 Heap은 느릴 수 있으므로 최적화를 위한 PQ가 필요할 수도 있음
				OpenList.HeapPush(SuccessorNode);
			}
		}
	}
	return {};
}

//다음 JumpPoint 찾는 함수
void UJPSPathFinding::IdentifySuccessors(const FJPSNode& Node, const FIntVector& GoalPos, UVoxelGrid* Grid, TArray<FIntVector>& OutSuccessors)
{
	TArray<FIntVector> PrunedNeighbors = GetPrunedNeighbors(Node.Position, Node.ParentPosition);
	for (const FIntVector& NeighborDir : PrunedNeighbors)
	{
		TOptional<FIntVector> JumpPoint = Jump(Node.Position, NeighborDir, GoalPos, Grid);
		if (JumpPoint.IsSet())
		{
			OutSuccessors.Add(JumpPoint.GetValue());
		}
	}
}

//한 방향으로 점프하며 JumpPoint를 찾는다.
TOptional<FIntVector> UJPSPathFinding::Jump(const FIntVector& CurrentPos, const FIntVector& Direction, const FIntVector& GoalPos, UVoxelGrid* Grid)
{
	const FIntVector NextPos = CurrentPos + Direction;

	//맵을 벗어나거나 벽에 부딪히면 점프 실패
	if (!IsWalkable(NextPos, Grid)){
		return {};
	}

	//목적지에 도달하면 점프 성공
	if (NextPos == GoalPos)
	{
		return NextPos;
	}

	//강제 이웃이 있는지 확인 -> 있으면 현재 위치를 JumpPoint확정
	const int32 dx = Direction.X;
	const int32 dy = Direction.Y;
	const int32 dz = Direction.Z;

	//대각선 이동일 경우
	if (dx != 0 && dy != 0 && dz != 0) //3D 대각선
	{
		//수평/수직 방향으로 추가적인 점프를 시도하여 JumpPoint를 찾는다.
		if (Jump(NextPos, {dx, 0, 0}, GoalPos, Grid).IsSet()||
			Jump(NextPos, {0, dy, 0}, GoalPos, Grid).IsSet()||
			Jump(NextPos, {0, 0, dz}, GoalPos, Grid).IsSet())
		{
			return NextPos;
		}
	}
	else //직선 또는 2D대각선 이동일 경우 (3D구현에서는 이 부분을 더 정교하게 확장해야함
	{
		//X축 직선 이동시
		if (dx != 0 && dy == 0 && dz == 0)
		{
			const int px = NextPos.X - dx; //부모의 X좌표와 같음

			// 추기경 방향 이웃 (Cardinal Neighbors) - 4방향
			if (IsWalkable({NextPos.X, NextPos.Y + 1, NextPos.Z}, Grid) && !IsWalkable({px, NextPos.Y + 1, NextPos.Z}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y - 1, NextPos.Z}, Grid) && !IsWalkable({px, NextPos.Y - 1, NextPos.Z}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y, NextPos.Z + 1}, Grid) && !IsWalkable({px, NextPos.Y, NextPos.Z + 1}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y, NextPos.Z - 1}, Grid) && !IsWalkable({px, NextPos.Y, NextPos.Z - 1}, Grid)) return NextPos;

			// 대각선 방향 이웃 (Diagonal Neighbors) - 4방향
			if (IsWalkable({NextPos.X, NextPos.Y + 1, NextPos.Z + 1}, Grid) && !IsWalkable({px, NextPos.Y + 1, NextPos.Z + 1}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y + 1, NextPos.Z - 1}, Grid) && !IsWalkable({px, NextPos.Y + 1, NextPos.Z - 1}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y - 1, NextPos.Z + 1}, Grid) && !IsWalkable({px, NextPos.Y - 1, NextPos.Z + 1}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y - 1, NextPos.Z - 1}, Grid) && !IsWalkable({px, NextPos.Y - 1, NextPos.Z - 1}, Grid)) return NextPos;
		}
		//Y축 직선 이동시
		if (dx == 0 && dy != 0 && dz == 0)
		{
			const int py = NextPos.Y - dy; //부모의 X좌표와 같음

			// 추기경 방향 이웃 (Cardinal Neighbors) - 4방향
			if (IsWalkable({NextPos.X + 1, NextPos.Y, NextPos.Z}, Grid) && !IsWalkable({NextPos.X + 1, py, NextPos.Z}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X - 1, NextPos.Y, NextPos.Z}, Grid) && !IsWalkable({NextPos.X - 1, py, NextPos.Z}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y, NextPos.Z + 1}, Grid) && !IsWalkable({NextPos.X, py, NextPos.Z + 1}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y, NextPos.Z - 1}, Grid) && !IsWalkable({NextPos.X, py, NextPos.Z - 1}, Grid)) return NextPos;

			// 대각선 방향 이웃 (Diagonal Neighbors) - 4방향
			if (IsWalkable({NextPos.X + 1, NextPos.Y, NextPos.Z + 1}, Grid) && !IsWalkable({NextPos.X + 1, py, NextPos.Z + 1}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X + 1, NextPos.Y, NextPos.Z - 1}, Grid) && !IsWalkable({NextPos.X + 1, py, NextPos.Z - 1}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X - 1, NextPos.Y, NextPos.Z + 1}, Grid) && !IsWalkable({NextPos.X - 1, py, NextPos.Z + 1}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X - 1, NextPos.Y, NextPos.Z - 1}, Grid) && !IsWalkable({NextPos.X - 1, py, NextPos.Z - 1}, Grid)) return NextPos;
		}
		//Z축 직선 이동시
		if (dx == 0 && dy == 0 && dz != 0)
		{
			const int pz = NextPos.Z - dz; //부모의 X좌표와 같음

			// 추기경 방향 이웃 (Cardinal Neighbors) - 4방향
			if (IsWalkable({NextPos.X, NextPos.Y + 1, NextPos.Z}, Grid) && !IsWalkable({NextPos.X, NextPos.Y + 1, pz}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X, NextPos.Y - 1, NextPos.Z}, Grid) && !IsWalkable({NextPos.X, NextPos.Y - 1, pz}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X + 1, NextPos.Y, NextPos.Z}, Grid) && !IsWalkable({NextPos.X + 1, NextPos.Y, pz}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X - 1, NextPos.Y, NextPos.Z}, Grid) && !IsWalkable({NextPos.X - 1, NextPos.Y, pz}, Grid)) return NextPos;

			// 대각선 방향 이웃 (Diagonal Neighbors) - 4방향
			if (IsWalkable({NextPos.X + 1, NextPos.Y + 1, NextPos.Z}, Grid) && !IsWalkable({NextPos.X + 1, NextPos.Y + 1, pz}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X + 1, NextPos.Y - 1, NextPos.Z}, Grid) && !IsWalkable({NextPos.X + 1, NextPos.Y - 1, pz}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X - 1, NextPos.Y + 1, NextPos.Z}, Grid) && !IsWalkable({NextPos.X - 1, NextPos.Y + 1, pz}, Grid)) return NextPos;
			if (IsWalkable({NextPos.X - 1, NextPos.Y - 1, NextPos.Z}, Grid) && !IsWalkable({NextPos.X - 1, NextPos.Y - 1, pz}, Grid)) return NextPos;
		}
	}

	//위 조건에 해당하지 않으면 같은 방향으로 계속 점프
	return Jump(NextPos, Direction, GoalPos, Grid);
}

//경로가 완성되면 목표지점부터 부모노드를 따라 거슬러 올라가며 경로 재구성
TArray<FVector> UJPSPathFinding::ReconstructPath(const FIntVector& GoalPos, const TMap<FIntVector, FIntVector>& CameFrom, UVoxelGrid* Grid)
{
	TArray<FVector> Path;
	FIntVector Current = GoalPos;
	while (CameFrom.Contains(Current))
	{
		Path.Add(Grid->GridToWorldCoordinates(Current));
		Current = CameFrom.FindRef(Current);
	}
	Path.Add(Grid->GridToWorldCoordinates(Current)); //시작점 추가

	Algo::Reverse(Path); //경로를 시작점->목적점 순으로 뒤집기
	return Path;
}

//불필요한 이웃을 제거하고 탐색할 방향만 남김
TArray<FIntVector> UJPSPathFinding::GetPrunedNeighbors(const FIntVector& CurrentPos, const FIntVector& ParentPos)
{
	TArray<FIntVector> Neighbors;
	if (ParentPos == FIntVector(TNumericLimits<int32>::Min())) //부모가 없을때 (시작 노드)
	{
		//26방향 탐색
		for (int x = -1; x<=1; x++)
		{
			for (int y = -1; y<=1; y++)
			{
				for (int z = -1; z<=1; z++)
				{
					if (x == 0 && y == 0 && z == 0) continue;
					Neighbors.Add({x,y,z});
				}
			}
		}

		return Neighbors;
	}

	//부모로부터 온 방향 계산
	const FIntVector Dir = GetSignIntVector(CurrentPos - ParentPos);

	//직선 이동인 경우 -> 응 아니야 
	if (Dir.X == 0 || Dir.Y == 0 || Dir.Z == 0) 
	{ 
		//TODO: 실제 3D에서는 더 복잡하므로 손볼 필요 있음
		//앞으로 가는 방향은 무조건 추가
		Neighbors.Add(Dir);
		//강제된 이웃을 확인할 방향들도 추가해야함 (간략화)
	}else //대각선 이동인 경우
	{
		//대각선 방향 및 관련 직선 방향들을 추가
		Neighbors.Add({Dir.X, 0, 0});
		Neighbors.Add({0, Dir.Y, 0});
		Neighbors.Add({0, 0, Dir.Z});
		Neighbors.Add(Dir);
	}
	return Neighbors;
}

//해당 그리드 좌표가 걸어갈 수 있는지 확인
bool UJPSPathFinding::IsWalkable(const FIntVector& GridPos, UVoxelGrid* Grid)
{
	FVector WorldPos = Grid->GridToWorldCoordinates(GridPos);
	//볼륨밖이면 못감 (지금 WorldPosition이 아니라 그리드좌표 주고 있어서 제대로 체크를 안하는중)
	if (!Grid->IsInVolume(WorldPos))
	{
		return false;
	}
	
	//맵에 없으면 갈 수 있음 (맵은 장애물만 있기때문)
	if (!Grid->VoxelMap.Contains(GridPos))
	{
		return true;
	}

	//맵에 있으면 안에 IsSolid확인
	return Grid->VoxelMap[GridPos].IsWalkable();
}

//Heuristic함수. 맨해튼 거리나 유클리드 거리를 사용
float UJPSPathFinding::Heuristic(const FIntVector& A, const FIntVector& B)
{
	return FVector::Distance(FVector(A), FVector(B));
}

FIntVector UJPSPathFinding::GetSignIntVector(const FIntVector& InVec)
{
	return FIntVector
	(
		FMath::FloatSelect(InVec.X, 1.f, -1.f),		// LWC_TODO: Templatize FMath functionality
		FMath::FloatSelect(InVec.Y, 1.f, -1.f),
		FMath::FloatSelect(InVec.Z, 1.f, -1.f)
	);
}