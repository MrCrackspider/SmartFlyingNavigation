#pragma once
#include "CoreMinimal.h" // FIntVector, FMath 등을 위해 포함
#include "Containers/BitArray.h" // TBitArray를 위해 포함

struct FThreadSafeBitmaskGrid
{
	TBitArray<> ObstacleBitmasks_X;
	FIntVector GridSize;

	// 이 구조체는 UObject를 포함하지 않으므로 여러 스레드에서 안전하게 읽을 수 있습니다.
	
	FThreadSafeBitmaskGrid() : GridSize(FIntVector::ZeroValue)
	{
	}
	
	FORCEINLINE int32 GetVoxelFlatIndex(const FIntVector& Index) const
	{
		// X축이 가장 빠르게 변하고, 다음 Y, 다음 Z 순서로 인덱스를 평탄화합니다.
		return Index.X + Index.Y * GridSize.X + Index.Z * GridSize.X * GridSize.Y;
	}

	bool IsIndexValid(const FIntVector& Index) const
	{
		return Index.X >= 0 && Index.X < GridSize.X &&
			Index.Y >= 0 && Index.Y < GridSize.Y &&
			Index.Z >= 0 && Index.Z < GridSize.Z;
	}

	// IsRegionEmpty_Bitmask의 스레드 안전 버전.
	// UObject 멤버에 접근하는 대신, 복사된 데이터로만 연산합니다.
	bool IsRegionEmpty(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
	{
		// 경계 값을 그리드 크기 안으로 제한합니다.
		const FIntVector ClampedMin(
			FMath::Max(MinCorner.X, 0),
			FMath::Max(MinCorner.Y, 0),
			FMath::Max(MinCorner.Z, 0)
		);

		const FIntVector ClampedMax(
			FMath::Min(MaxCorner.X, GridSize.X - 1),
			FMath::Min(MaxCorner.Y, GridSize.Y - 1),
			FMath::Min(MaxCorner.Z, GridSize.Z - 1)
		);

		// 클램프된 범위가 유효하지 않거나 비어있는 경우 (예: Max < Min), 비어있다고 간주합니다.
		if (ClampedMin.X > ClampedMax.X || ClampedMin.Y > ClampedMax.Y || ClampedMin.Z > ClampedMax.Z)
		{
			return true;
		}

		// Z, Y 축을 순회하며 각 X-세그먼트를 TConstSetBitIterator로 검사합니다.
		for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
		{
			for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
			{
				// 현재 (Y, Z) 평면에서 X축을 따라 쿼리할 복셀 범위에 해당하는 1D 플랫 인덱스를 계산합니다.
				// GetVoxelFlatIndex는 FIntVector(X,Y,Z)를 기반으로 작동합니다.
				const int32 SegmentStartFlatIndex = GetVoxelFlatIndex(FIntVector(ClampedMin.X, y, z));
				const int32 SegmentEndFlatIndex = GetVoxelFlatIndex(FIntVector(ClampedMax.X, y, z));

				// TConstSetBitIterator를 사용하여 SegmentStartFlatIndex부터 장애물(비트가 1)을 찾습니다.
				// 이 이터레이터는 설정된 다음 비트가 있는 위치로 "점프"하므로 효율적입니다.
				for (TConstSetBitIterator<> It(ObstacleBitmasks_X, SegmentStartFlatIndex); It; ++It)
				{
					const int32 CurrentObstacleFlatIndex = It.GetIndex();

					// 찾은 장애물의 플랫 인덱스가 현재 X-세그먼트의 끝을 넘어서면,
					// 이 X-세그먼트 내에는 더 이상 장애물이 없으므로 다음 (Y,Z) 슬라이스로 넘어갑니다.
					if (CurrentObstacleFlatIndex > SegmentEndFlatIndex)
					{
						break; // 현재 Y, Z 슬라이스의 X 범위 벗어남
					}

					// 찾은 장애물이 현재 X-세그먼트 [SegmentStartFlatIndex, SegmentEndFlatIndex] 내에 있다면,
					// 해당 영역은 비어있지 않으므로 즉시 false를 반환합니다.
					// (이터레이터는 SegmentStartFlatIndex부터 시작하므로 CurrentObstacleFlatIndex >= SegmentStartFlatIndex는 항상 참입니다.)
					// 따라서 단순히 SegmentEndFlatIndex보다 작은지만 확인하면 됩니다.
					return false; // 장애물 발견, 영역이 비어있지 않음
				}
			}
		}

		// 모든 Z, Y 슬라이스의 모든 X-세그먼트를 확인했지만 장애물을 찾지 못했습니다.
		return true; // 영역이 비어있음
	}

	bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const
	{
		// 에이전트 반경이 음수일 수 없습니다.
		if (AgentRadiusInVoxels < 0) return false;

		const FIntVector AgentRadius(AgentRadiusInVoxels);
		const FIntVector MinCorner = VoxelIndex - AgentRadius;
		const FIntVector MaxCorner = VoxelIndex + AgentRadius;

		// 계산된 영역이 그리드 범위 밖으로 완전히 벗어나는 경우, 걸을 수 없다고 판단할 수 있습니다.
		// 또는 IsRegionEmpty 내부의 클램핑이 이를 처리합니다.
		// 이 경우 GridSize를 통한 유효성 검사가 먼저 수행되어야 합니다.
		if (!IsIndexValid(MinCorner) || !IsIndexValid(MaxCorner)) // Min/Max 코너가 그리드 내에 있어야 합니다.
		{
			return false;
		}

		return IsRegionEmpty(MinCorner, MaxCorner);
	}

	bool FindNearestWalkableVoxel_Bitmask26(const FIntVector& TargetVoxel, int32 AgentRadiusInVoxels,
	                                        int32 SearchRadiusInVoxels, FIntVector& OutNearestWalkableVoxel) const
	{
	const FIntVector& Size = GridSize;
		const int32 Total = Size.X * Size.Y * Size.Z;

		// 1) 방문 플래그 비트셋 (스레드 안전성을 위해 함수 내부에서 생성)
		TBitArray<> Visited;
		Visited.Init(false, Total);

		// 2) 원형 큐 (스레드 안전성을 위해 함수 내부에서 생성)
		TArray<int32> Queue;
		Queue.SetNumUninitialized(Total); // 최대 크기로 미리 할당
		int32 Head = 0, Tail = 0;

		// 3D 좌표를 1D 플랫 인덱스로 변환하는 람다 (GetVoxelFlatIndex와 동일)
		auto ToIndex = [&](const FIntVector& V)
		{
			return V.X + V.Y * Size.X + V.Z * Size.X * Size.Y;
		};

		// 3) 시작점 처리
		// 대상 복셀이 유효한지 먼저 확인합니다.
		if (!IsIndexValid(TargetVoxel))
		{
			return false; // 대상 복셀이 유효하지 않으면 검색할 수 없습니다.
		}

		if (IsRegionWalkable(TargetVoxel, AgentRadiusInVoxels))
		{
			OutNearestWalkableVoxel = TargetVoxel;
			return true;
		}

		int32 StartIdx = ToIndex(TargetVoxel);
		
		// 시작 인덱스가 유효하고 방문하지 않았는지 확인 후 큐에 추가
		if (StartIdx >= 0 && StartIdx < Total && !Visited[StartIdx])
		{
			Queue[Tail++] = StartIdx;
			Visited[StartIdx] = true;
		}
		else
		{
			// 시작 복셀이 유효하지 않거나 이미 방문되었을 경우, 검색을 시작할 수 없습니다.
			return false;
		}

		// 4) 26방향 이웃 오프셋 생성 (정적으로 한 번만 생성)
		static const TArray<FIntVector> Dirs26 = []()
		{
			TArray<FIntVector> D;
			D.Reserve(26);
			for (int dz = -1; dz <= 1; ++dz)
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
						if (dx != 0 || dy != 0 || dz != 0) // 자기 자신(0,0,0) 제외
							D.Emplace(dx, dy, dz);
			return D;
		}();

		// 5) BFS (너비 우선 탐색)
		while (Head != Tail)
		{
			int32 Idx = Queue[Head++];
			if (Head >= Total) Head = 0; // 원형 큐 처리

			// 1차원 인덱스를 3D 좌표로 복원
			int z = Idx / (Size.X * Size.Y);
			int rem = Idx % (Size.X * Size.Y);
			int y = rem / Size.X;
			int x = rem % Size.X;
			FIntVector V(x, y, z);

			// 현재 복셀이 탐색 반경을 초과했는지 검사
			if (FMath::Abs(x - TargetVoxel.X) > SearchRadiusInVoxels ||
				FMath::Abs(y - TargetVoxel.Y) > SearchRadiusInVoxels ||
				FMath::Abs(z - TargetVoxel.Z) > SearchRadiusInVoxels)
			{
				continue; // 이 분기를 타면 이웃들도 대부분 반경을 초과할 가능성이 높음
			}

			// 26방향 이웃 탐색
			for (const FIntVector& Off : Dirs26)
			{
				FIntVector N = V + Off;
				
				// 이웃 복셀이 그리드 경계를 벗어나는지 확인
				if (!IsIndexValid(N))
					continue;

				int nIdx = ToIndex(N);
				
				// 이웃 복셀이 이미 방문되었는지 확인
				if (Visited[nIdx]) continue;
				
				Visited[nIdx] = true; // 방문 표시

				// 이웃 복셀이 걸을 수 있는 영역인지 확인 (에이전트 반경 고려)
				if (IsRegionWalkable(N, AgentRadiusInVoxels))
				{
					OutNearestWalkableVoxel = N;
					return true; // 가장 가까운 걸을 수 있는 복셀을 찾았으므로 즉시 반환
				}

				// 걸을 수 없다면 큐에 추가하여 나중에 탐색
				Queue[Tail++] = nIdx;
				if (Tail >= Total) Tail = 0; // 원형 큐 처리
			}
		}
		
		// 탐색 반경 내에서 걸을 수 있는 복셀을 찾지 못함
		return false;
	}
};
