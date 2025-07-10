#pragma once

struct FThreadSafeBitmaskGrid
{
	TArray<uint64> ObstacleBitmasks_X;
	FIntVector GridSize;

	// 이 구조체는 UObject를 포함하지 않으므로 여러 스레드에서 안전하게 읽을 수 있습니다.

	// 헬퍼 상수
	static const int32 CHUNK_SIZE = 64; // FenwickVoxelVolume의 CHUNK_SIZE와 일치해야 합니다.

	FThreadSafeBitmaskGrid() : GridSize(FIntVector::ZeroValue)
	{
	}

	// X축 비트마스크 배열의 1차원 인덱스를 계산하는 헬퍼 함수
	// (AFenwickVoxelVolume::GetFlatIndex_X 와 동일한 로직)
	uint32 GetFlatIndex_X(uint32 ChunkX, uint32 Y, uint32 Z) const
	{
		const uint32 ChunksAlongX = GridSize.X / CHUNK_SIZE;
		return (Z * GridSize.Y + Y) * ChunksAlongX + ChunkX;
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
		// 경계 값을 맵 크기 안으로 제한합니다.
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

		// Z, Y 축을 순회하며 비트마스크로 X축을 한 번에 검사합니다.
		for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
		{
			for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
			{
				const int32 StartChunkX = ClampedMin.X / CHUNK_SIZE;
				const int32 EndChunkX = ClampedMax.X / CHUNK_SIZE;

				for (int32 cx = StartChunkX; cx <= EndChunkX; ++cx)
				{
					const uint64 Mask = ObstacleBitmasks_X[GetFlatIndex_X(cx, y, z)];
					if (Mask == 0) continue; // 이 청크(64개 복셀)에 장애물이 없으면 통과

					const int32 VoxelStartX = (cx == StartChunkX) ? ClampedMin.X : cx * CHUNK_SIZE;
					const int32 VoxelEndX = (cx == EndChunkX) ? ClampedMax.X : (cx + 1) * CHUNK_SIZE - 1;
					const int32 StartBit = VoxelStartX % CHUNK_SIZE;
					const int32 EndBit = VoxelEndX % CHUNK_SIZE;

					// 검사할 범위에 해당하는 비트만 1로 채운 마스크를 생성합니다.
					const uint64 CheckMask = (~0ULL << StartBit) & (~0ULL >> (63 - EndBit));

					// 장애물 비트마스크와 검사 마스크를 AND 연산하여 겹치는 부분이 있는지 확인합니다.
					if ((Mask & CheckMask) != 0)
					{
						return false; // 장애물 발견!
					}
				}
			}
		}
		return true; // 장애물 없음
	}

	bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const
	{
		if (AgentRadiusInVoxels < 0) return false;

		const FIntVector AgentRadius(AgentRadiusInVoxels);
		const FIntVector MinCorner = VoxelIndex - AgentRadius;
		const FIntVector MaxCorner = VoxelIndex + AgentRadius;

		if (!IsIndexValid(MinCorner) || !IsIndexValid(MaxCorner)) return false;

		return IsRegionEmpty(MinCorner, MaxCorner);
	}

	bool FindNearestWalkableVoxel_Bitmask26(const FIntVector& TargetVoxel, int32 AgentRadiusInVoxels,
	                                        int32 SearchRadiusInVoxels, FIntVector& OutNearestWalkableVoxel) const
	{
		const FIntVector& Size = GridSize;
		const int32 Total = Size.X * Size.Y * Size.Z;

		// 1) 방문 플래그 비트셋
		TBitArray<> Visited;
		Visited.Init(false, Total);

		// 2) 원형 큐
		TArray<int32> Queue;
		Queue.SetNumUninitialized(Total);
		int32 Head = 0, Tail = 0;

		auto ToIndex = [&](const FIntVector& V)
		{
			return V.X + V.Y * Size.X + V.Z * Size.X * Size.Y;
		};

		// 3) 시작점
		if (IsRegionWalkable(TargetVoxel, AgentRadiusInVoxels))
		{
			OutNearestWalkableVoxel = TargetVoxel;
			return true;
		}
		int32 StartIdx = ToIndex(TargetVoxel);
		Queue[Tail++] = StartIdx;
		Visited[StartIdx] = true;

		// 4) 26방향 이웃 오프셋 생성
		static const TArray<FIntVector> Dirs26 = []()
		{
			TArray<FIntVector> D;
			D.Reserve(26);
			for (int dz = -1; dz <= 1; ++dz)
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
						if (dx != 0 || dy != 0 || dz != 0)
							D.Emplace(dx, dy, dz);
			return D;
		}();

		// 5) BFS
		while (Head != Tail)
		{
			int32 Idx = Queue[Head++];
			if (Head >= Total) Head = 0;

			// 좌표 복원
			int z = Idx / (Size.X * Size.Y);
			int rem = Idx % (Size.X * Size.Y);
			int y = rem / Size.X;
			int x = rem % Size.X;
			FIntVector V(x, y, z);

			// 반경 검사
			if (FMath::Abs(x - TargetVoxel.X) > SearchRadiusInVoxels ||
				FMath::Abs(y - TargetVoxel.Y) > SearchRadiusInVoxels ||
				FMath::Abs(z - TargetVoxel.Z) > SearchRadiusInVoxels)
				continue;

			for (const FIntVector& Off : Dirs26)
			{
				FIntVector N = V + Off;
				if (N.X < 0 || N.Y < 0 || N.Z < 0 || N.X >= Size.X || N.Y >= Size.Y || N.Z >= Size.Z)
					continue;
				int nIdx = ToIndex(N);
				if (Visited[nIdx]) continue;
				Visited[nIdx] = true;

				// 비트마스크 기반 장애물 체크
				if (IsRegionEmpty(N - FIntVector(AgentRadiusInVoxels),
				                                  N + FIntVector(AgentRadiusInVoxels)))
				{
					OutNearestWalkableVoxel = N;
					return true;
				}

				Queue[Tail++] = nIdx;
				if (Tail >= Total) Tail = 0;
			}
		}
		return false;
	}
};
