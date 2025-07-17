#pragma once
#include "CoreMinimal.h"
#include "Containers/Queue.h"
#include "Pathfinder/NavData/StaticVoxelChunkData.h" // F128BitChunk
// SIMD
#include "emmintrin.h" // SSE2
#include "smmintrin.h" // SSE4.1

struct FThreadSafeBitmaskGrid
{
	TArray<F128BitChunk> ObstacleBitmasks;
	FIntVector GridSize;
	int64 TotalVoxels;

	FThreadSafeBitmaskGrid() : GridSize(FIntVector::ZeroValue)
	{
	}

	FORCEINLINE bool IsIndexValid(const FIntVector& Index) const
	{
		return Index.X >= 0 && Index.X < GridSize.X &&
			Index.Y >= 0 && Index.Y < GridSize.Y &&
			Index.Z >= 0 && Index.Z < GridSize.Z;
	}


	/**
 * 1) 전역 128비트 그리드에서 단일 셀의 비트(장애물 여부)를 읽어옵니다.
 *    경계를 벗어나면 장애물로 처리(true 반환)
 */
	FORCEINLINE bool GetBitAt(const FIntVector& Idx) const
	{
		if (!IsIndexValid(Idx))
		{
			return true;
		}
		// flat index: Z * (Y*X) + Y*X + X
		const int64 Flat = (int64)Idx.Z * GridSize.Y * GridSize.X +
			(int64)Idx.Y * GridSize.X + Idx.X;
		const int32 ChunkIdx = Flat >> 7; // /128
		const int32 BitInChunk = Flat & 127; // %128

		const F128BitChunk& Chunk = ObstacleBitmasks[ChunkIdx];
		// 0~63 → Words[0], 64~127 → Words[1]
		if (BitInChunk < 64)
		{
			return (Chunk.Words[0] >> BitInChunk) & 1ULL;
		}
		else
		{
			return (Chunk.Words[1] >> (BitInChunk - 64)) & 1ULL;
		}
	}

	void GetObstacleMask128_Clearance(const FIntVector& StartPos, const FIntVector& Direction,
	                                  uint64_t outMask[2]) const
	{
		outMask[0] = 0;
		outMask[1] = 0;

		for (int32 i = 0; i < 128; ++i)
		{
			const FIntVector VoxelToCheck = StartPos + Direction * i;

			// ClearanceGrid의 비트를 직접 읽습니다. GetBitAt은 경계를 벗어나면 true(장애물)를 반환합니다.
			// 이 비트는 이미 에이전트의 부피를 고려한 결과여야 합니다.
			if (GetBitAt(VoxelToCheck))
			{
				if (i < 64)
				{
					outMask[0] |= (1ULL << i);
				}
				else
				{
					outMask[1] |= (1ULL << (i - 64));
				}
			}
		}
	}

	void GetObstacleMask128(const FIntVector& Curr, const FIntVector& Dir, uint64_t outMask[2]) const
	{
		// 초기화
		outMask[0] = 0;
		outMask[1] = 0;

		// 1..128칸에 대해 순회
		for (int i = 1; i <= 128; ++i)
		{
			// i번째 칸 좌표 계산
			FIntVector Pos = Curr + Dir * i;

			// 장애물(true)인지 확인 (경계 밖도 true로 처리됨)
			bool bBlocked = GetBitAt(Pos);

			if (bBlocked)
			{
				if (i <= 64)
				{
					// 0~63 비트에 세팅 (i-1)
					outMask[0] |= (1ULL << (i - 1));
				}
				else
				{
					// 64~127 비트에 세팅 (i-65)
					outMask[1] |= (1ULL << (i - 65));
				}
			}
		}
	}

	uint64_t GetNeighbourObstacleMask(const FIntVector& Curr) const
	{
		static const TArray<FIntVector> Dirs26 = []()
		{
			TArray<FIntVector> A;
			A.Reserve(26);
			for (int dz = -1; dz <= 1; ++dz)
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
						if (dx || dy || dz)
							A.Emplace(dx, dy, dz);
			return A;
		}();

		uint64_t mask = 0ULL;
		for (int32 i = 0; i < Dirs26.Num(); ++i)
		{
			const FIntVector N = Curr + Dirs26[i];
			if (GetBitAt(N)) // 장애물(true) 이면
			{
				mask |= (1ULL << i);
			}
		}
		return mask;
	}

	FORCEINLINE uint64_t GetNeighbourWalkableMask(const FIntVector& Curr) const
	{
		return ~GetNeighbourObstacleMask(Curr);
	}

	// IsRegionEmpty의 SIMD 최적화 버전
	// IsRegionEmpty의 SIMD 최적화 버전 (안정성 강화)
	bool IsRegionEmpty(const FIntVector& MinCorner, const FIntVector& MaxCorner) const
	{
		const FIntVector ClampedMin(
			FMath::Max(MinCorner.X, 0), FMath::Max(MinCorner.Y, 0), FMath::Max(MinCorner.Z, 0)
		);
		const FIntVector ClampedMax(
			FMath::Min(MaxCorner.X, GridSize.X - 1), FMath::Min(MaxCorner.Y, GridSize.Y - 1),
			FMath::Min(MaxCorner.Z, GridSize.Z - 1)
		);

		if (ClampedMin.X > ClampedMax.X || ClampedMin.Y > ClampedMax.Y || ClampedMin.Z > ClampedMax.Z)
		{
			return true;
		}

		if (ObstacleBitmasks.Num() == 0 || GridSize.X == 0)
		{
			return true;
		}

		const __m128i AllOnesMask = _mm_set_epi64x(-1LL, -1LL);

		// ★★★ 2. 미정의 동작을 피하도록 수정한 CreateMask 람다 ★★★
		auto CreateMask = [](int32 StartBit, int32 EndBit) -> __m128i
		{
			uint64 LowMask = 0, HighMask = 0;

			if (EndBit >= 0 && StartBit < 64)
			{
				LowMask = (~0ULL << StartBit);
				if (EndBit < 64)
				{
					LowMask &= (~0ULL >> (63 - EndBit));
				}
			}

			if (StartBit < 128 && EndBit >= 64)
			{
				HighMask = (~0ULL >> (127 - EndBit));
				if (StartBit >= 64)
				{
					HighMask &= (~0ULL << (StartBit - 64));
				}
			}

			return _mm_set_epi64x(HighMask, LowMask);
		};

		const int32 ChunksPerRow = GridSize.X / 128; // GridSize.X가 128의 배수이므로 이 계산은 안전

		for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
		{
			for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
			{
				const int32 RowStartChunkIndex = (z * GridSize.Y + y) * ChunksPerRow;
				const int32 StartChunkIndex = RowStartChunkIndex + ClampedMin.X / 128;
				const int32 EndChunkIndex = RowStartChunkIndex + ClampedMax.X / 128;
				const int32 StartBitInChunk = ClampedMin.X % 128;
				const int32 EndBitInChunk = ClampedMax.X % 128;

				if (StartChunkIndex == EndChunkIndex)
				{
					const __m128i Mask = CreateMask(StartBitInChunk, EndBitInChunk);
					// ★★★ 1. 충돌을 방지하는 안전한 Unaligned Load 사용 ★★★
					const __m128i ChunkData = _mm_loadu_si128(
						reinterpret_cast<const __m128i*>(&ObstacleBitmasks[StartChunkIndex]));
					if (_mm_testz_si128(ChunkData, Mask) == 0) return false;
				}
				else
				{
					__m128i StartMask = CreateMask(StartBitInChunk, 127);
					__m128i ChunkData = _mm_loadu_si128(
						reinterpret_cast<const __m128i*>(&ObstacleBitmasks[StartChunkIndex]));
					if (_mm_testz_si128(ChunkData, StartMask) == 0) return false;

					for (int32 i = StartChunkIndex + 1; i < EndChunkIndex; ++i)
					{
						const __m128i MiddleChunkData = _mm_loadu_si128(
							reinterpret_cast<const __m128i*>(&ObstacleBitmasks[i]));
						if (_mm_testz_si128(MiddleChunkData, AllOnesMask) == 0) return false;
					}

					__m128i EndMask = CreateMask(0, EndBitInChunk);
					ChunkData = _mm_loadu_si128(reinterpret_cast<const __m128i*>(&ObstacleBitmasks[EndChunkIndex]));
					if (_mm_testz_si128(ChunkData, EndMask) == 0) return false;
				}
			}
		}

		return true;
	}

	bool IsRegionWalkable(const FIntVector& VoxelIndex, int32 AgentRadiusInVoxels) const
	{
		if (AgentRadiusInVoxels < 0) return false;

		const FIntVector AgentRadius(AgentRadiusInVoxels);
		const FIntVector MinCorner = VoxelIndex - AgentRadius;
		const FIntVector MaxCorner = VoxelIndex + AgentRadius;

		// IsRegionEmpty 내부에서 경계를 클램핑하므로, 여기서는 별도의 IsIndexValid 검사가 필요 없습니다.
		return IsRegionEmpty(MinCorner, MaxCorner);
	}

	// BFS 탐색 함수의 최적화 버전
	bool FindNearestWalkableVoxel_BFS(const FIntVector& TargetVoxel, int32 AgentRadiusInVoxels,
	                                  int32 SearchRadiusInVoxels, FIntVector& OutNearestWalkableVoxel) const
	{
		if (!IsIndexValid(TargetVoxel))
		{
			return false; // 시작점이 유효하지 않으면 검색 불가
		}

		// 1. 시작점 자체가 걸을 수 있는지 먼저 확인 (가장 좋은 케이스)
		if (IsRegionWalkable(TargetVoxel, AgentRadiusInVoxels))
		{
			OutNearestWalkableVoxel = TargetVoxel;
			return true;
		}

		// 2. BFS를 위한 자료구조 초기화
		TQueue<FIntVector> Queue; // 언리얼의 표준 큐 사용
		TSet<FIntVector> Visited; // 방문한 노드만 저장하여 메모리 효율 증대

		Queue.Enqueue(TargetVoxel);
		Visited.Add(TargetVoxel);

		// 3. 26방향 이웃 오프셋 (정적 변수로 선언하여 매번 생성되는 것을 방지)
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

		// 4. BFS 루프
		FIntVector CurrentVoxel;
		while (Queue.Dequeue(CurrentVoxel))
		{
			// 5. 26방향 이웃 탐색
			for (const FIntVector& Dir : Dirs26)
			{
				const FIntVector NeighborVoxel = CurrentVoxel + Dir;

				// 6. 탐색 범위 검사 (큐에 넣기 전에 수행)
				if (FMath::Abs(NeighborVoxel.X - TargetVoxel.X) > SearchRadiusInVoxels ||
					FMath::Abs(NeighborVoxel.Y - TargetVoxel.Y) > SearchRadiusInVoxels ||
					FMath::Abs(NeighborVoxel.Z - TargetVoxel.Z) > SearchRadiusInVoxels)
				{
					continue; // 탐색 반경을 벗어났으므로 무시
				}

				// 7. 유효성 및 방문 여부 검사
				if (!IsIndexValid(NeighborVoxel) || Visited.Contains(NeighborVoxel))
				{
					continue;
				}

				// 8. 방문 처리
				Visited.Add(NeighborVoxel);

				// 9. 걸을 수 있는지 확인
				if (IsRegionWalkable(NeighborVoxel, AgentRadiusInVoxels))
				{
					OutNearestWalkableVoxel = NeighborVoxel;
					return true; // 가장 가까운 지점을 찾았으므로 즉시 성공 반환
				}

				// 10. 걸을 수 없다면 다음 탐색을 위해 큐에 추가
				Queue.Enqueue(NeighborVoxel);
			}
		}

		// 큐가 비워질 때까지 (즉, 탐색 반경 내 모든 노드를 방문할 때까지) 찾지 못함
		return false;
	}
};
