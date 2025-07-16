#pragma once
#include "Pathfinder/StaticVoxelChunkData.h"
#include "Pathfinder/NaviDataAssets/OctreeDataAsset.h"

struct FOctreePathfindingData
{
	// --- 정적 옥트리 데이터 ---
	TArray<FOctreeNode> StaticNodePool;
	int32 RootNodeIndex = -1;
	int32 MaxOctreeDepth = 0;
	FBox VolumeBounds; // 옥트리의 전체 월드 경계

	// --- 동적 장애물 데이터 ---
	TArray<F128BitChunk> DynamicObstacleBitmask;
	FIntVector MapSize; // 동적 그리드의 크기

	// --- 기타 필요한 정보 ---
	float VoxelSize = 100.0f;
	float AgentRadius = 0.0f;

private:
	FBox CalculateChildBounds(const FBox& ParentBounds, int32 ChildIndex) const
	{
		const FVector Center = ParentBounds.GetCenter();
		const FVector Min = ParentBounds.Min;
		const FVector Max = ParentBounds.Max;

		switch (ChildIndex)
		{
		case 0: return FBox(Min, Center); // ---
		case 1: return FBox(FVector(Center.X, Min.Y, Min.Z), FVector(Max.X, Center.Y, Center.Z)); // +--
		case 2: return FBox(FVector(Min.X, Center.Y, Min.Z), FVector(Center.X, Max.Y, Center.Z)); // -+-
		case 3: return FBox(FVector(Center.X, Center.Y, Min.Z), FVector(Max.X, Max.Y, Center.Z)); // ++-
		case 4: return FBox(FVector(Min.X, Min.Y, Center.Z), FVector(Center.X, Center.Y, Max.Z)); // --+
		case 5: return FBox(FVector(Center.X, Min.Y, Center.Z), FVector(Max.X, Center.Y, Max.Z)); // +-+
		case 6: return FBox(FVector(Min.X, Center.Y, Center.Z), FVector(Center.X, Max.Y, Max.Z)); // -++
		case 7: return FBox(Center, Max); // +++
		}
		return FBox(ForceInit);
	}

	int32 GetOctantContainingPosition(const FBox& NodeBounds, const FVector& Position) const
	{
		int32 Octant = 0;
		const FVector Center = NodeBounds.GetCenter();
		if (Position.X >= Center.X) Octant |= 1; // 001
		if (Position.Y >= Center.Y) Octant |= 2; // 010
		if (Position.Z >= Center.Z) Octant |= 4; // 100
		return Octant;
	}

	FBox GetNodeBoundsRecursive(int32 TargetNodeIndex, int32 CurrentNodeIndex, const FBox& CurrentNodeBounds) const
	{
		if (CurrentNodeIndex == TargetNodeIndex)
		{
			return CurrentNodeBounds;
		}

		if (!StaticNodePool.IsValidIndex(CurrentNodeIndex))
		{
			return FBox(ForceInit);
		}

		const FOctreeNode& Node = StaticNodePool[CurrentNodeIndex];
		if (Node.State != EOctreeNodeState::Mixed)
		{
			// 리프 노드인데 타겟이 아니므로 이 경로는 실패
			return FBox(ForceInit);
		}

		// 자식 노드들을 재귀적으로 탐색
		for (int i = 0; i < 8; ++i)
		{
			if (Node.ChildrenIndices[i] != -1)
			{
				const FBox ChildBounds = CalculateChildBounds(CurrentNodeBounds, i);
				const FBox Result = GetNodeBoundsRecursive(TargetNodeIndex, Node.ChildrenIndices[i], ChildBounds);
				if (Result.IsValid)
				{
					// 자식 경로에서 찾았으면 즉시 반환
					return Result;
				}
			}
		}

		return FBox(ForceInit);
	}

	FIntVector WorldToVoxelIndex(const FVector& WorldPos) const
	{
		if (MapSize.IsZero() || !VolumeBounds.IsValid) return FIntVector::ZeroValue;

		const FVector LocalPos = WorldPos - VolumeBounds.Min;
		const FVector VoxelDimensions = VolumeBounds.GetSize() / FVector(MapSize);

		// 0으로 나누기 방지
		if (VoxelDimensions.X == 0 || VoxelDimensions.Y == 0 || VoxelDimensions.Z == 0) return FIntVector::ZeroValue;

		return FIntVector(
			FMath::FloorToInt(LocalPos.X / VoxelDimensions.X),
			FMath::FloorToInt(LocalPos.Y / VoxelDimensions.Y),
			FMath::FloorToInt(LocalPos.Z / VoxelDimensions.Z)
		);
	}

public:
	// --- 공개 API 함수 구현 ---

	/** 월드 좌표를 포함하는 리프 노드의 인덱스를 찾습니다. */
	int32 FindLeafNodeAt(const FVector& WorldPos) const
	{
		if (!VolumeBounds.IsInside(WorldPos) || RootNodeIndex == -1)
		{
			return -1;
		}

		int32 CurrentNodeIndex = RootNodeIndex;
		FBox CurrentNodeBounds = VolumeBounds;
		int32 CurrentDepth = 0;

		while (StaticNodePool.IsValidIndex(CurrentNodeIndex))
		{
			const FOctreeNode& Node = StaticNodePool[CurrentNodeIndex];
			if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= MaxOctreeDepth)
			{
				return CurrentNodeIndex;
			}

			const int32 ChildOctant = GetOctantContainingPosition(CurrentNodeBounds, WorldPos);
			CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
			CurrentNodeBounds = CalculateChildBounds(CurrentNodeBounds, ChildOctant);
			CurrentDepth++;
		}
		return -1;
	}

	/** 특정 노드 인덱스에 해당하는 노드의 월드 경계를 계산합니다. */
	FBox GetNodeBounds(int32 NodeIndex) const
	{
		if (!StaticNodePool.IsValidIndex(NodeIndex))
		{
			return FBox(ForceInit);
		}
		if (NodeIndex == RootNodeIndex)
		{
			return VolumeBounds;
		}
		return GetNodeBoundsRecursive(NodeIndex, RootNodeIndex, VolumeBounds);
	}

	/** 특정 노드와 인접한 모든 'Empty' 리프 노드를 찾습니다. */
	void FindAdjacentEmptyLeaves(int32 NodeIndex, TArray<int32>& OutNeighbors) const
	{
		OutNeighbors.Empty();
		const FBox NodeBounds = GetNodeBounds(NodeIndex);
		if (!NodeBounds.IsValid) return;

		const float Epsilon = 1.0f;

		// 6개의 면(Face)에 대해 검사
		for (int32 i = 0; i < 6; ++i)
		{
			FVector TestPoint = NodeBounds.GetCenter();
			switch (i)
			{
			case 0: TestPoint.X = NodeBounds.Max.X + Epsilon;
				break; // +X
			case 1: TestPoint.X = NodeBounds.Min.X - Epsilon;
				break; // -X
			case 2: TestPoint.Y = NodeBounds.Max.Y + Epsilon;
				break; // +Y
			case 3: TestPoint.Y = NodeBounds.Min.Y - Epsilon;
				break; // -Y
			case 4: TestPoint.Z = NodeBounds.Max.Z + Epsilon;
				break; // +Z
			case 5: TestPoint.Z = NodeBounds.Min.Z - Epsilon;
				break; // -Z
			}

			if (!VolumeBounds.IsInside(TestPoint)) continue;

			const int32 AdjacentNodeIndex = FindLeafNodeAt(TestPoint);
			if (StaticNodePool.IsValidIndex(AdjacentNodeIndex) &&
				StaticNodePool[AdjacentNodeIndex].State == EOctreeNodeState::Empty &&
				!OutNeighbors.Contains(AdjacentNodeIndex))
			{
				OutNeighbors.Add(AdjacentNodeIndex);
			}
		}
	}

	/** 특정 월드 영역이 동적 장애물에 의해 막혔는지 확인합니다. */
	bool IsRegionDynamicallyBlocked(const FBox& QueryBox) const
	{
		const FIntVector MinCorner = WorldToVoxelIndex(QueryBox.Min);
		// Max 코너는 복셀의 끝 지점까지 포함해야 하므로 1을 뺀 인덱스를 사용
		const FIntVector MaxCorner = WorldToVoxelIndex(QueryBox.Max - FVector(KINDA_SMALL_NUMBER));

		const FIntVector ClampedMin(FMath::Max(MinCorner.X, 0), FMath::Max(MinCorner.Y, 0), FMath::Max(MinCorner.Z, 0));
		const FIntVector ClampedMax(FMath::Min(MaxCorner.X, MapSize.X - 1), FMath::Min(MaxCorner.Y, MapSize.Y - 1),
		                            FMath::Min(MaxCorner.Z, MapSize.Z - 1));

		if (ClampedMin.X > ClampedMax.X || ClampedMin.Y > ClampedMax.Y || ClampedMin.Z > ClampedMax.Z) return false;
		if (DynamicObstacleBitmask.Num() == 0) return false;

		auto CreateMask = [](int32 StartBit, int32 EndBit) -> __m128i
		{
			uint64 LowMask = (StartBit < 64) ? (~0ULL << StartBit) : 0;
			if (EndBit < 64) LowMask &= (~0ULL >> (63 - EndBit));
			uint64 HighMask = (EndBit >= 64) ? (~0ULL >> (127 - EndBit)) : 0;
			if (StartBit >= 64) HighMask &= (~0ULL << (StartBit - 64));
			return _mm_set_epi64x(HighMask, LowMask);
		};

		const int32 ChunksPerRow = MapSize.X / 128;
		for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
		{
			for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
			{
				const int32 RowStartChunkIndex = (z * MapSize.Y + y) * ChunksPerRow;
				const int32 StartChunkIndex = RowStartChunkIndex + ClampedMin.X / 128;
				const int32 EndChunkIndex = RowStartChunkIndex + ClampedMax.X / 128;
				const int32 StartBitInChunk = ClampedMin.X % 128;
				const int32 EndBitInChunk = ClampedMax.X % 128;

				if (StartChunkIndex == EndChunkIndex)
				{
					if (!DynamicObstacleBitmask.IsValidIndex(StartChunkIndex)) continue;
					const __m128i Mask = CreateMask(StartBitInChunk, EndBitInChunk);
					const __m128i ChunkData = _mm_loadu_si128(
						reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[StartChunkIndex]));
					if (_mm_testz_si128(ChunkData, Mask) == 0) return true; // 막힘
				}
				else
				{
					if (DynamicObstacleBitmask.IsValidIndex(StartChunkIndex))
					{
						__m128i StartMask = CreateMask(StartBitInChunk, 127);
						__m128i ChunkData = _mm_loadu_si128(
							reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[StartChunkIndex]));
						if (_mm_testz_si128(ChunkData, StartMask) == 0) return true;
					}

					for (int32 i = StartChunkIndex + 1; i < EndChunkIndex; ++i)
					{
						if (!DynamicObstacleBitmask.IsValidIndex(i)) continue;
						const __m128i MiddleChunkData = _mm_loadu_si128(
							reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[i]));
						if (_mm_testz_si128(MiddleChunkData, _mm_set1_epi32(-1)) == 0) return true;
					}

					if (DynamicObstacleBitmask.IsValidIndex(EndChunkIndex))
					{
						__m128i EndMask = CreateMask(0, EndBitInChunk);
						__m128i ChunkData = _mm_loadu_si128(
							reinterpret_cast<const __m128i*>(&DynamicObstacleBitmask[EndChunkIndex]));
						if (_mm_testz_si128(ChunkData, EndMask) == 0) return true;
					}
				}
			}
		}
		return false; // 안 막힘
	}
};
