// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavData/StaticVoxelChunkData.h"
#include "NavData/OctreeDataAsset.h"

struct FOctreePathfindingData
{
	const TArray<FOctreeNode>* StaticNodePoolPtr = nullptr;
	int32 RootNodeIndex = -1;
	int32 MaxOctreeDepth = 0;
	FBox VolumeBounds;

	TArray<F128BitChunk> DynamicObstacleBitmask;
	FIntVector MapSize;
	float VoxelSize = 100.0f;

	int32 FindNearestValidLeafForAgent(const FVector& Origin, float AgentRadius, float InitialSearchRadius,
	                                   int32 MaxIterations) const
	{
		if (RootNodeIndex == -1 || !StaticNodePoolPtr) return -1;

		const float MaxSearchRadius = InitialSearchRadius * MaxIterations;
		const FBox SearchBox(Origin - FVector(MaxSearchRadius), Origin + FVector(MaxSearchRadius));

		TArray<int32> CandidateLeaves;
		FindLeavesInBoxRecursive(CandidateLeaves, RootNodeIndex, SearchBox);

		int32 BestNodeIndex = -1;
		float MinDistSq = TNumericLimits<float>::Max();

		for (const int32 LeafIndex : CandidateLeaves)
		{
			if (StaticNodePoolPtr->IsValidIndex(LeafIndex))
			{
				const FOctreeNode& LeafNode = (*StaticNodePoolPtr)[LeafIndex];

				if (LeafNode.State == EOctreeNodeState::Empty)
				{
					const float DistSq = FVector::DistSquared(Origin, LeafNode.Bounds.GetCenter());
					if (DistSq < MinDistSq)
					{
						MinDistSq = DistSq;
						BestNodeIndex = LeafIndex;
					}
				}
			}
		}

		return BestNodeIndex;
	}

private:
	int32 GetOctantContainingPosition(const FBox& NodeBounds, const FVector& Position) const
	{
		int32 Octant = 0;
		const FVector Center = NodeBounds.GetCenter();
		if (Position.X >= Center.X) Octant |= 1; // 001
		if (Position.Y >= Center.Y) Octant |= 2; // 010
		if (Position.Z >= Center.Z) Octant |= 4; // 100
		return Octant;
	}

	FIntVector WorldToVoxelIndex(const FVector& WorldPos) const
	{
		if (MapSize.IsZero() || !VolumeBounds.IsValid) return FIntVector::ZeroValue;

		const FVector LocalPos = WorldPos - VolumeBounds.Min;
		const FVector VoxelDimensions = VolumeBounds.GetSize() / FVector(MapSize);

		if (VoxelDimensions.X == 0 || VoxelDimensions.Y == 0 || VoxelDimensions.Z == 0) return FIntVector::ZeroValue;

		return FIntVector(
			FMath::FloorToInt(LocalPos.X / VoxelDimensions.X),
			FMath::FloorToInt(LocalPos.Y / VoxelDimensions.Y),
			FMath::FloorToInt(LocalPos.Z / VoxelDimensions.Z)
		);
	}

	float SegmentBoxDistanceSquared(const FVector& SegStart, const FVector& SegEnd, const FBox& Box) const
	{
		// --- 1. 사전 검사: 빠른 예외 처리 ---

		// 선분이 박스 내부에 완전히 포함되거나, 일부라도 교차하는지 빠르게 확인합니다.
		// 이 검사는 가장 흔한 경우(충돌)를 매우 빠르게 처리해줍니다.
		if (Box.IsInside(SegStart) || Box.IsInside(SegEnd) || FMath::LineBoxIntersection(
			Box, SegStart, SegEnd, SegEnd - SegStart))
		{
			return 0.0f;
		}

		// 선분이 사실상 점인 경우(degenerate case), 점에서 박스까지의 거리로 처리합니다.
		if (FVector::PointsAreSame(SegStart, SegEnd))
		{
			const FVector PointOnBox = Box.GetClosestPointTo(SegStart);
			return FVector::DistSquared(SegStart, PointOnBox);
		}

		// --- 2. 반복 최적화: 가장 가까운 점 찾기 ---
		// 핵심 원리:
		// 1. 선분 위의 한 점(P_seg)을 잡습니다.
		// 2. 박스에서 P_seg와 가장 가까운 점(P_box)을 찾습니다. (Box.GetClosestPointTo)
		// 3. 다시 선분에서 P_box와 가장 가까운 점(P_seg)을 찾습니다. (FMath::ClosestPointOnSegment)
		// 4. 이 과정을 반복하면 P_seg와 P_box는 서로에게 가장 가까운 지점으로 빠르게 수렴합니다.

		FVector PointOnSegment = (SegStart + SegEnd) * 0.5f; // 시작점으로 선분의 중앙을 사용
		FVector PointOnBox;

		// 보통 5~10회 반복으로 충분히 수렴하지만, 넉넉하게 10회로 설정합니다.
		const int32 MaxIterations = 10;
		for (int32 i = 0; i < MaxIterations; ++i)
		{
			const FVector PrevPointOnSegment = PointOnSegment;

			// 현재 선분 위의 점에서 가장 가까운 박스 위의 점을 찾습니다.
			PointOnBox = Box.GetClosestPointTo(PointOnSegment);

			// 위에서 찾은 박스 위의 점에서 가장 가까운 선분 위의 점을 다시 찾습니다.
			PointOnSegment = FMath::ClosestPointOnSegment(PointOnBox, SegStart, SegEnd);

			// 이전 단계와 거의 변화가 없다면 수렴한 것으로 보고 조기 종료합니다.
			if (FVector::PointsAreSame(PointOnSegment, PrevPointOnSegment))
			{
				break;
			}
		}

		// --- 3. 최종 계산 ---
		// 최종적으로 수렴한 두 점 사이의 거리 제곱을 반환합니다.
		PointOnBox = Box.GetClosestPointTo(PointOnSegment);
		return FVector::DistSquared(PointOnSegment, PointOnBox);
	}

	bool IntersectsCapsule(const FBox& Box, const FVector& Start, const FVector& End, float Radius) const
	{
		const FBox ShrunkBox = Box.ExpandBy(-Radius);
		if (!ShrunkBox.IsValid)
		{
			return false;
		}
		// 이제 반지름 없이 선-박스 교차 검사
		return FMath::LineBoxIntersection(ShrunkBox, Start, End, End - Start);

	}

	void FindLeavesInBoxRecursive(TArray<int32>& OutLeaves, int32 NodeIndex, const FBox& QueryBox) const
	{
		if (!StaticNodePoolPtr) return;
		if (!StaticNodePoolPtr->IsValidIndex(NodeIndex)) return;

		const FBox NodeBounds = GetNodeBounds(NodeIndex);

		if (!NodeBounds.Intersect(QueryBox))
		{
			return;
		}

		const FOctreeNode& Node = (*StaticNodePoolPtr)[NodeIndex];

		if (Node.State != EOctreeNodeState::Mixed)
		{
			OutLeaves.Add(NodeIndex);
			return;
		}

		for (int32 i = 0; i < 8; ++i)
		{
			if (Node.ChildrenIndices[i] != -1)
			{
				FindLeavesInBoxRecursive(OutLeaves, Node.ChildrenIndices[i], QueryBox);
			}
		}
	}

public:
	bool CheckForSolidNodesInCapsule(const FVector& Start, const FVector& End, float Radius) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Octree_CheckForSolidNodesInCapsule_Iterative);

		if (RootNodeIndex == -1 || !StaticNodePoolPtr) return true;

		const int32 StartLeaf = FindLeafNodeAt(Start);
		if (StartLeaf != -1 && (*StaticNodePoolPtr)[StartLeaf].State == EOctreeNodeState::Solid) return true;


		const int32 EndLeaf = FindLeafNodeAt(End);
		if (EndLeaf != -1 && (*StaticNodePoolPtr)[EndLeaf].State == EOctreeNodeState::Solid) return true;
		
		TArray<int32> NodesToVisit;
		NodesToVisit.Push(RootNodeIndex);

		if (!StaticNodePoolPtr) return true;
		while (NodesToVisit.Num() > 0)
		{
			const int32 CurrentNodeIndex = NodesToVisit.Pop();
			if (!StaticNodePoolPtr->IsValidIndex(CurrentNodeIndex)) continue;

			const FOctreeNode& Node = (*StaticNodePoolPtr)[CurrentNodeIndex];
			const float MinDistSq = SegmentBoxDistanceSquared(Start, End, Node.Bounds);
			if (MinDistSq > (Radius * Radius))
			{
				continue;
			}
			if (Node.State == EOctreeNodeState::Mixed)
			{
				for (int32 i = 0; i < 8; ++i)
				{
					if (Node.ChildrenIndices[i] != -1) NodesToVisit.Push(Node.ChildrenIndices[i]);
				}
			}
			else
			{
				if (Node.State == EOctreeNodeState::Solid)
				{
					return true;
				}
			}
		}

		return false;
	}

	int32 FindLeafNodeAt(const FVector& WorldPos) const
	{
		if (!VolumeBounds.IsInside(WorldPos) || RootNodeIndex == -1)
		{
			return -1;
		}

		int32 CurrentNodeIndex = RootNodeIndex;
		int32 CurrentDepth = 0;

		while (StaticNodePoolPtr->IsValidIndex(CurrentNodeIndex))
		{
			const FOctreeNode& Node = (*StaticNodePoolPtr)[CurrentNodeIndex];
			if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= MaxOctreeDepth)
			{
				return CurrentNodeIndex;
			}

			const int32 ChildOctant = GetOctantContainingPosition(Node.Bounds, WorldPos);
			CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
			CurrentDepth++;
		}
		return -1;
	}

	FBox GetNodeBounds(int32 NodeIndex) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Octree_GetNodeBounds);
		if (!StaticNodePoolPtr)
		{
			return FBox(ForceInit);
		}
		if (!StaticNodePoolPtr->IsValidIndex(NodeIndex))
		{
			return FBox(ForceInit);
		}

		return (*StaticNodePoolPtr)[NodeIndex].Bounds;
	}

	void FindAdjacentEmptyLeaves(int32 NodeIndex, TArray<int32>& OutNeighbors) const
	{
		OutNeighbors.Empty();

		if (!StaticNodePoolPtr)
		{
			return;
		}

		if (!StaticNodePoolPtr->IsValidIndex(NodeIndex))
		{
			return;
		}

		const FOctreeNode& Node = (*StaticNodePoolPtr)[NodeIndex];

		for (const int32 NeighborIndex : Node.Neighbors)
		{
			if (StaticNodePoolPtr->IsValidIndex(NeighborIndex) &&
				(*StaticNodePoolPtr)[NeighborIndex].State == EOctreeNodeState::Empty)
			{
				OutNeighbors.Add(NeighborIndex);
			}
		}
	}

	bool IsPathDynamicallyBlocked_Sweep(const FVector& Start, const FVector& End, float AgentRadius) const
	{
		const FVector PathVector = End - Start;
		const float TotalDistance = PathVector.Size();
		if (TotalDistance < KINDA_SMALL_NUMBER)
		{
			return IsRegionDynamicallyBlocked(FBox::BuildAABB(Start, FVector(AgentRadius)));
		}

		const FVector Direction = PathVector / TotalDistance;

		const float StepSize = AgentRadius;

		for (float Distance = 0; Distance < TotalDistance; Distance += StepSize)
		{
			const FVector CurrentPosition = Start + Direction * Distance;

			const FBox QueryBox = FBox::BuildAABB(CurrentPosition, FVector(AgentRadius));
			if (IsRegionDynamicallyBlocked(QueryBox))
			{
				return true;
			}
		}

		const FBox EndQueryBox = FBox::BuildAABB(End, FVector(AgentRadius));
		if (IsRegionDynamicallyBlocked(EndQueryBox))
		{
			return true;
		}

		return false;
	}

	bool IsRegionDynamicallyBlocked(const FBox& QueryBox) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Octree_IsRegionDynamicallyBlocked);

		const FIntVector MinCorner = WorldToVoxelIndex(QueryBox.Min);

		const FVector LocalMax = QueryBox.Max - VolumeBounds.Min;
		const FVector VoxelDimensions = VolumeBounds.GetSize() / FVector(MapSize);

		if (VoxelDimensions.X == 0 || VoxelDimensions.Y == 0 || VoxelDimensions.Z == 0) return false;

		const FIntVector MaxCorner(
			FMath::CeilToInt(LocalMax.X / VoxelDimensions.X),
			FMath::CeilToInt(LocalMax.Y / VoxelDimensions.Y),
			FMath::CeilToInt(LocalMax.Z / VoxelDimensions.Z)
		);

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
					if (_mm_testz_si128(ChunkData, Mask) == 0) return true;
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
		return false;
	}
};
