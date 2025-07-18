// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/NavData/StaticVoxelChunkData.h"
#include "Pathfinder/NavData/OctreeDataAsset.h"

struct FOctreePathfindingData
{
	TArray<FOctreeNode> StaticNodePool;
	int32 RootNodeIndex = -1;
	int32 MaxOctreeDepth = 0;
	FBox VolumeBounds;

	TArray<F128BitChunk> DynamicObstacleBitmask;
	FIntVector MapSize;
	float VoxelSize = 100.0f;
	
	int32 FindNearestValidLeafForAgent(const FVector& Origin,float AgentRadius, float InitialSearchRadius, int32 MaxIterations) const
	{
		if (RootNodeIndex == -1) return -1;

		const float MaxSearchRadius = InitialSearchRadius * MaxIterations;
		const FBox SearchBox(Origin - FVector(MaxSearchRadius), Origin + FVector(MaxSearchRadius));

		TArray<int32> CandidateLeaves;
		FindLeavesInBoxRecursive(CandidateLeaves, RootNodeIndex, SearchBox);

		int32 BestNodeIndex = -1;
		float MinDistSq = TNumericLimits<float>::Max();

		for (const int32 LeafIndex : CandidateLeaves)
		{
			if (StaticNodePool.IsValidIndex(LeafIndex))
			{
				const FOctreeNode& LeafNode = StaticNodePool[LeafIndex];
            
				if (LeafNode.State == EOctreeNodeState::Empty && LeafNode.MaxAllowedRadius >= AgentRadius)
				{
					if (!IsRegionDynamicallyBlocked(LeafNode.Bounds))
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
		}

		return BestNodeIndex;
	}
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
		default: return FBox();
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

	bool IntersectsCapsule(const FBox& Box, const FVector& Start, const FVector& End, float Radius) const
	{
		const FVector PointOnSegment = FMath::ClosestPointOnSegment(Box.GetCenter(), Start, End);
		const FVector ClosestPointOnBox = Box.GetClosestPointTo(PointOnSegment);

		const float DistSq = FVector::DistSquared(ClosestPointOnBox, PointOnSegment);

		return DistSq <= (Radius * Radius);
	}
	
	bool CheckForSolidNodesInCapsuleRecursive(int32 NodeIndex, const FVector& Start, const FVector& End,
	                                          float Radius) const
	{
		if (!StaticNodePool.IsValidIndex(NodeIndex))
		{
			return false;
		}

		const FOctreeNode& Node = StaticNodePool[NodeIndex];
		const FBox& NodeBounds = GetNodeBounds(NodeIndex);

		if (!IntersectsCapsule(NodeBounds, Start, End, Radius))
		{
			return false;
		}

		if (Node.State != EOctreeNodeState::Mixed)
		{
			if (Node.State == EOctreeNodeState::Solid)
			{
				return true;
			}

			if (IsRegionDynamicallyBlocked(NodeBounds))
			{
				return true;
			}

			return false;
		}

		for (int32 i = 0; i < 8; ++i)
		{
			const int32 ChildIndex = Node.ChildrenIndices[i];
			if (ChildIndex != -1)
			{
				if (CheckForSolidNodesInCapsuleRecursive(ChildIndex, Start, End, Radius))
				{
					return true;
				}
			}
		}

		return false;
	}

	void FindLeavesInBoxRecursive(TArray<int32>& OutLeaves, int32 NodeIndex, const FBox& QueryBox) const
	{
		if (!StaticNodePool.IsValidIndex(NodeIndex)) return;

		const FBox NodeBounds = GetNodeBounds(NodeIndex);

		if (!NodeBounds.Intersect(QueryBox))
		{
			return;
		}

		const FOctreeNode& Node = StaticNodePool[NodeIndex];

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

	bool VoxelRayMarch(const FVector& Start, const FVector& End, TArray<int32>& OutPathNodes) const
	{
		OutPathNodes.Empty();
		FVector RayDirection = End - Start;
		const float RayLength = RayDirection.Size();
		if (RayLength < KINDA_SMALL_NUMBER) return true;
		RayDirection /= RayLength; // Normalize

		FVector CurrentPos = Start;
		int32 LastNodeIndex = -1;

		for (float Dist = 0; Dist < RayLength; Dist += VoxelSize * 0.5f)
		{
			CurrentPos = Start + RayDirection * Dist;
			const int32 CurrentNodeIndex = FindLeafNodeAt(CurrentPos);

			if (CurrentNodeIndex != -1 && CurrentNodeIndex != LastNodeIndex)
			{
				if (StaticNodePool[CurrentNodeIndex].State == EOctreeNodeState::Solid)
				{
					return false;
				}
				OutPathNodes.Add(CurrentNodeIndex);
				LastNodeIndex = CurrentNodeIndex;
			}
		}
		return true;
	}
	bool CheckForSolidNodesInCapsule(const FVector& Start, const FVector& End, float Radius) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Octree_CheckForSolidNodesInCapsule_Iterative);

		if (RootNodeIndex == -1) return true;

		TArray<int32> NodesToVisit;
		NodesToVisit.Push(RootNodeIndex);

		while (NodesToVisit.Num() > 0)
		{
			const int32 CurrentNodeIndex = NodesToVisit.Pop();
			if (!StaticNodePool.IsValidIndex(CurrentNodeIndex)) continue;

			const FOctreeNode& Node = StaticNodePool[CurrentNodeIndex];
			if (!IntersectsCapsule(Node.Bounds, Start, End, Radius))
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
				if (Node.State == EOctreeNodeState::Solid) return true;
				if (IsRegionDynamicallyBlocked(Node.Bounds)) return true;
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

		while (StaticNodePool.IsValidIndex(CurrentNodeIndex))
		{
			const FOctreeNode& Node = StaticNodePool[CurrentNodeIndex];
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

		if (!StaticNodePool.IsValidIndex(NodeIndex))
		{
			return FBox(ForceInit);
		}

		return StaticNodePool[NodeIndex].Bounds;
	}

	void FindAdjacentEmptyLeaves(int32 NodeIndex, TArray<int32>& OutNeighbors) const
	{
		OutNeighbors.Empty();
		if (!StaticNodePool.IsValidIndex(NodeIndex))
		{
			return;
		}

		const FOctreeNode& Node = StaticNodePool[NodeIndex];

		for (const int32 NeighborIndex : Node.Neighbors)
		{

			if (StaticNodePool.IsValidIndex(NeighborIndex) &&
				StaticNodePool[NeighborIndex].State == EOctreeNodeState::Empty)
			{
				OutNeighbors.Add(NeighborIndex);
			}
		}
	}
	
	bool IsRegionDynamicallyBlocked(const FBox& QueryBox) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Octree_IsRegionDynamicallyBlocked);

		const FIntVector MinCorner = WorldToVoxelIndex(QueryBox.Min);
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
		return false;
	}

	int32 FindNearestEmptyLeaf(const FVector& Origin, float InitialSearchRadius, int32 MaxIterations) const
	{
		if (RootNodeIndex == -1) return -1;

		const float MaxSearchRadius = InitialSearchRadius * MaxIterations;
		const FBox SearchBox(Origin - FVector(MaxSearchRadius), Origin + FVector(MaxSearchRadius));

		TArray<int32> CandidateLeaves;
		FindLeavesInBoxRecursive(CandidateLeaves, RootNodeIndex, SearchBox);

		int32 BestNodeIndex = -1;
		float MinDistSq = TNumericLimits<float>::Max();

		for (const int32 LeafIndex : CandidateLeaves)
		{
			if (StaticNodePool.IsValidIndex(LeafIndex) && StaticNodePool[LeafIndex].State == EOctreeNodeState::Empty)
			{
				const FBox& LeafBounds = StaticNodePool[LeafIndex].Bounds; // GetNodeBounds 대신 직접 접근
				if (!IsRegionDynamicallyBlocked(LeafBounds))
				{
					const float DistSq = FVector::DistSquared(Origin, LeafBounds.GetCenter());
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
};
