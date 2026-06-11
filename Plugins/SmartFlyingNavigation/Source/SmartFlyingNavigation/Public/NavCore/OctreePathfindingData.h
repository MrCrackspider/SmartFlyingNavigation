// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavCore/NavNodeTypes.h"
#include "NavCore/NavDistanceField.h"
#include "NavCore/DynamicObstacleRegistry.h"

/**
 * Baked navigation payload for one chunk. Owned via thread-safe shared pointer:
 * the volume holds the authoritative reference and every pathfinding snapshot
 * holds another, so in-flight worker tasks keep the data alive even when the
 * volume streams out or re-initializes mid-task.
 */
struct FLoadedNavChunkData
{
	TArray<FOctreeNode> NodePool;
	FAirwayGraph AirwayGraph;
	FNavDistanceField DistanceField;
};

/**
 * Thread-safe snapshot of navigation data for async pathfinding.
 * Dynamic obstacles are an owned snapshot of analytic primitives, composited
 * with the static distance field as min(static clearance, primitive distance).
 */
struct FOctreePathfindingData
{
	/**
	 * Keep-alive reference for the chunk payload. StaticNodePoolPtr,
	 * AirwayGraphPtr, and DistanceFieldPtr point into this object.
	 */
	TSharedPtr<const FLoadedNavChunkData, ESPMode::ThreadSafe> ChunkDataRef;

	const TArray<FOctreeNode>* StaticNodePoolPtr = nullptr;
	const FAirwayGraph* AirwayGraphPtr = nullptr;
	int32 RootNodeIndex = -1;
	int32 MaxOctreeDepth = 0;
	FBox VolumeBounds;

	/** Owned snapshot of dynamic obstacle primitives relevant to this volume. */
	TArray<FDynamicObstaclePrim> DynamicObstacles;

	FIntVector MapSize;
	float VoxelSize = 100.0f;

	/** Baked navigation distance field for this chunk (read-only after load; safe on worker threads). */
	const FNavDistanceField* DistanceFieldPtr = nullptr;

	/** Snapshot of the sfn.Path.UseSDF cvar at view-creation time. */
	bool bUseDistanceField = true;

	FORCEINLINE bool ShouldUseDistanceField() const
	{
		return bUseDistanceField && DistanceFieldPtr && DistanceFieldPtr->IsValid();
	}

	FORCEINLINE bool HasDynamicObstacleData() const
	{
		return DynamicObstacles.Num() > 0;
	}

	/** Minimum distance from a point to any dynamic obstacle (large value when none). */
	float DynamicObstacleDistance(const FVector& Point) const
	{
		float MinDistance = TNumericLimits<float>::Max();
		for (const FDynamicObstaclePrim& Prim : DynamicObstacles)
		{
			MinDistance = FMath::Min(MinDistance, Prim.Distance(Point));
			if (MinDistance <= 0.f)
			{
				break;
			}
		}
		return MinDistance;
	}

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

				if (LeafNode.State != EOctreeNodeState::Empty)
				{
					continue;
				}

				const FVector CandidateCenter = LeafNode.Bounds.GetCenter();

				const bool bIsStaticallySafe = !IsPointBlockedForAgent(CandidateCenter, AgentRadius, true, false);
				const bool bIsDynamicallySafe = !IsRegionDynamicallyBlocked(
					FBox::BuildAABB(CandidateCenter, FVector(AgentRadius)));

				if (bIsStaticallySafe && bIsDynamicallySafe)
				{
					const float DistSq = FVector::DistSquared(Origin, CandidateCenter);
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
		if (VoxelSize <= 0.f || !VolumeBounds.IsValid) return FIntVector::ZeroValue;

		const FVector LocalPos = WorldPos - VolumeBounds.Min;

		return FIntVector(
			FMath::FloorToInt(LocalPos.X / VoxelSize),
			FMath::FloorToInt(LocalPos.Y / VoxelSize),
			FMath::FloorToInt(LocalPos.Z / VoxelSize)
		);
	}

	float SegmentBoxDistanceSquared(const FVector& SegStart, const FVector& SegEnd, const FBox& Box) const
	{
		if (Box.IsInside(SegStart) || Box.IsInside(SegEnd) || FMath::LineBoxIntersection(
			Box, SegStart, SegEnd, SegEnd - SegStart))
		{
			return 0.0f;
		}

		if (FVector::PointsAreSame(SegStart, SegEnd))
		{
			const FVector PointOnBox = Box.GetClosestPointTo(SegStart);
			return FVector::DistSquared(SegStart, PointOnBox);
		}


		FVector PointOnSegment = (SegStart + SegEnd) * 0.5f;
		FVector PointOnBox;

		const int32 MaxIterations = 10;
		for (int32 i = 0; i < MaxIterations; ++i)
		{
			const FVector PrevPointOnSegment = PointOnSegment;

			PointOnBox = Box.GetClosestPointTo(PointOnSegment);

			PointOnSegment = FMath::ClosestPointOnSegment(PointOnBox, SegStart, SegEnd);

			if (FVector::PointsAreSame(PointOnSegment, PrevPointOnSegment))
			{
				break;
			}
		}

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

		const FBox CapsuleAABB(FVector::Min(Start, End) - FVector(Radius), FVector::Max(Start, End) + FVector(Radius));

		if (!(*StaticNodePoolPtr)[RootNodeIndex].Bounds.Intersect(CapsuleAABB))
		{
			return false;
		}

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

	/** Sphere-traced dynamic obstacle test along a segment using exact primitive distances. */
	bool IsPathDynamicallyBlocked(const FVector& Start, const FVector& End, float AgentRadius) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Octree_IsPathDynamicallyBlocked);

		if (AgentRadius <= 0.f || DynamicObstacles.Num() == 0)
		{
			return false;
		}

		const FVector Delta = End - Start;
		const float TotalDistance = Delta.Size();
		const float MinStep = FMath::Max(AgentRadius * 0.25f, 10.f);

		if (TotalDistance < KINDA_SMALL_NUMBER)
		{
			return DynamicObstacleDistance(Start) < AgentRadius;
		}

		const FVector Direction = Delta / TotalDistance;

		float Travelled = 0.f;
		while (Travelled < TotalDistance)
		{
			const FVector Sample = Start + Direction * Travelled;
			const float Distance = DynamicObstacleDistance(Sample);
			if (Distance < AgentRadius)
			{
				return true;
			}

			Travelled += FMath::Max(Distance - AgentRadius, MinStep);
		}

		return DynamicObstacleDistance(End) < AgentRadius;
	}

	/** Conservative AABB-vs-primitive overlap test for a region. */
	bool IsRegionDynamicallyBlocked(const FBox& QueryBox) const
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Octree_IsRegionDynamicallyBlocked);

		for (const FDynamicObstaclePrim& Prim : DynamicObstacles)
		{
			if (Prim.GetBounds().Intersect(QueryBox))
			{
				// Refine with the exact distance to the box center.
				if (Prim.Distance(QueryBox.GetCenter()) <= QueryBox.GetExtent().GetMax())
				{
					return true;
				}
			}
		}
		return false;
	}

	bool IsPointBlockedForAgent(const FVector& Location, float AgentRadius, bool bCheckStatic = true,
	                            bool bCheckDynamic = true) const
	{
		return IsSegmentBlockedForAgent(Location, Location, AgentRadius, bCheckStatic, bCheckDynamic);
	}

	bool IsSegmentBlockedForAgent(const FVector& Start, const FVector& End, float AgentRadius,
	                              bool bCheckStatic = true, bool bCheckDynamic = true) const
	{
		if (AgentRadius <= 0.f)
		{
			return false;
		}

		if (bCheckStatic)
		{
			// Sphere tracing against the baked distance field replaces the per-step
			// octree capsule descent (typically 3-8 samples per segment instead of dozens).
			if (ShouldUseDistanceField())
			{
				if (!DistanceFieldPtr->IsSegmentClear(Start, End, AgentRadius))
				{
					return true;
				}
			}
			else if (CheckForSolidNodesInCapsule(Start, End, AgentRadius))
			{
				return true;
			}
		}

		if (bCheckDynamic && IsPathDynamicallyBlocked(Start, End, AgentRadius))
		{
			return true;
		}

		return false;
	}
};
