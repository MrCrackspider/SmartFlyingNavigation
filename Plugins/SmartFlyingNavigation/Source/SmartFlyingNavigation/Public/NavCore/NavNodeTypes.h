// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "NavNodeTypes.generated.h"

UENUM(BlueprintType)
enum class EOctreeNodeState : uint8
{
	Empty,
	Solid,
	Mixed
};

UENUM()
enum class ENeighborDirection : uint8
{
	Right, Left, Front, Back, Up, Down, MAX
};

USTRUCT(BlueprintType)
struct FOctreeNode
{
	GENERATED_BODY()

	UPROPERTY()
	int32 ChildrenIndices[8];

	UPROPERTY()
	EOctreeNodeState State;

	UPROPERTY()
	FBox Bounds;

	UPROPERTY()
	int32 Neighbors[static_cast<int32>(ENeighborDirection::MAX)];

	UPROPERTY()
	float MaxAllowedRadius = 0.f;

	FOctreeNode() : State(EOctreeNodeState::Empty)
	{
		FMemory::Memset(ChildrenIndices, -1, sizeof(ChildrenIndices));
		FMemory::Memset(Neighbors, -1, sizeof(Neighbors));
	}
};

/**
 * Compact airway (free-space) graph in CSR (compressed sparse row) form.
 *
 * - Node: 8 bytes — half-voxel lattice coords (uint16×3, lossless: every octree
 *   leaf center lies on the half-voxel lattice) + quantized clearance.
 * - Edge: 4 bytes — 24-bit target index + 8-bit corridor clearance (8 cm units).
 * - Adjacency: offsets array, so only existing edges are stored (no fixed 6 slots).
 *
 * The in-memory CSR is symmetric (both directions present, required for A*
 * traversal). On disk only the half with Target > Source is stored — clearance
 * is symmetric (min of both endpoints) so the mirror is reconstructed losslessly
 * on load. This halves the edge payload (~ -2.3 MB on a 256³ maze chunk).
 */
struct SMARTFLYINGNAVIGATION_API FAirwayGraph
{
	struct FCompactNode
	{
		uint16 X = 0, Y = 0, Z = 0;   // half-voxel lattice coords from OriginWS
		uint8 ClearanceQ = 0;         // 8 cm units; 255 = "at least 20.4 m"
		uint8 Padding = 0;
	};

	static constexpr uint32 EdgeIndexMask = 0x00FFFFFFu;
	static constexpr float ClearanceUnitCm = 8.f;

	FVector3f OriginWS = FVector3f::ZeroVector;
	float HalfVoxelSize = 50.f;

	TArray<FCompactNode> Nodes;
	TArray<uint32> AdjOffsets;  // Nodes.Num()+1 entries
	TArray<uint32> Edges;       // (ClearanceQ << 24) | TargetIndex

	int32 Num() const { return Nodes.Num(); }
	bool IsEmpty() const { return Nodes.IsEmpty(); }
	bool IsValidIndex(int32 Index) const { return Nodes.IsValidIndex(Index); }

	FVector GetPosition(int32 Index) const
	{
		const FCompactNode& N = Nodes[Index];
		return FVector(OriginWS) + FVector(N.X, N.Y, N.Z) * HalfVoxelSize;
	}

	static float DequantizeClearance(uint8 Q)
	{
		return Q == 0xFF ? TNumericLimits<float>::Max() : static_cast<float>(Q) * ClearanceUnitCm;
	}

	static uint8 QuantizeClearance(float ClearanceCm)
	{
		// Conservative floor; 255 is reserved for "unbounded".
		return static_cast<uint8>(FMath::Clamp(FMath::FloorToInt(ClearanceCm / ClearanceUnitCm), 0, 254));
	}

	float GetClearance(int32 Index) const { return DequantizeClearance(Nodes[Index].ClearanceQ); }

	void GetEdgeRange(int32 Index, uint32& OutBegin, uint32& OutEnd) const
	{
		OutBegin = AdjOffsets[Index];
		OutEnd = AdjOffsets[Index + 1];
	}

	static int32 GetEdgeTarget(uint32 Edge) { return static_cast<int32>(Edge & EdgeIndexMask); }
	static float GetEdgeClearance(uint32 Edge) { return DequantizeClearance(static_cast<uint8>(Edge >> 24)); }

	int64 GetAllocatedBytes() const
	{
		return Nodes.GetAllocatedSize() + AdjOffsets.GetAllocatedSize() + Edges.GetAllocatedSize();
	}

	void Reset()
	{
		Nodes.Empty();
		AdjOffsets.Empty();
		Edges.Empty();
	}

	friend FArchive& operator<<(FArchive& Ar, FCompactNode& Node)
	{
		Ar << Node.X << Node.Y << Node.Z << Node.ClearanceQ << Node.Padding;
		return Ar;
	}

	/** Rebuilds the symmetric runtime CSR from half-edges (one entry per undirected pair, Target > Source). */
	void BuildFromUndirectedEdges(const TArray<uint32>& HalfOffsets, const TArray<uint32>& HalfEdges)
	{
		AdjOffsets.Reset();
		Edges.Reset();

		const int32 NodeCount = Nodes.Num();
		if (NodeCount == 0 || HalfOffsets.Num() != NodeCount + 1)
		{
			AdjOffsets.Init(0, NodeCount + 1);
			return;
		}

		TArray<uint32> Degree;
		Degree.SetNumZeroed(NodeCount);
		for (int32 Source = 0; Source < NodeCount; ++Source)
		{
			for (uint32 EdgeIndex = HalfOffsets[Source]; EdgeIndex < HalfOffsets[Source + 1]; ++EdgeIndex)
			{
				const int32 Target = GetEdgeTarget(HalfEdges[EdgeIndex]);
				if (Nodes.IsValidIndex(Target))
				{
					++Degree[Source];
					++Degree[Target];
				}
			}
		}

		AdjOffsets.SetNumUninitialized(NodeCount + 1);
		AdjOffsets[0] = 0;
		for (int32 Index = 0; Index < NodeCount; ++Index)
		{
			AdjOffsets[Index + 1] = AdjOffsets[Index] + Degree[Index];
		}

		// Reuse Degree as per-node write cursors while mirroring each half-edge.
		TArray<uint32>& Cursor = Degree;
		FMemory::Memcpy(Cursor.GetData(), AdjOffsets.GetData(), NodeCount * sizeof(uint32));

		Edges.SetNumUninitialized(AdjOffsets[NodeCount]);
		for (int32 Source = 0; Source < NodeCount; ++Source)
		{
			for (uint32 EdgeIndex = HalfOffsets[Source]; EdgeIndex < HalfOffsets[Source + 1]; ++EdgeIndex)
			{
				const uint32 Edge = HalfEdges[EdgeIndex];
				const int32 Target = GetEdgeTarget(Edge);
				if (!Nodes.IsValidIndex(Target))
				{
					continue;
				}
				const uint32 ClearanceBits = Edge & ~EdgeIndexMask;
				Edges[Cursor[Source]++] = ClearanceBits | static_cast<uint32>(Target);
				Edges[Cursor[Target]++] = ClearanceBits | static_cast<uint32>(Source);
			}
		}
	}

	friend FArchive& operator<<(FArchive& Ar, FAirwayGraph& Graph)
	{
		Ar << Graph.OriginWS;
		Ar << Graph.HalfVoxelSize;
		Ar << Graph.Nodes;

		if (Ar.IsLoading())
		{
			TArray<uint32> HalfOffsets;
			TArray<uint32> HalfEdges;
			Ar << HalfOffsets;
			Ar << HalfEdges;
			Graph.BuildFromUndirectedEdges(HalfOffsets, HalfEdges);
		}
		else
		{
			// Persist only the Target > Source half of each undirected pair.
			TArray<uint32> HalfOffsets;
			TArray<uint32> HalfEdges;
			const int32 NodeCount = Graph.AdjOffsets.Num() == Graph.Nodes.Num() + 1 ? Graph.Nodes.Num() : 0;
			HalfOffsets.Reserve(NodeCount + 1);
			HalfOffsets.Add(0);
			HalfEdges.Reserve(Graph.Edges.Num() / 2);
			for (int32 Source = 0; Source < NodeCount; ++Source)
			{
				for (uint32 EdgeIndex = Graph.AdjOffsets[Source]; EdgeIndex < Graph.AdjOffsets[Source + 1]; ++EdgeIndex)
				{
					const uint32 Edge = Graph.Edges[EdgeIndex];
					if (GetEdgeTarget(Edge) > Source)
					{
						HalfEdges.Add(Edge);
					}
				}
				HalfOffsets.Add(HalfEdges.Num());
			}
			Ar << HalfOffsets;
			Ar << HalfEdges;
		}
		return Ar;
	}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FOctreeNode& Node)
{
	for (int32& ChildIndex : Node.ChildrenIndices) { Ar << ChildIndex; }
	Ar << Node.State;
	Ar << Node.Bounds;
	for (int32& NeighborIndex : Node.Neighbors) { Ar << NeighborIndex; }
	Ar << Node.MaxAllowedRadius;
	return Ar;
}

