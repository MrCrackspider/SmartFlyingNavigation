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

	friend FArchive& operator<<(FArchive& Ar, FAirwayGraph& Graph)
	{
		Ar << Graph.OriginWS;
		Ar << Graph.HalfVoxelSize;
		Ar << Graph.Nodes;
		Ar << Graph.AdjOffsets;
		Ar << Graph.Edges;
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

