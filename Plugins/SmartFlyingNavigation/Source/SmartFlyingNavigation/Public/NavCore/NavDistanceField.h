// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Baked, quantized, brick-sparse narrow-band distance field for navigation.
 *
 * - One uint8 per voxel: distance to the nearest solid voxel face, in 1/8-voxel
 *   fixed point (max representable ≈ 31.9 voxels; 255 means "at least the band").
 * - Storage is split into 8x8x8 bricks. Bricks whose voxels share one value
 *   (deep free space, solid interiors) collapse into a single table entry, so
 *   typical scenes stay in the hundreds of KB even at 256³.
 * - CPU-only and read-only after load: safe to sample from worker threads.
 */
struct SMARTFLYINGNAVIGATION_API FNavDistanceField
{
	// --- Header ---
	FVector OriginWS = FVector::ZeroVector;     // Volume bounds minimum.
	float VoxelSize = 100.f;                    // Voxel extent in cm.
	FIntVector Dims = FIntVector::ZeroValue;    // Field dimensions, in voxels.
	FIntVector BrickDims = FIntVector::ZeroValue;

	static constexpr int32 BrickSize = 8;       // 8x8x8 voxels per brick.
	static constexpr int32 BrickVoxels = BrickSize * BrickSize * BrickSize;
	static constexpr int32 BrickPackedBytes = BrickVoxels / 2; // 4 bits per voxel = 256 B.
	static constexpr float QuantScale = 8.f;    // base value = distance_in_voxels * 8.
	static constexpr uint32 UniformFlag = 0x80000000u;
	static constexpr uint32 Packed4Flag = 0x40000000u;
	static constexpr uint32 IndexMask = 0x00FFFFFFu;

	// --- Storage (mixed 8-bit / lossless 4-bit bricks) ---
	/**
	 * One entry per brick:
	 *  - UniformFlag set: low 8 bits hold the single value.
	 *  - Packed4Flag set: lossless 4-bit brick (value range <= 15). Index addresses
	 *    BrickBase (1 B base) and BrickData4 (256 B of nibbles; value = Base + Nibble).
	 *  - Otherwise: full 8-bit brick. Index addresses BrickData8 (512 B).
	 */
	TArray<uint32> BrickTable;
	TArray<uint8> BrickBase;
	TArray<uint8> BrickData4;
	TArray<uint8> BrickData8;

	// --- Queries (thread-safe, read-only after load) ---

	bool IsValid() const
	{
		return Dims.X > 0 && Dims.Y > 0 && Dims.Z > 0 && BrickTable.Num() == BrickDims.X * BrickDims.Y * BrickDims.Z;
	}

	/** Max representable clearance in world units ("at least this much"). */
	float GetBandWorld() const
	{
		return (255.f / QuantScale) * VoxelSize;
	}

	/** Raw quantized sample at a voxel coordinate (no bounds check on input — caller clamps). */
	FORCEINLINE uint8 SampleRaw(int32 X, int32 Y, int32 Z) const
	{
		const int32 BX = X / BrickSize;
		const int32 BY = Y / BrickSize;
		const int32 BZ = Z / BrickSize;
		const uint32 Entry = BrickTable[(BZ * BrickDims.Y + BY) * BrickDims.X + BX];

		if (Entry & UniformFlag)
		{
			return static_cast<uint8>(Entry & 0xFF);
		}

		const int32 LX = X & (BrickSize - 1);
		const int32 LY = Y & (BrickSize - 1);
		const int32 LZ = Z & (BrickSize - 1);
		const int32 Local = (LZ * BrickSize + LY) * BrickSize + LX;
		const uint32 Index = Entry & IndexMask;

		if (Entry & Packed4Flag)
		{
			// Lossless 4-bit brick: value = Base + Nibble (brick range <= 15).
			const uint8 Packed = BrickData4[static_cast<int64>(Index) * BrickPackedBytes + (Local >> 1)];
			const uint8 Nibble = (Local & 1) ? (Packed >> 4) : (Packed & 0x0F);
			return static_cast<uint8>(BrickBase[Index] + Nibble);
		}

		return BrickData8[static_cast<int64>(Index) * BrickVoxels + Local];
	}

	/**
	 * Clearance (distance to nearest solid) at a world position, in world units.
	 * Positions outside the field return the band (no information => permissive,
	 * matching the octree query behavior outside volume bounds).
	 */
	float SampleClearance(const FVector& WorldPos) const
	{
		if (!IsValid())
		{
			return GetBandWorld();
		}

		const FVector Local = (WorldPos - OriginWS) / VoxelSize;
		const int32 X = FMath::FloorToInt(Local.X);
		const int32 Y = FMath::FloorToInt(Local.Y);
		const int32 Z = FMath::FloorToInt(Local.Z);

		if (X < 0 || Y < 0 || Z < 0 || X >= Dims.X || Y >= Dims.Y || Z >= Dims.Z)
		{
			return GetBandWorld();
		}

		return (static_cast<float>(SampleRaw(X, Y, Z)) / QuantScale) * VoxelSize;
	}

	/**
	 * Approximate outward gradient (away from the nearest obstacle) via central
	 * differences, 6 samples. Returns a normalized direction or zero when flat.
	 */
	FVector SampleGradient(const FVector& WorldPos) const
	{
		const float Step = VoxelSize;
		const FVector Gradient(
			SampleClearance(WorldPos + FVector(Step, 0, 0)) - SampleClearance(WorldPos - FVector(Step, 0, 0)),
			SampleClearance(WorldPos + FVector(0, Step, 0)) - SampleClearance(WorldPos - FVector(0, Step, 0)),
			SampleClearance(WorldPos + FVector(0, 0, Step)) - SampleClearance(WorldPos - FVector(0, 0, Step)));

		return Gradient.GetSafeNormal();
	}

	/**
	 * Sphere-traced segment test: marches from A to B with adaptive steps of
	 * max(clearance - radius, half voxel). Returns false when any sample's
	 * clearance drops below AgentRadius.
	 */
	bool IsSegmentClear(const FVector& A, const FVector& B, float AgentRadius) const
	{
		const FVector Delta = B - A;
		const float TotalDistance = Delta.Size();
		const float MinStep = VoxelSize * 0.5f;

		if (TotalDistance < KINDA_SMALL_NUMBER)
		{
			return SampleClearance(A) >= AgentRadius;
		}

		const FVector Direction = Delta / TotalDistance;

		float Travelled = 0.f;
		while (Travelled < TotalDistance)
		{
			const FVector Sample = A + Direction * Travelled;
			const float Clearance = SampleClearance(Sample);
			if (Clearance < AgentRadius)
			{
				return false;
			}

			Travelled += FMath::Max(Clearance - AgentRadius, MinStep);
		}

		return SampleClearance(B) >= AgentRadius;
	}

	int64 GetAllocatedBytes() const
	{
		return BrickTable.GetAllocatedSize() + BrickBase.GetAllocatedSize()
			+ BrickData4.GetAllocatedSize() + BrickData8.GetAllocatedSize();
	}

	void Reset()
	{
		OriginWS = FVector::ZeroVector;
		VoxelSize = 100.f;
		Dims = FIntVector::ZeroValue;
		BrickDims = FIntVector::ZeroValue;
		BrickTable.Empty();
		BrickBase.Empty();
		BrickData4.Empty();
		BrickData8.Empty();
	}

	friend FArchive& operator<<(FArchive& Ar, FNavDistanceField& Field)
	{
		Ar << Field.OriginWS;
		Ar << Field.VoxelSize;
		Ar << Field.Dims;
		Ar << Field.BrickDims;
		Ar << Field.BrickTable;
		Ar << Field.BrickBase;
		Ar << Field.BrickData4;
		Ar << Field.BrickData8;
		return Ar;
	}
};
