// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * Analytic dynamic obstacle primitive (sphere or box). Supports exact point
 * distance so it can be composited with the static distance field as
 * min(static clearance, dynamic distance).
 */
struct SMARTFLYINGNAVIGATION_API FDynamicObstaclePrim
{
	enum class EShape : uint8 { Sphere, Box };

	EShape Shape = EShape::Sphere;
	FVector Center = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	/** Sphere: X holds the radius. Box: half extents. */
	FVector HalfExtents = FVector::ZeroVector;
	FVector Velocity = FVector::ZeroVector;
	uint32 OwnerId = 0;

	/** Exact distance from a point to the primitive surface (0 when inside). */
	float Distance(const FVector& Point) const
	{
		if (Shape == EShape::Sphere)
		{
			return FMath::Max(0.f, FVector::Dist(Point, Center) - HalfExtents.X);
		}

		const FVector Local = Rotation.UnrotateVector(Point - Center);
		const FVector Excess = (Local.GetAbs() - HalfExtents).ComponentMax(FVector::ZeroVector);
		return Excess.Size();
	}

	/** Radius of the bounding sphere (for broadphase culling and ORCA). */
	float GetBoundingRadius() const
	{
		return Shape == EShape::Sphere ? HalfExtents.X : HalfExtents.Size();
	}

	/** World-space AABB of the primitive. */
	FBox GetBounds() const
	{
		const float BoundingRadius = GetBoundingRadius();
		return FBox::BuildAABB(Center, FVector(BoundingRadius));
	}
};

/**
 * World-level registry of dynamic obstacles. One registration point with two
 * consumers: A* path requests (macro avoidance, via thread-safe snapshots) and
 * the ORCA avoidance solver (micro avoidance).
 *
 * Replaces the per-volume voxel bitmask grid: no timers, no per-volume
 * duplication, and zero steady-state allocation.
 */
class SMARTFLYINGNAVIGATION_API FDynamicObstacleRegistry
{
public:
	/** Adds or refreshes a primitive (game thread, called from the obstacle's dirty tick). */
	void Update(const FDynamicObstaclePrim& Prim)
	{
		FRWScopeLock ScopeLock(Lock, SLT_Write);
		Prims.Add(Prim.OwnerId, Prim);
	}

	void Remove(uint32 OwnerId)
	{
		FRWScopeLock ScopeLock(Lock, SLT_Write);
		Prims.Remove(OwnerId);
	}

	/**
	 * Flat-array copy for async consumers. When RelevantBounds is provided,
	 * primitives whose AABB does not intersect it are skipped (the World
	 * Partition-friendly path for large obstacle counts).
	 */
	void Snapshot(TArray<FDynamicObstaclePrim>& OutPrims, const FBox* RelevantBounds = nullptr) const
	{
		OutPrims.Reset();

		FRWScopeLock ScopeLock(Lock, SLT_ReadOnly);
		OutPrims.Reserve(Prims.Num());
		for (const TPair<uint32, FDynamicObstaclePrim>& Pair : Prims)
		{
			if (RelevantBounds && !RelevantBounds->Intersect(Pair.Value.GetBounds()))
			{
				continue;
			}
			OutPrims.Add(Pair.Value);
		}
	}

	/** True when any primitive is within Radius of the point. Lock-held iteration, no copy. */
	bool IsPointBlocked(const FVector& Point, float Radius = 0.f) const
	{
		FRWScopeLock ScopeLock(Lock, SLT_ReadOnly);
		for (const TPair<uint32, FDynamicObstaclePrim>& Pair : Prims)
		{
			if (Pair.Value.Distance(Point) <= Radius)
			{
				return true;
			}
		}
		return false;
	}

	/**
	 * True when any primitive comes within Radius of the segment.
	 * Lock-held iteration, no copy — the cheap path for per-tick safety probes.
	 * (Exact for spheres; for boxes the closest-point-to-center approximation
	 * can under-report grazing corner contacts by a small margin.)
	 */
	bool IsSegmentBlocked(const FVector& Start, const FVector& End, float Radius) const
	{
		FRWScopeLock ScopeLock(Lock, SLT_ReadOnly);
		for (const TPair<uint32, FDynamicObstaclePrim>& Pair : Prims)
		{
			const FDynamicObstaclePrim& Prim = Pair.Value;

			// Broadphase: segment vs bounding sphere.
			const FVector ClosestToCenter = FMath::ClosestPointOnSegment(Prim.Center, Start, End);
			if (FVector::DistSquared(ClosestToCenter, Prim.Center) >
				FMath::Square(Prim.GetBoundingRadius() + Radius))
			{
				continue;
			}

			if (Prim.Distance(ClosestToCenter) <= Radius)
			{
				return true;
			}
		}
		return false;
	}

	int32 Num() const
	{
		FRWScopeLock ScopeLock(Lock, SLT_ReadOnly);
		return Prims.Num();
	}

	void Reset()
	{
		FRWScopeLock ScopeLock(Lock, SLT_Write);
		Prims.Empty();
	}

private:
	mutable FRWLock Lock;
	TMap<uint32, FDynamicObstaclePrim> Prims;
};
