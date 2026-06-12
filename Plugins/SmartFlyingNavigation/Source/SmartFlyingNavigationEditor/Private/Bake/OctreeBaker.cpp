// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Bake/OctreeBaker.h"

#include "NavVolume/OctreeVoxelVolume.h"
#include "NavData/VoxelObstacleProfile.h"
#include "Component/NavBakeExcludeComponent.h"

#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/OverlapResult.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "Async/ParallelFor.h"
#include "Misc/ScopedSlowTask.h"
#include "HAL/IConsoleManager.h"
#include "DrawDebugHelpers.h"

#define LOCTEXT_NAMESPACE "OctreeBaker"

static TAutoConsoleVariable<int32> CVarBakeParallel(
	TEXT("sfn.Bake.Parallel"),
	1,
	TEXT("When 1, the octree bake voxelizes components and runs the distance transform in parallel."),
	ECVF_Default);

namespace
{
	/** Distance clamp band upper bound, in voxels — the 8-bit 1/8-voxel quantization maximum. */
	constexpr int32 GMaxBakeDistanceBandVoxels = 32;

	/**
	 * Bias subtracted from center-to-center voxel distances, in voxels.
	 * 0.5 (solid voxel face) is exact for axially adjacent walls — the dominant case,
	 * and the one that keeps 1-voxel corridors passable for radius == voxel/2 agents.
	 * Pure diagonals can over-report by up to ~0.37 voxels; do NOT raise this to
	 * compensate (√3/2 makes tight corridors impassable — see v2.0 regression).
	 */
	constexpr float GConservativeDistanceBiasVoxels = 0.5f;
}

FString FVoxelVolumeDesc::GetLabel() const
{
#if WITH_EDITOR
	if (VolumeActor)
	{
		return VolumeActor->GetActorLabel();
	}
#endif
	return TEXT("Volume");
}

bool FOctreeBaker::BuildDescFromVolume(AOctreeVoxelVolume* Volume, FVoxelVolumeDesc& OutDesc)
{
	if (!IsValid(Volume))
	{
		return false;
	}

	const FBox Bounds = Volume->GetVolumeBounds();
	if (!Bounds.IsValid || Volume->VoxelExtent <= 0.f)
	{
		UE_LOG(LogTemp, Error, TEXT("OctreeBaker: '%s' has invalid bounds or non-positive voxel extent."), *Volume->GetActorLabel());
		return false;
	}

	const FVector TotalSize = Bounds.GetSize();
	const FIntVector CalculatedMapSize(
		FMath::RoundToInt(TotalSize.X / Volume->VoxelExtent),
		FMath::RoundToInt(TotalSize.Y / Volume->VoxelExtent),
		FMath::RoundToInt(TotalSize.Z / Volume->VoxelExtent));

	if (CalculatedMapSize.X <= 0 || CalculatedMapSize.Y <= 0 || CalculatedMapSize.Z <= 0)
	{
		UE_LOG(LogTemp, Error, TEXT("OctreeBaker: '%s' computed an invalid map size (%s)."),
			*Volume->GetActorLabel(), *CalculatedMapSize.ToString());
		return false;
	}

	if (Volume->GetMapSize() != CalculatedMapSize)
	{
		UE_LOG(LogTemp, Warning, TEXT("OctreeBaker: '%s' MapSize %s differs from bounds-derived size %s. Updating the volume."),
			*Volume->GetActorLabel(), *Volume->GetMapSize().ToString(), *CalculatedMapSize.ToString());
		Volume->Modify();
		Volume->SetMapSize(CalculatedMapSize);
	}

	if (CalculatedMapSize.X != CalculatedMapSize.Y || CalculatedMapSize.X != CalculatedMapSize.Z ||
		!FMath::IsPowerOfTwo(static_cast<uint32>(CalculatedMapSize.X)))
	{
		UE_LOG(LogTemp, Error, TEXT("OctreeBaker: '%s' requires a cubic power-of-two map size. Current: %s."),
			*Volume->GetActorLabel(), *CalculatedMapSize.ToString());
		return false;
	}

	OutDesc.Bounds = Bounds;
	OutDesc.VoxelExtent = Volume->VoxelExtent;
	OutDesc.MapSize = CalculatedMapSize;
	OutDesc.MaxDepth = Volume->GetMaxOctreeDepth();
	OutDesc.DistanceFieldBandVoxels = Volume->GetDistanceFieldBandVoxels();
	OutDesc.CollisionChannels = Volume->BakeCollisionChannels;
	OutDesc.bRespectCanAffectNavigation = Volume->bRespectCanAffectNavigation;
	OutDesc.ObstacleProfile = Volume->GetStaticObstacleProfile();
	OutDesc.VolumeActor = Volume;
#if WITH_EDITORONLY_DATA
	for (const AActor* ExcludeActor : Volume->BakeExcludeActors)
	{
		if (ExcludeActor)
		{
			OutDesc.ExcludeActors.Add(ExcludeActor);
		}
	}
#endif

	return true;
}

bool FOctreeBaker::BakeVolume(UWorld* World, const FVoxelVolumeDesc& Desc, TArray<FOctreeNode>& OutNodePool, FVoxelField* OutField)
{
	if (!World || Desc.NumVoxels() <= 0)
	{
		return false;
	}

	FScopedSlowTask BakeTask(3.f, FText::Format(LOCTEXT("BakeTaskTitle", "Baking Octree Data for {0}..."), FText::FromString(Desc.GetLabel())));
	BakeTask.MakeDialog(true);

	FVoxelField Field;
	Field.BandVoxels = FMath::Clamp(Desc.DistanceFieldBandVoxels, 4, GMaxBakeDistanceBandVoxels);

	// --- Stage A: occupancy ---
	BakeTask.EnterProgressFrame(1.f, LOCTEXT("BakeStageOccupancy", "Voxelizing obstacles... (1/3)"));
	const double StageAStart = FPlatformTime::Seconds();
	if (!BuildOccupancy(World, Desc, Field.Occupancy))
	{
		return false;
	}
	const double StageADuration = FPlatformTime::Seconds() - StageAStart;
	if (BakeTask.ShouldCancel()) return false;

	// --- Stage B: distance transform ---
	BakeTask.EnterProgressFrame(1.f, LOCTEXT("BakeStageDistance", "Computing distance field... (2/3)"));
	const double StageBStart = FPlatformTime::Seconds();
	ComputeSquaredDistanceField(Desc, Field.Occupancy, Field.BandVoxels, Field.SquaredDistance);
	const double StageBDuration = FPlatformTime::Seconds() - StageBStart;
	if (BakeTask.ShouldCancel()) return false;

	// --- Stage C: octree build ---
	BakeTask.EnterProgressFrame(1.f, LOCTEXT("BakeStageOctree", "Building octree... (3/3)"));
	const double StageCStart = FPlatformTime::Seconds();
	if (!BuildOctree(Desc, Field, OutNodePool))
	{
		return false;
	}
	const double StageCDuration = FPlatformTime::Seconds() - StageCStart;

	UE_LOG(LogTemp, Log, TEXT("OctreeBaker: '%s' baked %lld voxels into %d nodes. Occupancy=%.2fs DistanceField=%.2fs Octree=%.2fs"),
		*Desc.GetLabel(), Desc.NumVoxels(), OutNodePool.Num(), StageADuration, StageBDuration, StageCDuration);

	if (OutField)
	{
		*OutField = MoveTemp(Field);
	}

	return true;
}

void FOctreeBaker::BuildDistanceField(const FVoxelVolumeDesc& Desc, const FVoxelField& Field, FNavDistanceField& OutDistanceField)
{
	OutDistanceField.Reset();

	if (Field.SquaredDistance.Num() != Desc.NumVoxels() || Desc.NumVoxels() <= 0)
	{
		return;
	}

	OutDistanceField.OriginWS = Desc.Bounds.Min;
	OutDistanceField.VoxelSize = Desc.VoxelExtent;
	OutDistanceField.Dims = Desc.MapSize;
	OutDistanceField.BrickDims = FIntVector(
		FMath::DivideAndRoundUp(Desc.MapSize.X, FNavDistanceField::BrickSize),
		FMath::DivideAndRoundUp(Desc.MapSize.Y, FNavDistanceField::BrickSize),
		FMath::DivideAndRoundUp(Desc.MapSize.Z, FNavDistanceField::BrickSize));

	const int32 NumBricks = OutDistanceField.BrickDims.X * OutDistanceField.BrickDims.Y * OutDistanceField.BrickDims.Z;
	OutDistanceField.BrickTable.SetNumZeroed(NumBricks);

	// Quantize: distance to the nearest solid voxel *surface* (conservative half-diagonal
	// bias), in 1/8-voxel fixed point, saturating at 255 ("at least the band").
	auto QuantizeVoxel = [&Desc, &Field](int32 X, int32 Y, int32 Z) -> uint8
	{
		const int32 CX = FMath::Clamp(X, 0, Desc.MapSize.X - 1);
		const int32 CY = FMath::Clamp(Y, 0, Desc.MapSize.Y - 1);
		const int32 CZ = FMath::Clamp(Z, 0, Desc.MapSize.Z - 1);
		const float DistVoxels = FMath::Sqrt(static_cast<float>(Field.SquaredDistance[Desc.FlatIndex(CX, CY, CZ)]));
		const float Biased = FMath::Max(0.f, DistVoxels - GConservativeDistanceBiasVoxels);
		return static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(Biased * FNavDistanceField::QuantScale), 0, 255));
	};

	constexpr int32 BrickSize = FNavDistanceField::BrickSize;
	uint8 BrickValues[FNavDistanceField::BrickVoxels];
	uint8 PackedNibbles[FNavDistanceField::BrickPackedBytes];

	for (int32 BZ = 0; BZ < OutDistanceField.BrickDims.Z; ++BZ)
	{
		for (int32 BY = 0; BY < OutDistanceField.BrickDims.Y; ++BY)
		{
			for (int32 BX = 0; BX < OutDistanceField.BrickDims.X; ++BX)
			{
				uint8 MinValue = 255;
				uint8 MaxValue = 0;

				for (int32 LZ = 0; LZ < BrickSize; ++LZ)
				{
					for (int32 LY = 0; LY < BrickSize; ++LY)
					{
						for (int32 LX = 0; LX < BrickSize; ++LX)
						{
							const uint8 Value = QuantizeVoxel(BX * BrickSize + LX, BY * BrickSize + LY, BZ * BrickSize + LZ);
							BrickValues[(LZ * BrickSize + LY) * BrickSize + LX] = Value;
							MinValue = FMath::Min(MinValue, Value);
							MaxValue = FMath::Max(MaxValue, Value);
						}
					}
				}

				const int32 BrickIndex = (BZ * OutDistanceField.BrickDims.Y + BY) * OutDistanceField.BrickDims.X + BX;
				if (MinValue == MaxValue)
				{
					OutDistanceField.BrickTable[BrickIndex] = FNavDistanceField::UniformFlag | MinValue;
					continue;
				}

				if (MaxValue - MinValue <= 15)
				{
					// Lossless 4-bit brick: value = Base + Nibble (range fits in a nibble).
					FMemory::Memzero(PackedNibbles, sizeof(PackedNibbles));
					for (int32 Local = 0; Local < FNavDistanceField::BrickVoxels; ++Local)
					{
						const uint8 Nibble = BrickValues[Local] - MinValue;
						if (Local & 1)
						{
							PackedNibbles[Local >> 1] |= Nibble << 4;
						}
						else
						{
							PackedNibbles[Local >> 1] |= Nibble;
						}
					}

					const uint32 DataBrickIndex = static_cast<uint32>(OutDistanceField.BrickBase.Num());
					OutDistanceField.BrickTable[BrickIndex] = FNavDistanceField::Packed4Flag | DataBrickIndex;
					OutDistanceField.BrickBase.Add(MinValue);
					OutDistanceField.BrickData4.Append(PackedNibbles, FNavDistanceField::BrickPackedBytes);
				}
				else
				{
					// Wide-range brick (near walls): keep full 8-bit precision so tight
					// corridor clearances survive quantization.
					const uint32 DataBrickIndex = static_cast<uint32>(OutDistanceField.BrickData8.Num() / FNavDistanceField::BrickVoxels);
					OutDistanceField.BrickTable[BrickIndex] = DataBrickIndex;
					OutDistanceField.BrickData8.Append(BrickValues, FNavDistanceField::BrickVoxels);
				}
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("OctreeBaker: '%s' distance field: %d bricks (%d lossless-4bit, %d full-8bit), %.1f KB."),
		*Desc.GetLabel(), NumBricks,
		OutDistanceField.BrickBase.Num(),
		OutDistanceField.BrickData8.Num() / FNavDistanceField::BrickVoxels,
		OutDistanceField.GetAllocatedBytes() / 1024.f);
}

//~======================================================================
// Stage A — Occupancy
//~======================================================================

bool FOctreeBaker::BuildOccupancy(UWorld* World, const FVoxelVolumeDesc& Desc, TBitArray<>& OutOccupancy)
{
	OutOccupancy.Init(false, Desc.NumVoxels());

	// Build collision query params from either the Profile (advanced) or the direct channel settings.
	FCollisionObjectQueryParams ObjectQueryParams;
	if (Desc.ObstacleProfile && Desc.ObstacleProfile->CollisionChannels.Num() > 0)
	{
		for (const TEnumAsByte<ECollisionChannel>& Channel : Desc.ObstacleProfile->CollisionChannels)
		{
			ObjectQueryParams.AddObjectTypesToQuery(Channel);
		}
	}
	else if (Desc.CollisionChannels.Num() > 0)
	{
		for (const TEnumAsByte<ECollisionChannel>& Channel : Desc.CollisionChannels)
		{
			ObjectQueryParams.AddObjectTypesToQuery(Channel);
		}
	}
	else
	{
		ObjectQueryParams.AddObjectTypesToQuery(ECollisionChannel::ECC_WorldStatic);
	}

	// Single broadphase query over the whole volume to gather candidate components.
	FCollisionQueryParams CollisionQueryParams(SCENE_QUERY_STAT(VoxelBakeOverlap), false);
	if (Desc.VolumeActor)
	{
		CollisionQueryParams.AddIgnoredActor(Desc.VolumeActor);
	}

	TArray<FOverlapResult> OverlapResults;
	World->OverlapMultiByObjectType(
		OverlapResults,
		Desc.Bounds.GetCenter(),
		FQuat::Identity,
		ObjectQueryParams,
		FCollisionShape::MakeBox(Desc.Bounds.GetExtent()),
		CollisionQueryParams);

	TArray<UPrimitiveComponent*> RelevantComponents;
	RelevantComponents.Reserve(OverlapResults.Num());

	for (const FOverlapResult& Result : OverlapResults)
	{
		UPrimitiveComponent* PrimitiveComp = Result.GetComponent();

		if (!IsValid(PrimitiveComp) || !PrimitiveComp->IsQueryCollisionEnabled())
		{
			continue;
		}

		AActor* OwnerActor = PrimitiveComp->GetOwner();
		if (!OwnerActor) continue;

		if (OwnerActor->FindComponentByClass<UNavBakeExcludeComponent>() || OwnerActor->ActorHasTag(FName("NoNavBake")))
		{
			continue;
		}

		if (Desc.ExcludeActors.Contains(OwnerActor))
		{
			continue;
		}

		if (Desc.bRespectCanAffectNavigation && !PrimitiveComp->CanEverAffectNavigation())
		{
			continue;
		}

		if (Desc.ObstacleProfile && !Desc.ObstacleProfile->DoesActorMatch(OwnerActor, PrimitiveComp))
		{
			continue;
		}

		RelevantComponents.Add(PrimitiveComp);
	}

	FScopedSlowTask ComponentTask(RelevantComponents.Num(), LOCTEXT("VoxelizingComponents", "Voxelizing components..."));

	const float HalfVoxel = Desc.VoxelExtent * 0.5f;
	const FCollisionShape VoxelShape = FCollisionShape::MakeBox(FVector(HalfVoxel));

	// Z-slices map to disjoint word ranges in the bitset only when a slice is word-aligned.
	const bool bSliceWordAligned = ((static_cast<int64>(Desc.MapSize.X) * Desc.MapSize.Y) % NumBitsPerDWORD) == 0;
	const bool bParallel = CVarBakeParallel.GetValueOnGameThread() != 0 && bSliceWordAligned;

	for (UPrimitiveComponent* Comp : RelevantComponents)
	{
		ComponentTask.EnterProgressFrame(1.f);
		if (ComponentTask.ShouldCancel())
		{
			return false;
		}

		const FBox CompBounds = Comp->Bounds.GetBox();
		if (!CompBounds.IsValid) continue;

		// Narrow-phase: only test voxels inside this component's AABB, against this one component.
		const FIntVector MinVoxel = Desc.WorldToVoxelIndex(CompBounds.Min);
		const FIntVector MaxVoxel = Desc.WorldToVoxelIndex(CompBounds.Max);

		const FIntVector ClampedMin(
			FMath::Max(MinVoxel.X, 0), FMath::Max(MinVoxel.Y, 0), FMath::Max(MinVoxel.Z, 0));
		const FIntVector ClampedMax(
			FMath::Min(MaxVoxel.X, Desc.MapSize.X - 1), FMath::Min(MaxVoxel.Y, Desc.MapSize.Y - 1), FMath::Min(MaxVoxel.Z, Desc.MapSize.Z - 1));

		if (ClampedMin.X > ClampedMax.X || ClampedMin.Y > ClampedMax.Y || ClampedMin.Z > ClampedMax.Z)
		{
			continue;
		}

		const int32 NumSlices = ClampedMax.Z - ClampedMin.Z + 1;

		ParallelFor(NumSlices, [&](int32 SliceIndex)
		{
			const int32 z = ClampedMin.Z + SliceIndex;
			for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
			{
				for (int32 x = ClampedMin.X; x <= ClampedMax.X; ++x)
				{
					const int64 FlatIdx = Desc.FlatIndex(x, y, z);
					if (OutOccupancy[FlatIdx])
					{
						continue; // Already marked solid by a previous component.
					}

					const FVector VoxelCenter = Desc.VoxelIndexToWorld(FIntVector(x, y, z));
					if (Comp->OverlapComponent(VoxelCenter, FQuat::Identity, VoxelShape))
					{
						OutOccupancy[FlatIdx] = true;
					}
				}
			}
		}, bParallel ? EParallelForFlags::None : EParallelForFlags::ForceSingleThread);
	}

	return true;
}

//~======================================================================
// Stage B — Separable exact squared EDT (Saito–Toriwaki / Meijster)
//~======================================================================

void FOctreeBaker::ComputeSquaredDistanceField(const FVoxelVolumeDesc& Desc, const TBitArray<>& Occupancy, int32 BandVoxels, TArray<int32>& OutSquaredDistance)
{
	const int32 NX = Desc.MapSize.X;
	const int32 NY = Desc.MapSize.Y;
	const int32 NZ = Desc.MapSize.Z;
	const int64 N = Desc.NumVoxels();
	const int32 Band = FMath::Max(1, BandVoxels);
	const int32 Band2 = Band * Band;

	OutSquaredDistance.SetNumUninitialized(N);

	const bool bParallel = CVarBakeParallel.GetValueOnGameThread() != 0;
	const EParallelForFlags ParallelFlags = bParallel ? EParallelForFlags::None : EParallelForFlags::ForceSingleThread;

	// --- Pass 1 (X axis): per-row 1D distance (in voxels), clamped to the band, then squared. ---
	ParallelFor(NY * NZ, [&](int32 RowIdx)
	{
		const int32 y = RowIdx % NY;
		const int32 z = RowIdx / NY;
		const int64 Base = Desc.FlatIndex(0, y, z);

		int32 g = Band;
		for (int32 x = 0; x < NX; ++x)
		{
			g = Occupancy[Base + x] ? 0 : FMath::Min(g + 1, Band);
			OutSquaredDistance[Base + x] = g;
		}

		g = Band;
		for (int32 x = NX - 1; x >= 0; --x)
		{
			g = Occupancy[Base + x] ? 0 : FMath::Min(g + 1, Band);
			const int32 Dist = FMath::Min(OutSquaredDistance[Base + x], g);
			OutSquaredDistance[Base + x] = Dist * Dist;
		}
	}, ParallelFlags);

	// --- Passes 2 & 3: Meijster lower-envelope scan along Y then Z over squared costs. ---
	auto EnvelopePass = [&](int32 LineLen, int32 NumLines, auto&& FlatAt)
	{
		ParallelFor(NumLines, [&](int32 Line)
		{
			TArray<int32> Cost;   Cost.SetNumUninitialized(LineLen);
			TArray<int32> Out;    Out.SetNumUninitialized(LineLen);
			TArray<int32> S;      S.SetNumUninitialized(LineLen);
			TArray<int32> T;      T.SetNumUninitialized(LineLen);

			for (int32 i = 0; i < LineLen; ++i)
			{
				Cost[i] = OutSquaredDistance[FlatAt(Line, i)];
			}

			auto F = [&Cost](int32 X, int32 I) { return (X - I) * (X - I) + Cost[I]; };
			auto Sep = [&Cost](int32 I, int32 U) { return (U * U - I * I + Cost[U] - Cost[I]) / (2 * (U - I)); };

			int32 q = 0;
			S[0] = 0;
			T[0] = 0;

			for (int32 u = 1; u < LineLen; ++u)
			{
				while (q >= 0 && F(T[q], S[q]) > F(T[q], u))
				{
					q--;
				}
				if (q < 0)
				{
					q = 0;
					S[0] = u;
				}
				else
				{
					const int32 w = 1 + Sep(S[q], u);
					if (w < LineLen)
					{
						q++;
						S[q] = u;
						T[q] = w;
					}
				}
			}

			for (int32 u = LineLen - 1; u >= 0; --u)
			{
				Out[u] = F(u, S[q]);
				if (u == T[q])
				{
					q--;
				}
			}

			for (int32 i = 0; i < LineLen; ++i)
			{
				OutSquaredDistance[FlatAt(Line, i)] = Out[i];
			}
		}, ParallelFlags);
	};

	// Pass 2 (Y axis): one line per (x, z).
	EnvelopePass(NY, NX * NZ, [&](int32 Line, int32 i) -> int64
	{
		const int32 x = Line % NX;
		const int32 z = Line / NX;
		return Desc.FlatIndex(x, i, z);
	});

	// Pass 3 (Z axis): one line per (x, y).
	EnvelopePass(NZ, NX * NY, [&](int32 Line, int32 i) -> int64
	{
		const int32 x = Line % NX;
		const int32 y = Line / NX;
		return Desc.FlatIndex(x, y, i);
	});

	// Saturate at the band.
	ParallelFor(NY * NZ, [&](int32 RowIdx)
	{
		const int64 Base = static_cast<int64>(RowIdx) * NX;
		for (int32 x = 0; x < NX; ++x)
		{
			OutSquaredDistance[Base + x] = FMath::Min(OutSquaredDistance[Base + x], Band2);
		}
	}, ParallelFlags);
}

//~======================================================================
// Stage C — Octree build
//~======================================================================

bool FOctreeBaker::BuildOctree(const FVoxelVolumeDesc& Desc, const FVoxelField& Field, TArray<FOctreeNode>& OutNodePool)
{
	OutNodePool.Reset();

	int64 SolidCount = 0;
	for (TConstSetBitIterator<> It(Field.Occupancy); It; ++It)
	{
		SolidCount++;
	}

	const int32 EstimatedNodeCount = static_cast<int32>(FMath::Min<int64>(SolidCount * Desc.MaxDepth, 4 * 1024 * 1024));
	OutNodePool.Reserve(FMath::Max(EstimatedNodeCount, 64));

	const int32 RootIndex = OutNodePool.AddDefaulted();
	OutNodePool[RootIndex].Bounds = Desc.Bounds;

	for (TConstSetBitIterator<> It(Field.Occupancy); It; ++It)
	{
		const int64 FlatIdx = It.GetIndex();
		const int32 x = static_cast<int32>(FlatIdx % Desc.MapSize.X);
		const int32 y = static_cast<int32>((FlatIdx / Desc.MapSize.X) % Desc.MapSize.Y);
		const int32 z = static_cast<int32>(FlatIdx / (static_cast<int64>(Desc.MapSize.X) * Desc.MapSize.Y));

		InsertObstacleRecursive(Desc, OutNodePool, RootIndex, FIntVector(x, y, z), 0);
	}

	OptimizeAndCollapse(OutNodePool, RootIndex);
	PrecomputeNeighborsAndClearance(Desc, Field, OutNodePool);

	return true;
}

void FOctreeBaker::InsertObstacleRecursive(const FVoxelVolumeDesc& Desc, TArray<FOctreeNode>& NodePool, int32 NodeIndex, const FIntVector& ObstacleVoxel, int32 CurrentDepth)
{
	if (!NodePool.IsValidIndex(NodeIndex))
	{
		return;
	}

	if (NodePool[NodeIndex].State == EOctreeNodeState::Solid)
	{
		return;
	}

	if (CurrentDepth == Desc.MaxDepth)
	{
		NodePool[NodeIndex].State = EOctreeNodeState::Solid;
		return;
	}

	if (NodePool[NodeIndex].State == EOctreeNodeState::Empty)
	{
		NodePool[NodeIndex].State = EOctreeNodeState::Mixed;
		const FBox CurrentNodeBounds = NodePool[NodeIndex].Bounds;

		for (int i = 0; i < 8; ++i)
		{
			const int32 NewChildIndex = NodePool.Emplace(FOctreeNode());
			NodePool[NewChildIndex].Bounds = CalculateChildBounds(CurrentNodeBounds, i);
			NodePool[NodeIndex].ChildrenIndices[i] = NewChildIndex;
		}
	}

	const FVector VoxelWorldPos = Desc.VoxelIndexToWorld(ObstacleVoxel);
	const int32 ChildOctant = GetOctantContainingPoint(NodePool[NodeIndex].Bounds, VoxelWorldPos);
	const int32 ChildNodeIndex = NodePool[NodeIndex].ChildrenIndices[ChildOctant];

	if (ChildNodeIndex != -1)
	{
		InsertObstacleRecursive(Desc, NodePool, ChildNodeIndex, ObstacleVoxel, CurrentDepth + 1);
	}
}

void FOctreeBaker::OptimizeAndCollapse(TArray<FOctreeNode>& NodePool, int32 NodeIndex)
{
	if (!NodePool.IsValidIndex(NodeIndex)) return;

	FOctreeNode& Node = NodePool[NodeIndex];
	if (Node.State != EOctreeNodeState::Mixed)
	{
		return;
	}

	for (int i = 0; i < 8; ++i)
	{
		if (Node.ChildrenIndices[i] != -1)
		{
			OptimizeAndCollapse(NodePool, Node.ChildrenIndices[i]);
		}
	}

	const int32 FirstChildIndex = Node.ChildrenIndices[0];
	if (FirstChildIndex == -1 || !NodePool.IsValidIndex(FirstChildIndex)) return;

	const EOctreeNodeState FirstChildState = NodePool[FirstChildIndex].State;
	if (FirstChildState == EOctreeNodeState::Mixed)
	{
		return;
	}

	bool bAllChildrenAreSame = true;
	for (int i = 1; i < 8; ++i)
	{
		const int32 ChildNodeIndex = Node.ChildrenIndices[i];
		if (ChildNodeIndex == -1 || !NodePool.IsValidIndex(ChildNodeIndex) || NodePool[ChildNodeIndex].State != FirstChildState)
		{
			bAllChildrenAreSame = false;
			break;
		}
	}

	if (bAllChildrenAreSame)
	{
		FOctreeNode& ParentNodeToCollapse = NodePool[NodeIndex];
		ParentNodeToCollapse.State = FirstChildState;

		for (int i = 0; i < 8; ++i)
		{
			ParentNodeToCollapse.ChildrenIndices[i] = -1;
		}
	}
}

void FOctreeBaker::PrecomputeNeighborsAndClearance(const FVoxelVolumeDesc& Desc, const FVoxelField& Field, TArray<FOctreeNode>& NodePool)
{
	const FBox TotalVolumeBounds = Desc.Bounds;
	const int32 MaxDepth = Desc.MaxDepth;

	// Phase 1: Build a spatial lookup table (leaf grid cell → node index) to avoid repeated root traversals.
	const float LeafNodeSize = TotalVolumeBounds.GetSize().X / FMath::Pow(2.f, MaxDepth);
	const FVector BoundsMin = TotalVolumeBounds.Min;
	const int32 GridDim = FMath::RoundToInt(FMath::Pow(2.f, MaxDepth));

	const int64 LookupSize = static_cast<int64>(GridDim) * GridDim * GridDim;
	const bool bUseLookupTable = (LookupSize <= 32 * 1024 * 1024); // 32M entries max = 128MB

	TArray<int32> SpatialLookup;
	if (bUseLookupTable)
	{
		SpatialLookup.SetNumUninitialized(LookupSize);
		FMemory::Memset(SpatialLookup.GetData(), 0xFF, LookupSize * sizeof(int32)); // Fill with -1

		for (int32 i = 0; i < NodePool.Num(); ++i)
		{
			const FOctreeNode& Node = NodePool[i];
			if (Node.State == EOctreeNodeState::Mixed) continue;

			const FVector LocalCenter = Node.Bounds.GetCenter() - BoundsMin;
			const int32 gx = FMath::Clamp(FMath::FloorToInt(LocalCenter.X / LeafNodeSize), 0, GridDim - 1);
			const int32 gy = FMath::Clamp(FMath::FloorToInt(LocalCenter.Y / LeafNodeSize), 0, GridDim - 1);
			const int32 gz = FMath::Clamp(FMath::FloorToInt(LocalCenter.Z / LeafNodeSize), 0, GridDim - 1);
			const int64 FlatIdx = static_cast<int64>(gz) * GridDim * GridDim + static_cast<int64>(gy) * GridDim + gx;
			if (FlatIdx >= 0 && FlatIdx < LookupSize)
			{
				SpatialLookup[FlatIdx] = i;
			}
		}
	}

	// Helper: exact clearance for a leaf — minimum EDT distance over the leaf's voxels.
	const bool bHasDistanceField = Field.SquaredDistance.Num() == Desc.NumVoxels();
	auto ComputeLeafClearance = [&Desc, &Field](const FBox& LeafBounds) -> float
	{
		const FIntVector MinVoxel = Desc.WorldToVoxelIndex(LeafBounds.Min + FVector(Desc.VoxelExtent * 0.25f));
		const FIntVector MaxVoxel = Desc.WorldToVoxelIndex(LeafBounds.Max - FVector(Desc.VoxelExtent * 0.25f));

		const FIntVector ClampedMin(
			FMath::Max(MinVoxel.X, 0), FMath::Max(MinVoxel.Y, 0), FMath::Max(MinVoxel.Z, 0));
		const FIntVector ClampedMax(
			FMath::Min(MaxVoxel.X, Desc.MapSize.X - 1), FMath::Min(MaxVoxel.Y, Desc.MapSize.Y - 1), FMath::Min(MaxVoxel.Z, Desc.MapSize.Z - 1));

		int32 MinSq = Field.BandVoxels * Field.BandVoxels;
		for (int32 z = ClampedMin.Z; z <= ClampedMax.Z; ++z)
		{
			for (int32 y = ClampedMin.Y; y <= ClampedMax.Y; ++y)
			{
				for (int32 x = ClampedMin.X; x <= ClampedMax.X; ++x)
				{
					MinSq = FMath::Min(MinSq, Field.SquaredDistance[Desc.FlatIndex(x, y, z)]);
				}
			}
		}

		// Distances are center-to-center; subtract the half-diagonal to stay conservative
		// against the solid voxel surface in every direction.
		const float DistVoxels = FMath::Sqrt(static_cast<float>(MinSq));
		return FMath::Max(0.f, DistVoxels - GConservativeDistanceBiasVoxels) * Desc.VoxelExtent;
	};

	// Phase 2: Compute neighbors and clearance per leaf.
	const int32 NumNodes = NodePool.Num();
	FScopedSlowTask NeighborTask(NumNodes, LOCTEXT("PrecomputingNeighbors", "Precomputing Neighbors..."));

	constexpr float Epsilon = 1.0f;

	for (int32 i = 0; i < NumNodes; ++i)
	{
		NeighborTask.EnterProgressFrame(1);
		if (NeighborTask.ShouldCancel()) return;

		if (NodePool[i].State == EOctreeNodeState::Mixed)
		{
			continue;
		}

		if (NodePool[i].State == EOctreeNodeState::Empty)
		{
			NodePool[i].MaxAllowedRadius = bHasDistanceField
				? ComputeLeafClearance(NodePool[i].Bounds)
				: NodePool[i].Bounds.GetExtent().GetMin();
		}
		else
		{
			NodePool[i].MaxAllowedRadius = 0.f;
		}

		const FBox NodeBounds = NodePool[i].Bounds;
		for (int32 DirIndex = 0; DirIndex < static_cast<int32>(ENeighborDirection::MAX); ++DirIndex)
		{
			FVector TestPoint = NodeBounds.GetCenter();
			const ENeighborDirection Dir = static_cast<ENeighborDirection>(DirIndex);

			switch (Dir)
			{
			case ENeighborDirection::Right: TestPoint.X = NodeBounds.Max.X + Epsilon; break;
			case ENeighborDirection::Left:  TestPoint.X = NodeBounds.Min.X - Epsilon; break;
			case ENeighborDirection::Front: TestPoint.Y = NodeBounds.Max.Y + Epsilon; break;
			case ENeighborDirection::Back:  TestPoint.Y = NodeBounds.Min.Y - Epsilon; break;
			case ENeighborDirection::Up:    TestPoint.Z = NodeBounds.Max.Z + Epsilon; break;
			case ENeighborDirection::Down:  TestPoint.Z = NodeBounds.Min.Z - Epsilon; break;
			default: break;
			}

			if (TotalVolumeBounds.IsInside(TestPoint))
			{
				int32 NeighborNodeIndex = -1;

				if (bUseLookupTable)
				{
					const FVector LocalTest = TestPoint - BoundsMin;
					const int32 gx = FMath::Clamp(FMath::FloorToInt(LocalTest.X / LeafNodeSize), 0, GridDim - 1);
					const int32 gy = FMath::Clamp(FMath::FloorToInt(LocalTest.Y / LeafNodeSize), 0, GridDim - 1);
					const int32 gz = FMath::Clamp(FMath::FloorToInt(LocalTest.Z / LeafNodeSize), 0, GridDim - 1);
					const int64 FlatIdx = static_cast<int64>(gz) * GridDim * GridDim + static_cast<int64>(gy) * GridDim + gx;
					if (FlatIdx >= 0 && FlatIdx < LookupSize)
					{
						NeighborNodeIndex = SpatialLookup[FlatIdx];
					}

					// Lookup maps leaf-level cells; collapsed (larger) neighbors miss. Fall back to traversal.
					if (NeighborNodeIndex == -1)
					{
						NeighborNodeIndex = FindLeafNode(NodePool, 0, TestPoint, MaxDepth);
					}
				}
				else
				{
					NeighborNodeIndex = FindLeafNode(NodePool, 0, TestPoint, MaxDepth);
				}

				NodePool[i].Neighbors[DirIndex] = NeighborNodeIndex;
			}
			else
			{
				NodePool[i].Neighbors[DirIndex] = -1;
			}
		}
	}
}

//~======================================================================
// Portals
//~======================================================================

void FOctreeBaker::ConnectPortals(
	UWorld* World,
	AOctreeVoxelVolume* SourceVolume,
	TArray<FOctreeNode>& InOutMyNodePool,
	TMap<int32, TArray<FNavPortalBakeLink>>& OutPortalNodes,
	const TMap<AOctreeVoxelVolume*, TArray<FOctreeNode>>& AllVolumeData,
	const TArray<AOctreeVoxelVolume*>& AllCreatedVolumes)
{
	if (!World || !IsValid(SourceVolume))
	{
		UE_LOG(LogTemp, Error, TEXT("OctreeBaker::ConnectPortals: invalid world or source volume."));
		return;
	}

	constexpr float Epsilon = 1.0f;
	int32 CreatedPortalCount = 0;

	for (int32 MyNodeIndex = 0; MyNodeIndex < InOutMyNodePool.Num(); ++MyNodeIndex)
	{
		FOctreeNode& MyNode = InOutMyNodePool[MyNodeIndex];

		// Portals can only be created from empty nodes.
		if (MyNode.State != EOctreeNodeState::Empty)
		{
			continue;
		}

		for (int32 DirIndex = 0; DirIndex < static_cast<int32>(ENeighborDirection::MAX); ++DirIndex)
		{
			// Skip if there's already an internal neighbor (we only care about boundary nodes).
			if (MyNode.Neighbors[DirIndex] != -1)
			{
				continue;
			}

			const ENeighborDirection Direction = static_cast<ENeighborDirection>(DirIndex);

			FVector TestPoint = MyNode.Bounds.GetCenter();
			switch (Direction)
			{
			case ENeighborDirection::Right: TestPoint.X = MyNode.Bounds.Max.X + Epsilon; break;
			case ENeighborDirection::Left:  TestPoint.X = MyNode.Bounds.Min.X - Epsilon; break;
			case ENeighborDirection::Front: TestPoint.Y = MyNode.Bounds.Max.Y + Epsilon; break;
			case ENeighborDirection::Back:  TestPoint.Y = MyNode.Bounds.Min.Y - Epsilon; break;
			case ENeighborDirection::Up:    TestPoint.Z = MyNode.Bounds.Max.Z + Epsilon; break;
			case ENeighborDirection::Down:  TestPoint.Z = MyNode.Bounds.Min.Z - Epsilon; break;
			}

			// Find the neighboring volume: closest center among volumes containing the test point.
			AOctreeVoxelVolume* NeighborVolume = nullptr;
			float MinDistanceSq = TNumericLimits<float>::Max();

			for (AOctreeVoxelVolume* PotentialNeighbor : AllCreatedVolumes)
			{
				if (!PotentialNeighbor || PotentialNeighbor == SourceVolume) continue;

				const FBox NeighborBounds = PotentialNeighbor->GetVolumeBounds();

				if (NeighborBounds.IsInsideOrOn(TestPoint))
				{
					const float DistSq = FVector::DistSquared(TestPoint, NeighborBounds.GetCenter());
					if (DistSq < MinDistanceSq)
					{
						MinDistanceSq = DistSq;
						NeighborVolume = PotentialNeighbor;
					}
				}
			}

			// Defensive distance check to avoid bad matches with overlapping volumes.
			if (NeighborVolume)
			{
				const float VolumeHalfExtent = NeighborVolume->GetVolumeBounds().GetExtent().X;
				const float DistanceThresholdSq = FMath::Square(VolumeHalfExtent * 1.5f);

				if (MinDistanceSq > DistanceThresholdSq)
				{
					NeighborVolume = nullptr;
				}
			}

			if (NeighborVolume)
			{
				const TArray<FOctreeNode>* NeighborNodePoolPtr = AllVolumeData.Find(NeighborVolume);
				if (!NeighborNodePoolPtr)
				{
					continue;
				}

				const int32 NeighborNodeIndex = FindLeafNode(*NeighborNodePoolPtr, 0, TestPoint, NeighborVolume->GetMaxOctreeDepth());

				if (NeighborNodePoolPtr->IsValidIndex(NeighborNodeIndex))
				{
					const FOctreeNode& NeighborNode = (*NeighborNodePoolPtr)[NeighborNodeIndex];

					// A portal can only be created if the neighbor node is also empty.
					if (NeighborNode.State == EOctreeNodeState::Empty)
					{
						const float PortalClearance = FMath::Min(MyNode.MaxAllowedRadius, NeighborNode.MaxAllowedRadius);

						FNavPortalBakeLink PortalLink;
						PortalLink.TargetVolume = NeighborVolume;
						PortalLink.TargetNodeIndex = NeighborNodeIndex;
						PortalLink.MaxAllowedRadius = PortalClearance;
						OutPortalNodes.FindOrAdd(MyNodeIndex).Add(PortalLink);

						CreatedPortalCount++;
					}
					else
					{
						DrawDebugBox(World, NeighborNode.Bounds.GetCenter(), NeighborNode.Bounds.GetExtent(), FColor::Red, false, 30.0f, 0, 10.0f);
					}
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("OctreeBaker::ConnectPortals: '%s' could not find a valid leaf in neighbor volume at TestPoint(%s)."),
						*SourceVolume->GetActorLabel(), *TestPoint.ToString());
				}
			}
		}
	}

	UE_LOG(LogTemp, Verbose, TEXT("OctreeBaker::ConnectPortals: '%s' created %d portal links."), *SourceVolume->GetActorLabel(), CreatedPortalCount);
}

//~======================================================================
// Shared math
//~======================================================================

FBox FOctreeBaker::CalculateChildBounds(const FBox& ParentBounds, int32 ChildIndex)
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
	default: return FBox(ForceInit);
	}
}

int32 FOctreeBaker::GetOctantContainingPoint(const FBox& NodeBounds, const FVector& Point)
{
	int32 Octant = 0;
	const FVector Center = NodeBounds.GetCenter();
	if (Point.X >= Center.X) Octant |= 1;
	if (Point.Y >= Center.Y) Octant |= 2;
	if (Point.Z >= Center.Z) Octant |= 4;
	return Octant;
}

int32 FOctreeBaker::FindLeafNode(const TArray<FOctreeNode>& NodePool, int32 RootIndex, const FVector& WorldPos, int32 MaxDepth)
{
	int32 CurrentNodeIndex = RootIndex;
	int32 CurrentDepth = 0;

	while (NodePool.IsValidIndex(CurrentNodeIndex))
	{
		const FOctreeNode& Node = NodePool[CurrentNodeIndex];

		if (Node.State != EOctreeNodeState::Mixed || CurrentDepth >= MaxDepth)
		{
			return CurrentNodeIndex;
		}

		const int32 ChildOctant = GetOctantContainingPoint(Node.Bounds, WorldPos);
		CurrentNodeIndex = Node.ChildrenIndices[ChildOctant];
		CurrentDepth++;
	}
	return -1;
}

#undef LOCTEXT_NAMESPACE
