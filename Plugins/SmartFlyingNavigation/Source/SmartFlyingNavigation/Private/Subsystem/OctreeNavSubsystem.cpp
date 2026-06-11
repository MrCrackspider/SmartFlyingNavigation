// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Subsystem/OctreeNavSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "NavData/WorldNavigationDataAsset.h"
#include "Subsystem/AvoidanceSubsystem.h"
#include "HAL/IConsoleManager.h"
#include "Engine/World.h"

/**
 * One-shot, logcat-friendly nav stats dump.
 * Usage: `sfn.DumpNavStats` from the console (PC) or
 * `adb shell am broadcast -a android.intent.action.RUN -e cmd "sfn.DumpNavStats"` on device.
 */
static FAutoConsoleCommandWithWorldAndArgs GDumpNavStatsCmd(
	TEXT("sfn.DumpNavStats"),
	TEXT("Logs loaded nav chunk count/memory, dynamic obstacle count, and registered agent count."),
	FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(
		[](const TArray<FString>& Args, UWorld* World)
		{
			if (!World)
			{
				return;
			}

			const UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>();
			if (!NavSubsystem)
			{
				UE_LOG(LogTemp, Log, TEXT("[SFN] No OctreeNavSubsystem in this world."));
				return;
			}

			int32 LoadedChunks = 0;
			int64 TotalBytes = 0;
			for (const AOctreeVoxelVolume* Volume : NavSubsystem->GetRegisteredVolumes())
			{
				if (IsValid(Volume) && Volume->IsReadyForNavigation())
				{
					LoadedChunks++;
					TotalBytes += Volume->GetLoadedChunkBytes();

					int64 NodePoolBytes = 0, AirwayBytes = 0, SdfBytes = 0;
					Volume->GetLoadedChunkMemoryBreakdown(NodePoolBytes, AirwayBytes, SdfBytes);
					UE_LOG(LogTemp, Log, TEXT("[SFN]   Chunk %d ('%s'): NodePool=%.2f MB Airway=%.2f MB SDF=%.2f MB"),
						Volume->GetLoadedChunkId(), *Volume->GetName(),
						NodePoolBytes / (1024.0 * 1024.0),
						AirwayBytes / (1024.0 * 1024.0),
						SdfBytes / (1024.0 * 1024.0));
				}
			}

			const int32 ObstacleCount = NavSubsystem->GetObstacleRegistry().Num();
			const FPlatformMemoryStats MemStats = FPlatformMemory::GetStats();

			UE_LOG(LogTemp, Log, TEXT("[SFN] NavStats: LoadedChunks=%d ChunkMemory=%.2f MB DynamicObstacles=%d RegisteredVolumes=%d | Process=%.0f MB DeviceFree=%.0f/%.0f MB"),
				LoadedChunks,
				TotalBytes / (1024.0 * 1024.0),
				ObstacleCount,
				NavSubsystem->GetRegisteredVolumes().Num(),
				MemStats.UsedPhysical / (1024.0 * 1024.0),
				MemStats.AvailablePhysical / (1024.0 * 1024.0),
				MemStats.TotalPhysical / (1024.0 * 1024.0));
		}));

void UOctreeNavSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	bIsReady = false;
}

void UOctreeNavSubsystem::RegisterVolumeChunk(AOctreeVoxelVolume* Volume)
{
	if (IsValid(Volume))
	{
		RegisteredVolumeChunks.AddUnique(Volume);
		if (!Volume->IsReadyForNavigation() && InitializeVolumeFromManifestPayload(Volume))
		{
			OnVolumeRegistered.Broadcast(Volume);
		}
	}
}

void UOctreeNavSubsystem::UnregisterVolumeChunk(AOctreeVoxelVolume* Volume)
{
	if (IsValid(Volume))
	{
		RegisteredVolumeChunks.Remove(Volume);

		const int32 ChunkId = Volume->GetLoadedChunkId();
		if (ChunkId != INDEX_NONE)
		{
			ChunkIdToVolumeMap.Remove(ChunkId);
		}
	}
}

void UOctreeNavSubsystem::SetWorldNavigationData(UWorldNavigationDataAsset* InWorldNavigationData)
{
	CurrentWorldNavigationData = nullptr;
	ChunkIdToVolumeMap.Reset();

	if (!InWorldNavigationData)
	{
		return;
	}

	if (!InWorldNavigationData->IsFormatSupported())
	{
		return;
	}

	CurrentWorldNavigationData = InWorldNavigationData;

	// Initialize any volumes that registered before the manifest arrived.
	for (AOctreeVoxelVolume* Volume : RegisteredVolumeChunks)
	{
		if (IsValid(Volume) && !Volume->IsReadyForNavigation() && InitializeVolumeFromManifestPayload(Volume))
		{
			OnVolumeRegistered.Broadcast(Volume);
		}
	}

	UE_LOG(LogTemp, Log, TEXT("OctreeNavSubsystem received world navigation manifest '%s' with %d chunks and %d portals."),
		*CurrentWorldNavigationData->GetName(),
		CurrentWorldNavigationData->Chunks.Num(),
		CurrentWorldNavigationData->Portals.Num());
}

AOctreeVoxelVolume* UOctreeNavSubsystem::GetVolumeChunkAt(const FVector& WorldLocation) const
{
	for (AOctreeVoxelVolume* Volume : RegisteredVolumeChunks)
	{
		if (Volume && Volume->IsReadyForNavigation() && Volume->GetVolumeBounds().IsInside(WorldLocation))
		{
			return Volume;
		}
	}
	return nullptr;
}

AOctreeVoxelVolume* UOctreeNavSubsystem::GetVolumeByChunkId(int32 ChunkId) const
{
	if (const TObjectPtr<AOctreeVoxelVolume>* FoundVolume = ChunkIdToVolumeMap.Find(ChunkId))
	{
		return *FoundVolume;
	}
	return nullptr;
}

bool UOctreeNavSubsystem::InitializeVolumeFromManifestPayload(AOctreeVoxelVolume* Volume)
{
	if (!CurrentWorldNavigationData || !IsValid(Volume))
	{
		return false;
	}

	const int32 ChunkId = FindChunkIdForVolume(Volume);
	if (ChunkId == INDEX_NONE)
	{
		return false;
	}

	TArray<FOctreeNode> NodePool;
	FAirwayGraph AirwayGraph;
	FNavDistanceField DistanceField;
	if (!CurrentWorldNavigationData->DecompressChunkPayload(ChunkId, NodePool, AirwayGraph, &DistanceField))
	{
		return false;
	}

	if (!Volume->InitializeFromManifestPayload(ChunkId, MoveTemp(NodePool), MoveTemp(AirwayGraph), MoveTemp(DistanceField)))
	{
		return false;
	}

	ChunkIdToVolumeMap.Add(ChunkId, Volume);
	return true;
}

int32 UOctreeNavSubsystem::FindChunkIdForVolume(const AOctreeVoxelVolume* Volume) const
{
	if (!CurrentWorldNavigationData || !IsValid(Volume))
	{
		return INDEX_NONE;
	}

	const FBox VolumeBounds = Volume->GetVolumeBounds();
	const FVector VolumeCenter = VolumeBounds.GetCenter();
	const FVector VolumeExtent = VolumeBounds.GetExtent();

	for (const FNavigationChunkRecord& Chunk : CurrentWorldNavigationData->Chunks)
	{
		if (Chunk.Bounds.IsInsideOrOn(VolumeCenter) &&
			Chunk.Bounds.GetExtent().Equals(VolumeExtent, 1.0f))
		{
			return Chunk.ChunkId;
		}
	}

	return INDEX_NONE;
}

#if WITH_EDITOR
AOctreeVoxelVolume* UOctreeNavSubsystem::FindVolumeContainingPoint_BakeTime(const FVector& WorldLocation) const
{
	for (AOctreeVoxelVolume* Volume : RegisteredVolumeChunks)
	{
		if (IsValid(Volume) && Volume->GetVolumeBounds().IsInside(WorldLocation))
		{
			return Volume;
		}
	}
	return nullptr;
}

void UOctreeNavSubsystem::ClearAllRegisteredVolumes()
{
	RegisteredVolumeChunks.Empty();
	ChunkIdToVolumeMap.Empty();
}
#endif
