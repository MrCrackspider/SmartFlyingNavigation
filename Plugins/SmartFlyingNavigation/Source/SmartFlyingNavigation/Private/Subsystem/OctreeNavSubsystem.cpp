// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Subsystem/OctreeNavSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "NavData/OctreeDataAsset.h"

void UOctreeNavSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
}

void UOctreeNavSubsystem::RegisterVolumeChunk(AOctreeVoxelVolume* Volume)
{
	if (IsValid(Volume))
	{
		RegisteredVolumeChunks.AddUnique(Volume);
		if (Volume->GetBakedOctreeData().IsValid())
		{
			VolumeDataAssetMap.Add(Volume->GetBakedOctreeData(), Volume);
		}
	}
}

void UOctreeNavSubsystem::UnregisterVolumeChunk(AOctreeVoxelVolume* Volume)
{
	if (IsValid(Volume))
	{
		RegisteredVolumeChunks.Remove(Volume);
		if (Volume->GetBakedOctreeData().IsValid())
		{
			VolumeDataAssetMap.Remove(Volume->GetBakedOctreeData());
		}
	}
}

AOctreeVoxelVolume* UOctreeNavSubsystem::GetVolumeChunkAt(const FVector& WorldLocation) const
{
	for (AOctreeVoxelVolume* Volume : RegisteredVolumeChunks)
	{
		if (Volume && Volume->GetIsSpatiallyLoaded() && Volume->IsReadyForNavigation() && Volume->GetVolumeBounds().IsInside(WorldLocation))
		{
			return Volume;
		}
	}
	return nullptr;
}

AOctreeVoxelVolume* UOctreeNavSubsystem::GetVolumeByDataAsset(const TSoftObjectPtr<UOctreeDataAsset>& DataAsset) const
{
	if (const TObjectPtr<AOctreeVoxelVolume>* FoundVolume = VolumeDataAssetMap.Find(DataAsset))
	{
		return *FoundVolume;
	}
	return nullptr;
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
	VolumeDataAssetMap.Empty();
}
#endif