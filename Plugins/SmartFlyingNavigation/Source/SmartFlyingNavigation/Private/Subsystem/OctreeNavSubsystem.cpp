// Fill out your copyright notice in the Description page of Project Settings.


#include "Subsystem/OctreeNavSubsystem.h"
#include "NavVolume/OctreeVoxelVolume.h"

void UOctreeNavSubsystem::RegisterVolumeChunk(AOctreeVoxelVolume* Volume)
{
	if (IsValid(Volume))
	{
		RegisteredVolumeChunks.AddUnique(Volume);
	}
}

void UOctreeNavSubsystem::UnregisterVolumeChunk(AOctreeVoxelVolume* Volume)
{
	if (IsValid(Volume))
	{
		RegisteredVolumeChunks.Remove(Volume);
	}
}

AOctreeVoxelVolume* UOctreeNavSubsystem::GetVolumeChunkAt(const FVector& WorldLocation) const
{
	for (AOctreeVoxelVolume* Volume : RegisteredVolumeChunks)
	{
		// 데이터가 완전히 로드되었고, 요청된 위치가 해당 볼륨의 바운더리 내에 있는지 확인합니다.
		if (Volume && Volume->IsReadyForNavigation() && Volume->GetVolumeBounds().IsInside(WorldLocation))
		{
			return Volume;
		}
	}
	return nullptr;
}
