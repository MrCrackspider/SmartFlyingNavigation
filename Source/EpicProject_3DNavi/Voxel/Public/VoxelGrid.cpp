#include "VoxelGrid.h"

void UVoxelGrid::Initialize(FVector InVolumeOrigin, FVector InBoxExtent, float InVoxleSize)
{
	VolumeOrigin = InVolumeOrigin;
	BoxExtent = InBoxExtent;
	VoxelSize = InVoxleSize;
	GridOrigin = VolumeOrigin - BoxExtent;
	VolumeBounds = FBox(GridOrigin, GridOrigin + (BoxExtent * 2));
}

FIntVector UVoxelGrid::WorldToGridCoordinates(const FVector& WorldPosition) const
{
	const FVector RelativePos = WorldPosition - GridOrigin;
		
	return FIntVector(
		FMath::FloorToInt(RelativePos.X / VoxelSize),
		FMath::FloorToInt(RelativePos.Y / VoxelSize),
		FMath::FloorToInt(RelativePos.Z / VoxelSize)
	);
}

FVector UVoxelGrid::GridToWorldCoordinates(const FIntVector& GridPos) const
{
	const FVector RelativePos = FVector(GridPos) * VoxelSize;
	const FVector WorldPos = GridOrigin + RelativePos;
	return WorldPos + FVector(VoxelSize / 2.0f);
}

void UVoxelGrid::SetVoxel(const FIntVector& GridCoords, const FVoxelGridData& Data)
{
	if (Data.bIsSolid)
	{
		//장애물이면 추가
		VoxelMap.Add(GridCoords, Data);
	}
	else
	{
		//장애물이 아니면 제거
		VoxelMap.Remove(GridCoords);
	}
}

bool UVoxelGrid::GetVoxel(const FIntVector& GridCoords, FVoxelGridData& OutData) const
{
	const FVoxelGridData* FoundData = VoxelMap.Find(GridCoords);
	if (FoundData)
	{
		OutData = *FoundData;
		return true;
	}

	OutData.bIsSolid = false;
	return false;
}

bool UVoxelGrid::IsInVolume(const FVector& WorldPosition) const
{
	return VolumeBounds.IsInsideOrOn(WorldPosition);
}
