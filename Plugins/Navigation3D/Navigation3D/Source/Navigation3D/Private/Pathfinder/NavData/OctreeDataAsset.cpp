#include "Pathfinder/NavData/OctreeDataAsset.h"

const FName UOctreeDataAsset::OctreeAssetType = TEXT("OctreeData");

FPrimaryAssetId UOctreeDataAsset::GetPrimaryAssetId() const
{
	return FPrimaryAssetId(OctreeAssetType, GetFName());
}