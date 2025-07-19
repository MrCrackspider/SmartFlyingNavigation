// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "NavData/OctreeDataAsset.h"

const FName UOctreeDataAsset::OctreeAssetType = TEXT("OctreeData");

FPrimaryAssetId UOctreeDataAsset::GetPrimaryAssetId() const
{
	return FPrimaryAssetId(OctreeAssetType, GetFName());
}