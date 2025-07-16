#include "Pathfinder/NaviDataAssets/OctreeDataAsset.h"

// 에셋 타입을 위한 FName을 정적으로 정의합니다.
const FName UOctreeDataAsset::OctreeAssetType = TEXT("OctreeData");

FPrimaryAssetId UOctreeDataAsset::GetPrimaryAssetId() const
{
	return FPrimaryAssetId(OctreeAssetType, GetFName());
}