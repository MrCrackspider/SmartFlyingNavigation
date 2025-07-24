// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "SPathfindingTunerSettings.h"
#include "NavData/VoxelObstacleProfile.h"

UPathfindingTunerSettings::UPathfindingTunerSettings()
{
	bIsPerformanceCategoryExpanded = true;
	bIsOctreeSettingsCategoryExpanded = false;
	bIsBakedDataCategoryExpanded = false;
	bIsAgentSettingsCategoryExpanded = false;
	bIsDebugSettingsCategoryExpanded = false;
	bAgentDebugSettingsExpanded = false;

	WorldBake_ChunkSize = 25600.0f;
	WorldBake_VoxelExtent = 100.0f;
	WorldBake_MaxOctreeDepth = 7;
	bIsWorldBakeCategoryExpanded = true;
}

