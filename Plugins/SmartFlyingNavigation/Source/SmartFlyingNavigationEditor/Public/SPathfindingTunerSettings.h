// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
//#include "WorldPartition/DataLayer/DataLayerAsset.h"
#include "SPathfindingTunerSettings.generated.h"

class UVoxelObstacleProfile;

UCLASS(Config = EditorPerProjectUserSettings)
class UPathfindingTunerSettings : public UObject
{
	GENERATED_BODY()

public:
	UPathfindingTunerSettings();
	
	UPROPERTY(Config)
	bool bIsPerformanceCategoryExpanded;

	UPROPERTY(Config)
	bool bIsOctreeSettingsCategoryExpanded;

	UPROPERTY(Config)
	bool bIsBakedDataCategoryExpanded;
	
	UPROPERTY(Config)
	bool bIsAgentSettingsCategoryExpanded;

	UPROPERTY(Config)
	bool bIsDebugSettingsCategoryExpanded;

	UPROPERTY(Config)
	bool bAgentDebugSettingsExpanded;

	// Begin Test
	/** The size of the chunks (in world units) to use when baking the entire world. */
	UPROPERTY(Config)
	float WorldBake_VolumeSize = 25600.0f;

	/** The size of the Voxels to use when baking the entire world. */
	UPROPERTY(Config)
	float WorldBake_VoxelExtent = 100.0f;

	/** The maximum depth of the Octree to use when baking the entire world. */
	UPROPERTY(Config)
	int32 WorldBake_MaxOctreeDepth = 7;

	/** The default path to save generated assets to when baking the entire world. */
	UPROPERTY(Config)
	FDirectoryPath WorldBake_SaveDirectory;

	/** The obstacle profile to use when baking the entire world. */
	UPROPERTY(Config)
	TSoftObjectPtr<UVoxelObstacleProfile> WorldBake_ObstacleProfile;

	/** Variable to save the expansion state of the UI section. */
	UPROPERTY(Config)
	bool bIsWorldBakeCategoryExpanded = true;

	//UPROPERTY(EditAnywhere, Category = "World Bake Settings")
	//TSoftObjectPtr<UDataLayerAsset> WorldBake_TargetDataLayer;
	// End Test
};