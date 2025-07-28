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
	/** 월드 전체를 Bake할 때 사용할 청크의 크기 (월드 단위) */
	UPROPERTY(Config)
	float WorldBake_VolumeSize = 25600.0f;

	/** 월드 전체를 Bake할 때 사용할 Voxel의 크기 */
	UPROPERTY(Config)
	float WorldBake_VoxelExtent = 100.0f;

	/** 월드 전체를 Bake할 때 사용할 Octree의 최대 깊이 */
	UPROPERTY(Config)
	int32 WorldBake_MaxOctreeDepth = 7;

	/** 월드 전체 Bake 시 생성된 에셋들을 저장할 기본 경로 */
	UPROPERTY(Config)
	FDirectoryPath WorldBake_SaveDirectory;

	/** 월드 전체 Bake 시 사용할 장애물 프로파일 */
	UPROPERTY(Config)
	TSoftObjectPtr<UVoxelObstacleProfile> WorldBake_ObstacleProfile;

	/** UI 섹션 확장 상태 저장을 위한 변수 */
	UPROPERTY(Config)
	bool bIsWorldBakeCategoryExpanded = true;

	//UPROPERTY(EditAnywhere, Category = "World Bake Settings")
	//TSoftObjectPtr<UDataLayerAsset> WorldBake_TargetDataLayer;
	// End Test
};
