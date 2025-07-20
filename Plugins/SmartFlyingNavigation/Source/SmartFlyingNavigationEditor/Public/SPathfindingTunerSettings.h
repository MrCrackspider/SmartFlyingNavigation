// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "SPathfindingTunerSettings.generated.h"

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
};
