// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "PathfindingTunerSettings.generated.h"

/**
 * 
 */
UCLASS(Config = EditorPerProjectUserSettings)
class SMARTFLYINGNAVIGATION_API UPathfindingTunerSettings : public UObject
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
};
