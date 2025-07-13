// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FNavigation3DModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
private:
	/** 탭(창)을 생성하는 콜백 함수입니다. */
	TSharedRef<class SDockTab> SpawnPathfindingTunerTab(const class FSpawnTabArgs& SpawnTabArgs);
};
