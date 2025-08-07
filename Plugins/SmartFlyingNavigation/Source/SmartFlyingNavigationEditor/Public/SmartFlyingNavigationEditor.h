// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"
#include "Framework/MultiBox/MultiBoxBuilder.h" 
#include "Widgets/Docking/SDockTab.h"      

class FSmartFlyingNavigationEditorModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

private:
    TSharedRef<SDockTab> SpawnPathfindingTunerTab(const FSpawnTabArgs& SpawnTabArgs);
    TSharedPtr<FExtender> MenuExtender; 
};