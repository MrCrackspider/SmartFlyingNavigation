// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Navigation3D.h"
#include "LevelEditor.h"
#include "Widgets/Docking/SDockTab.h"
#include "Pathfinder/UI/PathfindingTuner.h"

#include "WorkspaceMenuStructure.h"
#include "WorkspaceMenuStructureModule.h"
#include "EditorStyleSet.h"

static const FName PathfindingTunerTabName("PathfindingTuner");
static const FName PathfindingTunerIconName = "Icons.Folder"; 

#define LOCTEXT_NAMESPACE "FNavigation3DModule"

void FNavigation3DModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
    if (!IsRunningCommandlet() && !IsRunningGame())
    {
        FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
        
        TSharedPtr<FExtender> MenuExtender = MakeShareable(new FExtender());
        MenuExtender->AddMenuExtension(
            "WindowLayout",
            EExtensionHook::After,
            nullptr,
            FMenuExtensionDelegate::CreateLambda([](FMenuBuilder& Builder)
            {
                Builder.AddMenuEntry(
                    FText::FromString("Pathfinding Tuner"),
                    FText::FromString("Open the Pathfinding Tuner window."),
                    FSlateIcon(FAppStyle::GetAppStyleSetName(), PathfindingTunerIconName),
                    FUIAction(
                        FExecuteAction::CreateLambda([]()
                        {
                            FGlobalTabmanager::Get()->TryInvokeTab(PathfindingTunerTabName);
                        })
                    )
                );
            })
        );
        
        LevelEditorModule.GetMenuExtensibilityManager()->AddExtender(MenuExtender);

        FGlobalTabmanager::Get()->RegisterTabSpawner(PathfindingTunerTabName, 
            FOnSpawnTab::CreateRaw(this, &FNavigation3DModule::SpawnPathfindingTunerTab))
            .SetDisplayName(FText::FromString("Pathfinding Tuner"))
        .SetIcon(FSlateIcon(FAppStyle::GetAppStyleSetName(), PathfindingTunerIconName)) 
        .SetGroup(WorkspaceMenu::GetMenuStructure().GetDeveloperToolsMiscCategory());
    }
}

void FNavigation3DModule::ShutdownModule()
{
    if (FModuleManager::Get().IsModuleLoaded("LevelEditor"))
    {
        FGlobalTabmanager::Get()->UnregisterTabSpawner(PathfindingTunerTabName);
    }
}

TSharedRef<SDockTab> FNavigation3DModule::SpawnPathfindingTunerTab(const FSpawnTabArgs& SpawnTabArgs)
{
    return SNew(SDockTab)
        .TabRole(ETabRole::NomadTab)
        [
            SNew(SPathfindingTuner)
        ];
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FNavigation3DModule, Navigation3D)