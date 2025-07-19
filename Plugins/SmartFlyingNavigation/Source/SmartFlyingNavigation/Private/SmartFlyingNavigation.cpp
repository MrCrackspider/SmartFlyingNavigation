// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "SmartFlyingNavigation.h"
#include "LevelEditor.h"
#include "Widgets/Docking/SDockTab.h"
#include "UI/PathfindingTuner.h"

#include "WorkspaceMenuStructure.h"
#include "WorkspaceMenuStructureModule.h"

#include "Styling/SlateStyle.h"
#include "Styling/SlateStyleRegistry.h"
#include "Interfaces/IPluginManager.h" // 

static const FName PathfindingTunerTabName("PathfindingTuner");
static const FName PathfindingTunerIconName = "Icons.Folder"; 

#define LOCTEXT_NAMESPACE "FSmartFlyingNavigationModule"

namespace PathfindingTunerStyleInternal
{
    TSharedPtr<FSlateStyleSet> StyleSet;

    FName GetStyleSetName()
    {
        static FName StyleSetName(TEXT("PathfindingTunerStyle"));
        return StyleSetName;
    }

    FString GetResourcePath(const FString& RelativePath, const TCHAR* Extension)
    {
        static FString ResourceDir = IPluginManager::Get().FindPlugin(TEXT("SmartFlyingNavigation"))->GetBaseDir() / TEXT("Resources");
        return (ResourceDir / RelativePath) + Extension;
    }

    #define IMAGE_BRUSH_FROM_RESOURCE( RelativePath, ... ) FSlateImageBrush( GetResourcePath( RelativePath, TEXT(".png") ), __VA_ARGS__ )


    void Initialize()
    {
        if (!StyleSet.IsValid())
        {
            StyleSet = MakeShareable(new FSlateStyleSet(GetStyleSetName()));
            StyleSet->Set("PathfindingTuner.CustomIcon", new IMAGE_BRUSH_FROM_RESOURCE("Ham", FVector2D(16.0f, 16.0f)));

            FSlateStyleRegistry::RegisterSlateStyle(*StyleSet);
        }
    }

    void Shutdown()
    {
        if (StyleSet.IsValid())
        {
            FSlateStyleRegistry::UnRegisterSlateStyle(*StyleSet);
            StyleSet.Reset();
        }
    }
} // namespace PathfindingTunerStyleInternal
 

void FSmartFlyingNavigationModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
    if (!IsRunningCommandlet() && !IsRunningGame())
    {
        PathfindingTunerStyleInternal::Initialize();
        
        FLevelEditorModule& LevelEditorModule = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
        
        TSharedPtr<FExtender> MenuExtender = MakeShareable(new FExtender());
        MenuExtender->AddMenuExtension(
            "WindowLayout",
            EExtensionHook::After,
            nullptr,
            FMenuExtensionDelegate::CreateLambda([](FMenuBuilder& Builder)
            {
                Builder.AddWidget(
                     SNew(SBorder)
                     .Padding(FMargin(5, 2))
                     .BorderImage(FAppStyle::GetBrush("Menu.Background"))
                     [
                         SNew(SHorizontalBox)

                         + SHorizontalBox::Slot()
                         .Padding(FMargin(5.f, 0.0f, 5.0f, 0.0f))
                         .AutoWidth()
                         .VAlign(VAlign_Center)
                         [
                             SNew(STextBlock)
                             .Text(FText::FromString("PLUGIN"))
                             .Font(FCoreStyle::GetDefaultFontStyle("Bold", 7.5))
                             .ColorAndOpacity(FLinearColor(0.15f, 0.15f, 0.15f, 1.0f))  
                         ]

                         // 우측 선
                         + SHorizontalBox::Slot()
                         .FillWidth(1.0f)
                         .Padding(FMargin(5.0f, 0.0f, 5.0f, 0.0f))
                         .VAlign(VAlign_Center)
                         [
                             SNew(SBox)
                             .HeightOverride(1.0f) 
                             [
                                 SNew(SImage)
                                 .Image(FCoreStyle::Get().GetBrush("GenericWhiteBox"))
                                 .ColorAndOpacity(FLinearColor(0.15f, 0.15f, 0.15f, 1.0f))
                             ]
                         ]
                     ],
                     FText::GetEmpty()
                 );
                                
                Builder.AddMenuEntry(
                    FText::FromString("Pathfinding Tuner"),
                    FText::FromString("Open the Pathfinding Tuner window."),
                    FSlateIcon(PathfindingTunerStyleInternal::GetStyleSetName(), "PathfindingTuner.CustomIcon"),
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
            FOnSpawnTab::CreateRaw(this, &FSmartFlyingNavigationModule::SpawnPathfindingTunerTab))
            .SetDisplayName(FText::FromString("Pathfinding Tuner"))
        .SetIcon(FSlateIcon(PathfindingTunerStyleInternal::GetStyleSetName(), "PathfindingTuner.CustomIcon")) 
        .SetGroup(WorkspaceMenu::GetMenuStructure().GetDeveloperToolsMiscCategory());
    }
}

void FSmartFlyingNavigationModule::ShutdownModule()
{
    if (FModuleManager::Get().IsModuleLoaded("LevelEditor"))
    {
        FGlobalTabmanager::Get()->UnregisterTabSpawner(PathfindingTunerTabName);
    }
}

TSharedRef<SDockTab> FSmartFlyingNavigationModule::SpawnPathfindingTunerTab(const FSpawnTabArgs& SpawnTabArgs)
{
    return SNew(SDockTab)
        .TabRole(ETabRole::NomadTab)
        [
            SNew(SPathfindingTuner)
        ];
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FSmartFlyingNavigationModule, SmartFlyingNavigation)