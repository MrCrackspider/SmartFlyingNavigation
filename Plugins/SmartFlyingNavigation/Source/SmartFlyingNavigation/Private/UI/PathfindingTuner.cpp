// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "UI/PathfindingTuner.h"

// Engine
#include "Editor.h"
#include "Editor/EditorEngine.h"
#include "Engine/Selection.h"
#include "Internationalization/Text.h"
#include "Styling/AppStyle.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/MessageDialog.h"
#include "ScopedTransaction.h"

// Editor
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"
#include "PropertyCustomizationHelpers.h"

// Widgets
#include "Widgets/SBoxPanel.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SExpandableArea.h"
#include "Widgets/Text/STextBlock.h"

// Project
#include "NavData/VoxelObstacleProfile.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "Subsystem/PathfindingSubsystem.h"
#include "PathfindingComponent.h"
#include "UI/PathfindingTunerSettings.h"

#define LOCTEXT_NAMESPACE "PathfindingTuner"

//=================================================================================================
// SPathfindingTuner: Slate Widget Construction
//=================================================================================================

void SPathfindingTuner::Construct(const FArguments& InArgs)
{
    TunerSettings = GetMutableDefault<UPathfindingTunerSettings>();
    PathfindingSubsystem = GEngine->GetEngineSubsystem<UPathfindingSubsystem>();

    // Initialize ComboBox options
    MapSizeOptions.Add(MakeShared<int32>(128));
    MapSizeOptions.Add(MakeShared<int32>(256));
    MapSizeOptions.Add(MakeShared<int32>(512));

    // Set up a ticker to monitor actor selection changes in the editor
    if (GEditor)
    {
        TWeakPtr<SPathfindingTuner> WeakThisPtr = SharedThis(this);
        ActorSelectionTickerHandle = FTSTicker::GetCoreTicker().AddTicker(
            FTickerDelegate::CreateSP(this, &SPathfindingTuner::TickActorSelection)
        );

        // Perform an initial selection update
        OnActorSelectionChanged();
    }

    ChildSlot
    [
        // To make the content scrollable, we wrap everything in an SScrollBox.
        SNew(SScrollBox)
        + SScrollBox::Slot()
        [
            // The SVerticalBox now becomes a child of the SScrollBox.
            SNew(SVerticalBox)

            // Performance Settings Section
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(2)
            [
                BuildPerformanceSettingsSection()
            ]

            // Octree Volume Settings Section
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(2)
            [
                BuildOctreeVolumeSettingsSection()
            ]

            // Baked Data Settings Section
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(2)
            [
                BuildBakedDataSettingsSection()
            ]

            // Debug Section
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(2)
            [
                BuildVoxelDebugSettingsSection() 
            ]

            // Agent Settings Section
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(2)
            [
                BuildAgentSettingsSection()
            ]

            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(0, 5)
            [
                BuildAgentDebugSection() // 새로 추가된 섹션 호출
            ]
        ]
    ];
}

SPathfindingTuner::~SPathfindingTuner()
{
    // Clean up the ticker when the widget is destroyed
    if (ActorSelectionTickerHandle.IsValid())
    {
        FTSTicker::GetCoreTicker().RemoveTicker(ActorSelectionTickerHandle);
    }
}

//=================================================================================================
// UI Section Builders
//=================================================================================================

TSharedRef<SWidget> SPathfindingTuner::BuildPerformanceSettingsSection()
{
    return SNew(SExpandableArea)
        .HeaderContent()
        [
            SNew(STextBlock)
            .Text(LOCTEXT("PerformanceSettingsCategory", "Performance Settings"))
            .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
            .ShadowOffset(FVector2D(1.0f, 1.0f))
        ]
        .BodyContent()
        [
            SNew(SVerticalBox)
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                .ToolTipText(LOCTEXT("NumThreadsTooltip", "Sets the number of worker threads for pathfinding calculations.\nHigher values can process more requests simultaneously but will increase CPU usage."))
                
                + SHorizontalBox::Slot()
                .FillWidth(0.4f)
                .VAlign(VAlign_Center)
                .Padding(0, 0, 10, 0)
                [
                    SNew(STextBlock)
                    .Text(LOCTEXT("NumThreadsLabel", "Number of Worker Threads"))
                    .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                
                + SHorizontalBox::Slot()
                .FillWidth(0.6f)
                .VAlign(VAlign_Center)
                [
                    SNew(SNumericEntryBox<int32>)
                    .Value(this, &SPathfindingTuner::GetCurrentNumThreads)
                    .OnValueCommitted(this, &SPathfindingTuner::OnNumThreadsCommitted)
                    .MinValue(1)
                    .MaxValue(64)
                    .MinSliderValue(1)
                    .MaxSliderValue(16)
                ]
            ]
        ]
       .InitiallyCollapsed(!TunerSettings->bIsPerformanceCategoryExpanded)
        .OnAreaExpansionChanged(this, &SPathfindingTuner::OnPerformanceCategoryExpansionChanged);
}

TSharedRef<SWidget> SPathfindingTuner::BuildOctreeVolumeSettingsSection()
{
    return SNew(SExpandableArea)
        .IsEnabled(this, &SPathfindingTuner::AreOnlyOctreeVolumesSelected)
        .HeaderContent()
        [
            SNew(STextBlock)
            .Text(LOCTEXT("OctreeSettingsCategory", "Octree Volume Settings"))
            .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
            .ShadowOffset(FVector2D(1.0f, 1.0f))
        ]
        .BodyContent()
        [
            SNew(SVerticalBox)

            // Selected Volume Display
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("SelectedVolumeLabel", "Selected Volume(s):")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                    .ColorAndOpacity(FAppStyle::Get().GetWidgetStyle<FTextBlockStyle>("TextBlock.Warning").ColorAndOpacity)
                ]
                + SHorizontalBox::Slot().FillWidth(0.6f)
                .VAlign(VAlign_Center)
                .Padding(FMargin(5.0f, 0.0f, 0.0f, 0.0f)) // Left, Top, Right, Bottom
                [
                    SNew(STextBlock).Text(this, &SPathfindingTuner::GetSelectedVolumeDisplayText).Font(FAppStyle::GetFontStyle("PropertyWindow.BoldFont"))
                    .ColorAndOpacity(FAppStyle::Get().GetWidgetStyle<FTextBlockStyle>("TextBlock.Warning").ColorAndOpacity)
                ]
            ]
            // Max Octree Depth
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("MaxOctreeDepthLabel", "Max Octree Depth:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                + SHorizontalBox::Slot().FillWidth(0.6f)
                [
                    SNew(SNumericEntryBox<int32>)
                    .Value(this, &SPathfindingTuner::GetMaxOctreeDepth)
                    .OnValueCommitted(this, &SPathfindingTuner::OnMaxOctreeDepthCommitted)
                    .MinValue(0).MaxValue(10).MinSliderValue(0).MaxSliderValue(8)
                ]
            ]

            // Voxel Extent
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("VoxelExtentLabel", "Voxel Extent:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                + SHorizontalBox::Slot().FillWidth(0.6f)
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SPathfindingTuner::GetVoxelExtent)
                    .OnValueCommitted(this, &SPathfindingTuner::OnVoxelExtentCommitted)
                    .MinValue(1.0f).MaxValue(1000.0f).MinSliderValue(10.0f).MaxSliderValue(500.0f)
                ]
            ]

            // Map Size
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("MapSizeLabel", "Map Size:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                + SHorizontalBox::Slot().FillWidth(0.6f)
                [
                    SNew(SComboBox<TSharedPtr<int32>>)
                    .OptionsSource(&MapSizeOptions)
                    .OnSelectionChanged(this, &SPathfindingTuner::OnMapSizeSelectionChanged)
                    .OnGenerateWidget_Lambda([](TSharedPtr<int32> Item)
                    {
                        return SNew(STextBlock).Text(FText::AsNumber(*Item));
                    })
                    .Content()
                    [
                        SNew(STextBlock).Text(this, &SPathfindingTuner::GetSelectedMapSizeText)
                    ]
                ]
            ]
        ]
        .InitiallyCollapsed(!TunerSettings->bIsOctreeSettingsCategoryExpanded)
        .OnAreaExpansionChanged(this, &SPathfindingTuner::OnOctreeSettingsCategoryExpansionChanged);
}

TSharedRef<SWidget> SPathfindingTuner::BuildBakedDataSettingsSection()
{
    return SNew(SExpandableArea)
        .IsEnabled(this, &SPathfindingTuner::AreOnlyOctreeVolumesSelected)
        .HeaderContent()
        [
            SNew(STextBlock)
            .Text(LOCTEXT("BakedDataSettingsCategory", "Baked Data Settings"))
            .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
            .ShadowOffset(FVector2D(1.0f, 1.0f))
        ]
        .BodyContent()
        [
            
            // Top-level vertical box for all content in this section
            SNew(SVerticalBox)
            // Item1 Save Directory
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                  SNew(SHorizontalBox)
                  + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                  [
                      SNew(STextBlock).Text(LOCTEXT("BakedDataSaveDirectoryLabel", "Baked Data Save Directory:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                  ]
                  + SHorizontalBox::Slot().FillWidth(0.6f)
                  [
                      SNew(SHorizontalBox)
                      + SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
                      [
                          SNew(SEditableTextBox)
                          .Text(this, &SPathfindingTuner::GetBakedDataSaveDirectoryAsText)
                          .OnTextCommitted(this, &SPathfindingTuner::OnBakedDataSaveDirectoryTextCommitted)
                          .HintText(LOCTEXT("DirectoryPathHint", "Path to save directory..."))
                      ]
                      + SHorizontalBox::Slot().AutoWidth().Padding(4.0f, 0.0f, 0.0f, 0.0f).VAlign(VAlign_Center)
                      [
                          SNew(SButton)
                          .Text(FText::FromString(TEXT("...")))
                          .ToolTipText(LOCTEXT("BakedDataSaveDirectoryBrowseTooltip", "Choose a directory from this computer"))
                          .OnClicked(this, &SPathfindingTuner::OnBrowseButtonClicked)
                      ]
                  ]
            ]
            // --- Item 2: Baked Octree Data Asset ---
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                // Label
                + SHorizontalBox::Slot()
                .FillWidth(0.4f)
                .VAlign(VAlign_Center)
                [
                    SNew(STextBlock)
                    .Text(LOCTEXT("BakedOctreeDataLabel", "Baked Octree Data Asset:"))
                    .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                // Input widget + path display
                + SHorizontalBox::Slot()
                .FillWidth(0.6f)
                [
                    SNew(SVerticalBox)
                    // Object Property Entry Box
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    [
                        SNew(SObjectPropertyEntryBox)
                        .AllowedClass(UOctreeDataAsset::StaticClass())
                        .ObjectPath(this, &SPathfindingTuner::GetBakedOctreeDataPath)
                        .OnObjectChanged(this, &SPathfindingTuner::OnBakedOctreeDataChanged)
                    ]
                    // Path text
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(0, 4, 0, 0)
                    [
                        SNew(SHorizontalBox)
                        + SHorizontalBox::Slot()
                        .AutoWidth()
                        .VAlign(VAlign_Center)
                        .Padding(0, 0, 5, 0)
                        [
                            SNew(STextBlock)
                            .Text(LOCTEXT("PathLabel", "Path:"))
                            .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                            .ColorAndOpacity(FSlateColor(FLinearColor(0.7f, 0.7f, 0.7f)))
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth(1.0f)
                        .VAlign(VAlign_Center)
                        [
                            SNew(STextBlock)
                            .Text(this, &SPathfindingTuner::GetBakedDataFullPathAsText)
                            .Font(FAppStyle::GetFontStyle("PropertyWindow.ItalicFont"))
                            .ColorAndOpacity(FSlateColor(FLinearColor(0.5f, 0.5f, 0.5f)))
                        ]
                    ]
                ]
            ]

            // --- Item 2: Voxel Obstacle Profile (Defines WHAT to bake) ---
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                // Label
                + SHorizontalBox::Slot()
                .FillWidth(0.4f)
                .VAlign(VAlign_Center)
                [
                    SNew(STextBlock)
                    .Text(LOCTEXT("ObstacleProfileLabel", "Voxel Obstacle Profile:"))
                    .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                // Input widget + path display
                + SHorizontalBox::Slot()
                .FillWidth(0.6f)
                [
                    SNew(SVerticalBox)
                    // Object Property Entry Box
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    [
                        SNew(SObjectPropertyEntryBox)
                        .AllowedClass(UVoxelObstacleProfile::StaticClass()) // Changed to the new Profile class
                        .ObjectPath(this, &SPathfindingTuner::GetObstacleProfilePath) // Changed delegate
                        .OnObjectChanged(this, &SPathfindingTuner::OnObstacleProfileChanged) // Changed delegate
                        .ToolTipText(LOCTEXT("ObstacleProfileTooltip", "Specifies the Profile that defines which actors or geometry types are considered obstacles for baking."))
                    ]
                    // Path text
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(0, 4, 0, 0)
                    [
                        SNew(SHorizontalBox)
                        + SHorizontalBox::Slot()
                        .AutoWidth()
                        .VAlign(VAlign_Center)
                        .Padding(0, 0, 5, 0)
                        [
                            SNew(STextBlock)
                            .Text(LOCTEXT("PathLabel", "Path:"))
                            .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                            .ColorAndOpacity(FSlateColor(FLinearColor(0.7f, 0.7f, 0.7f)))
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth(1.0f)
                        .VAlign(VAlign_Center)
                        [
                            SNew(STextBlock)
                            .Text(this, &SPathfindingTuner::GetObstacleProfileFullPathAsText) // Changed delegate
                            .Font(FAppStyle::GetFontStyle("PropertyWindow.ItalicFont"))
                            .ColorAndOpacity(FSlateColor(FLinearColor(0.5f, 0.5f, 0.5f)))
                        ]
                    ]
                ]
            ]

            // --- Item 3: Bake Action (Description on Left, Button on Right) ---
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(8.0f, 15.0f, 8.0f, 8.0f) // Add vertical padding for separation
            [
                SNew(SHorizontalBox)

                // Left Slot: Description Text
                + SHorizontalBox::Slot()
                .FillWidth(0.4f)
                .VAlign(VAlign_Center)
                .Padding(0, 0, 8.0f, 0) // Padding between text and button
                [
                    SNew(STextBlock)
                    .Text(LOCTEXT("BakeDescription", "Generate data asset from selected volumes."))
                    .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                    .ColorAndOpacity(FSlateColor(FLinearColor(0.8f, 0.8f, 0.8f)))
                    .AutoWrapText(true)
                ]

                // Right Slot: Bake Button (Now fills the slot)
                + SHorizontalBox::Slot()
                .FillWidth(0.6f)
                .VAlign(VAlign_Center)
                .Padding(0.0f, 0.0f, 15.0f, 0.0f)
                [
                    SNew(SButton)
                    .ButtonStyle(FAppStyle::Get(), "PrimaryButton")
                    .TextStyle(FAppStyle::Get(), "PrimaryButtonText")
                    .Text(LOCTEXT("BakeButtonText", "Bake Octree Data"))
                    .ToolTipText(LOCTEXT("BakeButtonTooltip", "Bakes the static geometry within the selected Octree Volumes into a data asset. This can be a lengthy process."))
                    .IsEnabled(this, &SPathfindingTuner::AreOnlyOctreeVolumesSelected)
                    .OnClicked(this, &SPathfindingTuner::OnBakeButtonClicked)
                ]
            ]
        ]
        .InitiallyCollapsed(!TunerSettings->bIsBakedDataCategoryExpanded)
        .OnAreaExpansionChanged(this, &SPathfindingTuner::OnBakedDataCategoryExpansionChanged);
}
 
TSharedRef<SWidget> SPathfindingTuner::BuildAgentSettingsSection()
{
    return SNew(SExpandableArea)
        .IsEnabled(this, &SPathfindingTuner::AreOnlyAgentsSelected)
        .HeaderContent()
        [
            SNew(STextBlock)
            .Text(LOCTEXT("AgentSettingsCategory", "Agent Settings"))
            .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
            .ShadowOffset(FVector2D(1.0f, 1.0f))
        ]
        .BodyContent()
        [
            SNew(SVerticalBox)

            // Selected Agent Display
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("SelectedActorLabel", "Selected Agent(s):")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                    .ColorAndOpacity(FAppStyle::Get().GetWidgetStyle<FTextBlockStyle>("TextBlock.Warning").ColorAndOpacity)
                ]
                + SHorizontalBox::Slot()
                .FillWidth(0.6f).VAlign(VAlign_Center)
                .Padding(FMargin(5.0f, 0.0f, 0.0f, 0.0f)) // Left, Top, Right, Bottom
                [
                    SNew(STextBlock).Text(this, &SPathfindingTuner::GetSelectedAgentName).Font(FAppStyle::GetFontStyle("PropertyWindow.BoldFont"))
                    .ColorAndOpacity(FAppStyle::Get().GetWidgetStyle<FTextBlockStyle>("TextBlock.Warning").ColorAndOpacity)
                ]
            ]

            // Target Actor
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("TargetActorLabel", "Target Actor:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                + SHorizontalBox::Slot().FillWidth(0.6f)
                [
                    SNew(SObjectPropertyEntryBox)
                    .AllowedClass(AActor::StaticClass())
                    .ObjectPath(this, &SPathfindingTuner::GetTargetActorPath)
                    .OnObjectChanged(this, &SPathfindingTuner::OnTargetActorChanged)
                    .DisplayBrowse(true)
                ]
            ]

            // Target Volume
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("TargetVolumeLabel", "Target Volume:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                + SHorizontalBox::Slot().FillWidth(0.6f)
                [
                    SNew(SObjectPropertyEntryBox)
                    .AllowedClass(AOctreeVoxelVolume::StaticClass())
                    .ObjectPath(this, &SPathfindingTuner::GetTargetVolumePath)
                    .OnObjectChanged(this, &SPathfindingTuner::OnTargetVolumeChanged)
                    .DisplayBrowse(true)
                ]
            ]

            // Agent Radius
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("AgentRadiusLabel", "Agent Radius:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                + SHorizontalBox::Slot().FillWidth(0.6f)
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SPathfindingTuner::GetAgentRadius)
                    .OnValueCommitted(this, &SPathfindingTuner::OnAgentRadiusCommitted)
                    .MinValue(0.1f).MaxValue(1000.0f).MinSliderValue(10.0f).MaxSliderValue(200.0f)
                ]
            ]

            // Movement Speed
            + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
                [
                    SNew(STextBlock).Text(LOCTEXT("MovementSpeedLabel", "Movement Speed:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
                ]
                + SHorizontalBox::Slot().FillWidth(0.6f)
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SPathfindingTuner::GetMovementSpeed)
                    .OnValueCommitted(this, &SPathfindingTuner::OnMovementSpeedCommitted)
                    .MinValue(1.0f).MaxValue(1000.0f).MinSliderValue(100.0f).MaxSliderValue(500.0f)
                ]
            ]

            // Avoidance Radius
          + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
          [
              SNew(SHorizontalBox)
              + SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
              [
                  SNew(STextBlock).Text(LOCTEXT("Avoidance Radius Label", "Avoidance Radius:")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
              ]
              + SHorizontalBox::Slot().FillWidth(0.6f)
              [
                  SNew(SNumericEntryBox<float>)
                  .Value(this, &SPathfindingTuner::GetAvoidanceRadius)
                  .OnValueCommitted(this, &SPathfindingTuner::OnAvoidanceRadiusCommitted)
                  .MinValue(1.0f).MaxValue(1000.0f).MinSliderValue(100.0f).MaxSliderValue(500.0f)
              ]
          ]
        ]
        
        .InitiallyCollapsed(!TunerSettings->bIsAgentSettingsCategoryExpanded)
        .OnAreaExpansionChanged(this, &SPathfindingTuner::OnAgentSettingsCategoryExpansionChanged);
}


TSharedRef<SWidget> SPathfindingTuner::BuildVoxelDebugSettingsSection()
{
    // Use SExpandableArea as the root widget to match the design
    return SNew(SExpandableArea)
         .IsEnabled(this, &SPathfindingTuner::AreOnlyOctreeVolumesSelected) // Example: enable only if VoxelVolumes are selected
        .HeaderContent()
        [
            SNew(STextBlock)
            .Text(LOCTEXT("DebugSettingsCategory", "Debug Settings")) // Category title
            .Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle")) // Matching font style
            .ShadowOffset(FVector2D(1.0f, 1.0f)) // Matching shadow offset
        ]
        .BodyContent()
        [
            // Wrap the existing debug controls in a SVerticalBox for the body content
            SNew(SVerticalBox)

            // bDrawDebugBox property
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(8.0f, 5.0f) // Matching padding for individual properties
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot()
                .FillWidth(0.4f)
                .VAlign(VAlign_Center)
                [
                    SNew(STextBlock)
                    .Text(LOCTEXT("DrawDebugBox", "Draw Debug Box"))
                    .ToolTipText(LOCTEXT("DrawDebugBoxTooltip", "Whether to draw the main volume debug box."))
                    .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont")) // Matching font style for property labels
                ]
                + SHorizontalBox::Slot()
                .FillWidth(0.6f)
                [
                    SNew(SCheckBox)
                    .OnCheckStateChanged(this, &SPathfindingTuner::OnDrawDebugBoxChanged)
                    .IsChecked_Lambda([this]() -> ECheckBoxState {
                        TOptional<ECheckBoxState> State = this->GetDrawDebugBoxState();
                        if (State.IsSet())
                        {
                            return State.GetValue();
                        }
                        return ECheckBoxState::Undetermined;
                    })
                    .ToolTipText(LOCTEXT("DrawDebugBoxTooltip", "Whether to draw the main volume debug box."))
                ]
            ]

            // DebugLineWidth property - Now matches Voxel Extent's UI exactly (including slider)
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding(8.0f, 5.0f) // Matching padding for individual properties
            [
                SNew(SHorizontalBox)
                + SHorizontalBox::Slot()
                .FillWidth(0.4f)
                .VAlign(VAlign_Center)
                [
                    SNew(STextBlock)
                    .Text(LOCTEXT("DebugLineWidth", "Debug Line Width"))
                    .ToolTipText(LOCTEXT("DebugLineWidthTooltip", "The thickness of the debug lines."))
                    .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont")) // Matching font style for property labels
                ]
                + SHorizontalBox::Slot()
                .FillWidth(0.6f)
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SPathfindingTuner::GetDebugLineWidth)
                    .OnValueCommitted(this, &SPathfindingTuner::OnDebugLineWidthCommitted)
                    .MinValue(1.0f).MaxValue(1000.0f) // Matches Voxel Extent's min/max range for input
                    .MinSliderValue(10.0f).MaxSliderValue(500.0f) // Re-added to match Voxel Extent's slider behavior
                    // .AllowSpin(true) and .Delta(1.0f) are often defaults or can be explicitly added if desired for VoxelExtent too.
                    // For exact match, if VoxelExtent doesn't have them explicitly, we omit them too.
                    .UndeterminedString(LOCTEXT("Unselected", "Unselected"))
                ]
            ]
        ]
        // Control initial collapse state based on settings
        .InitiallyCollapsed(!TunerSettings->bIsDebugSettingsCategoryExpanded)
        // Bind expansion change to save state
        .OnAreaExpansionChanged(this, &SPathfindingTuner::OnDebugSettingsCategoryExpansionChanged);
}

TSharedRef<SWidget> SPathfindingTuner::BuildAgentDebugSection()
{
    return SNew(SExpandableArea)
       .AreaTitle(LOCTEXT("AgentDebugSettingsTitle", "Agent Debug Settings")) 
        .IsEnabled(this, &SPathfindingTuner::AreOnlyAgentsSelected)
       .BodyContent()
       [
           SNew(SVerticalBox)
           + SVerticalBox::Slot()
           .AutoHeight()
           .Padding(8.0f, 5.0f)
           [
               SNew(SHorizontalBox)
               + SHorizontalBox::Slot()
               .FillWidth(0.4f)
               .VAlign(VAlign_Center)
               [
                   SNew(STextBlock)
                   .Text(LOCTEXT("DrawDebugPathLabel", "Draw Debug Path"))
                   .Font(FAppStyle::Get().GetFontStyle("PropertyWindow.NormalFont"))
               ]
               + SHorizontalBox::Slot()
               .FillWidth(0.6f)
               .Padding(5.0f, 0.0f, 0.0f, 0.0f)
               [
                   SNew(SCheckBox)
                    
                   .IsChecked_Lambda([this]() -> ECheckBoxState {
                        TOptional<ECheckBoxState> State = this->GetDrawDebugPathState();
                        if (State.IsSet())
                        {
                            return State.GetValue();
                        }
                        return ECheckBoxState::Undetermined;
                    })
                   .OnCheckStateChanged(this, &SPathfindingTuner::OnDrawDebugPathChanged)
               ]
               ]

                + SVerticalBox::Slot()
                   .AutoHeight()
                   .Padding(8.0f, 5.0f) // Standard property row padding
                   [
                       SNew(SHorizontalBox)
                       + SHorizontalBox::Slot()
                       .FillWidth(0.4f) // Label takes 40% width
                       .VAlign(VAlign_Center)
                       [
                           SNew(STextBlock)
                           .Text(LOCTEXT("RequestPathLabel", "Request Path")) // Label for the button
                           .Font(FAppStyle::Get().GetFontStyle("PropertyWindow.NormalFont"))
                           .ToolTipText(LOCTEXT("RequestPathTooltip", "Requests a path for the selected agent(s) to their target actor."))
                       ]
                       + SHorizontalBox::Slot()
                       .FillWidth(0.6f) // Button takes 60% width
                       .Padding(5.0f, 0.0f, 0.0f, 0.0f) // Left padding for alignment
                       [
                           SNew(SButton)
                           .Text(LOCTEXT("RequestPathButtonText", "Request Path to Target")) // Button text
                           .ContentPadding(FMargin(4.0f, 2.0f)) // Standard button padding
                           .HAlign(HAlign_Center) // Center align text horizontally
                           .VAlign(VAlign_Center) // Center align text vertically
                           .OnClicked(this, &SPathfindingTuner::OnRequestPathButtonClicked) // Bind to new handler function
                           .ToolTipText(LOCTEXT("RequestPathButtonTooltip", "Click to request a path for the selected agent(s) to their target actor."))
                       ]
                   ]

                
            ]
        
         .InitiallyCollapsed(!TunerSettings->bAgentDebugSettingsExpanded)
      // Bind expansion change to save state
        .OnAreaExpansionChanged(this, &SPathfindingTuner::OnAgentDebugCategoryExpansionChanged);
}

//=================================================================================================
// Actor Selection Handling
//=================================================================================================
bool SPathfindingTuner::TickActorSelection(float DeltaTime)
{
    if (!GEditor) return false; // Stop ticking if the editor is gone

    TArray<UObject*> CurrentSelectedObjects;
    if (USelection* Sel = GEditor->GetSelectedActors())
    {
        Sel->GetSelectedObjects(CurrentSelectedObjects);
    }

    TSet<TWeakObjectPtr<AActor>> CurrentSelectedActorsSet;
    for (UObject* Obj : CurrentSelectedObjects)
    {
        if (AActor* Actor = Cast<AActor>(Obj))
        {
            // We only care about actors that are volumes or have a pathfinding component
            if (Actor->IsA<AOctreeVoxelVolume>() || Actor->FindComponentByClass<UPathfindingComponent>())
            {
                CurrentSelectedActorsSet.Add(Actor);
            }
        }
    }

    bool bSelectionChanged = false;
    const TSet<TWeakObjectPtr<AActor>> LastSelectedActorsSet(LastSelectedActors);

    if (CurrentSelectedActorsSet.Num() != LastSelectedActorsSet.Num())
    {
        bSelectionChanged = true;
    }
    else
    {
        for (const TWeakObjectPtr<AActor>& CurrentActor : CurrentSelectedActorsSet)
        {
            if (!LastSelectedActorsSet.Contains(CurrentActor))
            {
                bSelectionChanged = true;
                break;
            }
        }
    }

    if (bSelectionChanged)
    {
        LastSelectedActors = CurrentSelectedActorsSet.Array();
        OnActorSelectionChanged();
    }
    
    return true; // Continue ticking
}

void SPathfindingTuner::OnActorSelectionChanged()
{
    // This function is called when the selection changes.
    // It's responsible for updating the UI state.
    // The actual selection is already updated in LastSelectedActors by the ticker.
    // We just need to invalidate the widget to force a redraw and data refresh.
    Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
}




//=================================================================================================
// Performance Settings
//=================================================================================================

TOptional<int32> SPathfindingTuner::GetCurrentNumThreads() const
{
    return PathfindingSubsystem.IsValid() ? TOptional(PathfindingSubsystem->GetNumWorkerThreads()) : TOptional<int32>();
}

void SPathfindingTuner::OnNumThreadsCommitted(int32 NewValue, ETextCommit::Type CommitInfo)
{
    if (PathfindingSubsystem.IsValid())
    {
        PathfindingSubsystem->SetNumWorkerThreads(NewValue);
        PathfindingSubsystem->SaveConfig();
        Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
    }
}


//=================================================================================================
// Octree Volume Settings
//=================================================================================================

bool SPathfindingTuner::AreOnlyOctreeVolumesSelected() const
{
    if (LastSelectedActors.IsEmpty()) return false;
    
    for (const auto& ActorPtr : LastSelectedActors)
    {
        if (!ActorPtr.IsValid() || !ActorPtr->IsA<AOctreeVoxelVolume>())
        {
            return false;
        }
    }
    return true;
}

FText SPathfindingTuner::GetSelectedVolumeDisplayText() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    const int32 NumSelected = SelectedVolumes.Num();

    if (NumSelected == 0) return LOCTEXT("NoVolumeSelected", "None");
    if (NumSelected == 1) return FText::FromString(SelectedVolumes[0]->GetActorLabel());
    
    return FText::Format(LOCTEXT("MultipleVolumesSelected", "{0} Volumes Selected"), FText::AsNumber(NumSelected));
}

FText SPathfindingTuner::GetBakedDataSaveDirectoryAsText() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    TOptional<FString> CommonPath = GetCommonPropertyValue<AOctreeVoxelVolume, FString>(SelectedVolumes,
        [](const AOctreeVoxelVolume* V) { return V->BakedDataSaveDirectory.Path; });

    return CommonPath.IsSet() ? FText::FromString(CommonPath.GetValue()) : FText();
}
  
void SPathfindingTuner::OnBakedDataSaveDirectoryTextCommitted(const FText& InText, ETextCommit::Type CommitInfo)
{
    if (CommitInfo == ETextCommit::OnEnter || CommitInfo == ETextCommit::OnUserMovedFocus)
    {
        const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
        SetCommonPropertyValue<AOctreeVoxelVolume, FString>(SelectedVolumes, InText.ToString(), LOCTEXT("SetSaveDirectoryTransaction", "Set Baked Data Save Directory"),
            [](AOctreeVoxelVolume* V, const FString& Path) {
                V->BakedDataSaveDirectory.Path = Path;
                V->MarkPackageDirty();
            });
    }
}

/**
 * Gets the object path for the Voxel Obstacle Profile.
 * Used by SObjectPropertyEntryBox. Returns an empty string for multiple or null values.
 */
FString SPathfindingTuner::GetObstacleProfilePath() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    
    // Get the common ObstacleProfile from all selected volumes.
    TOptional<UVoxelObstacleProfile*> CommonProfile = GetCommonPropertyValue<AOctreeVoxelVolume, UVoxelObstacleProfile*>(SelectedVolumes,
        [](const AOctreeVoxelVolume* V) { return V->GetStaticObstacleProfile(); });

    // If a common, valid profile exists, return its path. Otherwise, return an empty string.
    return (CommonProfile.IsSet() && CommonProfile.GetValue()) ? CommonProfile.GetValue()->GetPathName() : FString();
}

/**
 * Called when the user changes the Voxel Obstacle Profile in the UI.
 * This sets the new profile on all selected volumes.
 */
void SPathfindingTuner::OnObstacleProfileChanged(const FAssetData& AssetData)
{
    UVoxelObstacleProfile* NewProfile = Cast<UVoxelObstacleProfile>(AssetData.GetAsset());
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();

    // Use the helper to set the property on all selected volumes within a single transaction.
    SetCommonPropertyValue<AOctreeVoxelVolume, UVoxelObstacleProfile*>(SelectedVolumes, NewProfile, LOCTEXT("SetObstacleProfileTransaction", "Set Voxel Obstacle Profile"),
        [](AOctreeVoxelVolume* V, UVoxelObstacleProfile* Profile) {
            V->SetStaticObstacleProfile(Profile);
            V->MarkPackageDirty();
        });
}

/**
 * Gets the display text for the directory path of the Voxel Obstacle Profile.
 * Correctly handles "Multiple Values" and "None" cases.
 */
FText SPathfindingTuner::GetObstacleProfileFullPathAsText() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    
    // Get the common ObstacleProfile from all selected volumes.
    TOptional<UVoxelObstacleProfile*> CommonProfile = GetCommonPropertyValue<AOctreeVoxelVolume, UVoxelObstacleProfile*>(SelectedVolumes,
        [](const AOctreeVoxelVolume* V) { return V->GetStaticObstacleProfile(); });


    // Case 0: Non Selected
    if (SelectedVolumes.Num() == 0)
    {
        return LOCTEXT("NonePath", "None");
    }
    
    // Case 1: The selected volumes have different profile values.
    if (!CommonProfile.IsSet())
    {
        return LOCTEXT("MultipleValues", "Multiple Values");
    }
    
    // Case 2: The common profile is null (all are set to None).
    if (!CommonProfile.GetValue())
    {
        return LOCTEXT("NonePath", "None");
    }

    // Case 3: A common, valid profile exists. Display its directory path.
    return FText::FromString(FPaths::GetPath(CommonProfile.GetValue()->GetPathName()));
}

FReply SPathfindingTuner::OnBrowseButtonClicked()
{
    IDesktopPlatform* DesktopPlatform = FDesktopPlatformModule::Get();
    if (DesktopPlatform)
    {
        const void* ParentWindowHandle = FSlateApplication::Get().GetActiveTopLevelWindow()->GetNativeWindow()->GetOSWindowHandle();
        FString SelectedDirectory;
        
        if (DesktopPlatform->OpenDirectoryDialog(ParentWindowHandle, TEXT("Select Save Directory"), FPaths::ProjectContentDir(), SelectedDirectory))
        {
            const FString FullContentDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir());
            if (!SelectedDirectory.StartsWith(FullContentDir))
            {
                FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("InvalidPathMsg", "Please select a directory inside the project's Content folder."));
                return FReply::Handled();
            }

            FString RelativePath = SelectedDirectory;
            FPaths::MakePathRelativeTo(RelativePath, *FullContentDir);
            
            OnBakedDataSaveDirectoryTextCommitted(FText::FromString(RelativePath), ETextCommit::OnEnter);
        }
    }
    return FReply::Handled();
}

TOptional<int32> SPathfindingTuner::GetMaxOctreeDepth() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    return GetCommonPropertyValue<AOctreeVoxelVolume, int32>(SelectedVolumes,
        [](const AOctreeVoxelVolume* V) { return V->GetMaxOctreeDepth(); });
}

void SPathfindingTuner::OnMaxOctreeDepthCommitted(int32 NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
        SetCommonPropertyValue<AOctreeVoxelVolume, int32>(SelectedVolumes, NewValue, LOCTEXT("SetMaxOctreeDepthTransaction", "Set Max Octree Depth"),
            [](AOctreeVoxelVolume* V, int32 Value) {
                V->SetMaxOctreeDepth(Value);
                V->MarkPackageDirty();
            });
    }
}

TOptional<float> SPathfindingTuner::GetVoxelExtent() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    return GetCommonPropertyValue<AOctreeVoxelVolume, float>(SelectedVolumes,
        [](const AOctreeVoxelVolume* V) { return V->VoxelExtent; });
}

void SPathfindingTuner::OnVoxelExtentCommitted(float NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
        SetCommonPropertyValue<AOctreeVoxelVolume, float>(SelectedVolumes, NewValue, LOCTEXT("SetVoxelSizeTransaction", "Set Voxel Size"),
            [](AOctreeVoxelVolume* V, float Value) {
                V->VoxelExtent = Value;
                V->MarkPackageDirty();
                V->RerunConstructionScripts();
            });
    }
}

FText SPathfindingTuner::GetSelectedMapSizeText() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    if (SelectedVolumes.IsEmpty()) return LOCTEXT("NoVolumeSelected", "---");

    TOptional<FIntVector> CommonMapSize = GetCommonPropertyValue<AOctreeVoxelVolume, FIntVector>(SelectedVolumes,
        [](const AOctreeVoxelVolume* V) { return V->MapSize; });

    if (!CommonMapSize.IsSet()) return LOCTEXT("MultipleMapSizes", "Multiple Values");

    const FIntVector& Size = CommonMapSize.GetValue();
    if (Size.X == Size.Y && Size.Y == Size.Z) return FText::AsNumber(Size.X);
    
    return LOCTEXT("CustomMapSize", "Custom Size");
}

void SPathfindingTuner::OnMapSizeSelectionChanged(TSharedPtr<int32> NewSelection, ESelectInfo::Type SelectType)
{
    if ((SelectType == ESelectInfo::OnMouseClick || SelectType == ESelectInfo::OnKeyPress) && NewSelection.IsValid())
    {
        const int32 NewSize = *NewSelection;
        const FIntVector NewMapSize(NewSize, NewSize, NewSize);
        const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();

        SetCommonPropertyValue<AOctreeVoxelVolume, FIntVector>(SelectedVolumes, NewMapSize, LOCTEXT("SetMapSizeTransaction", "Set Map Size"),
            [](AOctreeVoxelVolume* V, const FIntVector& Size) {
                V->MapSize = Size;
                V->RerunConstructionScripts();
            });
    }
}


//=================================================================================================
// Baked Data Settings
//=================================================================================================

FString SPathfindingTuner::GetBakedOctreeDataPath() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    TOptional<UOctreeDataAsset*> CommonAsset = GetCommonPropertyValue<AOctreeVoxelVolume, UOctreeDataAsset*>(SelectedVolumes,
        [](const AOctreeVoxelVolume* V) { return V->GetBakedOctreeData(); });

    return (CommonAsset.IsSet() && CommonAsset.GetValue()) ? CommonAsset.GetValue()->GetPathName() : FString();
}

void SPathfindingTuner::OnBakedOctreeDataChanged(const FAssetData& AssetData)
{
    UOctreeDataAsset* NewDataAsset = Cast<UOctreeDataAsset>(AssetData.GetAsset());
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();

    SetCommonPropertyValue<AOctreeVoxelVolume, UOctreeDataAsset*>(SelectedVolumes, NewDataAsset, LOCTEXT("SetBakedDataTransaction", "Set Baked Octree Data"),
        [](AOctreeVoxelVolume* V, UOctreeDataAsset* Asset) {
            V->SetBakedOctreeData(Asset);
            V->MarkPackageDirty();
        });
}

FText SPathfindingTuner::GetBakedDataFullPathAsText() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    TOptional<UOctreeDataAsset*> CommonAsset = GetCommonPropertyValue<AOctreeVoxelVolume, UOctreeDataAsset*>
    (SelectedVolumes,[](const AOctreeVoxelVolume* V) { return V->GetBakedOctreeData(); });

    // Case 0: Non Selected
    if (SelectedVolumes.Num() == 0)
    {
        return LOCTEXT("NonePath", "None");
    }
    
    if (!CommonAsset.IsSet())
    {
        return LOCTEXT("MultipleValues", "Multiple Values");
    }
    
    if (!CommonAsset.GetValue())
    {
        return LOCTEXT("NonePath", "None");
    }
    
    return FText::FromString(FPaths::GetPath(CommonAsset.GetValue()->GetPathName()));
}

//=================================================================================================
// Debug Setting
//=================================================================================================

TOptional<ECheckBoxState> SPathfindingTuner::GetDrawDebugBoxState() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AVoxelVolumeBase>();
    TOptional<bool> CommonValue = GetCommonPropertyValue<AVoxelVolumeBase, bool>(SelectedVolumes,
        [](const AVoxelVolumeBase* V) { return V->GetbDrawDebugBox(); });

    if (CommonValue.IsSet())
    {
        return CommonValue.GetValue() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
    }
    return ECheckBoxState::Undetermined; // Values differ or no actors selected
}

void SPathfindingTuner::OnDrawDebugBoxChanged(ECheckBoxState NewState)
{
    bool bNewValue = (NewState == ECheckBoxState::Checked);
    
    // Only commit if it's a definite checked/unchecked state
    if (NewState != ECheckBoxState::Undetermined)
    {
        const auto SelectedVolumes = GetSelectedActorsOfType<AVoxelVolumeBase>();
        SetCommonPropertyValue<AVoxelVolumeBase, bool>(SelectedVolumes, bNewValue, LOCTEXT("SetDrawDebugBoxTransaction", "Set Draw Debug Box"),
            [](AVoxelVolumeBase* V, bool Value) {
                V->SetbDrawDebugBox(Value);
                // Important: Call OnConstruction to re-trigger debug drawing update
                V->OnConstruction(V->GetActorTransform());
                V->MarkPackageDirty();
            });
    }
}

TOptional<float> SPathfindingTuner::GetDebugLineWidth() const
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AVoxelVolumeBase>();
    return GetCommonPropertyValue<AVoxelVolumeBase, float>(SelectedVolumes,
        [](const AVoxelVolumeBase* V) { return V->GetDebugLineWidth(); });
}

void SPathfindingTuner::OnDebugLineWidthCommitted(float NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        const auto SelectedVolumes = GetSelectedActorsOfType<AVoxelVolumeBase>();
        SetCommonPropertyValue<AVoxelVolumeBase, float>(SelectedVolumes, NewValue, LOCTEXT("SetDebugLineWidthTransaction", "Set Debug Line Width"),
            [](AVoxelVolumeBase* V, float Value) {
                V->SetDebugLineWidth(Value);
                // Important: Call OnConstruction to re-trigger debug drawing update
                V->OnConstruction(V->GetActorTransform()); 
                V->MarkPackageDirty();
            });
    }
}

TOptional<ECheckBoxState> SPathfindingTuner::GetDrawDebugPathState() const
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    if (SelectedComponents.Num() == 0)
    {
        return ECheckBoxState::Undetermined;
    }

    bool bFirstValue = SelectedComponents[0]->GetbDrawDebugPath(); // UPathfindingComponent의 Getter 사용
    for (int32 i = 1; i < SelectedComponents.Num(); ++i)
    {
        if (SelectedComponents[i]->GetbDrawDebugPath() != bFirstValue) // UPathfindingComponent의 Getter 사용
        {
            return ECheckBoxState::Undetermined; // Mixed state
        }
    }
    return bFirstValue ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
}

void SPathfindingTuner::OnDrawDebugPathChanged(ECheckBoxState NewState)
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    SetCommonPropertyValue<UPathfindingComponent, bool>(SelectedComponents, NewState == ECheckBoxState::Checked, LOCTEXT("SetDrawDebugPath", "Set Draw Debug Path"),
        [](UPathfindingComponent* Component, bool bDraw) {
            Component->SetbDrawDebugPath(bDraw); // UPathfindingComponent의 Setter 사용
        });
}
//=================================================================================================
// Agent Settings
//=================================================================================================

bool SPathfindingTuner::AreOnlyAgentsSelected() const
{
    if (LastSelectedActors.IsEmpty()) return false;

    for (const auto& ActorPtr : LastSelectedActors)
    {
        if (!ActorPtr.IsValid() || !ActorPtr->FindComponentByClass<UPathfindingComponent>())
        {
            return false;
        }
    }
    return true;
}
 
FText SPathfindingTuner::GetSelectedAgentName() const
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();

    const int32 NumSelected = SelectedComponents.Num();

    if (NumSelected == 0)
    {
        return LOCTEXT("NoAgentSelected", "No Agent Selected");
    }
    if (NumSelected == 1)
    {
        return FText::FromString(SelectedComponents[0]->GetOwner()->GetActorLabel());
    }
    
    return FText::Format(LOCTEXT("MultipleAgentsSelected", "{0} Agents Selected"), FText::AsNumber(NumSelected));
}
FString SPathfindingTuner::GetTargetActorPath() const
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    TOptional<AActor*> CommonTarget = GetCommonPropertyValue<UPathfindingComponent, AActor*>(SelectedComponents,
        [](const UPathfindingComponent* C) { return C->TargetActor; });

    return (CommonTarget.IsSet() && CommonTarget.GetValue()) ? CommonTarget.GetValue()->GetPathName() : FString();
}

void SPathfindingTuner::OnTargetActorChanged(const FAssetData& AssetData)
{
    AActor* NewTargetActor = Cast<AActor>(AssetData.GetAsset());
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
 
    SetCommonPropertyValue<UPathfindingComponent, AActor*>(SelectedComponents, NewTargetActor, LOCTEXT("SetTargetActor", "Set Target Actor"),
        [](UPathfindingComponent* C, AActor* Target) {
            C->SetTargetActor(Target);
        });
}

FString SPathfindingTuner::GetTargetVolumePath() const
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    TOptional<AVoxelVolumeBase*> CommonVolume = GetCommonPropertyValue<UPathfindingComponent, AVoxelVolumeBase*>(SelectedComponents,
        [](const UPathfindingComponent* C) { return C->TargetVolume; });

    return (CommonVolume.IsSet() && CommonVolume.GetValue()) ? CommonVolume.GetValue()->GetPathName() : FString();
}

void SPathfindingTuner::OnTargetVolumeChanged(const FAssetData& AssetData)
{
    AOctreeVoxelVolume* NewTargetVolume = Cast<AOctreeVoxelVolume>(AssetData.GetAsset());
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();

    SetCommonPropertyValue<UPathfindingComponent, AVoxelVolumeBase*>(SelectedComponents, NewTargetVolume, LOCTEXT("SetAgentTargetVolume", "Set Agent Target Volume"),
        [](UPathfindingComponent* C, AVoxelVolumeBase* Volume) {
            C->TargetVolume = Volume;
        });
}

TOptional<float> SPathfindingTuner::GetAgentRadius() const
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    return GetCommonPropertyValue<UPathfindingComponent, float>(SelectedComponents,
        [](const UPathfindingComponent* C) { return C->BaseAgentRadius; });
}

void SPathfindingTuner::OnAgentRadiusCommitted(float NewValue, ETextCommit::Type CommitType)
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    SetCommonPropertyValue<UPathfindingComponent, float>(SelectedComponents, NewValue, LOCTEXT("SetAgentRadius", "Set Agent Radius"),
        [](UPathfindingComponent* C, float Radius) {
            C->BaseAgentRadius = Radius;
        });
}

TOptional<float> SPathfindingTuner::GetMovementSpeed() const
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    return GetCommonPropertyValue<UPathfindingComponent, float>(SelectedComponents,
        [](const UPathfindingComponent* C) { return C->MovementSpeed; });
}

void SPathfindingTuner::OnMovementSpeedCommitted(float NewValue, ETextCommit::Type CommitType)
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    SetCommonPropertyValue<UPathfindingComponent, float>(SelectedComponents, NewValue, LOCTEXT("SetMovementSpeed", "Set Movement Speed"),
        [](UPathfindingComponent* C, float Speed) {
            C->MovementSpeed = Speed;
        });
}

TOptional<float> SPathfindingTuner::GetAvoidanceRadius() const
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    return GetCommonPropertyValue<UPathfindingComponent, float>(SelectedComponents,
        [](const UPathfindingComponent* C) { return C->GetAvoidanceRadius(); });
}

void SPathfindingTuner::OnAvoidanceRadiusCommitted(float NewValue,
    ETextCommit::Type CommitType)
{
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    SetCommonPropertyValue<UPathfindingComponent, float>(SelectedComponents, NewValue, LOCTEXT("SetMovementSpeed", "Set Movement Speed"),
        [](UPathfindingComponent* C, float NewRadius) {
            C->SetAvoidanceRadius(NewRadius);
        });
}


//=================================================================================================
// Actions and Events
//=================================================================================================

FReply SPathfindingTuner::OnBakeButtonClicked()
{
    const auto SelectedVolumes = GetSelectedActorsOfType<AOctreeVoxelVolume>();
    if (SelectedVolumes.IsEmpty())
    {
        ShowWarningMessage(LOCTEXT("NoOctreeSelectedForBake", "No Octree Volume selected to bake."));
        return FReply::Handled();
    }

    for (AOctreeVoxelVolume* Volume : SelectedVolumes)
    {
        if (Volume->BakedDataSaveDirectory.Path.IsEmpty())
        {
            FText ErrorMsg = FText::Format(LOCTEXT("BakePathNotSetForOne", "Cannot start baking. 'Baked Data Save Directory' is not set for {0}."), FText::FromString(Volume->GetActorLabel()));
            ShowWarningMessage(ErrorMsg);
            return FReply::Handled();
        }
    }

    if (SelectedVolumes.Num() > 1)
    {
        FText ConfirmationMessage = FText::Format(
            LOCTEXT("MultiBakeConfirmation", "You have selected {0} volumes to bake.\nThis process could take a long time and may cause the editor to become unresponsive.\n\nDo you want to continue?"),
            FText::AsNumber(SelectedVolumes.Num())
        );

        if (FMessageDialog::Open(EAppMsgType::YesNo, ConfirmationMessage) != EAppReturnType::Yes)
        {
            return FReply::Handled();
        }
    }

    const FScopedTransaction Transaction(LOCTEXT("BakeOctreeDataTransaction", "Bake Octree Data"));
    for (AOctreeVoxelVolume* Volume : SelectedVolumes)
    {
        if (Volume)
        {
            Volume->Modify();
            Volume->BakeStaticData();
        }
    }

    return FReply::Handled();
}

void SPathfindingTuner::ShowWarningMessage(const FText& Message) const
{
    FMessageDialog::Open(EAppMsgType::Ok, Message);
}

FReply SPathfindingTuner::OnRequestPathButtonClicked()
{
    if (!IsPIEActive())
    {
        ShowWarningMessage(LOCTEXT("NotPIEWarning", "Path requests can only be made during Play In Editor (PIE) or Standalone Game."));
        return FReply::Handled();
    }
    
    const auto SelectedComponents = GetSelectedComponentsOfType<UPathfindingComponent>();
    if (SelectedComponents.Num() == 0)
    {
        ShowWarningMessage(LOCTEXT("NoAgentSelected", "Please select at least one agent with a Pathfinding Component."));
        return FReply::Handled();
    }

    bool bAnyPathRequested = false;
    for (UPathfindingComponent* Component : SelectedComponents)
    {
        if (Component && Component->RequestPathToTargetActor()) // UPathfindingComponent의 함수 호출
        {
            bAnyPathRequested = true;
        }
    }

    if (!bAnyPathRequested)
    {
        ShowWarningMessage(LOCTEXT("PathRequestFailed", "Path request failed for all selected agents. Check logs for details (e.g., invalid target actor)."));
    }
    else
    {
        // Optionally, provide feedback that path request was sent
        // GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Green, TEXT("Path request sent!"));
    }

    return FReply::Handled();
}

//=================================================================================================
// UI State & Expansion Handlers
//=================================================================================================

void SPathfindingTuner::OnPerformanceCategoryExpansionChanged(bool bIsExpanded)
{
    if (TunerSettings)
    {
        TunerSettings->bIsPerformanceCategoryExpanded = bIsExpanded;
        TunerSettings->SaveConfig();
    }
}

void SPathfindingTuner::OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded)
{
    if (TunerSettings)
    {
        TunerSettings->bIsOctreeSettingsCategoryExpanded = bIsExpanded;
        TunerSettings->SaveConfig();
    }
}

void SPathfindingTuner::OnBakedDataCategoryExpansionChanged(bool bIsExpanded)
{
    if (TunerSettings)
    {
        TunerSettings->bIsBakedDataCategoryExpanded = bIsExpanded;
        TunerSettings->SaveConfig();
    }
}

void SPathfindingTuner::OnAgentSettingsCategoryExpansionChanged(bool bIsExpanded)
{
    if (TunerSettings)
    {
        TunerSettings->bIsAgentSettingsCategoryExpanded = bIsExpanded;
        TunerSettings->SaveConfig();
    }
}

void SPathfindingTuner::OnDebugSettingsCategoryExpansionChanged(bool bIsExpanded)
{
    if (TunerSettings)
    {
        TunerSettings->bIsDebugSettingsCategoryExpanded = bIsExpanded;
        TunerSettings->SaveConfig(); // Save the new state to config
    }
}

void SPathfindingTuner::OnAgentDebugCategoryExpansionChanged(bool bIsExpanded)
{
    if (TunerSettings)
    {
        TunerSettings->bAgentDebugSettingsExpanded = bIsExpanded;
        TunerSettings->SaveConfig(); // Save the new state to config
    }
}

bool SPathfindingTuner::IsPIEActive() const
{
    // GEditor가 유효하고, 현재 게임이 실행 중인지 확인합니다.
    // GEditor->PlayWorld는 PIE 월드를 나타냅니다.
    return GEditor && GEditor->PlayWorld && GEditor->PlayWorld->IsGameWorld();
}

#undef LOCTEXT_NAMESPACE
