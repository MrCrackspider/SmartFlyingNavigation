// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "SPathfindingTuner.h"

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
#include "Widgets/Notifications/SNotificationList.h"
#include "Framework/Notifications/NotificationManager.h"

// Widgets
#include "Widgets/SBoxPanel.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SExpandableArea.h"
#include "Widgets/Text/STextBlock.h"
#include "PropertyEditorModule.h"

// Project
#include "NavData/VoxelObstacleProfile.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "Subsystem/PathfindingSubsystem.h"
#include "Subsystem/AvoidanceSubsystem.h"
#include "Component/PathfindingComponent.h"
#include "SPathfindingTunerSettings.h"
#include "Containers/UnrealString.h"

#include "PathfindingEditorSubsystem.h"


#define LOCTEXT_NAMESPACE "PathfindingTuner"

//=================================================================================================
// SPathfindingTuner: Slate Widget Construction
//=================================================================================================


void SPathfindingTuner::Construct(const FArguments& InArgs)
{
	FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	FDetailsViewArgs DetailsViewArgs;
	DetailsViewArgs.bHideSelectionTip = true;
	DetailsViewArgs.bLockable = false;

	AgentDetailsView = PropertyEditorModule.CreateDetailView(DetailsViewArgs);
	MapSizeOptions.Add(MakeShared<int32>(128));
	MapSizeOptions.Add(MakeShared<int32>(256));
	
	ChildSlot
	[
		SNew(SScrollBox)
		+ SScrollBox::Slot()
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(2)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::GetBrush("ToolPanel.GroupBorder"))
				.Padding(4.f)
				[
					BuildPerformanceSettingsSection()
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::GetBrush("ToolPanel.GroupBorder"))
				.Padding(4.f)
				[
					BuildAvoidanceSettingsSection()
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::GetBrush("ToolPanel.GroupBorder"))
				.Padding(4.f)
				[
					BuildOctreeVolumeSettingsSection()
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::GetBrush("ToolPanel.GroupBorder"))
				.Padding(4.f)
				[
					BuildVoxelDebugSettingsSection()
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::GetBrush("ToolPanel.GroupBorder"))
				.Padding(4.f)
				[
					BuildBakedDataSettingsSection()
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::GetBrush("ToolPanel.GroupBorder"))
				.Padding(4.f)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(2.0f)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("SelectedAgentDetails", "Selected Agent Details (Pathfinding Component)"))
						.Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
					]
					+ SVerticalBox::Slot().AutoHeight()
					[
						AgentDetailsView.ToSharedRef()
					]
				]
			]
		]
	];

	if (GEditor)
	{
		ActorSelectionTickerHandle = FTSTicker::GetCoreTicker().AddTicker(
			FTickerDelegate::CreateSP(this, &SPathfindingTuner::TickActorSelection)
		);
		OnActorSelectionChanged();
	}
}

SPathfindingTuner::~SPathfindingTuner()
{
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
	const UPathfindingTunerSettings* TunerSettings = GetDefault<UPathfindingTunerSettings>();
	const UAvoidanceSubsystem* AvoidanceCDO = GetDefault<UAvoidanceSubsystem>();

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
				.ToolTipText(LOCTEXT("NumThreadsTooltip",
				                     "Sets the number of worker threads for pathfinding calculations.\nHigher values can process more requests simultaneously but will increase CPU usage."))

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
	const UPathfindingTunerSettings* TunerSettings = GetDefault<UPathfindingTunerSettings>();

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
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("SelectedVolumeLabel", "Selected Volume(s):"))
					.Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
					.ColorAndOpacity(FAppStyle::Get().GetWidgetStyle<FTextBlockStyle>("NormalText").ColorAndOpacity)
				]
				+ SHorizontalBox::Slot()
				.FillWidth(0.6f)
				.VAlign(VAlign_Center)
				.Padding(FMargin(5.0f, 0.0f, 0.0f, 0.0f))
				[
					SNew(STextBlock)
					.Text(this, &SPathfindingTuner::GetSelectedVolumeDisplayText)
					.Font(FAppStyle::GetFontStyle("PropertyWindow.BoldFont"))
					.ColorAndOpacity(FAppStyle::Get().GetWidgetStyle<FTextBlockStyle>("NormalText").ColorAndOpacity)
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
				[
					SNew(STextBlock).Text(LOCTEXT("VoxelExtentLabel", "Voxel Extent:")).Font(
						FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f)
				[
					SNew(SNumericEntryBox<float>)
					.Value(this, &SPathfindingTuner::GetVoxelExtent)
					.OnValueCommitted(this, &SPathfindingTuner::OnVoxelExtentCommitted)
					.MinValue(1.0f)
					.MaxValue(1000.0f)
					.MinSliderValue(10.0f)
					.MaxSliderValue(500.0f)
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
				[
					SNew(STextBlock).Text(LOCTEXT("MapSizeLabel", "Map Size:")).Font(
						FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
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
	const UPathfindingTunerSettings* TunerSettings = GetDefault<UPathfindingTunerSettings>();

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
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center)
				[
					SNew(STextBlock).Text(LOCTEXT("BakedDataSaveDirectoryLabel", "Baked Data Save Directory:")).Font(
						FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
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
						.ToolTipText(LOCTEXT("BakedDataSaveDirectoryBrowseTooltip",
						                     "Choose a directory from this computer"))
						.OnClicked(this, &SPathfindingTuner::OnBrowseButtonClicked)
					]
				]
			]
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
					.Text(LOCTEXT("BakedOctreeDataLabel", "Baked Octree Data Asset:"))
					.Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot()
				.FillWidth(0.6f)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot()
					.AutoHeight()
					[
						SNew(SObjectPropertyEntryBox)
						.AllowedClass(UOctreeDataAsset::StaticClass())
						.ObjectPath(this, &SPathfindingTuner::GetBakedOctreeDataPath)
						.OnObjectChanged(this, &SPathfindingTuner::OnBakedOctreeDataChanged)
					]
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
					.Text(LOCTEXT("ObstacleProfileLabel", "Voxel Obstacle Profile:"))
					.Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot()
				.FillWidth(0.6f)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot()
					.AutoHeight()
					[
						SNew(SObjectPropertyEntryBox)
						                             .AllowedClass(UVoxelObstacleProfile::StaticClass())
						                             .ObjectPath(this, &SPathfindingTuner::GetObstacleProfilePath)
						                             .OnObjectChanged(
							                             this, &SPathfindingTuner::OnObstacleProfileChanged)
						                             .ToolTipText(LOCTEXT(
							                             "ObstacleProfileTooltip",
							                             "Specifies the Profile that defines which actors or geometry types are considered obstacles for baking."))
					]
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
							                .Text(this, &SPathfindingTuner::GetObstacleProfileFullPathAsText)
							                .Font(FAppStyle::GetFontStyle("PropertyWindow.ItalicFont"))
							                .ColorAndOpacity(FSlateColor(FLinearColor(0.5f, 0.5f, 0.5f)))
						]
					]
				]
			]
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(8.0f, 15.0f, 8.0f, 8.0f)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.FillWidth(0.4f)
				.VAlign(VAlign_Center)
				.Padding(0, 0, 8.0f, 0)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("BakeDescription", "Generate data asset from selected volumes."))
					.Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
					.ColorAndOpacity(FSlateColor(FLinearColor(0.8f, 0.8f, 0.8f)))
					.AutoWrapText(true)
				]
				+ SHorizontalBox::Slot()
				.FillWidth(0.6f)
				.VAlign(VAlign_Center)
				.Padding(0.0f, 0.0f, 15.0f, 0.0f)
				[
					SNew(SButton)
					.ButtonStyle(FAppStyle::Get(), "PrimaryButton")
					.TextStyle(FAppStyle::Get(), "PrimaryButtonText")
					.Text(LOCTEXT("BakeButtonText", "Bake Octree Data"))
					.ToolTipText(LOCTEXT("BakeButtonTooltip",
					                     "Bakes the static geometry within the selected Octree Volumes into a data asset. This can be a lengthy process."))
					.IsEnabled(this, &SPathfindingTuner::AreOnlyOctreeVolumesSelected)
					.OnClicked(this, &SPathfindingTuner::OnBakeButtonClicked)
				]
			]
		]
		.InitiallyCollapsed(!TunerSettings->bIsBakedDataCategoryExpanded)
		.OnAreaExpansionChanged(this, &SPathfindingTuner::OnBakedDataCategoryExpansionChanged);
}

TSharedRef<SWidget> SPathfindingTuner::BuildAvoidanceSettingsSection()
{
	const UPathfindingTunerSettings* TunerSettings = GetDefault<UPathfindingTunerSettings>();
	const UAvoidanceSubsystem* AvoidanceCDO = GetDefault<UAvoidanceSubsystem>();

	return SNew(SExpandableArea)
		.HeaderContent()
		[
			SNew(STextBlock)
			.Text(LOCTEXT("AvoidanceSettingsCategory", "Avoidance Tuning"))
			.Font(FAppStyle::Get().GetFontStyle("DetailsView.CategoryFontStyle"))
			.ShadowOffset(FVector2D(1.0f, 1.0f))
		]
		.BodyContent()
		[
			SNew(SVerticalBox)

			// Informational Note
			+ SVerticalBox::Slot()
			.AutoHeight()
			.Padding(4.0f, 4.0f, 4.0f, 12.0f)
			[
				SNew(SBorder)
				.BorderImage(FAppStyle::GetBrush("ToolPanel.GroupBorder"))
				.Padding(FMargin(8.0f))
				[
					SNew(SHorizontalBox)

					// Info Icon
					+ SHorizontalBox::Slot()
					.AutoWidth()
					.VAlign(VAlign_Center)
					.Padding(0.0f, 0.0f, 6.0f, 0.0f)
					[
						SNew(SImage)
						.Image(FAppStyle::GetBrush("Icons.Info"))
					]

					// Description Text
					+ SHorizontalBox::Slot()
					.FillWidth(1.0f)
					.VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("AvoidanceWarningText", "Note: Incorrectly tuned parameters can result in poor or non-functional avoidance behavior."))
						.Font(FAppStyle::GetFontStyle("PropertyWindow.ItalicFont"))
						.ColorAndOpacity(FSlateColor(FLinearColor(0.8f, 0.8f, 0.8f)))
						.AutoWrapText(true)
					]
				]
			]

			// Use Agent Avoidance
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("UseAvoidanceTooltip", "Enables or disables agent avoidance behavior globally."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("UseAvoidanceLabel", "Use Agent Avoidance")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SCheckBox)
					.IsChecked_Lambda([AvoidanceCDO]() { return AvoidanceCDO ? (AvoidanceCDO->GetbUseAvoidance() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked) : ECheckBoxState::Undetermined; })
					.OnCheckStateChanged(this, &SPathfindingTuner::OnbAvoidanceComitted)
				]
			]

			// Show Velocity Debug Line
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("ShowAvoidanceVelocityDebugTooltip", "If enabled, shows a debug line representing the agent's avoidance velocity."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("ShowAvoidanceVelocityDebugLabel", "Show Velocity Debug Line")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SCheckBox)
					.IsChecked_Lambda([AvoidanceCDO]() { return AvoidanceCDO ? (AvoidanceCDO->GetbShowVelocityDebugLine() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked) : ECheckBoxState::Undetermined; })
					.OnCheckStateChanged(this, &SPathfindingTuner::OnbAvoidanceVelocityDebugComitted)
				]
			]
			
			// TimeHorizon
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("TimeHorizonTooltip", "How far in the future to predict collisions (in seconds)."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("TimeHorizonLabel", "Time Horizon")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SNumericEntryBox<float>)
					.Value_Lambda([AvoidanceCDO]() -> TOptional<float> { return AvoidanceCDO ? TOptional(AvoidanceCDO->GetTimeHorizon()) : TOptional<float>(); })
					.OnValueCommitted(this, &SPathfindingTuner::OnTimeHorizonCommitted)
				]
			]

			// MaxNeighbors
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("MaxNeighborsTooltip", "Maximum number of other agents to consider for avoidance.\nHigher values can improve avoidance quality but may cause performance degradation."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("MaxNeighborsLabel", "Max Neighbors")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SNumericEntryBox<int32>)
					.Value_Lambda([AvoidanceCDO]() -> TOptional<int32> { return AvoidanceCDO ? TOptional(AvoidanceCDO->GetMaxNeighbors()) : TOptional<int32>(); })
					.OnValueCommitted(this, &SPathfindingTuner::OnMaxNeighborsCommitted)
					.MinValue(0).MaxValue(32)
				]
			]

			// GridCellSize
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("GridCellSizeTooltip", "Size of the grid cells used for broad-phase neighbor search.\nIt is highly recommended to match this value with the 'Voxel Extent' for optimal performance."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("GridCellSizeLabel", "Grid Cell Size")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SNumericEntryBox<float>)
					.Value_Lambda([AvoidanceCDO]() -> TOptional<float> { return AvoidanceCDO ? TOptional(AvoidanceCDO->GetGridCellSize()) : TOptional<float>(); })
					.OnValueCommitted(this, &SPathfindingTuner::OnGridCellSizeCommitted)
				]
			]

			// SymmetryBreakingRangeMultiplier
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("SymmetryBreakingTooltip", "Multiplier to break symmetry when agents are head-on."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("SymmetryBreakingLabel", "Symmetry Breaking Multiplier")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SNumericEntryBox<float>)
					.Value_Lambda([AvoidanceCDO]() -> TOptional<float> { return AvoidanceCDO ? TOptional(AvoidanceCDO->GetSymmetryBreakingRangeMultiplier()) : TOptional<float>(); })
					.OnValueCommitted(this, &SPathfindingTuner::OnSymmetryBreakingRangeMultiplierCommitted)
				]
			]

			// SolverIterations
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("SolverIterationsTooltip", "Number of iterations to run the velocity solver."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("SolverIterationsLabel", "Solver Iterations")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SNumericEntryBox<int32>)
					.Value_Lambda([AvoidanceCDO]() -> TOptional<int32> { return AvoidanceCDO ? TOptional(AvoidanceCDO->GetSolverIterations()) : TOptional<int32>(); })
					.OnValueCommitted(this, &SPathfindingTuner::OnSolverIterationsCommitted)
					.MinValue(1).MaxValue(20)
				]
			]
			
			// SideBiasMagnitude
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("SideBiasMagnitudeTooltip", "Magnitude of the bias to encourage agents to pass on a specific side."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("SideBiasMagnitudeLabel", "Side Bias Magnitude")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SNumericEntryBox<float>)
					.Value_Lambda([AvoidanceCDO]() -> TOptional<float> { return AvoidanceCDO ? TOptional(AvoidanceCDO->GetSideBiasMagnitude()) : TOptional<float>(); })
					.OnValueCommitted(this, &SPathfindingTuner::OnSideBiasMagnitudeCommitted)
				]
			]

			// InterpSpeed
			+ SVerticalBox::Slot().AutoHeight().Padding(8.0f, 5.0f)
			[
				SNew(SHorizontalBox)
				.ToolTipText(LOCTEXT("InterpSpeedTooltip", "Interpolation speed for smoothing the final velocity."))
				+ SHorizontalBox::Slot().FillWidth(0.4f).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("InterpSpeedLabel", "Interpolation Speed")).Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
				]
				+ SHorizontalBox::Slot().FillWidth(0.6f).VAlign(VAlign_Center)
				[
					SNew(SNumericEntryBox<float>)
					.Value_Lambda([AvoidanceCDO]() -> TOptional<float> { return AvoidanceCDO ? TOptional(AvoidanceCDO->GetInterpSpeed()) : TOptional<float>(); })
					.OnValueCommitted(this, &SPathfindingTuner::OnInterpSpeedCommitted)
				]
			]
		]
		.InitiallyCollapsed(!TunerSettings->bIsAvoidanceCategoryExpanded)
		.OnAreaExpansionChanged(this, &SPathfindingTuner::OnAvoidanceCategoryExpansionChanged);	
}
TSharedRef<SWidget> SPathfindingTuner::BuildVoxelDebugSettingsSection()
{
	const UPathfindingTunerSettings* TunerSettings = GetDefault<UPathfindingTunerSettings>();

	return SNew(SExpandableArea)
	                            .IsEnabled(this, &SPathfindingTuner::AreOnlyOctreeVolumesSelected)
	                            .HeaderContent()
	                            [
		                            SNew(STextBlock)
		                                            .Text(LOCTEXT("DebugSettingsCategory", "Octree Volume Debug Settings"))
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
			                            + SHorizontalBox::Slot()
			                            .FillWidth(0.4f)
			                            .VAlign(VAlign_Center)
			                            [
				                            SNew(STextBlock)
				                            .Text(LOCTEXT("DrawDebugBox", "Draw Debug Box"))
				                            .ToolTipText(LOCTEXT("DrawDebugBoxTooltip", "Whether to draw the main volume debug box."))
				                            .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
			                            ]
			                            + SHorizontalBox::Slot()
			                            .FillWidth(0.6f)
			                            [
				                            SNew(SCheckBox)
				                            .OnCheckStateChanged(this, &SPathfindingTuner::OnDrawDebugBoxChanged)
				                            .IsChecked_Lambda([this]() -> ECheckBoxState
				                            {
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
				                            .Text(LOCTEXT("DebugLineWidth", "Debug Line Width"))
				                            .ToolTipText(LOCTEXT("DebugLineWidthTooltip", "The thickness of the debug lines."))
				                            .Font(FAppStyle::GetFontStyle("PropertyWindow.NormalFont"))
			                            ]
			                            + SHorizontalBox::Slot()
			                            .FillWidth(0.6f)
			                            [
				                            SNew(SNumericEntryBox<float>)
				                                                         .Value(this, &SPathfindingTuner::GetDebugLineWidth)
				                                                         .OnValueCommitted(this, &SPathfindingTuner::OnDebugLineWidthCommitted)
				                                                         .MinValue(1.0f)
				                                                         .MaxValue(1000.0f)
				                                                         .MinSliderValue(10.0f)
				                                                         .MaxSliderValue(500.0f)
				                                                         .UndeterminedString(LOCTEXT("Unselected", "Unselected"))
			                            ]
		                            ]
	                            ]
	                            .InitiallyCollapsed(!TunerSettings->bIsDebugSettingsCategoryExpanded)
	                            .OnAreaExpansionChanged(this, &SPathfindingTuner::OnDebugSettingsCategoryExpansionChanged);
}

//=================================================================================================
// Actor Selection Handling
//=================================================================================================
bool SPathfindingTuner::TickActorSelection(float DeltaTime)
{
	if (!GEditor) return false;

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

	return true;
}

void SPathfindingTuner::OnActorSelectionChanged()
{
	if (!AgentDetailsView.IsValid())
	{
		return;
	}

	TArray<UObject*> AgentComponentsToDisplay;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AActor* Actor = WeakActor.Get())
		{
		
			if (UPathfindingComponent* PathfindingComp = Actor->FindComponentByClass<UPathfindingComponent>())
			{
				AgentComponentsToDisplay.Add(PathfindingComp);
			}
		}
	}

	AgentDetailsView->SetObjects(AgentComponentsToDisplay);
	Invalidate(EInvalidateWidgetReason::Layout);
}

//=================================================================================================
// Performance Settings
//=================================================================================================

TOptional<int32> SPathfindingTuner::GetCurrentNumThreads() const
{
	if (const UPathfindingSubsystem* SubsystemCDO = GetDefault<UPathfindingSubsystem>())
	{
		return SubsystemCDO->GetNumWorkerThreads();
	}
	return TOptional<int32>();
}

void SPathfindingTuner::OnNumThreadsCommitted(int32 NewValue, ETextCommit::Type CommitInfo)
{
	if (UPathfindingSubsystem* SubsystemCDO = GetMutableDefault<UPathfindingSubsystem>())
	{
		SubsystemCDO->SetNumWorkerThreads(NewValue);
		SubsystemCDO->SaveConfig();
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
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	const int32 NumSelected = SelectedVolumes.Num();

	if (NumSelected == 0)
	{
		return LOCTEXT("NoVolumeSelected", "None");
	}

	if (NumSelected == 1)
	{
		return FText::FromString(SelectedVolumes[0]->GetActorLabel());
	}

	return FText::Format(LOCTEXT("MultipleVolumesSelected", "{0} Volumes Selected"), FText::AsNumber(NumSelected));
}

FText SPathfindingTuner::GetBakedDataSaveDirectoryAsText() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	TOptional<FString> CommonPath = GetCommonPropertyValue<AOctreeVoxelVolume, FString>(SelectedVolumes,
		[](const AOctreeVoxelVolume* V) { return V->BakedDataSaveDirectory.Path; });

	return CommonPath.IsSet() ? FText::FromString(CommonPath.GetValue()) : FText();
}

void SPathfindingTuner::OnBakedDataSaveDirectoryTextCommitted(const FText& InText, ETextCommit::Type CommitInfo)
{
	if (CommitInfo == ETextCommit::OnEnter || CommitInfo == ETextCommit::OnUserMovedFocus)
	{
		TArray<AOctreeVoxelVolume*> SelectedVolumes;
		for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
		{
			if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
			{
				SelectedVolumes.Add(Volume);
			}
		}

		SetCommonPropertyValue<AOctreeVoxelVolume, FString>(SelectedVolumes, InText.ToString(),
		                                                    LOCTEXT("SetSaveDirectoryTransaction",
		                                                            "Set Baked Data Save Directory"),
		                                                    [](AOctreeVoxelVolume* V, const FString& Path)
		                                                    {
			                                                    V->BakedDataSaveDirectory.Path = Path;
			                                                    V->MarkPackageDirty();
		                                                    });
	}
}

FString SPathfindingTuner::GetObstacleProfilePath() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	TOptional<UVoxelObstacleProfile*> CommonProfile = GetCommonPropertyValue<
		AOctreeVoxelVolume, UVoxelObstacleProfile*>(SelectedVolumes,
		                                            [](const AOctreeVoxelVolume* V)
		                                            {
			                                            return V->GetStaticObstacleProfile();
		                                            });

	return (CommonProfile.IsSet() && CommonProfile.GetValue()) ? CommonProfile.GetValue()->GetPathName() : FString();
}

void SPathfindingTuner::OnObstacleProfileChanged(const FAssetData& AssetData)
{
	UVoxelObstacleProfile* NewProfile = Cast<UVoxelObstacleProfile>(AssetData.GetAsset());
	
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	SetCommonPropertyValue<AOctreeVoxelVolume, UVoxelObstacleProfile*>(SelectedVolumes, NewProfile,
	                                                                   LOCTEXT("SetObstacleProfileTransaction",
	                                                                           "Set Voxel Obstacle Profile"),
	                                                                   [](AOctreeVoxelVolume* V,
	                                                                      UVoxelObstacleProfile* Profile)
	                                                                   {
		                                                                   V->SetStaticObstacleProfile(Profile);
		                                                                   V->MarkPackageDirty();
	                                                                   });
}

FText SPathfindingTuner::GetObstacleProfileFullPathAsText() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	TOptional<UVoxelObstacleProfile*> CommonProfile = GetCommonPropertyValue<
		AOctreeVoxelVolume, UVoxelObstacleProfile*>(SelectedVolumes,
		                                            [](const AOctreeVoxelVolume* V)
		                                            {
			                                            return V->GetStaticObstacleProfile();
		                                            });

	if (SelectedVolumes.Num() == 0)
	{
		return LOCTEXT("NonePath", "None");
	}

	if (!CommonProfile.IsSet())
	{
		return LOCTEXT("MultipleValues", "Multiple Values");
	}

	if (!CommonProfile.GetValue())
	{
		return LOCTEXT("NonePath", "None");
	}

	return FText::FromString(FPaths::GetPath(CommonProfile.GetValue()->GetPathName()));
}

FReply SPathfindingTuner::OnBrowseButtonClicked()
{
	IDesktopPlatform* DesktopPlatform = FDesktopPlatformModule::Get();
	if (DesktopPlatform)
	{
		const void* ParentWindowHandle = FSlateApplication::Get().GetActiveTopLevelWindow()->GetNativeWindow()->
		                                                          GetOSWindowHandle();
		FString SelectedDirectory;

		if (DesktopPlatform->OpenDirectoryDialog(ParentWindowHandle, TEXT("Select Save Directory"),
		                                         FPaths::ProjectContentDir(), SelectedDirectory))
		{
			const FString FullContentDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir());
			if (!SelectedDirectory.StartsWith(FullContentDir))
			{
				FMessageDialog::Open(EAppMsgType::Ok,
				                     LOCTEXT("InvalidPathMsg",
				                             "Please select a directory inside the project's Content folder."));
				return FReply::Handled();
			}

			FString RelativePath = SelectedDirectory;
			FPaths::MakePathRelativeTo(RelativePath, *FullContentDir);

			OnBakedDataSaveDirectoryTextCommitted(FText::FromString(RelativePath), ETextCommit::OnEnter);
		}
	}
	return FReply::Handled();
}

TOptional<float> SPathfindingTuner::GetVoxelExtent() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	return GetCommonPropertyValue<AOctreeVoxelVolume, float>(SelectedVolumes,
	                                                         [](const AOctreeVoxelVolume* V)
	                                                         {
		                                                         return V->VoxelExtent;
	                                                         });
}

void SPathfindingTuner::OnVoxelExtentCommitted(float NewValue, ETextCommit::Type CommitType)
{
	if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
	{
		TArray<AOctreeVoxelVolume*> SelectedVolumes;
		for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
		{
			if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
			{
				SelectedVolumes.Add(Volume);
			}
		}

		SetCommonPropertyValue<AOctreeVoxelVolume, float>(SelectedVolumes, NewValue,
		                                                  LOCTEXT("SetVoxelSizeTransaction", "Set Voxel Size"),
		                                                  [](AOctreeVoxelVolume* V, float Value)
		                                                  {
			                                                  V->VoxelExtent = Value;
			                                                  V->MarkPackageDirty();
			                                                  V->RerunConstructionScripts();
		                                                  });
	}
}

FText SPathfindingTuner::GetSelectedMapSizeText() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	if (SelectedVolumes.IsEmpty()) return LOCTEXT("NoVolumeSelected", "---");

	TOptional<FIntVector> CommonMapSize = GetCommonPropertyValue<AOctreeVoxelVolume, FIntVector>(SelectedVolumes,
		[](const AOctreeVoxelVolume* V) { return V->GetMapSize(); });

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

		SetCommonPropertyValue<AOctreeVoxelVolume, FIntVector>(SelectedVolumes, NewMapSize,
															   LOCTEXT("SetMapSizeTransaction", "Set Map Size"),
															   [](AOctreeVoxelVolume* V, const FIntVector& Size)
															   {
																   V->SetMapSize(Size);
																   V->RerunConstructionScripts();
															   });
	}
}

//=================================================================================================
// Baked Data Settings
//=================================================================================================

FString SPathfindingTuner::GetBakedOctreeDataPath() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	TOptional<UOctreeDataAsset*> CommonAsset = GetCommonPropertyValue<AOctreeVoxelVolume, UOctreeDataAsset*>(
		SelectedVolumes,
		[](const AOctreeVoxelVolume* V) { return V->GetBakedOctreeData().Get(); });

	return (CommonAsset.IsSet() && CommonAsset.GetValue()) ? CommonAsset.GetValue()->GetPathName() : FString();
}

void SPathfindingTuner::OnBakedOctreeDataChanged(const FAssetData& AssetData)
{
	UOctreeDataAsset* NewDataAsset = Cast<UOctreeDataAsset>(AssetData.GetAsset());
	
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	SetCommonPropertyValue<AOctreeVoxelVolume, UOctreeDataAsset*>(SelectedVolumes, NewDataAsset,
	                                                              LOCTEXT("SetBakedDataTransaction",
	                                                                      "Set Baked Octree Data"),
	                                                              [](AOctreeVoxelVolume* V, UOctreeDataAsset* Asset)
	                                                              {
		                                                              V->SetBakedOctreeData(Asset);
		                                                              V->MarkPackageDirty();
	                                                              });
}

FText SPathfindingTuner::GetBakedDataFullPathAsText() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	TOptional<UOctreeDataAsset*> CommonAsset = GetCommonPropertyValue<AOctreeVoxelVolume, UOctreeDataAsset*>
		(SelectedVolumes, [](const AOctreeVoxelVolume* V) { return V->GetBakedOctreeData().Get(); });

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
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	TOptional<bool> CommonValue = GetCommonPropertyValue<AOctreeVoxelVolume, bool>(SelectedVolumes,
		[](const AOctreeVoxelVolume* V) { return V->GetbDrawDebugBox(); });

	if (CommonValue.IsSet())
	{
		return CommonValue.GetValue() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
	}
	return ECheckBoxState::Undetermined;
}

void SPathfindingTuner::OnDrawDebugBoxChanged(ECheckBoxState NewState)
{
	if (NewState != ECheckBoxState::Undetermined)
	{
		TArray<AOctreeVoxelVolume*> SelectedVolumes;
		for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
		{
			if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
			{
				SelectedVolumes.Add(Volume);
			}
		}

		SetCommonPropertyValue<AOctreeVoxelVolume, bool>(SelectedVolumes, NewState == ECheckBoxState::Checked,
		                                               LOCTEXT("SetDrawDebugBoxTransaction", "Set Draw Debug Box"),
		                                               [](AOctreeVoxelVolume* V, bool Value)
		                                               {
			                                               V->SetbDrawDebugBox(Value);
			                                               V->OnConstruction(V->GetActorTransform());
		                                               });
	}
}

TOptional<float> SPathfindingTuner::GetDebugLineWidth() const
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
		{
			SelectedVolumes.Add(Volume);
		}
	}

	return GetCommonPropertyValue<AOctreeVoxelVolume, float>(SelectedVolumes,
	                                                       [](const AOctreeVoxelVolume* V)
	                                                       {
		                                                       return V->GetDebugLineWidth();
	                                                       });
}

void SPathfindingTuner::OnDebugLineWidthCommitted(float NewValue, ETextCommit::Type CommitType)
{
	if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
	{
		TArray<AOctreeVoxelVolume*> SelectedVolumes;
		for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
		{
			if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(WeakActor.Get()))
			{
				SelectedVolumes.Add(Volume);
			}
		}

		SetCommonPropertyValue<AOctreeVoxelVolume, float>(SelectedVolumes, NewValue,
		                                                LOCTEXT("SetDebugLineWidthTransaction", "Set Debug Line Width"),
		                                                [](AOctreeVoxelVolume* V, float Value)
		                                                {
			                                                V->SetDebugLineWidth(Value);
			                                                V->OnConstruction(V->GetActorTransform());
			                                                V->MarkPackageDirty();
		                                                });
	}
}

TOptional<ECheckBoxState> SPathfindingTuner::GetDrawDebugPathState() const
{
	TArray<UPathfindingComponent*> SelectedComponents;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AActor* Actor = WeakActor.Get())
		{
			if (UPathfindingComponent* Comp = Actor->FindComponentByClass<UPathfindingComponent>())
			{
				SelectedComponents.Add(Comp);
			}
		}
	}

	TOptional<bool> CommonValue = GetCommonPropertyValue<UPathfindingComponent, bool>(SelectedComponents,
		[](const UPathfindingComponent* C) { return C->GetbDrawDebugPath(); });

	if (CommonValue.IsSet())
	{
		return CommonValue.GetValue() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
	}
	return ECheckBoxState::Undetermined;
}

void SPathfindingTuner::OnDrawDebugPathChanged(ECheckBoxState NewState)
{
	if (NewState != ECheckBoxState::Undetermined)
	{
		TArray<UPathfindingComponent*> SelectedComponents;
		for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
		{
			if (AActor* Actor = WeakActor.Get())
			{
				if (UPathfindingComponent* Comp = Actor->FindComponentByClass<UPathfindingComponent>())
				{
					SelectedComponents.Add(Comp);
				}
			}
		}

		SetCommonPropertyValue<UPathfindingComponent, bool>(SelectedComponents, NewState == ECheckBoxState::Checked,
		                                                    LOCTEXT("SetDrawDebugPath", "Set Draw Debug Path"),
		                                                    [](UPathfindingComponent* C, bool bDraw)
		                                                    {
			                                                    C->SetbDrawDebugPath(bDraw);
		                                                    });
	}
}

//=================================================================================================
// Agent Settings
//=================================================================================================

void SPathfindingTuner::OnbAvoidanceComitted(ECheckBoxState NewState)
{
	if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
	{
		const bool bUseAvoidance = (NewState == ECheckBoxState::Checked);
		AvoidanceCDO->SetbUseAvoidance(bUseAvoidance);
		AvoidanceCDO->SaveConfig();
		Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
	}
}

void SPathfindingTuner::OnTimeHorizonCommitted(float NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
        {
            AvoidanceCDO->SetTimeHorizon(NewValue);
            AvoidanceCDO->SaveConfig();
            Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
        }
    }
}

void SPathfindingTuner::OnMaxNeighborsCommitted(int32 NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
        {
            AvoidanceCDO->SetMaxNeighbors(NewValue);
            AvoidanceCDO->SaveConfig();
            Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
        }
    }
}

void SPathfindingTuner::OnGridCellSizeCommitted(float NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
        {
            AvoidanceCDO->SetGridCellSize(NewValue);
            AvoidanceCDO->SaveConfig();
            Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
        }
    }
}

void SPathfindingTuner::OnSymmetryBreakingRangeMultiplierCommitted(float NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
        {
            AvoidanceCDO->SetSymmetryBreakingRangeMultiplier(NewValue);
            AvoidanceCDO->SaveConfig();
            Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
        }
    }
}

void SPathfindingTuner::OnSolverIterationsCommitted(int32 NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
        {
            AvoidanceCDO->SetSolverIterations(NewValue);
            AvoidanceCDO->SaveConfig();
            Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
        }
    }
}

void SPathfindingTuner::OnSideBiasMagnitudeCommitted(float NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
        {
            AvoidanceCDO->SetSideBiasMagnitude(NewValue);
            AvoidanceCDO->SaveConfig();
            Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
        }
    }
}

void SPathfindingTuner::OnInterpSpeedCommitted(float NewValue, ETextCommit::Type CommitType)
{
    if (CommitType == ETextCommit::OnEnter || CommitType == ETextCommit::OnUserMovedFocus)
    {
        if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
        {
            AvoidanceCDO->SetInterpSpeed(NewValue);
            AvoidanceCDO->SaveConfig();
            Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
        }
    }
}

void SPathfindingTuner::OnbAvoidanceVelocityDebugComitted(ECheckBoxState NewState)
{
	if (UAvoidanceSubsystem* AvoidanceCDO = GetMutableDefault<UAvoidanceSubsystem>())
	{
		const bool bUseAvoidance = (NewState == ECheckBoxState::Checked);
		AvoidanceCDO->SetbShowVelocityDebugLine(bUseAvoidance);
		AvoidanceCDO->SaveConfig();
		Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
	}
}

bool SPathfindingTuner::AreOnlyAgentsSelected() const
{
	if (LastSelectedActors.IsEmpty()) return false;

	for (const auto& ActorPtr : LastSelectedActors)
	{
		if (AActor* Actor = ActorPtr.Get())
		{
			if (!Actor->FindComponentByClass<UPathfindingComponent>())
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	return true;
}

//=================================================================================================
// Actions and Events
//=================================================================================================

FReply SPathfindingTuner::OnBakeButtonClicked()
{
	TArray<AOctreeVoxelVolume*> SelectedVolumes;
	if (USelection* SelectedActors = GEditor->GetSelectedActors())
	{
		TArray<AActor*> Actors;
		SelectedActors->GetSelectedObjects<AActor>(Actors);
		for (AActor* Actor : Actors)
		{
			if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(Actor))
			{
				SelectedVolumes.Add(Volume);
			}
		}
	}

	if (SelectedVolumes.IsEmpty())
	{
		ShowWarningMessage(LOCTEXT("NoOctreeSelectedForBake", "No Octree Volume selected to bake."));
		return FReply::Handled();
	}

	for (AOctreeVoxelVolume* Volume : SelectedVolumes)
	{
		if (Volume->BakedDataSaveDirectory.Path.IsEmpty())
		{
			FText ErrorMsg = FText::Format(
				LOCTEXT("BakePathNotSetForOne", "Cannot start baking. 'Baked Data Save Directory' is not set for {0}."),
				FText::FromString(Volume->GetActorLabel()));
			ShowWarningMessage(ErrorMsg);
			return FReply::Handled();
		}
	}

	if (SelectedVolumes.Num() > 1)
	{
		FText ConfirmationMessage = FText::Format(
			LOCTEXT("MultiBakeConfirmation",
			        "You have selected {0} volumes to bake.\nThis process could take a long time and may cause the editor to become unresponsive.\n\nDo you want to continue?"),
			FText::AsNumber(SelectedVolumes.Num())
		);

		if (FMessageDialog::Open(EAppMsgType::YesNo, ConfirmationMessage) != EAppReturnType::Yes)
		{
			return FReply::Handled();
		}
	}
	
	if (UPathfindingEditorSubsystem* EditorSubsystem = GEditor->GetEditorSubsystem<UPathfindingEditorSubsystem>())
	{
		EditorSubsystem->BakeSpecificVolumes(SelectedVolumes);
		FNotificationInfo NotifyInfo(FText::FromString(TEXT("Bake Complete!")));
		NotifyInfo.bUseLargeFont = true;
		NotifyInfo.FadeOutDuration = 4.5f;
		FSlateNotificationManager::Get().AddNotification(NotifyInfo);
	}
	else
	{
		ShowWarningMessage(LOCTEXT("NoEditorSubsystem", "Pathfinding Editor Subsystem not found. Cannot start bake."));
	}

	return FReply::Handled();
}

FReply SPathfindingTuner::OnRequestPathButtonClicked()
{
	if (!IsPIEActive())
	{
		ShowWarningMessage(LOCTEXT("NotPIEWarning", "Path requests can only be made during Play In Editor."));
		return FReply::Handled();
	}

	TArray<UPathfindingComponent*> SelectedComponents;
	for (const TWeakObjectPtr<AActor>& WeakActor : LastSelectedActors)
	{
		if (AActor* Actor = WeakActor.Get())
		{
			if (UPathfindingComponent* Comp = Actor->FindComponentByClass<UPathfindingComponent>())
			{
				SelectedComponents.Add(Comp);
			}
		}
	}

	if (SelectedComponents.Num() == 0)
	{
		ShowWarningMessage(LOCTEXT("NoAgentSelected", "Please select at least one agent with a Pathfinding Component."));
		return FReply::Handled();
	}

	bool bTargetActorNotSetWarningShown = false;
	for (UPathfindingComponent* Component : SelectedComponents)
	{
		if (Component && !IsValid(Component->GetTargetActor()))
		{
			if (!bTargetActorNotSetWarningShown)
			{
				ShowWarningMessage(LOCTEXT("TargetActorNotSetSummary", "Path request failed for some agents: Target Actor is not set."));
				bTargetActorNotSetWarningShown = true;
			}
			continue;
		}

		if (Component)
		{
			Component->RequestPathToTargetActor();
		}
	}

	return FReply::Handled();
}

//=================================================================================================
// UI State & Expansion Handlers
//=================================================================================================

void SPathfindingTuner::OnAvoidanceCategoryExpansionChanged(bool bIsExpanded)
{
	if (UPathfindingTunerSettings* TunerSettings = GetMutableDefault<UPathfindingTunerSettings>())
	{
		TunerSettings->bIsAvoidanceCategoryExpanded = bIsExpanded;
		TunerSettings->SaveConfig();
	}
}


void SPathfindingTuner::OnPerformanceCategoryExpansionChanged(bool bIsExpanded)
{
	if (UPathfindingTunerSettings* TunerSettings = GetMutableDefault<UPathfindingTunerSettings>())
	{
		TunerSettings->bIsPerformanceCategoryExpanded = bIsExpanded;
		TunerSettings->SaveConfig();
	}
}

void SPathfindingTuner::OnOctreeSettingsCategoryExpansionChanged(bool bIsExpanded)
{
	if (UPathfindingTunerSettings* TunerSettings = GetMutableDefault<UPathfindingTunerSettings>())
	{
		TunerSettings->bIsOctreeSettingsCategoryExpanded = bIsExpanded;
		TunerSettings->SaveConfig();
	}
}

void SPathfindingTuner::OnBakedDataCategoryExpansionChanged(bool bIsExpanded)
{
	if (UPathfindingTunerSettings* TunerSettings = GetMutableDefault<UPathfindingTunerSettings>())
	{
		TunerSettings->bIsBakedDataCategoryExpanded = bIsExpanded;
		TunerSettings->SaveConfig();
	}
}

void SPathfindingTuner::OnDebugSettingsCategoryExpansionChanged(bool bIsExpanded)
{
	if (UPathfindingTunerSettings* TunerSettings = GetMutableDefault<UPathfindingTunerSettings>())
	{
		TunerSettings->bIsDebugSettingsCategoryExpanded = bIsExpanded;
		TunerSettings->SaveConfig();
	}
}

void SPathfindingTuner::OnAgentDebugCategoryExpansionChanged(bool bIsExpanded)
{
	if (UPathfindingTunerSettings* TunerSettings = GetMutableDefault<UPathfindingTunerSettings>())
	{
		TunerSettings->bAgentDebugSettingsExpanded = bIsExpanded;
		TunerSettings->SaveConfig();
	}
}

bool SPathfindingTuner::IsPIEActive() const
{
	return GEditor && GEditor->PlayWorld && GEditor->PlayWorld->IsGameWorld();
}

void SPathfindingTuner::ShowWarningMessage(const FText& Message) const
{
	FMessageDialog::Open(EAppMsgType::Ok, Message);
}

#undef LOCTEXT_NAMESPACE