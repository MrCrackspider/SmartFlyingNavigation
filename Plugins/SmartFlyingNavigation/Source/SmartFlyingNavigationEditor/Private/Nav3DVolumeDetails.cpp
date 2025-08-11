// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Nav3DVolumeDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"
#include "PropertyEditorModule.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "PathfindingTunerTabName.h"
#include "Subsystem/OctreeNavSubsystem.h" 
#include "Editor.h"
#include "PathfindingEditorSubsystem.h"

#define LOCTEXT_NAMESPACE "FNav3DVolumeDetails"
TSharedRef<IDetailCustomization> FNav3DVolumeDetails::MakeInstance()
{
	return MakeShareable(new FNav3DVolumeDetails);
}

void FNav3DVolumeDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    // --- 1. Organize existing properties into categories (this part remains unchanged) ---
    IDetailCategoryBuilder& VolumeCategory = DetailBuilder.EditCategory("Volume Setup", FText::GetEmpty(), ECategoryPriority::Important);
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AVoxelVolumeBase, VoxelExtent));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, BakedDataSaveDirectory));

    IDetailCategoryBuilder& DebuggingCategory = DetailBuilder.EditCategory("Debugging", FText::GetEmpty(), ECategoryPriority::Important);
    DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, bDrawDebugBox));

    // --- 2. Add a "Bake Actions" category and a new bake button ---
    IDetailCategoryBuilder& BakeCategory = DetailBuilder.EditCategory("Bake Actions", FText::GetEmpty(), ECategoryPriority::Important);

    // Get the list of currently selected objects in the details panel.
    TArray<TWeakObjectPtr<UObject>> SelectedObjects;
    DetailBuilder.GetObjectsBeingCustomized(SelectedObjects);

    BakeCategory.AddCustomRow(LOCTEXT("BakeButtonFilter", "Bake Volume"))
        .NameContent()
        [
            SNew(STextBlock)
                .Text(LOCTEXT("BakeActionLabel", "Bake Actions"))
                .Font(IDetailLayoutBuilder::GetDetailFont())
        ];

    // --- 3. Keep the existing "Pathfinding Tuner UI" button ---
    IDetailCategoryBuilder& TunerCategory = DetailBuilder.EditCategory("Tuner", FText::GetEmpty(), ECategoryPriority::Important);
    TunerCategory.AddCustomRow(LOCTEXT("OpenPathfindingTunerFilter", "Open Pathfinding Tuner"))
        .NameContent()
        [
            SNew(STextBlock)
                .Text(LOCTEXT("OpenPathfindingTunerLabel", "Pathfinding Tuner UI"))
                .Font(IDetailLayoutBuilder::GetDetailFont())
        ]
        .ValueContent()
        [
            SNew(SButton)
                .Text(LOCTEXT("OpenPathfindingTunerButton", "Open Tuner"))
                .OnClicked_Lambda([]() -> FReply
                    {
                        FGlobalTabmanager::Get()->TryInvokeTab(PathfindingTunerTabName);
                        return FReply::Handled();
                    })
        ];
}
#undef LOCTEXT_NAMESPACE