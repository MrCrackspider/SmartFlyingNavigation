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
        ]
        .ValueContent()
        [
            SNew(SButton)
                .Text(LOCTEXT("BakeSelectedButton", "Bake Selected Volume(s)"))
                .ToolTipText(LOCTEXT("BakeSelectedButton_Tooltip", "Bakes the navigation data for the currently selected volume(s), including portal connections between them."))
                .HAlign(HAlign_Center)
                .OnClicked_Lambda([SelectedObjects]() -> FReply
                    {
                        TArray<AOctreeVoxelVolume*> VolumesToBake;
                        for (const TWeakObjectPtr<UObject>& ObjPtr : SelectedObjects)
                        {
                            if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(ObjPtr.Get()))
                            {
                                VolumesToBake.Add(Volume);
                            }
                        }

                        if (VolumesToBake.IsEmpty())
                        {
                            UE_LOG(LogTemp, Warning, TEXT("No AOctreeVoxelVolume selected to bake."));
                            return FReply::Handled();
                        }

                        // <<<< This is the key modification >>>>
                        // Get the editor subsystem instead of the runtime subsystem.
                        if (UPathfindingEditorSubsystem* EditorSubsystem = GEditor->GetEditorSubsystem<UPathfindingEditorSubsystem>())
                        {
                            // Call the bake function on the editor subsystem.
                            EditorSubsystem->BakeSpecificVolumes(VolumesToBake);
                        }
                        else
                        {
                            UE_LOG(LogTemp, Error, TEXT("PathfindingEditorSubsystem not found! Cannot start bake."));
                        }
                        // <<<< End of modification >>>>

                        return FReply::Handled();
                    })
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