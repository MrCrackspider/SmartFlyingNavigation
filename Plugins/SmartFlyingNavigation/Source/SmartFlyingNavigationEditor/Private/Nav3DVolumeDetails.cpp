// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Nav3DVolumeDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"
#include "PropertyEditorModule.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "PathfindingTunerTabName.h"

TSharedRef<IDetailCustomization> FNav3DVolumeDetails::MakeInstance()
{
	return MakeShareable(new FNav3DVolumeDetails);
}

void FNav3DVolumeDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    IDetailCategoryBuilder& VolumeCategory = DetailBuilder.EditCategory("Volume Setup", FText::GetEmpty(), ECategoryPriority::Important);
    IDetailCategoryBuilder& DebuggingCategory = DetailBuilder.EditCategory("Tuner", FText::GetEmpty(), ECategoryPriority::Important);

	// Begin Test
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AVoxelVolumeBase, VoxelExtent));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AVoxelVolumeBase, MapSize));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, MaxOctreeDepth));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, BakedDataSaveDirectory));
    // VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, VolumeBoundsVisualizer));
    // VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, VisualVolumeMeshAsset));
    // VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, BakedDataSaveDirectory));
    // VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, StreamingRangeInChunks));
	// End Test

	TArray<TWeakObjectPtr<UObject>> SelectedObjects;
	DetailBuilder.GetObjectsBeingCustomized(SelectedObjects);
	
	auto OnRequestPathClicked = [SelectedObjects]()-> FReply
	{
		for (const TWeakObjectPtr<UObject>& SelectedObject : SelectedObjects)
		{
			if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(SelectedObject.Get()))
			{
				Volume->BakeStaticData();
			}
		}
		return FReply::Handled();
	};
	
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, bDrawDebugBox));
	
	DebuggingCategory.AddCustomRow(LOCTEXT("OpenPathfindingTunerFilter", "Open Pathfinding Tuner")) // Filter text
	.NameContent()
	[
		 SNew(STextBlock)
		 .Text(LOCTEXT("OpenPathfindingTunerLabel", "Pathfinding Tuner UI")) // Label text
		 .Font(IDetailLayoutBuilder::GetDetailFont()) // Use standard detail font
	]
	.ValueContent()
	[
		 SNew(SButton)
		 .Text(LOCTEXT("OpenPathfindingTunerButton", "Open Tuner")) // Button text
		 .ContentPadding(FMargin(4.0f, 2.0f)) // Standard button padding
		 .HAlign(HAlign_Center) // Center align text horizontally
		 .VAlign(VAlign_Center) // Center align text vertically
		 .OnClicked_Lambda([]() -> FReply // Lambda for button click event
		 {
			 // Invoke the Pathfinding Tuner tab
			 FGlobalTabmanager::Get()->TryInvokeTab(PathfindingTunerTabName);
			 return FReply::Handled();
		 })
	];
}