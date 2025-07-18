#include "Nav3DVolumeDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"
#include "PropertyEditorModule.h"
#include "Pathfinder/NavVolume/OctreeVoxelVolume.h"

TSharedRef<IDetailCustomization> FNav3DVolumeDetails::MakeInstance()
{
	return MakeShareable(new FNav3DVolumeDetails);
}

void FNav3DVolumeDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    IDetailCategoryBuilder& VolumeCategory = DetailBuilder.EditCategory("Volume Setup", FText::GetEmpty(), ECategoryPriority::Important);
    IDetailCategoryBuilder& DebuggingCategory = DetailBuilder.EditCategory("Debugging", FText::GetEmpty(), ECategoryPriority::Important);

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
	
	DebuggingCategory.AddCustomRow(FText::FromString(TEXT("Baking"))).NameContent()
	[
		SNew(STextBlock)
		.Text(FText::FromString(TEXT("Bake Action")))
		.Font(IDetailLayoutBuilder::GetDetailFont())
	].ValueContent()
	[
		SNew(SButton)
		.Text(FText::FromString(TEXT("Bake Static Data to Chunks")))
		.OnClicked_Lambda(OnRequestPathClicked) 
	];

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, bDrawDebugBox));
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, BakeExcludeActors));
	
}