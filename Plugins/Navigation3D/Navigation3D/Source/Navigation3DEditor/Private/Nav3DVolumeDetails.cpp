#include "Nav3DVolumeDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"
#include "PropertyEditorModule.h"
// Begin Test
//#include "Pathfinder/NavVolume/FenwickVoxelVolume.h"
#include "Pathfinder/NavVolume/OctreeVoxelVolume.h"
// End Test

// End Test
TSharedRef<IDetailCustomization> FNav3DVolumeDetails::MakeInstance()
{
	return MakeShareable(new FNav3DVolumeDetails);
}

void FNav3DVolumeDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    IDetailCategoryBuilder& VolumeCategory = DetailBuilder.EditCategory("Volume Setup", FText::GetEmpty(), ECategoryPriority::Important);
    IDetailCategoryBuilder& DebuggingCategory = DetailBuilder.EditCategory("Debugging", FText::GetEmpty(), ECategoryPriority::Important);

    // "Volume Setup" 카테고리에 핸들을 이용해 프로퍼티들을 추가(재배치)합니다.
	// Begin Test
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, VoxelExtent));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, MapSize));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, DebugOctreeMaxDepth));
    //VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, VoxelExtent));
    //VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, MapSize));
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
		//왼쪽: 버튼의 이름/ 설명
		SNew(STextBlock)
		.Text(FText::FromString(TEXT("Bake Action")))
		.Font(IDetailLayoutBuilder::GetDetailFont())
	].ValueContent()
	[
		//오른쪽: 실제 버튼
		SNew(SButton)
		.Text(FText::FromString(TEXT("Bake Static Data to Chunks")))
		.OnClicked_Lambda(OnRequestPathClicked) // 람다함수를 버튼 클릭 이벤트에 바인딩
	];

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, bDrawDebugBox));
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AOctreeVoxelVolume, BakeExcludeActors));
	
}