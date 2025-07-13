#include "Nav3DVolumeDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "PropertyEditorModule.h"
#include "Pathfinder/FenwickVoxelVolume.h"

TSharedRef<IDetailCustomization> FNav3DVolumeDetails::MakeInstance()
{
	return MakeShareable(new FNav3DVolumeDetails);
}

void FNav3DVolumeDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    IDetailCategoryBuilder& VolumeCategory = DetailBuilder.EditCategory("Volume Setup", FText::GetEmpty(), ECategoryPriority::Important);
    IDetailCategoryBuilder& DebuggingCategory = DetailBuilder.EditCategory("Debugging", FText::GetEmpty(), ECategoryPriority::Important);

    // "Volume Setup" 카테고리에 핸들을 이용해 프로퍼티들을 추가(재배치)합니다.
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, VoxelExtent));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, MapSize));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, VolumeBoundsVisualizer));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, VisualVolumeMeshAsset));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, BakedDataSaveDirectory));
    VolumeCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, StreamingRangeInChunks));
	
	TArray<TWeakObjectPtr<UObject>> SelectedObjects;
	DetailBuilder.GetObjectsBeingCustomized(SelectedObjects);
	
	auto OnRequestPathClicked = [SelectedObjects]()-> FReply
	{
		for (const TWeakObjectPtr<UObject>& SelectedObject : SelectedObjects)
		{
			if (AFenwickVoxelVolume* Volume = Cast<AFenwickVoxelVolume>(SelectedObject.Get()))
			{
				Volume->BakeStaticDataToChunks();
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

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, bDrawDebugBox));
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(AFenwickVoxelVolume, BakeExcludeActors));
	
}