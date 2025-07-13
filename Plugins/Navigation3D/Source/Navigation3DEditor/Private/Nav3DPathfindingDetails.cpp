#include "Nav3DPathfindingDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "PropertyEditorModule.h"
#include "Pathfinder/PathfindingComponent.h"

TSharedRef<IDetailCustomization> FNav3DPathfindingDetails::MakeInstance()
{
	return MakeShareable(new FNav3DPathfindingDetails);
}

void FNav3DPathfindingDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuidler)
{
	// 새로운 카테고리를 생성하거나 기존 카테고리를 가져옵니다.
	IDetailCategoryBuilder& PathfindingCategory = DetailBuidler.EditCategory("Navigation Configuration", FText::GetEmpty(), ECategoryPriority::Important);
	IDetailCategoryBuilder& DebuggingCategory = DetailBuidler.EditCategory("Debugging", FText::GetEmpty(), ECategoryPriority::Important);

	//카테고리에 기존 프로퍼리틑 추가합니다.
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, TargetActor));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, TargetVolume));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PathfindingStrategyClass));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RotationAxisLock));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, AgentRadius));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, MovementSpeed));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RotationInterpSpeed));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RepathCooldown));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, bPeriodicRepath));

	TArray<TWeakObjectPtr<UObject>> SelectedObjects;
	DetailBuidler.GetObjectsBeingCustomized(SelectedObjects);
	
	auto OnRequestPathClicked = [SelectedObjects]()-> FReply
	{
		for (const TWeakObjectPtr<UObject>& SelectedObject : SelectedObjects)
		{
			if (UPathfindingComponent* PathfindingComp = Cast<UPathfindingComponent>(SelectedObject.Get()))
			{
				PathfindingComp->RequestPathToTargetActor();
			}
		}
		return FReply::Handled();
	};
	
	DebuggingCategory.AddCustomRow(FText::FromString(TEXT("Request Path"))).NameContent()
	[
		//왼쪽: 버튼의 이름/ 설명
		SNew(STextBlock)
		.Text(FText::FromString(TEXT("Execute Pathfinding")))
		.Font(IDetailLayoutBuilder::GetDetailFont())
	].ValueContent()
	[
		//오른쪽: 실제 버튼
		SNew(SButton)
		.Text(FText::FromString(TEXT("Request Path to Target")))
		.OnClicked_Lambda(OnRequestPathClicked) // 람다함수를 버튼 클릭 이벤트에 바인딩
	];

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, bDrawDebugPath));
}