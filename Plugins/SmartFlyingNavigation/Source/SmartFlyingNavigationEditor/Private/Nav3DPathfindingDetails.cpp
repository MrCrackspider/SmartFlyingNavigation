#include "Nav3DPathfindingDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h" 
#include "PropertyEditorModule.h"
#include "PathfindingComponent.h"


#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

#include "Framework/Application/SlateApplication.h" 
#include "Widgets/Docking/SDockTab.h"
#include "PathfindingTunerTabName.h"


#define LOCTEXT_NAMESPACE "FNav3DPathfindingDetails"


TSharedRef<IDetailCustomization> FNav3DPathfindingDetails::MakeInstance()
{
	return MakeShareable(new FNav3DPathfindingDetails);
}

void FNav3DPathfindingDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuidler)
{
	IDetailCategoryBuilder& DebuggingCategory = DetailBuidler.EditCategory("Tuner", FText::GetEmpty(), ECategoryPriority::Important);
	IDetailCategoryBuilder& PathfindingCategory = DetailBuidler.EditCategory("Navigation Configuration", FText::GetEmpty(), ECategoryPriority::Important);

	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, TargetActor));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, TargetVolume));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PathfindingStrategyClass));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RotationAxisLock));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, BaseAgentRadius));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, AgentRadius));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, MovementSpeed));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RotationInterpSpeed));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RepathCooldown));
	PathfindingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, bPeriodicRepath));

	TArray<TWeakObjectPtr<UObject>> SelectedObjects;
	DetailBuidler.GetObjectsBeingCustomized(SelectedObjects);

	//Path finding Button in Editor (Moved to Slate UI)
//	auto OnRequestPathClicked = [SelectedObjects]()-> FReply
//	{
//		for (const TWeakObjectPtr<UObject>& SelectedObject : SelectedObjects)
//		{
//			if (UPathfindingComponent* PathfindingComp = Cast<UPathfindingComponent>(SelectedObject.Get()))
//			{
//				PathfindingComp->RequestPathToTargetActor();
//			}
//		}
//		return FReply::Handled();
//	};
//	
//	DebuggingCategory.AddCustomRow(FText::FromString(TEXT("Request Path"))).NameContent()
//	[
//		SNew(STextBlock)
//		.Text(FText::FromString(TEXT("Execute Pathfinding")))
//		.Font(IDetailLayoutBuilder::GetDetailFont())
//	].ValueContent()
//	[
//		SNew(SButton)
//		.Text(FText::FromString(TEXT("Request Path to Target")))
//		.OnClicked_Lambda(OnRequestPathClicked) 
//	];
//
//	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, bDrawDebugPath));
	
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
		.HAlign(HAlign_Center)
		.VAlign(VAlign_Center)
		.OnClicked_Lambda([]() -> FReply // Lambda for button click event
		{
			// Invoke the Pathfinding Tuner tab
			FGlobalTabmanager::Get()->TryInvokeTab(PathfindingTunerTabName);
			return FReply::Handled();
		})
	];
}