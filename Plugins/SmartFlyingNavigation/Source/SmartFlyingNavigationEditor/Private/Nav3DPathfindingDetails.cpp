// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Nav3DPathfindingDetails.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h" 
#include "PropertyEditorModule.h"
#include "Component/PathfindingComponent.h"

#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"
#include "Editor.h" 
#include "IPropertyUtilities.h"
#include "Framework/Application/SlateApplication.h" 
#include "Widgets/Docking/SDockTab.h"
#include "PathfindingTunerTabName.h"

TSharedRef<IDetailCustomization> FNav3DPathfindingDetails::MakeInstance()
{
	return MakeShareable(new FNav3DPathfindingDetails);
}

void FNav3DPathfindingDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	IDetailCategoryBuilder& CoreCategory = DetailBuilder.EditCategory("Core Configuration", LOCTEXT("CoreConfigCategory", "Core Configuration"), ECategoryPriority::Important);
	IDetailCategoryBuilder& AgentCategory = DetailBuilder.EditCategory("Agent Properties", LOCTEXT("AgentPropsCategory", "Agent Properties"), ECategoryPriority::Important);
	IDetailCategoryBuilder& MovementCategory = DetailBuilder.EditCategory("Movement", LOCTEXT("MovementCategory", "Movement"), ECategoryPriority::Important);
	IDetailCategoryBuilder& RepathingCategory = DetailBuilder.EditCategory("Dynamic Pathfinding", LOCTEXT("RepathingCategory", "Dynamic Pathfinding"), ECategoryPriority::Important);
	IDetailCategoryBuilder& OptimizationCategory = DetailBuilder.EditCategory("Optimization", LOCTEXT("OptimizationCategory", "Optimization"), ECategoryPriority::Important);
	IDetailCategoryBuilder& DebuggingCategory = DetailBuilder.EditCategory("Debugging & Tools", LOCTEXT("DebuggingCategory", "Debugging & Tools"), ECategoryPriority::Important);
	
	TSharedPtr<IPropertyHandle> MovementModeHandle = DetailBuilder.GetProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, MovementMode));

	CoreCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, TargetActor));
	CoreCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PathfindingStrategyClass));

	AgentCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, BaseAgentRadius));
	AgentCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, AgentRadius)).DisplayName(LOCTEXT("AgentRadiusLabel", "Pathfinding Radius (Scaled)"));
	AgentCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, AvoidanceRadius));

	MovementCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, MovementSpeed));
	MovementCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RotationInterpSpeed));
	MovementCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RotationAxisLock));
	MovementCategory.AddProperty(MovementModeHandle);
	MovementCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PartialPointSplineNum));

	IDetailPropertyRow& SafetyCheckStepSizeRow = MovementCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, SafetyCheckStepSize));

	SafetyCheckStepSizeRow.Visibility(TAttribute<EVisibility>::Create(
		[MovementModeHandle]() -> EVisibility
		{
			if (!MovementModeHandle.IsValid() || !MovementModeHandle->IsValidHandle())
			{
				return EVisibility::Collapsed;
			}
			
			TArray<UObject*> OuterObjects;
			MovementModeHandle->GetOuterObjects(OuterObjects);
			if (OuterObjects.Num() == 0 || !IsValid(OuterObjects[0]))
			{
				return EVisibility::Collapsed;
			}
			
			uint8 MovementModeValue;
			if (MovementModeHandle->GetValue(MovementModeValue) == FPropertyAccess::Success)
			{
				if (static_cast<EPathMovementMode>(MovementModeValue) == EPathMovementMode::Spline)
				{
					return EVisibility::Visible;
				}
			}
			return EVisibility::Collapsed;
		}
	));

	RepathingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, bPeriodicRepath));
	RepathingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RepathCooldown));
	RepathingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, GoalUpdateThreshold));
	RepathingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PathValidationLookAhead));

	OptimizationCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PathOptimizationCheckInterval));
	OptimizationCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, OptimizationHeuristicFactor));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, bDrawDebugPath));
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, CurrentState));
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, CurrentTargetLocation));
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, FoundPath));
	
	DebuggingCategory.AddCustomRow(LOCTEXT("OpenPathfindingTunerFilter", "Open Pathfinding Tuner"))
	.ValueContent()
	.HAlign(HAlign_Center)
	[
		SNew(SButton)
		.Text(LOCTEXT("OpenPathfindingTunerButton", "Open Pathfinding Tuner"))
		.OnClicked_Lambda([]() -> FReply
		{
			FGlobalTabmanager::Get()->TryInvokeTab(PathfindingTunerTabName);
			return FReply::Handled();
		})
	];
	
	DetailBuilder.HideCategory("Pathfinding|Configuration");
	DetailBuilder.HideCategory("Pathfinding|Dynamic Repath");
	DetailBuilder.HideCategory("Pathfinding|Spline");
	DetailBuilder.HideCategory("Pathfinding|Dynamic Goal");
	DetailBuilder.HideCategory("Pathfinding|Optimization");
	DetailBuilder.HideCategory("Pathfinding|State");
	DetailBuilder.HideCategory("Pathfinding");
}