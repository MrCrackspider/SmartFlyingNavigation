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

#include "Framework/Application/SlateApplication.h" 
#include "Widgets/Docking/SDockTab.h"
#include "PathfindingTunerTabName.h"

#define LOCTEXT_NAMESPACE "FNav3DPathfindingDetails"

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

	CoreCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, TargetActor))
		.ToolTip(LOCTEXT("TargetActor_Tooltip", "The actor that this agent will attempt to move towards. Its location is used as the pathfinding destination."));

	CoreCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, AcceptanceRadius))
		.ToolTip(LOCTEXT("AcceptanceRadius_Tooltip", "The distance from a waypoint or the final destination at which it is considered 'reached'. Prevents the agent from circling its target."));

	AgentCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, BaseAgentRadius))
		.ToolTip(LOCTEXT("BaseAgentRadius_Tooltip", "The agent's fundamental radius, assuming its scale is 1.0. The actual pathfinding radius is this value multiplied by the actor's scale."));

	AgentCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, AgentRadius))
		.DisplayName(LOCTEXT("AgentRadiusLabel", "Pathfinding Radius (Scaled)"))
		.ToolTip(LOCTEXT("AgentRadius_Tooltip", "The final, calculated radius used for pathfinding (BaseAgentRadius * Actor Scale). This is a read-only value for debugging."));

	AgentCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, AvoidanceRadius))
			.ToolTip(LOCTEXT("AvoidanceRadius_Tooltip", "The 'personal space' radius for real-time dynamic avoidance. For predictable results, it is recommended to set this value equal to the 'Pathfinding Radius (Scaled)'."));
	
	MovementCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, MovementSpeed))
		.ToolTip(LOCTEXT("MovementSpeed_Tooltip", "The target speed (in UU/s) at which the agent moves along its calculated path."));

	MovementCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RotationInterpSpeed))
		.ToolTip(LOCTEXT("RotationInterpSpeed_Tooltip", "Controls how quickly the agent rotates to face its direction of travel. Higher values result in sharper turns."));

	MovementCategory.AddProperty(MovementModeHandle)
		.ToolTip(LOCTEXT("MovementMode_Tooltip", "Read-Only: Shows the current movement strategy (Spline or Linear). This is determined automatically by the component for optimal, safe movement."));

	MovementCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PartialPointSplineNum))
		.ToolTip(LOCTEXT("PartialPointSplineNum_Tooltip", "The number of future path points to use when generating the spline for smooth following."));

	RepathingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, bPeriodicRepath))
		.ToolTip(LOCTEXT("bPeriodicRepath_Tooltip", "If true, the agent will automatically request a new path on a timer, defined by Repath Cooldown. Essential for tracking moving targets."));

	RepathingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, RepathCooldown))
		.ToolTip(LOCTEXT("RepathCooldown_Tooltip", "The time in seconds to wait between automatic path requests when bPeriodicRepath is enabled."));

	RepathingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, GoalUpdateThreshold))
		.ToolTip(LOCTEXT("GoalUpdateThreshold_Tooltip", "Triggers a new path request only if the target has moved more than this distance (in cm) since the last path was calculated. A performance-saving alternative to periodic repathing."));
	
	// --- Debugging & Tools Category ---
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, bDrawDebugPath))
		.ToolTip(LOCTEXT("bDrawDebugPath_Tooltip", "If true, the agent's calculated path will be visualized in the world."));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, Duration))
		.ToolTip(LOCTEXT("Duration_Tooltip", "How long (in seconds) the debug path visualization persists in the viewport."));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, LineThickness))
		.ToolTip(LOCTEXT("LineThickness_Tooltip", "The thickness of the debug path lines."));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, BoxThickness))
		.ToolTip(LOCTEXT("BoxThickness_Tooltip", "The thickness of the outline for the debug boxes at each waypoint."));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, BoxHalfSize))
		.ToolTip(LOCTEXT("BoxHalfSize_Tooltip", "The size of the debug boxes drawn at each waypoint."));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PathColor))
		.ToolTip(LOCTEXT("PathColor_Tooltip", "The color of the debug path lines connecting waypoints."));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, PathPointColor))
		.ToolTip(LOCTEXT("PathPointColor_Tooltip", "The color of the debug boxes at each waypoint."));
	
	// --- 상태 변수 (Read-Only) ---
	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, CurrentState))
		.ToolTip(LOCTEXT("CurrentState_Tooltip", "Read-Only: Shows the agent's current high-level behavior (e.g., Idle, FollowPath)."));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, CurrentTargetLocation))
		.ToolTip(LOCTEXT("CurrentTargetLocation_Tooltip", "Read-Only: The world location the agent is currently trying to reach."));

	DebuggingCategory.AddProperty(GET_MEMBER_NAME_CHECKED(UPathfindingComponent, FoundPath))
		.ToolTip(LOCTEXT("FoundPath_Tooltip", "Read-Only: The array of FVectors representing the waypoints of the current path."));
	
	DetailBuilder.HideCategory("Pathfinding|Configuration");
	DetailBuilder.HideCategory("Pathfinding|Dynamic Repath");
	DetailBuilder.HideCategory("Pathfinding|Spline");
	DetailBuilder.HideCategory("Pathfinding|Dynamic Goal");
	DetailBuilder.HideCategory("Pathfinding|Optimization");
	DetailBuilder.HideCategory("Pathfinding|State");
	DetailBuilder.HideCategory("Pathfinding");
}
#undef LOCTEXT_NAMESPACE