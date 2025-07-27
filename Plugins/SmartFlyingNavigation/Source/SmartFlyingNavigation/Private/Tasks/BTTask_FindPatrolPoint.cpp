// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Tasks/BTTask_FindPatrolPoint.h"
#include "AIController.h"
#include "GameFramework/Pawn.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "NavVolume/OctreeVoxelVolume.h"
#include "Component/PathfindingComponent.h"

UBTTask_FindPatrolPoint::UBTTask_FindPatrolPoint()
{
	NodeName = "Find Patrol Point (3D)";
	BlackboardKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_FindPatrolPoint, BlackboardKey));
}

EBTNodeResult::Type UBTTask_FindPatrolPoint::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	APawn* ControlledPawn = AIController ? AIController->GetPawn() : nullptr;
	if (!ControlledPawn)
	{
		return EBTNodeResult::Failed;
	}

	UPathfindingComponent* PathComp = ControlledPawn->FindComponentByClass<UPathfindingComponent>();
	AOctreeVoxelVolume* NavVolume = PathComp ? Cast<AOctreeVoxelVolume>(PathComp->GetTargetVolume()) : nullptr;
	if (!NavVolume)
	{
		UE_LOG(LogTemp, Error, TEXT("FindPatrolPoint: Navigation Volume is not a valid OctreeVoxelVolume."));
		return EBTNodeResult::Failed;
	}

	const FVector Origin = ControlledPawn->GetActorLocation();

	for (int32 i = 0; i < MaxAttempts; ++i)
	{
		const FVector RandomDirection = FMath::VRand();
		const float RandomDistance = FMath::FRandRange(PatrolRadius * 0.5f, PatrolRadius);
		const FVector PotentialPatrolPoint = Origin + RandomDirection * RandomDistance;

		if (!NavVolume->IsLocationInBlockedVoxel(PotentialPatrolPoint))
		{
			OwnerComp.GetBlackboardComponent()->SetValueAsVector(BlackboardKey.SelectedKeyName, PotentialPatrolPoint);
			return EBTNodeResult::Succeeded;
		}
	}
	
	UE_LOG(LogTemp, Warning, TEXT("FindPatrolPoint: Failed to find a valid patrol point after %d attempts."), MaxAttempts);
	return EBTNodeResult::Failed;
}