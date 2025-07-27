// Copyright © 2025 TeamHamDev. All Rights Reserved.


#include "Tasks/BTTask_MoveToTarget.h"
#include "AIController.h"
#include "GameFramework/Pawn.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Component/PathfindingComponent.h"


UBTTask_MoveToTarget::UBTTask_MoveToTarget()
{
	NodeName = "Move To Target Location";
	bNotifyTick = true;
	BlackboardKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_MoveToTarget, BlackboardKey));
}

EBTNodeResult::Type UBTTask_MoveToTarget::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (!AIController) return EBTNodeResult::Failed;

	APawn* ControlledPawn = AIController->GetPawn();
	if (!ControlledPawn) return EBTNodeResult::Failed;

	UPathfindingComponent* PathComp = ControlledPawn->FindComponentByClass<UPathfindingComponent>();
	if (!PathComp) return EBTNodeResult::Failed;

	// 블랙보드에서 목표 위치(Vector)를 가져옵니다.
	const FVector TargetLocation = OwnerComp.GetBlackboardComponent()->GetValueAsVector(BlackboardKey.SelectedKeyName);

	if (TargetLocation == FAISystem::InvalidLocation)
	{
		return EBTNodeResult::Failed;
	}

	// 경로 탐색을 요청합니다.
	if (PathComp->RequestPathToLocation(TargetLocation))
	{
		return EBTNodeResult::InProgress;
	}

	return EBTNodeResult::Failed;
}

void UBTTask_MoveToTarget::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	APawn* ControlledPawn = OwnerComp.GetAIOwner()->GetPawn();
	if (!ControlledPawn)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}
	
	const FVector TargetLocation = OwnerComp.GetBlackboardComponent()->GetValueAsVector(BlackboardKey.SelectedKeyName);
	if (TargetLocation == FAISystem::InvalidLocation)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}

	const float DistSq = FVector::DistSquared(ControlledPawn->GetActorLocation(), TargetLocation);

	// 조건 1: 최종 목표의 반경 안에 들어왔는가?
	if (DistSq <= FMath::Square(AcceptanceRadius))
	{
		if (UPathfindingComponent* PathComp = ControlledPawn->FindComponentByClass<UPathfindingComponent>())
		{
			PathComp->StopMovement();
		}
		FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
		return;
	}


	if (UPathfindingComponent* PathComp = ControlledPawn->FindComponentByClass<UPathfindingComponent>())
	{
		if (PathComp->HasReachedDestination())
		{
			//FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
		}
	}
}

EBTNodeResult::Type UBTTask_MoveToTarget::AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	AAIController* AIController = OwnerComp.GetAIOwner();
	if (AIController && AIController->GetPawn())
	{
		if (UPathfindingComponent* PathComp = AIController->GetPawn()->FindComponentByClass<UPathfindingComponent>())
		{
			PathComp->StopMovement();
		}
	}
	
	return EBTNodeResult::Aborted;
}