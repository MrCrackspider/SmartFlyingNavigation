// Copyright © 2025 TeamHamDev. All Rights Reserved.


#include "Tasks/BTService_FindChaseTarget.h"
#include "AIController.h"
#include "GameFramework/Pawn.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Kismet/GameplayStatics.h"


UBTService_FindChaseTarget::UBTService_FindChaseTarget()
{
	NodeName = "Find Chase Target";
	Interval = 0.2f;
	RandomDeviation = 0.05f;
}

void UBTService_FindChaseTarget::TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	Super::TickNode(OwnerComp, NodeMemory, DeltaSeconds);

	APawn* ControlledPawn = OwnerComp.GetAIOwner() ? OwnerComp.GetAIOwner()->GetPawn() : nullptr;
	if (!ControlledPawn) return;

	TArray<AActor*> FoundActors;
	if (TargetActorClass)
	{
		UGameplayStatics::GetAllActorsOfClass(ControlledPawn->GetWorld(), TargetActorClass, FoundActors);
	}
	else if (!TargetTag.IsNone())
	{
		UGameplayStatics::GetAllActorsWithTag(ControlledPawn->GetWorld(), TargetTag, FoundActors);
	}
	else
	{
		return;
	}

	AActor* ClosestTarget = nullptr;
	float MinDistanceSq = ChaseRadius * ChaseRadius;

	for (AActor* PotentialTarget : FoundActors)
	{
		if (!PotentialTarget || PotentialTarget == ControlledPawn) continue;

		const float DistanceSq = FVector::DistSquared(ControlledPawn->GetActorLocation(), PotentialTarget->GetActorLocation());
		if (DistanceSq < MinDistanceSq)
		{
			// TODO: 여기에 시야(Line of Sight) 체크 로직을 추가하면 좋습니다.
			MinDistanceSq = DistanceSq;
			ClosestTarget = PotentialTarget;
		}
	}

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	
	if (ClosestTarget)
	{
		BlackboardComp->SetValueAsObject(BlackboardKey.SelectedKeyName, ClosestTarget);
	}
	else
	{
		BlackboardComp->ClearValue(BlackboardKey.SelectedKeyName);
	}
}