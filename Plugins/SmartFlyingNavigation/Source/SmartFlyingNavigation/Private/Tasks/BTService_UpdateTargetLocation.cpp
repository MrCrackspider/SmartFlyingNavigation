// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Tasks/BTService_UpdateTargetLocation.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "GameFramework/Actor.h"

UBTService_UpdateTargetLocation::UBTService_UpdateTargetLocation()
{
	NodeName = "Update Target Location";
}

void UBTService_UpdateTargetLocation::TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	Super::TickNode(OwnerComp, NodeMemory, DeltaSeconds);

	UBlackboardComponent* BlackboardComp = OwnerComp.GetBlackboardComponent();
	if (!BlackboardComp) return;

	// 1. 블랙보드에서 TargetActor를 가져옵니다.
	AActor* Target = Cast<AActor>(BlackboardComp->GetValueAsObject(TargetActorKey.SelectedKeyName));
	
	if (Target)
	{
		// 2. 타겟이 유효하면, 그 액터의 현재 위치를 TargetLocation 키에 업데이트합니다.
		BlackboardComp->SetValueAsVector(TargetLocationKey.SelectedKeyName, Target->GetActorLocation());
	}
	else
	{
		// 3. 타겟이 없다면(null), TargetLocation 키를 비웁니다.
		BlackboardComp->ClearValue(TargetLocationKey.SelectedKeyName);
	}
}