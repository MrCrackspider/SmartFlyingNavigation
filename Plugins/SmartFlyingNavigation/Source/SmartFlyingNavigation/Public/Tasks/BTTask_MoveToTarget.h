// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/Tasks/BTTask_BlackboardBase.h"
#include "BTTask_MoveToTarget.generated.h"

/**
 * 
 */
UCLASS()
class SMARTFLYINGNAVIGATION_API UBTTask_MoveToTarget : public UBTTask_BlackboardBase
{
	GENERATED_BODY()

public:
	UBTTask_MoveToTarget();

	UPROPERTY(EditAnywhere, Category = "AI", meta = (ClampMin = "0.0"))
	float AcceptanceRadius = 150.f;
	
protected:
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual EBTNodeResult::Type AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
};
