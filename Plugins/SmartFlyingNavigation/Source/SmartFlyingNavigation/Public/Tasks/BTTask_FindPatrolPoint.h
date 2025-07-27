// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BehaviorTreeTypes.h"
#include "Runtime/AIModule/Classes/BehaviorTree/Tasks/BTTask_BlackboardBase.h"
#include "BTTask_FindPatrolPoint.generated.h"

/**
 * 
 */
UCLASS()
class SMARTFLYINGNAVIGATION_API UBTTask_FindPatrolPoint : public UBTTask_BlackboardBase
{
	GENERATED_BODY()

public:
	UBTTask_FindPatrolPoint();

protected:
	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;

private:

	UPROPERTY(EditAnywhere, Category = "AI")
	float PatrolRadius = 2000.f;

	/** 유효한 지점을 찾기 위해 시도할 최대 횟수입니다. */
	UPROPERTY(EditAnywhere, Category = "AI", meta = (ClampMin = "1"))
	int32 MaxAttempts = 10;
};