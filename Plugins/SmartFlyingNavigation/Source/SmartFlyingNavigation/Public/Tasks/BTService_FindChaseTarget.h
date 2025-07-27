// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/Services/BTService_BlackboardBase.h"
#include "BTService_FindChaseTarget.generated.h"

/**
 * 
 */
UCLASS()
class SMARTFLYINGNAVIGATION_API UBTService_FindChaseTarget : public UBTService_BlackboardBase
{
	GENERATED_BODY()

public:
	UBTService_FindChaseTarget();
	virtual void TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;

private:
	UPROPERTY(EditAnywhere, Category = "AI|Targeting")
	TSubclassOf<AActor> TargetActorClass;

	UPROPERTY(EditAnywhere, Category = "AI|Targeting")
	FName TargetTag;
	
	UPROPERTY(EditAnywhere, Category = "AI")
	float ChaseRadius = 5000.f;
};
