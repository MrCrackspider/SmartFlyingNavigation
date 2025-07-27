// Copyright © 2025 TeamHamDev. All Rights Reserved.
#pragma once

#include "CoreMinimal.h"
#include "BehaviorTree/BTService.h"
#include "BTService_UpdateTargetLocation.generated.h"

/**
 * 
 */
UCLASS()
class SMARTFLYINGNAVIGATION_API UBTService_UpdateTargetLocation : public UBTService
{
	GENERATED_BODY()

public:
	UBTService_UpdateTargetLocation();
	
protected:
	virtual void TickNode(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;

public:
	/** 위치를 가져올 원본 액터가 저장된 블랙보드 키입니다. (입력) */
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector TargetActorKey;

	/** 위치 벡터를 저장할 블랙보드 키입니다. (출력) */
	UPROPERTY(EditAnywhere, Category = "Blackboard")
	FBlackboardKeySelector TargetLocationKey;
};
