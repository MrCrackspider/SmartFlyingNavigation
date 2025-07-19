// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Agent.generated.h"

class UPathfindingComponent;

UCLASS()
class SMARTFLYINGNAVIGATION_API AAgent : public AActor
{
	GENERATED_BODY()
	
public:	
	AAgent();
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;


protected:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Pathfinding")
	TObjectPtr<UPathfindingComponent> PathfindingComponent;
	
	void ExecuteFindPath();
};
