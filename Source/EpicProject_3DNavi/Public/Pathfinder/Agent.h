// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Agent.generated.h"

class UPathfindingComponent;

UCLASS()
class EPICPROJECT_3DNAVI_API AAgent : public AActor
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
	
	UFUNCTION(CallInEditor, Category = "Pathfinding|Test")
	void ExecuteFindPath();
};
