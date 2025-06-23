// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NaviController.generated.h"

class ANavigationVolume;

UCLASS()
class EPICPROJECT_3DNAVI_API ANaviController : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ANaviController();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void MoveToLocation(FVector ToLocation);

	UPROPERTY(EditAnywhere, Category = "PathFinding")
	ANavigationVolume* NaviVolume;

	UPROPERTY(EditAnywhere, Category = "PathFinding")
	AActor* GoalActor;
};
