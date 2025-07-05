// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PathActor.generated.h"

class USplineComponent;

UCLASS()
class EPICPROJECT_3DNAVI_API APathActor : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	APathActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	USplineComponent* GetSplineComponent();

private:
	UPROPERTY()
	USplineComponent* Spline;
};
