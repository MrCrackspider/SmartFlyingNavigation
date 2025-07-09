// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TestActor.generated.h"

class USplineComponent;

UCLASS()
class NAVIGATION3D_API ATestActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ATestActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	USplineComponent* PathSpline;
    
	// 경로 포인트를 받아 스플라인을 설정하는 함수
	void UpdateSpline(const TArray<FVector>& PathPoints);
	
public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
};