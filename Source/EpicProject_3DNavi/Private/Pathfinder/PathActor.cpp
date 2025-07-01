// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/PathActor.h"

#include "Components/SplineComponent.h"

// Sets default values
APathActor::APathActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	Spline = CreateDefaultSubobject<USplineComponent>(TEXT("Spline"));
	RootComponent = Spline;
}

// Called when the game starts or when spawned
void APathActor::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void APathActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

