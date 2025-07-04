// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder/Agent.h"

#include "Components/SplineComponent.h"
#include "Pathfinder/PathfindingComponent.h"

AAgent::AAgent()
{
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("Root")));
    PathfindingComponent = CreateDefaultSubobject<UPathfindingComponent>(TEXT("PathfindingComponent"));
}

void AAgent::BeginPlay()
{
	Super::BeginPlay();
	ExecuteFindPath();
}

void AAgent::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AAgent::ExecuteFindPath()
{
    if (PathfindingComponent)
    {
	    UE_LOG(LogTemp, Log, TEXT("--- Executing Find Path Test ---"));
    	if (IsValid(TargetActor))
    	{
    		PathfindingComponent->FindPathToLocation(TargetActor->GetActorLocation());
    	}
	    else
	    {
		    UE_LOG(LogTemp, Error, TEXT("Agent: Target Actor is InValid!"));
	    }
    }
}