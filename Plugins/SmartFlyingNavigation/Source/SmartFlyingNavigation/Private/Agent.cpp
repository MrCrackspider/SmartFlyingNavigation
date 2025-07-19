// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Agent.h"
#include "PathfindingComponent.h"

AAgent::AAgent()
{
	PrimaryActorTick.bCanEverTick = true;
	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("Root")));
    PathfindingComponent = CreateDefaultSubobject<UPathfindingComponent>(TEXT("PathfindingComponent"));
}

void AAgent::BeginPlay()
{
	Super::BeginPlay();
	// ExecuteFindPath();
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
    	PathfindingComponent->RequestPathToTargetActor();
    }
}