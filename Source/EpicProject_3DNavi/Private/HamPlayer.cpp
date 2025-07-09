// Fill out your copyright notice in the Description page of Project Settings.


#include "HamPlayer.h"

#include "EngineUtils.h"
#include "Pathfinder/PathfindingComponent.h"
#include "Pathfinder/Agent.h"

// Sets default values
AHamPlayer::AHamPlayer()
{
	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AHamPlayer::BeginPlay()
{
	Super::BeginPlay();

	//테스트용
	for (TActorIterator<AAgent> It(GetWorld()); It; ++It)
	{
		AAgent* AICharacter = *It;
		if (AICharacter)
		{
			if (UPathfindingComponent* PathComp = AICharacter->FindComponentByClass<UPathfindingComponent>())
			{
				PathComp->SetTargetActor(this);
				PathComp->RequestPathToTargetActor();
			}
		}
	}
}

// Called every frame
void AHamPlayer::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// Called to bind functionality to input
void AHamPlayer::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}
