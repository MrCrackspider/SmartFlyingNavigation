// Fill out your copyright notice in the Description page of Project Settings.


#include "GameMode/EpicGameMode.h"
#include "Pathfinder/Subsystem/PathfindingSubsystem.h"
#include "Engine/Engine.h" // GEngine를 위해 필요
#include "Engine/GameInstance.h"

void AEpicGameMode::BeginPlay()
{
    UGameInstance* GameInstance = GetGameInstance();
    UWorld* World = GameInstance->GetWorld();
    if (!GameInstance)
    {
        UE_LOG(LogTemp, Error, TEXT("World is null!"));
        return;
    }

    
    //UPathfindingSubsystem* PathSubsystem = GameInstance->GetSubsystem<UPathfindingSubsystem>();
    UPathfindingSubsystem* PathfindingSubsystem = GEngine->GetEngineSubsystem<UPathfindingSubsystem>();
    if (!PathfindingSubsystem)
    {
        UE_LOG(LogTemp, Error, TEXT("PathfindingSubsystem is null!"));
        return;
    }
}
