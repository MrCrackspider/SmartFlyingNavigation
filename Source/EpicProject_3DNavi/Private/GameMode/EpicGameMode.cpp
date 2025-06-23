// Fill out your copyright notice in the Description page of Project Settings.


#include "GameMode/EpicGameMode.h"
#include "Pathfinder/PathfindingSubsystem.h"
#include "Pathfinder/VoxelVolume.h"
#include "Engine/GameInstance.h"

void AEpicGameMode::BeginPlay()
{
    UGameInstance* GameInstance = GetGameInstance();
    if (!GameInstance)
    {
        UE_LOG(LogTemp, Error, TEXT("GameInstance is null!"));
        return;
    }

    
    UPathfindingSubsystem* PathSubsystem = GameInstance->GetSubsystem<UPathfindingSubsystem>();
    if (!PathSubsystem)
    {
        UE_LOG(LogTemp, Error, TEXT("PathfindingSubsystem is null!"));
        return;
    }

    PathSubsystem->RenderVoxelVolume(FLinearColor::White, FLinearColor::Red);
}
