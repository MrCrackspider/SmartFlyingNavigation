// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/Strategy/PathfindingStrategy.h"

void UPathfindingStrategy::Initialize(AVoxelVolume* InVolume)
{
}

bool UPathfindingStrategy::FindPathWithStrategy(
    const FIntVector& StartIndex,
    const FIntVector& EndIndex,
    TArray<FIntVector>& OutPath
)
{
    return false;
}

FName UPathfindingStrategy::GetStrategyName() const
{
	return NAME_None;
}