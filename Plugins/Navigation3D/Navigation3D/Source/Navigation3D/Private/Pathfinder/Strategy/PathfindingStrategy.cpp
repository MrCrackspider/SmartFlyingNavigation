// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/Strategy/PathfindingStrategy.h"

#include "Pathfinder/PathfindingRequest.h"

void UPathfindingStrategy::Initialize()
{
}

bool UPathfindingStrategy::FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FIntVector>& OutPath)
{
	return false;
}

bool UPathfindingStrategy::ShouldRepathForOptimization(const FVector& CurrentAgentLocation,
                                                       const TArray<FVector>& CurrentPath, int32 CurrentPathIndex,
                                                       const FVector& GoalLocation, uint32 AgentRadius) const
{
	return false;
}

FName UPathfindingStrategy::GetStrategyName() const
{
	return NAME_None;
}
