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

bool UPathfindingStrategy::FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath)
{
	return false;
}

bool UPathfindingStrategy::ShouldRepathForOptimization(const FIntVector& CurrentAgentVoxel,
                                                       const TArray<FIntVector>& CurrentPathVoxels, int32 CurrentPathIndex, const FIntVector& GoalVoxel,
                                                       uint32 AgentRadiusVoxels) const
{
	return false;
}


FName UPathfindingStrategy::GetStrategyName() const
{
	return NAME_None;
}
