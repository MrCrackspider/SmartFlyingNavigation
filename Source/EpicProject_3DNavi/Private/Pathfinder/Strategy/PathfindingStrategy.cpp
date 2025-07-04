// Fill out your copyright notice in the Description page of Project Settings.

#include "Pathfinder/Strategy/PathfindingStrategy.h"

void UPathfindingStrategy::Initialize(AVoxelVolumeBase* InVolume)
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

bool UPathfindingStrategy::FindPathWithStrategy(const FIntVector& StartIndex, const FIntVector& EndIndex,
                                                int32 AgentRadiusInVoxels, TArray<FIntVector>& OutPath)
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
