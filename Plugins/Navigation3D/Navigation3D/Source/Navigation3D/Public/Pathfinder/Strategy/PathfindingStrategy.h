// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/Utility/Parallel/PathfindingRequest.h"
#include "PathfindingStrategy.generated.h"

class UPathfindingSubsystem;
class AVoxelVolumeBase;

UENUM(BlueprintType)
enum class EPathfindingResult : uint8
{
	Success,
	Success_Partial,
	Failed_Unreachable,
	Failed_InvalidStart,
	Failed_InvalidGoal,
	Failed_Timeout,
	Failed_OutOfMemory
};

/**
 * 
 */
UCLASS(Abstract)
class NAVIGATION3D_API UPathfindingStrategy : public UObject
{
	GENERATED_BODY()

public:
	virtual void Initialize();

	virtual bool FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FIntVector>& OutPath);
	virtual bool FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FVector>& OutPath);
	
	virtual bool ShouldRepathForOptimization(const FIntVector& CurrentAgentVoxel,
											 const TArray<FIntVector>& CurrentPathVoxels,
											 int32 CurrentPathIndex,
											 const FIntVector& GoalVoxel,
											 uint32 AgentRadiusVoxels) const;
	virtual FName GetStrategyName() const;
};
