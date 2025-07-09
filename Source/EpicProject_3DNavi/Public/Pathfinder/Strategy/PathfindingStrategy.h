// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/PathfindingRequest.h"
#include "UObject/NoExportTypes.h"
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
	Failed_Timeout
};

/**
 * 
 */
UCLASS(Abstract)
class EPICPROJECT_3DNAVI_API UPathfindingStrategy : public UObject
{
	GENERATED_BODY()

public:
	virtual void Initialize(AVoxelVolumeBase* InVolume);

	virtual bool FindPathWithStrategy(const FPathfindingRequest& InRequest, TArray<FIntVector>& OutPath);
	
	virtual bool ShouldRepathForOptimization(const FVector& CurrentAgentLocation, const TArray<FVector>& CurrentPath,
	                                         int32 CurrentPathIndex, const FVector& GoalLocation,
	                                         uint32 AgentRadius) const;
	virtual FName GetStrategyName() const;
};
