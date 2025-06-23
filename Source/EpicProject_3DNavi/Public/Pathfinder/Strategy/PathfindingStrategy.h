// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "PathfindingStrategy.generated.h"

class UPathfindingSubsystem;
class AVoxelVolume;
/**
 * 
 */
UCLASS(Abstract)
class EPICPROJECT_3DNAVI_API UPathfindingStrategy : public UObject
{
	GENERATED_BODY()

public:
    virtual void Initialize(AVoxelVolume* InVolume);

    virtual bool FindPathWithStrategy(
        const FIntVector& StartIndex,
        const FIntVector& EndIndex,
        TArray<FIntVector>& OutPath
    );

    virtual FName GetStrategyName() const;
};
