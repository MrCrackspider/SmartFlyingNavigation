// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "PathfindingStrategy.h"
#include "UObject/WeakObjectPtr.h"
#include "JPSStrategy.generated.h"

class AVoxelVolume;
struct FPathNode;

/**
 * 
 */
UCLASS()
class EPICPROJECT_3DNAVI_API UJPSStrategy: public UPathfindingStrategy
{
    GENERATED_BODY()

public:
    virtual void Initialize(AVoxelVolume* InVolume) override;

    virtual bool FindPathWithStrategy(
        const FIntVector& StartIndex,
        const FIntVector& EndIndex,
        TArray<FIntVector>& OutPath
    ) override;

    virtual FName GetStrategyName() const override;


public:
    bool JumpPointSearch(
        const FIntVector& StartIndex,
        const FIntVector& EndIndex,
        TArray<FIntVector>& OutPath
    );

    TArray<FIntVector> IdentifySuccessors(const TSharedPtr<FPathNode>& Node, const FIntVector& Goal) const;

    void RetracePath(TSharedRef<FPathNode> EndNode, TArray<FIntVector>& OutPath);

private:
    TArray<FIntVector> FindPrunedNeighbors(const TSharedPtr<FPathNode>& Node) const;

    TOptional<FIntVector> Jump(FIntVector Current, const FIntVector& Direction, const FIntVector& Goal) const;

    TWeakObjectPtr<AVoxelVolume> Volume; 

};
