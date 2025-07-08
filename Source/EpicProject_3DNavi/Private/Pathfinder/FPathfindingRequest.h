// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FPathfindingRequest.generated.h"

DECLARE_DYNAMIC_DELEGATE_TwoParams(FPathfindingResultDelegate, const TArray<FIntVector>&, Path, bool, bSuccess);

class UPathfindingComponent;
class UPathfindingStrategy;

USTRUCT(BlueprintType)
struct FPathfindingRequest
{
	GENERATED_BODY()

	UPROPERTY()
	int32 RequestID = -1;

	UPROPERTY()
	TWeakObjectPtr<UPathfindingComponent> RequestingComponent;

	UPROPERTY()
	TWeakObjectPtr<UPathfindingStrategy> Strategy;

	UPROPERTY()
	FIntVector StartIndex;

	UPROPERTY()
	FIntVector EndIndex;

	UPROPERTY()
	int32 AgentRadiusInVoxels = 0;

	UPROPERTY()
	int32 Priority = 0;
	
	TSharedPtr<FThreadSafeBool, ESPMode::ThreadSafe> bIsCancelled;
	FPathfindingResultDelegate OnPathfindingComplete;
};