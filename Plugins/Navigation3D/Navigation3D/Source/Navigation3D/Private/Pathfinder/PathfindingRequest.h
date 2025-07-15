// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Pathfinder/ThreadSafeBitmask.h"
#include "PathfindingRequest.generated.h"

DECLARE_DYNAMIC_DELEGATE_TwoParams(FPathfindingResultDelegate, const TArray<FIntVector>&, Path, bool, bSuccess);

class UPathfindingComponent;
class UPathfindingStrategy;
struct FPathNodeMemoryPool;
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

	/** 길찾기를 요청한 컴포넌트 또는 액터. 콜백 시 유효성 검사에 사용됩니다. */
	UPROPERTY()
	TWeakObjectPtr<UObject> RequestingObject;
	
	FPathfindingResultDelegate OnPathfindingComplete;

	FThreadSafeBitmaskGrid WalkabilityGrid;

	FPathNodeMemoryPool* MemoryPool = nullptr;
};