// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NaviCore/OctreePathfindingData.h"
#include "Pathfinder/ThreadSafeBitmask.h"
#include "Pathfinder/Utility/MemoryPool.h"
#include "PathfindingRequest.generated.h"

DECLARE_DYNAMIC_DELEGATE_TwoParams(FVoxelPathfindingResultDelegate, const TArray<FIntVector>&, Path, bool, bSuccess);
DECLARE_DYNAMIC_DELEGATE_TwoParams(FWorldPathfindingResultDelegate, const TArray<FVector>&, Path, bool, bSuccess);

class UPathfindingComponent;
class UPathfindingStrategy;
enum class EPathfindingNodeType : uint8;

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
	FVector StartLocation;

	UPROPERTY()
	FVector EndLocation;

	
	UPROPERTY()
	int32 AgentRadiusInVoxels = 0;

	UPROPERTY()
	int32 AgentRadius = 0;

	UPROPERTY()
	int32 Priority = 0;

	/** 길찾기를 요청한 컴포넌트 또는 액터. 콜백 시 유효성 검사에 사용됩니다. */
	UPROPERTY()
	TWeakObjectPtr<UObject> RequestingObject;
	
	FVoxelPathfindingResultDelegate OnVoxelPathComplete;
	FWorldPathfindingResultDelegate OnWorldPathComplete;

	TVariant<FThreadSafeBitmaskGrid, FOctreePathfindingData> NavigationData;
	
	FPathNodeMemoryPoolBase* MemoryPool = nullptr;

	EPathfindingNodeType NodeType;
};