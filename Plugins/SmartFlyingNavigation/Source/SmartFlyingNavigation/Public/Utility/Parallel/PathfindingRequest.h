// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once
#include "CoreMinimal.h"
#include "NavCore/OctreePathfindingData.h"
#include "Utility/Parallel/MemoryPool.h"
#include "PathfindingRequest.generated.h"

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
	FVector StartLocation = FVector::ZeroVector;

	UPROPERTY()
	FVector EndLocation = FVector::ZeroVector;
	
	UPROPERTY()
	int32 AgentRadius = 0;

	UPROPERTY()
	int32 Priority = 0;

	UPROPERTY()
	TWeakObjectPtr<UObject> RequestingObject;
	
	FWorldPathfindingResultDelegate OnWorldPathComplete;

	FOctreePathfindingData NavigationData;
	
	FPathNodeMemoryPoolBase* MemoryPool = nullptr;

	EPathfindingNodeType NodeType;
};