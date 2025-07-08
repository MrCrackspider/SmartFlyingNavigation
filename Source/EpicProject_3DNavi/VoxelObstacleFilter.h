// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameplayTagContainer.h"
#include "Engine/DataAsset.h"
#include "VoxelObstacleFilter.generated.h"

// Enum to define how filter criteria are combined
UENUM(BlueprintType)
enum class EObstacleFilterLogic : uint8
{
	// Match if any of the criteria are met (OR)
	MatchAny,
	// Match only if all criteria are met (AND)
	MatchAll
};


UCLASS(BlueprintType, Blueprintable, EditInlineNew, meta=(DisplayName="Voxel Obstacle Filter"))
class EPICPROJECT_3DNAVI_API UVoxelObstacleFilter : public UDataAsset
{
	GENERATED_BODY()

public:
	// The logic used to combine the filtering rules.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Filter Logic")
	EObstacleFilterLogic Logic = EObstacleFilterLogic::MatchAny;

	// 1. Filter by collision channel
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Filter Criteria")
	TArray<TEnumAsByte<ECollisionChannel>> CollisionChannels;

	// 2. Filter by actor tag
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Filter Criteria")
	TArray<FName> ActorTags;

	// 3. Filter by the presence of a specific component
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Filter Criteria")
	TSubclassOf<UActorComponent> MarkerComponentClass;

	UFUNCTION(BlueprintCallable, Category="Filter")
	bool DoesActorMatch(const AActor* InActor, const UPrimitiveComponent* InPrimitiveComponent) const;
};
