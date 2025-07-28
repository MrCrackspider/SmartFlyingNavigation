// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PathfindingStreamingProbe.generated.h"

class UWorldPartitionStreamingSourceComponent;

UCLASS(NotBlueprintable, NotPlaceable, Transient)
class SMARTFLYINGNAVIGATION_API APathfindingStreamingProbe : public AActor
{
	GENERATED_BODY()

public:
	APathfindingStreamingProbe();

	void SetupStreamingSource(float Radius);

	/**
	 * Activates this probe at a specific world location.
	 * When this function is called, the streaming source becomes active and begins to load the world around the TargetLocation.
	 * @param TargetLocation The world coordinate that will trigger streaming.
	 */
	void ActivateProbe(const FVector& TargetLocation);

	/**
	 * Deactivates this probe.
	 * The streaming source is disabled, so this probe will no longer affect world loading.
	 */
	void DeactivateProbe();

private:
	UPROPERTY()
	TObjectPtr<UWorldPartitionStreamingSourceComponent> StreamingSource;
};