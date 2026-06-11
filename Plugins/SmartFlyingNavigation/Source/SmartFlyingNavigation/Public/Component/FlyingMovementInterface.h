// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "FlyingMovementInterface.generated.h"

UINTERFACE(MinimalAPI, Blueprintable)
class UFlyingMovementExecutor : public UInterface
{
	GENERATED_BODY()
};

/**
 * Interface for custom movement execution.
 * Implement this on any UObject to override the default AddActorWorldOffset-based movement
 * in UPathfindingComponent. Assign your implementation to PathfindingComponent::MovementExecutor.
 */
class IFlyingMovementExecutor
{
	GENERATED_BODY()

public:
	/**
	 * Called each frame to execute movement with the given velocity.
	 * The implementor is responsible for actually moving the actor.
	 *
	 * @param Velocity The desired movement velocity (units/second).
	 * @param DeltaTime Frame delta time.
	 * @param FacingDirection Normalized direction the agent should face.
	 */
	UFUNCTION(BlueprintNativeEvent, Category = "Pathfinding|Movement")
	void ExecuteMovement(const FVector& Velocity, float DeltaTime, const FVector& FacingDirection);
};
