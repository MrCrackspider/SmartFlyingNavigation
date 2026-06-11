// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DynamicObstacleComponent.generated.h"

/**
 * Marks the owning actor as a dynamic navigation obstacle.
 *
 * On every tick where the owner moved, the component pushes an analytic primitive
 * (sphere for a sphere root, AABB box otherwise) into the world-level
 * FDynamicObstacleRegistry. A* path requests and the ORCA avoidance solver both
 * consume that single registry.
 */
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class SMARTFLYINGNAVIGATION_API UDynamicObstacleComponent  : public UActorComponent
{
	GENERATED_BODY()

public:
	UDynamicObstacleComponent();

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

public:
	FBox GetCurrentBounds() const;

	/** Estimated velocity of this obstacle (computed from position delta). */
	FORCEINLINE FVector GetVelocity() const { return Velocity; }

	/** Effective radius for avoidance (half of max AABB extent). */
	FORCEINLINE float GetAvoidanceRadius() const { return AvoidanceRadius; }

	/** World position (center of bounds). */
	FORCEINLINE FVector GetObstaclePosition() const { return GetOwner() ? GetOwner()->GetActorLocation() : FVector::ZeroVector; }

private:
	/** Pushes the current primitive shape into the world obstacle registry. */
	void PushToRegistry();

	/** Removes this obstacle from the world registry. */
	void RemoveFromRegistry();

	FTransform LastKnownTransform;

	/** Velocity estimated from position changes between ticks. */
	FVector Velocity = FVector::ZeroVector;

	/** Previous tick position for velocity computation. */
	FVector PreviousPosition = FVector::ZeroVector;

	/** Effective avoidance radius computed from bounds. */
	float AvoidanceRadius = 0.f;
};
