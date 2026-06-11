// Copyright © 2025 TeamHamDev. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PawnMovementComponent.h"
#include "FlyingNavMovementComponent.generated.h"

/**
 * Velocity-driven flying movement for pathfinding agents.
 *
 * UPathfindingComponent (after avoidance) feeds a desired velocity each frame via
 * SetDesiredVelocity. This component accelerates toward it, sweeps the updated
 * component (SafeMoveUpdatedComponent + slide along blocking surfaces), and reports
 * Velocity through the standard movement-component channel — so AActor::GetVelocity(),
 * animation blueprints, and physics interactions all observe the real motion instead
 * of the teleport-style AddActorWorldOffset fallback.
 */
UCLASS(ClassGroup=(Movement), meta=(BlueprintSpawnableComponent))
class SMARTFLYINGNAVIGATION_API UFlyingNavMovementComponent : public UPawnMovementComponent
{
	GENERATED_BODY()

public:
	UFlyingNavMovementComponent();

	/**
	 * Authoritative velocity command for this frame (usually the avoidance-resolved
	 * pathfinding velocity). Decays to braking when not refreshed.
	 */
	UFUNCTION(BlueprintCallable, Category = "FlyingNav|Movement")
	void SetDesiredVelocity(const FVector& InVelocity);

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual float GetMaxSpeed() const override { return MaxSpeed; }

	/**
	 * True when this component can actually move the owner: active, with a valid
	 * UpdatedComponent and a Pawn owner (UPawnMovementComponent requires a Pawn).
	 */
	bool IsAbleToMove() const { return IsActive() && UpdatedComponent != nullptr && PawnOwner != nullptr; }

	/** Consecutive ticks the sweep was fully blocked; pathfinding polls this to trigger a repath. */
	UFUNCTION(BlueprintPure, Category = "FlyingNav|Movement")
	int32 GetBlockedFrameCount() const { return BlockedFrames; }

	UFUNCTION(BlueprintCallable, Category = "FlyingNav|Movement")
	void ResetBlockedFrameCount() { BlockedFrames = 0; }

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlyingNav|Movement", meta=(ClampMin="0.0"))
	float MaxSpeed = 1200.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlyingNav|Movement", meta=(ClampMin="0.0"))
	float Acceleration = 4000.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlyingNav|Movement", meta=(ClampMin="0.0"))
	float Deceleration = 8000.f;

	/** When true, the owner is rotated to face the movement direction. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlyingNav|Rotation")
	bool bOrientRotationToMovement = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlyingNav|Rotation", meta=(ClampMin="0.0"))
	float RotationInterpSpeed = 5.f;

private:
	FVector DesiredVelocity = FVector::ZeroVector;

	/** Set each time SetDesiredVelocity is called; cleared after one tick so stale commands brake. */
	bool bHasDesiredVelocity = false;

	int32 BlockedFrames = 0;
};
