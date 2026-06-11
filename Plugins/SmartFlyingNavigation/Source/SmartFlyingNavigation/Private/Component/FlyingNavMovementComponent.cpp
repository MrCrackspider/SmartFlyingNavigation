// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Component/FlyingNavMovementComponent.h"
#include "GameFramework/Pawn.h"

UFlyingNavMovementComponent::UFlyingNavMovementComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UFlyingNavMovementComponent::SetDesiredVelocity(const FVector& InVelocity)
{
	DesiredVelocity = InVelocity.GetClampedToMaxSize(MaxSpeed);
	bHasDesiredVelocity = true;
}

void UFlyingNavMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!PawnOwner || !UpdatedComponent || ShouldSkipUpdate(DeltaTime))
	{
		return;
	}

	// Without a fresh command this frame, brake toward zero.
	const FVector TargetVelocity = bHasDesiredVelocity ? DesiredVelocity : FVector::ZeroVector;
	bHasDesiredVelocity = false;

	// Accelerate / decelerate toward the target velocity.
	const FVector VelocityDelta = TargetVelocity - Velocity;
	const float DeltaSize = VelocityDelta.Size();
	if (DeltaSize > KINDA_SMALL_NUMBER)
	{
		const bool bSpeedingUp = TargetVelocity.SizeSquared() >= Velocity.SizeSquared();
		const float Rate = (bSpeedingUp ? Acceleration : Deceleration) * DeltaTime;
		Velocity += VelocityDelta * FMath::Min(1.f, Rate / DeltaSize);
	}

	Velocity = Velocity.GetClampedToMaxSize(MaxSpeed);

	const FVector MoveDelta = Velocity * DeltaTime;
	if (!MoveDelta.IsNearlyZero())
	{
		FHitResult Hit;
		SafeMoveUpdatedComponent(MoveDelta, UpdatedComponent->GetComponentQuat(), true, Hit);

		if (Hit.IsValidBlockingHit())
		{
			// Slide along the surface with the remaining time, then once more for any
			// secondary surface (corner case), mirroring UFloatingPawnMovement.
			const float InitialTime = Hit.Time;
			SlideAlongSurface(MoveDelta, 1.f - Hit.Time, Hit.Normal, Hit, true);

			if (Hit.IsValidBlockingHit())
			{
				SlideAlongSurface(MoveDelta, 1.f - Hit.Time, Hit.Normal, Hit, true);
			}

			// Count a tick as blocked when almost no forward progress was possible.
			if (InitialTime < 0.05f && Hit.IsValidBlockingHit())
			{
				BlockedFrames++;
			}
			else
			{
				BlockedFrames = 0;
			}
		}
		else
		{
			BlockedFrames = 0;
		}
	}

	if (bOrientRotationToMovement && !Velocity.IsNearlyZero())
	{
		const FRotator CurrentRotation = UpdatedComponent->GetComponentRotation();
		const FRotator TargetRotation = Velocity.ToOrientationRotator();
		const FRotator NewRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime, RotationInterpSpeed);
		MoveUpdatedComponent(FVector::ZeroVector, NewRotation, false);
	}

	// Publish Velocity so GetVelocity(), animation, and physics interactions see real motion.
	UpdateComponentVelocity();
}
