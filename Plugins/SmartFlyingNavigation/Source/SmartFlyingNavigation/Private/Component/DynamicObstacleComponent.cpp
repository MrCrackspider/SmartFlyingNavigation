// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Component/DynamicObstacleComponent.h"
#include "Components/SphereComponent.h"
#include "Subsystem/OctreeNavSubsystem.h"
#include "NavCore/DynamicObstacleRegistry.h"

UDynamicObstacleComponent::UDynamicObstacleComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void UDynamicObstacleComponent::BeginPlay()
{
	Super::BeginPlay();

	AActor* OwnerActor = GetOwner();
	if (!OwnerActor)
	{
		SetComponentTickEnabled(false);
		return;
	}

	LastKnownTransform = OwnerActor->GetActorTransform();
	PreviousPosition = OwnerActor->GetActorLocation();

	PushToRegistry();
}

void UDynamicObstacleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	RemoveFromRegistry();
	Super::EndPlay(EndPlayReason);
}

void UDynamicObstacleComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	const AActor* OwnerActor = GetOwner();
	if (!OwnerActor)
	{
		return;
	}

	const FVector CurrentPosition = OwnerActor->GetActorLocation();

	// Compute velocity from position delta.
	if (DeltaTime > KINDA_SMALL_NUMBER)
	{
		Velocity = (CurrentPosition - PreviousPosition) / DeltaTime;
	}
	PreviousPosition = CurrentPosition;

	// Update avoidance radius from current bounds.
	const FBox CurrentBounds = GetCurrentBounds();
	if (CurrentBounds.IsValid)
	{
		AvoidanceRadius = CurrentBounds.GetExtent().GetMax();
	}

	// Only push to the registry when the obstacle actually moved.
	const FTransform CurrentTransform = OwnerActor->GetActorTransform();
	if (!CurrentTransform.Equals(LastKnownTransform, 0.1f))
	{
		LastKnownTransform = CurrentTransform;
		PushToRegistry();
	}
}

FBox UDynamicObstacleComponent::GetCurrentBounds() const
{
	if (const UPrimitiveComponent* PrimComp = Cast<UPrimitiveComponent>(GetOwner()->GetRootComponent()))
	{
		return PrimComp->Bounds.GetBox();
	}
	return FBox(ForceInit);
}

void UDynamicObstacleComponent::PushToRegistry()
{
	UWorld* World = GetWorld();
	UOctreeNavSubsystem* NavSubsystem = World ? World->GetSubsystem<UOctreeNavSubsystem>() : nullptr;
	const AActor* OwnerActor = GetOwner();
	if (!NavSubsystem || !OwnerActor)
	{
		return;
	}

	const FBox Bounds = GetCurrentBounds();
	if (!Bounds.IsValid)
	{
		return;
	}

	FDynamicObstaclePrim Prim;
	Prim.OwnerId = GetUniqueID();
	Prim.Center = Bounds.GetCenter();
	Prim.Velocity = Velocity;

	if (const USphereComponent* SphereRoot = Cast<USphereComponent>(OwnerActor->GetRootComponent()))
	{
		Prim.Shape = FDynamicObstaclePrim::EShape::Sphere;
		Prim.HalfExtents = FVector(SphereRoot->GetScaledSphereRadius(), 0.f, 0.f);
	}
	else
	{
		Prim.Shape = FDynamicObstaclePrim::EShape::Box;
		Prim.Rotation = FQuat::Identity;
		Prim.HalfExtents = Bounds.GetExtent();
	}

	NavSubsystem->GetObstacleRegistry().Update(Prim);
}

void UDynamicObstacleComponent::RemoveFromRegistry()
{
	if (UWorld* World = GetWorld())
	{
		if (UOctreeNavSubsystem* NavSubsystem = World->GetSubsystem<UOctreeNavSubsystem>())
		{
			NavSubsystem->GetObstacleRegistry().Remove(GetUniqueID());
		}
	}
}
