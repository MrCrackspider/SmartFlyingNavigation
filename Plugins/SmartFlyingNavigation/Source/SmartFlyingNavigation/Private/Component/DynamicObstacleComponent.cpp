// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Component/DynamicObstacleComponent.h"
#include "Components/BoxComponent.h"
#include "Kismet/GameplayStatics.h"
#include "NavVolume/OctreeVoxelVolume.h"

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
		return;
	}

	TArray<AActor*> FoundVolumes;
	UGameplayStatics::GetAllActorsOfClass(this, AOctreeVoxelVolume::StaticClass(), FoundVolumes);

	if (FoundVolumes.Num() == 0)
	{
		SetComponentTickEnabled(false);
		return;
	}

	for (AActor* VolumeActor : FoundVolumes)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(VolumeActor))
		{
			if (Volume->GetVolumeBox() && Volume->GetVolumeBox()->Bounds.GetBox().IsInside(OwnerActor->GetActorLocation()))
			{
				AssociatedVolume = Volume;
				Volume->RegisterDynamicObstacle(this);
				
				LastKnownTransform = OwnerActor->GetActorTransform();
				UpdateLastKnownBounds();
				
				return;
			}
		}
	}

	SetComponentTickEnabled(false);
}


void UDynamicObstacleComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (AssociatedVolume.IsValid())
	{
		AssociatedVolume->UnregisterDynamicObstacle(this);
	}
	Super::EndPlay(EndPlayReason);
}

void UDynamicObstacleComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	const FTransform CurrentTransform = GetOwner()->GetActorTransform();
	if (!CurrentTransform.Equals(LastKnownTransform, 0.1f))
	{
		bIsDirty = true;
		LastKnownTransform = CurrentTransform;
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