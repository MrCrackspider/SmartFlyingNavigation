 // Fill out your copyright notice in the Description page of Project Settings.


#include "DynamicObstacle.h"

#include "Components/BoxComponent.h"
#include "Kismet/GameplayStatics.h"
#include "Pathfinder/NaviVolume/OctreeVoxelVolume.h"

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

	// 1. 월드에 있는 모든 네비게이션 볼륨을 찾습니다.
	TArray<AActor*> FoundVolumes;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AOctreeVoxelVolume::StaticClass(), FoundVolumes);

	if (FoundVolumes.Num() == 0)
	{
		// 등록할 볼륨이 없으므로, 이 컴포넌트의 틱을 비활성화하여 성능을 절약합니다.
		SetComponentTickEnabled(false);
		return;
	}

	// 2. ★★★ 핵심 로직 ★★★
	//    모든 볼륨을 순회하며, 이 액터가 어떤 볼륨의 경계 안에 있는지 확인합니다.
	for (AActor* VolumeActor : FoundVolumes)
	{
		if (AOctreeVoxelVolume* Volume = Cast<AOctreeVoxelVolume>(VolumeActor))
		{
			// 볼륨의 경계 상자(VolumeBox)가 이 액터의 위치를 포함하는지 확인합니다.
			if (Volume->GetVolumeBox() && Volume->GetVolumeBox()->Bounds.GetBox().IsInside(OwnerActor->GetActorLocation()))
			{
				AssociatedVolume = Volume;
				Volume->RegisterDynamicObstacle(this);
				
				LastKnownTransform = OwnerActor->GetActorTransform();
				UpdateLastKnownBounds();
				
				UE_LOG(LogTemp, Log, TEXT("DynamicObstacleComponent on '%s' registered with volume '%s'."), *OwnerActor->GetName(), *Volume->GetName());
				return;
			}
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("DynamicObstacleComponent on '%s' is not inside any OctreeVoxelVolume. Disabling tick."), *OwnerActor->GetName());
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
	if (!CurrentTransform.Equals(LastKnownTransform, 0.1f)) // 0.1 유닛 이상 움직이면 Dirty
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