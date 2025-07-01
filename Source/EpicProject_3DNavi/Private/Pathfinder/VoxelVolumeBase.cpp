// Fill out your copyright notice in the Description page of Project Settings.


#include "VoxelVolumeBase.h"

#include "Components/InstancedStaticMeshComponent.h"

// Sets default values
AVoxelVolumeBase::AVoxelVolumeBase()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	bDrawDebugObstacles = true; // Default to on, can be changed in editor
	DebugObstaclesComponent = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("DebugObstaclesComponent"));
	DebugObstaclesComponent->SetupAttachment(RootComponent);
	DebugObstaclesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void AVoxelVolumeBase::BeginPlay()
{
	Super::BeginPlay();
	if (bDrawDebugObstacles)
	{
		GetWorldTimerManager().SetTimer(DebugUpdateTimer, this, &AVoxelVolumeBase::UpdateDebugObstacleVisualization, 1.0f, true, 0.5f);
	}
}

void AVoxelVolumeBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}
void AVoxelVolumeBase::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
}
void AVoxelVolumeBase::UpdateDebugObstacleVisualization()
{
}

