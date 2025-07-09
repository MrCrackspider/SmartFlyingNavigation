// Fill out your copyright notice in the Description page of Project Settings.


#include "VoxelVolumeBase.h"

#include "Components/InstancedStaticMeshComponent.h"

// Sets default values
AVoxelVolumeBase::AVoxelVolumeBase()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	bDrawDebugBox = true; // Default to on, can be changed in editor
#if WITH_EDITORONLY_DATA

	static ConstructorHelpers::FObjectFinder<UStaticMesh> DebugMeshFinder(TEXT("StaticMesh'/Game/PathfinderPlugin/Debug/SM_DebugObstacle.SM_DebugObstacle'"));

	
	DebugObstaclesComponent = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("DebugObstaclesComponent"));
	if (DebugMeshFinder.Succeeded())
	{
		DebugObstaclesComponent->SetStaticMesh(DebugMeshFinder.Object);
	}
	DebugObstaclesComponent->SetupAttachment(RootComponent);
	DebugObstaclesComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
#endif
}

void AVoxelVolumeBase::BeginPlay()
{
	Super::BeginPlay();
	if (bDrawDebugBox)
	{
		GetWorldTimerManager().SetTimer(DebugUpdateTimer, this, &AVoxelVolumeBase::UpdateDebugObstacleVisualization, 0.8f, true, 0.5f);
	}
}

void AVoxelVolumeBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AVoxelVolumeBase::BakeStaticDataToChunks()
{
}

void AVoxelVolumeBase::GetDebugObstacleVoxelIndices(TArray<FIntVector>& OutObstacleIndices) const
{
}
void AVoxelVolumeBase::UpdateDebugObstacleVisualization()
{
}

void AVoxelVolumeBase::RegisterStreamingAgent(AActor* Agent)
{
}

void AVoxelVolumeBase::UnregisterStreamingAgent(AActor* Agent)
{
}

