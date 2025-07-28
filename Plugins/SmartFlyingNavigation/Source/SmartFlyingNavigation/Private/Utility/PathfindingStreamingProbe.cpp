// Copyright © 2025 TeamHamDev. All Rights Reserved.

#include "Utility/PathfindingStreamingProbe.h"
#include "Components/WorldPartitionStreamingSourceComponent.h"

APathfindingStreamingProbe::APathfindingStreamingProbe()
{
	PrimaryActorTick.bCanEverTick = false;
	SetActorEnableCollision(false);

	StreamingSource = CreateDefaultSubobject<UWorldPartitionStreamingSourceComponent>(TEXT("StreamingSource"));
	USceneComponent* SceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("SceneComponent"));
	SetRootComponent(SceneComponent);
	//SceneComponent->SetMobility(EComponentMobility::Movable);

	// Set a high streaming priority.
	// This ensures that load requests from this probe are prioritized over the player's streaming source.
	StreamingSource->Priority = EStreamingSourcePriority::Highest;
}

void APathfindingStreamingProbe::SetupStreamingSource(float Radius)
{
	if (StreamingSource)
	{
		StreamingSource->Shapes.Empty();

		FStreamingSourceShape NewShape;
		NewShape.bIsSector = false;
		NewShape.Radius = Radius;
		NewShape.Location = FVector::ZeroVector;

		StreamingSource->Shapes.Add(NewShape);
	}
}

void APathfindingStreamingProbe::ActivateProbe(const FVector& TargetLocation)
{
	SetActorLocation(TargetLocation);

	UE_LOG(LogTemp, Warning, TEXT("Probe's location AFTER SetActorLocation: %s"), *GetActorLocation().ToString());

	if (!StreamingSource->IsStreamingSourceEnabled())
	{
		StreamingSource->EnableStreamingSource();
	}
}

void APathfindingStreamingProbe::DeactivateProbe()
{
	if (StreamingSource->IsStreamingSourceEnabled())
	{
		StreamingSource->DisableStreamingSource();
	}
}